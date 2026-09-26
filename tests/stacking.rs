//! M4 stacking behavior — public API only. Bounds live in `common` and come
//! from spec.md (SC-001, SC-002, SC-005) and its edge cases.
//!
//! Run with `--nocapture` to see the metrics that drive tuning.

mod common;

use common::*;
use mirage::World;

/// Everything worth knowing about a tower run, for asserting and for the
/// tuning tables in research.md.
struct TowerRun {
    violations: Vec<String>,
    sink_at: [f32; 3],   // t = 2, 10, 60 s
    max_drift: f32,
    max_tilt: f32,
    last_speed_over_max: f32, // last t with speed > SPEED_MAX
    peak_sink: (f32, f32),    // (largest top-box compression, when)
}

/// Runs an `n`-box tower for `seconds`, checking SC-001/SC-002 every step.
fn run_tower(n: usize, seconds: f32) -> TowerRun {
    let mut world = World::new();
    let ids = tower(&mut world, n);
    let top = *ids.last().unwrap();
    let ideal_top = n as f32 - 0.5;

    let mut out = TowerRun {
        violations: Vec::new(),
        sink_at: [f32::NAN; 3],
        max_drift: 0.0,
        max_tilt: 0.0,
        last_speed_over_max: 0.0,
        peak_sink: (0.0, 0.0),
    };
    let mut sink_settled = f32::NAN;
    let note = |out: &mut TowerRun, msg: String| {
        if out.violations.len() < 8 {
            out.violations.push(msg);
        }
    };

    let steps = (seconds / DT).round() as usize;
    for step in 1..=steps {
        world.step(DT);
        let t = step as f32 * DT;
        let s = sink(&world, top, ideal_top);
        let drift = max_drift(&world, &ids, 0.0);
        let tilt = max_tilt(&world, &ids);
        let sp = max_speed(&world, &ids);

        if s > out.peak_sink.0 {
            out.peak_sink = (s, t);
        }
        out.max_drift = out.max_drift.max(drift);
        out.max_tilt = out.max_tilt.max(tilt);
        if sp > SPEED_MAX {
            out.last_speed_over_max = t;
        }
        for (slot, mark) in [2.0_f32, 10.0, 60.0].iter().enumerate() {
            if (t - mark).abs() < DT / 2.0 {
                out.sink_at[slot] = s;
            }
        }
        if (t - SINK_SETTLE_SECONDS).abs() < DT / 2.0 {
            sink_settled = s;
        }
        if step % 60 == 0 && (step / 60) % 5 == 0 {
            report(&format!("tower{n}"), t, s, drift, sp);
        }

        if !all_finite(&world, &ids) {
            note(&mut out, format!("t={t:.2}: non-finite state"));
            break;
        }
        if drift > DRIFT_MAX {
            note(&mut out, format!("t={t:.2}: drift {drift:.4} > {DRIFT_MAX}"));
        }
        if tilt > 0.05 {
            note(&mut out, format!("t={t:.2}: tilt {tilt:.4} rad > 0.05"));
        }
        let sink_limit = if t < SINK_STARTUP_SECONDS { SINK_STARTUP_MAX } else { SINK_MAX };
        if s > sink_limit {
            note(&mut out, format!("t={t:.2}: sink {s:.4} > {sink_limit}"));
        }
        if t >= SINK_SETTLE_SECONDS {
            if s > SINK_SETTLED_MAX {
                note(&mut out, format!("t={t:.2}: settled sink {s:.4} > {SINK_SETTLED_MAX}"));
            }
            if s - sink_settled > CREEP_MAX {
                note(&mut out, format!("t={t:.2}: creep {:.4} > {CREEP_MAX}", s - sink_settled));
            }
        }
        if t >= SPEED_EARLY_SECONDS && sp > SPEED_EARLY_MAX {
            note(&mut out, format!("t={t:.2}: speed {sp:.4} > {SPEED_EARLY_MAX}"));
        }
        if t >= SPEED_SETTLE_SECONDS && sp > SPEED_MAX {
            note(&mut out, format!("t={t:.2}: speed {sp:.4} > {SPEED_MAX}"));
        }
    }

    println!(
        "  SUMMARY tower{n}: peak sink {:.4} at {:.2}s, sink@2/10/60 = {:.4}/{:.4}/{:.4}, max drift {:.4}, max tilt {:.4}, last speed>{SPEED_MAX} at {:.2}s, violations {}",
        out.peak_sink.0, out.peak_sink.1, out.sink_at[0], out.sink_at[1], out.sink_at[2], out.max_drift, out.max_tilt, out.last_speed_over_max, out.violations.len()
    );
    out
}

#[test]
fn tower_stands_for_sixty_seconds() {
    let run = run_tower(10, 60.0);
    assert!(run.violations.is_empty(), "10-box tower violated its bounds:\n  {}", run.violations.join("\n  "));
}

// --- Stress scenes the tuning must also satisfy ---------------------------

use mirage::{BodyId, RigidBody, Shape, Vec2};

/// US3-1 / SC-005: a 5-row pyramid stands for 30 s.
#[test]
fn pyramid_stays_standing_for_thirty_seconds() {
    let mut world = World::new();
    let ids = pyramid(&mut world, 5);
    let start: Vec<Vec2> = ids.iter().map(|&id| world.body(id).position).collect();

    let mut violations: Vec<String> = Vec::new();
    let (mut max_dx, mut max_dy, mut max_tilt_seen, mut last_slow) = (0.0_f32, 0.0_f32, 0.0_f32, 0.0_f32);
    for step in 1..=1800 {
        world.step(DT);
        let t = step as f32 * DT;
        for (i, &id) in ids.iter().enumerate() {
            let b = world.body(id);
            let dx = (b.position.x - start[i].x).abs();
            let below = start[i].y - b.position.y;
            let tilt = b.orientation.angle().abs();
            max_dx = max_dx.max(dx);
            max_dy = max_dy.max(below);
            max_tilt_seen = max_tilt_seen.max(tilt);
            let sp = speed(b, 0.5);
            if sp > SPEED_MAX {
                last_slow = t;
            }
            let mut fail = |msg: String| {
                if violations.len() < 8 {
                    violations.push(format!("t={t:.2} box {i}: {msg}"));
                }
            };
            if !b.position.x.is_finite() || !b.position.y.is_finite() {
                fail("non-finite".into());
            }
            if dx > 0.1 {
                fail(format!("drifted {dx:.4} > 0.1"));
            }
            if below > 0.1 {
                fail(format!("sank {below:.4} > 0.1"));
            }
            if tilt > 0.05 {
                fail(format!("tilted {tilt:.4} > 0.05"));
            }
            if t >= 10.0 && sp > 0.05 {
                fail(format!("speed {sp:.4} > 0.05"));
            }
        }
    }
    println!("  SUMMARY pyramid: max drift {max_dx:.4}, max sink {max_dy:.4}, max tilt {max_tilt_seen:.4}, last speed>{SPEED_MAX} at {last_slow:.2}s, violations {}", violations.len());
    assert!(violations.is_empty(), "pyramid violated its bounds:\n  {}", violations.join("\n  "));
}

/// US3-2: boxes and circles landing on a floor come to rest.
#[test]
fn mixed_boxes_and_circles_come_to_rest() {
    let mut world = World::new();
    world.add_body(floor(0.5));
    let mut ids: Vec<BodyId> = Vec::new();
    for (i, x) in [-7.5_f32, -4.5, -1.5].iter().enumerate() {
        ids.push(world.add_body(RigidBody::new_dynamic(Vec2::new(*x, 3.0 + 2.0 * i as f32), 1.0, unit_box())));
    }
    for (i, x) in [1.5_f32, 4.5, 7.5].iter().enumerate() {
        ids.push(world.add_body(RigidBody::new_dynamic(Vec2::new(*x, 4.0 + 2.0 * i as f32), 1.0, Shape::circle(0.5))));
    }

    let mut prev: Vec<Vec2> = ids.iter().map(|&id| world.body(id).position).collect();
    let mut violations: Vec<String> = Vec::new();
    let (mut worst_speed, mut worst_step, mut worst_sink) = (0.0_f32, 0.0_f32, 0.0_f32);
    for step in 1..=1200 {
        world.step(DT);
        let t = step as f32 * DT;
        for (i, &id) in ids.iter().enumerate() {
            let b = world.body(id);
            let moved = (b.position - prev[i]).length();
            prev[i] = b.position;
            if t >= 15.0 {
                worst_speed = worst_speed.max(speed(b, 0.5));
                worst_step = worst_step.max(moved);
                let mut fail = |msg: String| {
                    if violations.len() < 8 {
                        violations.push(format!("t={t:.2} body {i}: {msg}"));
                    }
                };
                if speed(b, 0.5) > 0.02 {
                    fail(format!("speed {:.4} > 0.02", speed(b, 0.5)));
                }
                if moved >= 1e-3 {
                    fail(format!("moved {moved:.5} this step (>= 1e-3)"));
                }
            }
            // Resting center height is 0.5 for both shapes; allow 2% of size.
            let sunk = 0.5 - b.position.y;
            worst_sink = worst_sink.max(sunk);
            if sunk > 0.02 && t >= 2.0 && violations.len() < 8 {
                violations.push(format!("t={t:.2} body {i}: sunk {sunk:.4} > 0.02"));
            }
        }
    }
    println!("  SUMMARY mixed: last-5s worst speed {worst_speed:.4}, worst step {worst_step:.5}, worst sink {worst_sink:.4}, violations {}", violations.len());
    assert!(violations.is_empty(), "mixed scene violated its bounds:\n  {}", violations.join("\n  "));
}

/// Edge case: a stack taller than 10 may be soft but must not explode.
#[test]
fn tall_tower_does_not_explode() {
    let mut world = World::new();
    let ids = tower(&mut world, 15);
    let mut worst = 0.0_f32;
    for step in 1..=1800 {
        world.step(DT);
        assert!(all_finite(&world, &ids), "non-finite state at step {step}");
        // The first second is the start-up free-fall while contacts are first
        // detected (speeds of ~2.5 m/s are normal there); after that a soft
        // tower sways slowly but must stay well below 1 m/s.
        if step as f32 * DT >= 1.0 {
            worst = worst.max(max_speed(&world, &ids));
            assert!(max_speed(&world, &ids) < 1.0, "speed {} at step {step}", max_speed(&world, &ids));
        }
    }
    println!("  SUMMARY tall15: worst speed after 1 s {worst:.4}, top sink {:.4}", sink(&world, *ids.last().unwrap(), 14.5));
}

/// Edge case: a very heavy body on a very light one must stay stable.
#[test]
fn heavy_on_light_stays_finite_and_bounded() {
    let mut world = World::new();
    world.add_body(floor(0.5));
    let light = world.add_body(RigidBody::new_dynamic(Vec2::new(0.0, 0.5), 1.0, unit_box()));
    let heavy = world.add_body(RigidBody::new_dynamic(Vec2::new(0.0, 1.5), 1000.0, unit_box()));
    let ids = [light, heavy];
    let mut worst = 0.0_f32;
    for step in 1..=600 {
        world.step(DT);
        assert!(all_finite(&world, &ids), "non-finite state at step {step}");
        if step as f32 * DT >= 2.0 {
            worst = worst.max(max_speed(&world, &ids));
            assert!(max_speed(&world, &ids) < 1.0, "speed {} at step {step}", max_speed(&world, &ids));
        }
    }
    println!("  SUMMARY heavy-on-light: worst speed after 2 s {worst:.4}, heavy y {:.4}", world.body(heavy).position.y);
}

/// SC-007 / FR-013: the same scene run twice is bit-identical after 60 s.
#[test]
fn tower_is_bit_identical_across_runs() {
    let build = || {
        let mut world = World::new();
        let ids = tower(&mut world, 10);
        (world, ids)
    };
    let ((mut w1, ids), (mut w2, _)) = (build(), build());
    for _ in 0..3600 {
        w1.step(DT);
        w2.step(DT);
    }
    for &id in &ids {
        assert_eq!(w1.body(id), w2.body(id), "body {id:?} diverged between runs");
    }
}

/// Constitution II: rendering framerate must not matter. The demos call
/// `step(get_frame_time())` with whatever the display gives them; a tower
/// driven by a mix of 144, 75 and 30 Hz frames for 60 s must end as settled
/// as the fixed-step run.
#[test]
fn tower_is_stable_under_variable_frame_times() {
    let mut world = World::new();
    let ids = tower(&mut world, 10);
    let top = *ids.last().unwrap();
    let frames = [1.0 / 144.0, 1.0 / 75.0, 1.0 / 30.0, 1.0 / 144.0, 1.0 / 60.0];
    let mut t = 0.0_f32;
    let mut i = 0;
    while t < 60.0 {
        let dt = frames[i % frames.len()];
        world.step(dt);
        t += dt;
        i += 1;
        assert!(all_finite(&world, &ids), "non-finite state at t = {t}");
    }
    let (s, drift, sp) = (sink(&world, top, 9.5), max_drift(&world, &ids, 0.0), max_speed(&world, &ids));
    report("tower10 mixed frames", t, s, drift, sp);
    assert!(drift <= DRIFT_MAX, "drift {drift}");
    assert!(s <= SINK_SETTLED_MAX, "sink {s}");
    assert!(sp <= SPEED_MAX, "speed {sp}");
}
