//! M7 distance / spring joint behavior — public API only.
//!
//! Analytic references:
//! - critical damping `c = 2√(k·m)`;
//! - damped period `T_d = 2π / (ω·√(1−ζ²))`, `ω = √(k/m)`, `ζ = c / (2√(k·m))`;
//! - simple pendulum `T = 2π√(L/g)` (a bob anchored at its own center on a
//!   massless rod — the rigid joint applies no torque, so the bob's own
//!   inertia never enters).

mod common;

use common::*;
use mirage::{BodyId, DistanceJoint, RevoluteJoint, RigidBody, Shape, Vec2, World};
use std::f32::consts::PI;

/// Length error tolerated for a rigid joint — generous relative to the
/// engine's `JOINT_SLOP` (0.002), as in `tests/joints.rs`.
const ANCHOR_TOL: f32 = 0.02;

const HOOK: Vec2 = Vec2 { x: 0.0, y: 8.0 };
const REST: f32 = 2.0;
const HALF: f32 = 0.5;
const MASS: f32 = 1.0;
const K: f32 = 10.0;

fn world_anchor(world: &World, id: BodyId, anchor_local: Vec2) -> Vec2 {
    let body = world.body(id);
    body.position + body.orientation.rotate(anchor_local)
}

fn separation(world: &World, a: BodyId, anchor_a: Vec2, b: BodyId, anchor_b: Vec2) -> f32 {
    (world_anchor(world, b, anchor_b) - world_anchor(world, a, anchor_a)).length()
}

fn assert_finite(body: &RigidBody, context: &str) {
    let values = [
        body.position.x,
        body.position.y,
        body.velocity.x,
        body.velocity.y,
        body.angular_velocity,
        body.orientation.angle(),
    ];
    assert!(values.iter().all(|v| v.is_finite()), "{context}: non-finite state {body:?}");
}

// --- User Story 1: a box hanging on a spring --------------------------------

/// Equilibrium spring length for the hanging box: rest length plus the
/// static stretch from the box's weight.
fn equilibrium(k: f32) -> f32 {
    REST + MASS * G / k
}

/// A static hook at `HOOK` and a 1 kg, 1 m box hanging from it by its
/// top-centre anchor on a spring of rest length `REST`. The box starts at
/// rest, `displacement` past its equilibrium length, directly below the
/// hook.
fn hanging_box(world: &mut World, k: f32, c: f32, displacement: f32) -> (BodyId, BodyId) {
    let hook = world.add_body(RigidBody::new_static(HOOK, Shape::circle(0.05)));
    let anchor_y = HOOK.y - (equilibrium(k) + displacement);
    let bx = world.add_body(RigidBody::new_dynamic(
        Vec2::new(HOOK.x, anchor_y - HALF),
        MASS,
        box_shape(HALF),
    ));
    world.add_distance_joint(
        DistanceJoint::new(hook, bx, Vec2::ZERO, Vec2::new(0.0, HALF))
            .with_length(REST)
            .with_spring(k, c),
    );
    (hook, bx)
}

fn stretch_from_equilibrium(world: &World, hook: BodyId, bx: BodyId, k: f32) -> f32 {
    separation(world, hook, Vec2::ZERO, bx, Vec2::new(0.0, HALF)) - equilibrium(k)
}

/// Step indices where `x` crosses zero going upward.
fn upward_crossings(xs: &[f32]) -> Vec<usize> {
    (1..xs.len()).filter(|&i| xs[i - 1] < 0.0 && xs[i] >= 0.0).collect()
}

fn mean_spacing(idx: &[usize]) -> f32 {
    (idx[idx.len() - 1] - idx[0]) as f32 / (idx.len() - 1) as f32
}

#[test]
fn critically_damped_spring_settles_without_overshoot() {
    let mut world = World::new();
    let (hook, bx) = hanging_box(&mut world, K, DistanceJoint::critical_damping(K, MASS), 1.0);

    let mut min_stretch = f32::MAX;
    for step in 0..(20.0 / DT) as usize {
        world.step(DT);
        let stretch = stretch_from_equilibrium(&world, hook, bx, K);
        min_stretch = min_stretch.min(stretch);
        assert!(stretch >= -0.01, "step {step}: overshot equilibrium by {}", -stretch);
        if step as f32 * DT >= 10.0 {
            assert!(stretch.abs() <= 0.01, "step {step}: not settled, stretch {stretch}");
        }
    }
    println!("critical damping: max overshoot past equilibrium = {:.5} m", (-min_stretch).max(0.0));
}

#[test]
fn undamped_spring_conserves_energy() {
    let mut world = World::new();
    let (hook, bx) = hanging_box(&mut world, K, 0.0, 1.0);
    let inertia = box_shape(HALF).inertia(MASS);
    let e_osc = 0.5 * K * 1.0 * 1.0;

    let energy = |world: &World| {
        let body = world.body(bx);
        let l = separation(world, hook, Vec2::ZERO, bx, Vec2::new(0.0, HALF));
        0.5 * MASS * body.velocity.length_squared()
            + 0.5 * inertia * body.angular_velocity * body.angular_velocity
            + MASS * G * body.position.y
            + 0.5 * K * (l - REST) * (l - REST)
    };

    let e0 = energy(&world);
    let mut es = Vec::new();
    for _ in 0..(60.0 / DT) as usize {
        world.step(DT);
        es.push(energy(&world));
    }

    let max_dev = es.iter().map(|e| (e - e0).abs()).fold(0.0, f32::max) / e_osc;
    let period = (2.0 * PI / (K / MASS).sqrt() / DT).round() as usize;
    let mean = |s: &[f32]| s.iter().sum::<f32>() / s.len() as f32;
    let drift = (mean(&es[es.len() - period..]) - mean(&es[..period])) / e_osc;
    println!("undamped spring: max |E − E0| / E_osc = {max_dev:.4}, period-mean drift = {drift:+.5}");

    assert!(max_dev < 0.05, "instantaneous energy strayed {max_dev:.4} of E_osc");
    assert!(drift.abs() < 0.01, "energy drifted {drift:+.5} of E_osc over 60 s");
}

#[test]
fn underdamped_spring_period_matches_analytic() {
    let zeta: f32 = 0.1;
    let mut world = World::new();
    let (hook, bx) = hanging_box(&mut world, K, zeta * DistanceJoint::critical_damping(K, MASS), 1.0);

    let mut xs = Vec::new();
    for _ in 0..(30.0 / DT) as usize {
        world.step(DT);
        xs.push(stretch_from_equilibrium(&world, hook, bx, K));
    }

    let crossings = upward_crossings(&xs);
    assert!(crossings.len() >= 4, "too few oscillations: {}", crossings.len());
    let measured = mean_spacing(&crossings) * DT;
    let analytic = 2.0 * PI / ((K / MASS).sqrt() * (1.0 - zeta * zeta).sqrt());
    let err = (measured - analytic).abs() / analytic;
    println!("under-damped period: measured {measured:.4} s, analytic {analytic:.4} s, error {:.2}%", err * 100.0);
    assert!(err < 0.02, "period error {:.2}%", err * 100.0);

    let swing = |range: &[f32]| range.iter().map(|x| x.abs()).fold(0.0, f32::max);
    let (first, last) = (crossings[0], crossings[crossings.len() - 1]);
    let period = crossings[1] - crossings[0];
    assert!(swing(&xs[last - period..last]) < swing(&xs[first..first + period]), "did not ring down");
    assert!(xs[xs.len() - 60..].iter().all(|x| x.abs() < 0.01), "did not come to rest");
}

#[test]
fn spring_between_two_free_bodies_conserves_momentum() {
    let mut world = World::new();
    let a = world.add_body(RigidBody::new_dynamic(Vec2::new(-1.5, 50.0), 1.0, Shape::circle(0.1)));
    let b = world.add_body(RigidBody::new_dynamic(Vec2::new(1.5, 50.0), 1.0, Shape::circle(0.1)));
    let reference = world.add_body(RigidBody::new_dynamic(Vec2::new(10.0, 50.0), 1.0, Shape::circle(0.1)));
    world.add_distance_joint(
        DistanceJoint::new(a, b, Vec2::ZERO, Vec2::ZERO).with_length(2.0).with_spring(K, 0.0),
    );

    let mut min_sep = f32::MAX;
    for step in 0..(5.0 / DT) as usize {
        world.step(DT);
        let (pa, pb) = (world.body(a).position, world.body(b).position);
        let mid = (pa + pb) * 0.5;
        let vx_sum = world.body(a).velocity.x + world.body(b).velocity.x;
        let sep = (pb - pa).length();
        min_sep = min_sep.min(sep);

        assert!(mid.x.abs() < 1e-4, "step {step}: midpoint drifted to x = {}", mid.x);
        assert!(vx_sum.abs() < 1e-4, "step {step}: x-momentum {vx_sum}");
        assert!((mid.y - world.body(reference).position.y).abs() < 1e-3, "step {step}: spring added net force");
        assert!((0.9..=3.1).contains(&sep), "step {step}: separation {sep}");
    }
    assert!(min_sep < 1.1, "never swung through rest: min separation {min_sep}");
}

#[test]
fn spring_with_coincident_anchors_stays_finite() {
    // Anchors start coincident with rest length 0: the line is undefined.
    let mut world = World::new();
    let hook = world.add_body(RigidBody::new_static(Vec2::new(0.0, 5.0), Shape::circle(0.05)));
    let bob = world.add_body(RigidBody::new_dynamic(Vec2::new(0.0, 5.0), 1.0, Shape::circle(0.1)));
    world.add_distance_joint(
        DistanceJoint::new(hook, bob, Vec2::ZERO, Vec2::ZERO).with_length(0.0).with_spring(K, 1.0),
    );
    for step in 0..(2.0 / DT) as usize {
        world.step(DT);
        assert_finite(world.body(bob), &format!("coincident, step {step}"));
    }

    // Rest length 0, starting 1 m apart: a spring pulling toward a point.
    let mut world = World::new();
    let hook = world.add_body(RigidBody::new_static(Vec2::new(0.0, 5.0), Shape::circle(0.05)));
    let bob = world.add_body(RigidBody::new_dynamic(Vec2::new(1.0, 5.0), 1.0, Shape::circle(0.1)));
    world.add_distance_joint(
        DistanceJoint::new(hook, bob, Vec2::ZERO, Vec2::ZERO).with_length(0.0).with_spring(K, 1.0),
    );
    for step in 0..(2.0 / DT) as usize {
        world.step(DT);
        assert_finite(world.body(bob), &format!("zero rest length, step {step}"));
        // Gravity also pulls the bob down, so judge the spring by the
        // horizontal offset, which only the spring acts on: about a
        // quarter period (π/2ω ≈ 0.5 s) in, it should be near zero.
        if step == (0.5 / DT) as usize {
            let dx = (world.body(bob).position.x - world.body(hook).position.x).abs();
            assert!(dx < 0.5, "not pulled toward the hook: horizontal offset {dx}");
        }
    }
}

// --- User Story 2: rigid distance -------------------------------------------

const PIVOT: Vec2 = Vec2 { x: 0.0, y: 5.0 };
const ROD: f32 = 2.0;

/// A 0.1 m bob on a rigid rod of length `ROD` from a static pivot, anchored
/// at its own centre, released `angle` radians off straight down. No
/// `with_length`: the rest length comes from the starting separation.
fn rigid_pendulum(world: &mut World, angle: f32) -> (BodyId, BodyId, mirage::DistanceJointId) {
    let pivot = world.add_body(RigidBody::new_static(PIVOT, Shape::circle(0.05)));
    let bob_pos = PIVOT + Vec2::new(ROD * angle.sin(), -ROD * angle.cos());
    let bob = world.add_body(RigidBody::new_dynamic(bob_pos, 1.0, Shape::circle(0.1)));
    let id = world.add_distance_joint(DistanceJoint::new(pivot, bob, Vec2::ZERO, Vec2::ZERO));
    (pivot, bob, id)
}

#[test]
fn rigid_distance_holds_length_under_gravity() {
    let mut world = World::new();
    let (pivot, bob, id) = rigid_pendulum(&mut world, 0.5);
    assert!((world.distance_joint(id).length.unwrap() - ROD).abs() < 1e-5);

    let mut worst = 0.0f32;
    for step in 0..600 {
        world.step(DT);
        let err = (separation(&world, pivot, Vec2::ZERO, bob, Vec2::ZERO) - ROD).abs();
        worst = worst.max(err);
        assert!(err < ANCHOR_TOL, "step {step}: length error {err}");
    }
    println!("rigid distance: worst length error {worst:.5} m");
}

#[test]
fn rigid_distance_pendulum_period_matches_simple_pendulum() {
    let mut world = World::new();
    let (_, bob, _) = rigid_pendulum(&mut world, 0.1);

    let mut xs = Vec::new();
    for _ in 0..(20.0 / DT) as usize {
        world.step(DT);
        xs.push(world.body(bob).position.x - PIVOT.x);
        assert!(world.body(bob).angular_velocity.abs() < 1e-3, "bob started spinning");
    }

    let crossings = upward_crossings(&xs);
    let measured = mean_spacing(&crossings) * DT;
    let analytic = 2.0 * PI * (ROD / G).sqrt();
    let err = (measured - analytic).abs() / analytic;
    println!("rigid pendulum period: measured {measured:.4} s, analytic {analytic:.4} s, error {:.2}%", err * 100.0);
    assert!(err < 0.02, "period error {:.2}%", err * 100.0);
}

#[test]
fn rigid_distance_corrects_initial_gap_gradually() {
    // (start separation, target length): stretched, then compressed.
    for (start, target) in [(3.0f32, 2.0f32), (1.0, 2.0)] {
        let mut world = World::new();
        let a = world.add_body(RigidBody::new_dynamic(Vec2::new(0.0, 100.0), 1.0, Shape::circle(0.1)));
        let b = world.add_body(RigidBody::new_dynamic(Vec2::new(start, 100.0), 1.0, Shape::circle(0.1)));
        world.add_distance_joint(DistanceJoint::new(a, b, Vec2::ZERO, Vec2::ZERO).with_length(target));

        world.step(DT);
        let after_one = separation(&world, a, Vec2::ZERO, b, Vec2::ZERO);
        let moved = (after_one - start).abs();
        // From rest, Cdot = 0 and gravity is shared, so only the positional
        // correction acts: at most 0.4 · (|C| − slop).
        let max_step = 0.4 * ((start - target).abs() - 0.002);
        assert!(moved > 0.0, "{start} → {target}: no correction");
        assert!(moved <= max_step + 1e-4, "{start} → {target}: snapped by {moved} (max {max_step})");
        assert!((after_one - target).abs() < (start - target).abs(), "{start} → {target}: moved the wrong way");

        for _ in 1..120 {
            world.step(DT);
        }
        let err = (separation(&world, a, Vec2::ZERO, b, Vec2::ZERO) - target).abs();
        assert!(err < ANCHOR_TOL, "{start} → {target}: still {err} off after 2 s");
    }
}

// --- Cross-cutting: mixed joints, contacts, determinism ---------------------

#[test]
fn spring_and_revolute_on_one_body() {
    // A 2 m lid hinged at its left end, held up by a damped spring from its
    // free end to a hook above it.
    let mut world = World::new();
    let pin_pos = Vec2::new(0.0, 5.0);
    let pin = world.add_body(RigidBody::new_static(pin_pos, Shape::circle(0.05)));
    let hook = world.add_body(RigidBody::new_static(Vec2::new(2.0, 7.0), Shape::circle(0.05)));
    let lid = world.add_body(RigidBody::new_dynamic(Vec2::new(1.0, 5.0), 1.0, rect_shape(1.0, 0.05)));
    let hinge = Vec2::new(-1.0, 0.0);
    world.add_joint(RevoluteJoint::new(pin, lid, Vec2::ZERO, hinge));
    world.add_distance_joint(
        DistanceJoint::new(hook, lid, Vec2::ZERO, Vec2::new(1.0, 0.0))
            .with_length(1.0)
            .with_spring(30.0, 6.0),
    );

    for step in 0..(10.0 / DT) as usize {
        world.step(DT);
        assert_finite(world.body(lid), &format!("step {step}"));
        let gap = separation(&world, pin, Vec2::ZERO, lid, hinge);
        assert!(gap < ANCHOR_TOL, "step {step}: hinge gap {gap}");
    }
    let w = world.body(lid).angular_velocity;
    assert!(w.abs() < 0.01, "lid still swinging after 10 s: ω = {w}");
}

fn mixed_scene() -> (World, Vec<BodyId>) {
    let mut world = World::new();
    let (hook, bx) = hanging_box(&mut world, K, 0.1 * DistanceJoint::critical_damping(K, MASS), 1.0);
    let (pivot, bob, _) = rigid_pendulum(&mut world, 0.5);
    (world, vec![hook, bx, pivot, bob])
}

#[test]
fn distance_joint_scene_is_deterministic() {
    let (mut first, ids) = mixed_scene();
    let (mut second, _) = mixed_scene();
    for _ in 0..600 {
        first.step(DT);
        second.step(DT);
    }
    for &id in &ids {
        let (a, b) = (first.body(id), second.body(id));
        assert_eq!(a.position, b.position);
        assert_eq!(a.velocity, b.velocity);
        assert_eq!(a.orientation, b.orientation);
        assert_eq!(a.angular_velocity, b.angular_velocity);
    }
}

/// A 1 m box resting on the floor, hung from a hook at (0, 3) by a rigid
/// joint at its top-centre anchor. No `with_length`, so the rest length is
/// the 2.0 the box already satisfies while resting: the joint and the
/// floor contact can both hold at once.
fn box_on_floor_and_rod(kick: f32) -> (World, BodyId, BodyId) {
    let mut world = World::new();
    world.add_body(floor(0.5));
    let hook = world.add_body(RigidBody::new_static(Vec2::new(0.0, 3.0), Shape::circle(0.05)));
    let mut body = RigidBody::new_dynamic(Vec2::new(0.0, HALF), 1.0, box_shape(HALF));
    body.velocity.x = kick;
    let bx = world.add_body(body);
    world.add_distance_joint(DistanceJoint::new(hook, bx, Vec2::ZERO, Vec2::new(0.0, HALF)));
    (world, hook, bx)
}

#[test]
fn rigid_distance_joint_with_contact() {
    let anchor = Vec2::new(0.0, HALF);

    // At rest: both constraints already satisfied, so it must stay put.
    let (mut world, hook, bx) = box_on_floor_and_rod(0.0);
    for step in 0..(5.0 / DT) as usize {
        world.step(DT);
        let body = world.body(bx);
        assert_finite(body, &format!("at rest, step {step}"));
        let err = (separation(&world, hook, Vec2::ZERO, bx, anchor) - 2.0).abs();
        assert!(err < ANCHOR_TOL, "at rest, step {step}: length error {err}");
        let penetration = -(body.position.y - HALF);
        assert!(penetration < 0.05, "at rest, step {step}: penetration {penetration}");
        if step as f32 * DT >= 3.0 {
            assert!(speed(body, HALF) < 0.05, "at rest, step {step}: speed {}", speed(body, HALF));
        }
    }

    // Kicked: it swings up off the floor on its rod and comes back down onto
    // it. How the tilted box meets the floor is ordinary contact handling;
    // the joint's own guarantee is only its length.
    let (mut world, hook, bx) = box_on_floor_and_rod(1.0);
    for step in 0..(5.0 / DT) as usize {
        world.step(DT);
        assert_finite(world.body(bx), &format!("kicked, step {step}"));
        let err = (separation(&world, hook, Vec2::ZERO, bx, anchor) - 2.0).abs();
        assert!(err < ANCHOR_TOL, "kicked, step {step}: length error {err}");
    }
}
