//! Sequential-impulse solver — README § Resolution.
//!
//! Notation follows the README: `n` is the contact normal (from body `a`
//! toward body `b`), `r_a`/`r_b` the vectors from each body's center to the
//! contact point, `vr` the relative velocity at the contact, and `j` the
//! normal impulse.

use crate::{Manifold, RigidBody, Vec2};

/// Velocity-solver passes per fixed step (README § Resolution: ~8).
pub(crate) const VELOCITY_ITERATIONS: usize = 8;

/// Penetration tolerated before positional correction kicks in, so resting
/// contacts don't jitter.
pub(crate) const PENETRATION_SLOP: f32 = 0.01;

/// Fraction of the excess penetration removed per step.
pub(crate) const CORRECTION_PERCENT: f32 = 0.4;

/// Approach speed below which a contact doesn't bounce (float-noise floor).
const MIN_BOUNCE_SPEED: f32 = 1e-4;

/// Per-contact-point solver state, built once per step and discarded.
struct ContactState {
    a: usize,
    b: usize,
    /// Unit normal, `a → b`.
    n: Vec2,
    r_a: Vec2,
    r_b: Vec2,
    /// `K = 1/m_a + 1/m_b + (r_a×n)²/I_a + (r_b×n)²/I_b`.
    k: f32,
    /// Target normal velocity `−e·(vr·n)₀`, or `0` for a resting contact.
    bounce: f32,
    /// Accumulated normal impulse; invariant `j_acc >= 0`.
    j_acc: f32,
    penetration: f32,
    /// `1 / points in the source manifold`.
    share: f32,
}

/// Builds solver state for every contact point. Contacts between two
/// bodies that can't move (`K == 0`) are dropped.
fn build_contacts(
    bodies: &[RigidBody],
    manifolds: &[Manifold],
    gravity: Vec2,
    dt: f32,
) -> Vec<ContactState> {
    let g_dt = gravity * dt;
    let mut contacts = Vec::new();
    for manifold in manifolds {
        let a = manifold.body_a.0 as usize;
        let b = manifold.body_b.0 as usize;
        let (body_a, body_b) = (&bodies[a], &bodies[b]);
        let share = 1.0 / manifold.points.len() as f32;

        for point in &manifold.points {
            let n = point.normal;
            let r_a = point.point - body_a.position;
            let r_b = point.point - body_b.position;
            let ra_x_n = r_a.cross(n);
            let rb_x_n = r_b.cross(n);
            let k = body_a.inv_mass
                + body_b.inv_mass
                + ra_x_n * ra_x_n * body_a.inv_inertia
                + rb_x_n * rb_x_n * body_b.inv_inertia;
            if k == 0.0 {
                continue;
            }

            // Target normal velocity −e·(vr·n)₀, so (1 + e) is applied once,
            // not re-applied every iteration. (vr·n)₀ is the approach velocity
            // *before* this step's gravity kick, which World::step has already
            // added: a body resting on a floor then approaches at 0 and never
            // bounces, and a bouncing body decays as v → e·v instead of
            // locking into the limit cycle v = e·(v + g·dt). The pair's
            // restitution is the larger of the two.
            let vr0 = (body_b.velocity + Vec2::cross_sv(body_b.angular_velocity, r_b))
                - (body_a.velocity + Vec2::cross_sv(body_a.angular_velocity, r_a));
            let kicked = |body: &RigidBody| if body.is_static { 0.0 } else { 1.0 };
            let vn_approach =
                vr0.dot(n) - g_dt.dot(n) * (kicked(body_b) - kicked(body_a));
            let e = body_a.restitution.max(body_b.restitution);
            let bounce = if -vn_approach > MIN_BOUNCE_SPEED { -e * vn_approach } else { 0.0 };

            contacts.push(ContactState {
                a,
                b,
                n,
                r_a,
                r_b,
                k,
                bounce,
                j_acc: 0.0,
                penetration: point.penetration,
                share,
            });
        }
    }
    contacts
}

/// Resolves every contact in `manifolds`: velocity iterations, then
/// positional correction. Position integration stays in `World::step`.
pub(crate) fn resolve(
    bodies: &mut [RigidBody],
    manifolds: &[Manifold],
    gravity: Vec2,
    dt: f32,
) {
    let mut contacts = build_contacts(bodies, manifolds, gravity, dt);

    for _ in 0..VELOCITY_ITERATIONS {
        for c in &mut contacts {
            let (v_a, w_a) = (bodies[c.a].velocity, bodies[c.a].angular_velocity);
            let (v_b, w_b) = (bodies[c.b].velocity, bodies[c.b].angular_velocity);

            // vr = (v_b + ω_b × r_b) − (v_a + ω_a × r_a)
            let vr = (v_b + Vec2::cross_sv(w_b, c.r_b)) - (v_a + Vec2::cross_sv(w_a, c.r_a));
            let vn = vr.dot(c.n);

            // Iterated form of j = −(1 + e)(vr·n) / K: drive vr·n toward the
            // target `bounce`, clamping the *accumulated* impulse to >= 0 so
            // contacts push but never pull.
            let dj = -(vn - c.bounce) / c.k;
            let j_new = (c.j_acc + dj).max(0.0);
            let j = j_new - c.j_acc;
            c.j_acc = j_new;

            let impulse = c.n * j;
            let body_a = &mut bodies[c.a];
            if !body_a.is_static {
                body_a.velocity -= impulse * body_a.inv_mass;
                body_a.angular_velocity -= c.r_a.cross(impulse) * body_a.inv_inertia;
            }
            let body_b = &mut bodies[c.b];
            if !body_b.is_static {
                body_b.velocity += impulse * body_b.inv_mass;
                body_b.angular_velocity += c.r_b.cross(impulse) * body_b.inv_inertia;
            }
        }
    }

    correct_positions(bodies, &contacts);
}

/// Baumgarte-style positional bias, applied as a position projection rather
/// than a velocity bias so the restitution solve stays energy-clean: move the
/// bodies apart along `n` by
/// `percent · share · max(penetration − slop, 0) / (1/m_a + 1/m_b)`,
/// weighted by each body's inverse mass. `share = 1 / points`, so a
/// two-point manifold isn't corrected twice as hard as a one-point one.
fn correct_positions(bodies: &mut [RigidBody], contacts: &[ContactState]) {
    for c in contacts {
        let (inv_a, inv_b) = (bodies[c.a].inv_mass, bodies[c.b].inv_mass);
        let inv_sum = inv_a + inv_b;
        if inv_sum == 0.0 {
            continue;
        }
        let correction =
            CORRECTION_PERCENT * c.share * (c.penetration - PENETRATION_SLOP).max(0.0) / inv_sum;
        let shift = c.n * correction;
        if !bodies[c.a].is_static {
            bodies[c.a].position -= shift * inv_a;
        }
        if !bodies[c.b].is_static {
            bodies[c.b].position += shift * inv_b;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::collision::detect_contacts;
    use crate::collision::manifold::Contact;
    use crate::world::FIXED_DT;
    use crate::{BodyId, Rot2, Shape, World};

    /// Floor top surface sits at `y = 0`.
    fn floor() -> RigidBody {
        RigidBody::new_static(
            Vec2::new(0.0, -10.0),
            Shape::polygon(vec![
                Vec2::new(-100.0, -10.0),
                Vec2::new(100.0, -10.0),
                Vec2::new(100.0, 10.0),
                Vec2::new(-100.0, 10.0),
            ]),
        )
    }

    fn square() -> Shape {
        Shape::polygon(vec![
            Vec2::new(-1.0, -1.0),
            Vec2::new(1.0, -1.0),
            Vec2::new(1.0, 1.0),
            Vec2::new(-1.0, 1.0),
        ])
    }

    /// A world holding `body` and the floor, added in either order — the
    /// `(Circle, Polygon)` ordering is where M2 returned a reversed normal.
    fn scene(body: RigidBody, floor_first: bool) -> (World, BodyId, BodyId) {
        let mut world = World::new();
        if floor_first {
            let f = world.add_body(floor());
            let b = world.add_body(body);
            (world, b, f)
        } else {
            let b = world.add_body(body);
            let f = world.add_body(floor());
            (world, b, f)
        }
    }

    fn run(world: &mut World, steps: usize) {
        for _ in 0..steps {
            world.step(FIXED_DT);
        }
    }

    fn ball(y: f32) -> RigidBody {
        RigidBody::new_dynamic(Vec2::new(0.0, y), 1.0, Shape::circle(1.0))
    }

    #[test]
    fn e0_ball_stops_dead_on_floor() {
        for floor_first in [true, false] {
            let (mut world, id, _) = scene(ball(5.0), floor_first);

            let mut impact_speed = 0.0_f32;
            let mut landed = false;
            for _ in 0..200 {
                world.step(FIXED_DT);
                if !landed {
                    impact_speed = impact_speed.max(-world.body(id).velocity.y);
                }
                if !world.contacts().is_empty() {
                    landed = true;
                    assert!(
                        world.body(id).velocity.y.abs() < 1e-3 * impact_speed,
                        "floor_first={floor_first}: vy = {} after landing at {impact_speed}",
                        world.body(id).velocity.y
                    );
                    assert!(world.body(id).position.y <= 1.0 + 1e-3, "ball rebounded");
                }
            }
            assert!(landed, "ball never touched the floor");
        }
    }

    #[test]
    fn ball_settles_on_floor_without_falling_through() {
        for floor_first in [true, false] {
            let (mut world, id, _) = scene(ball(5.0), floor_first);
            run(&mut world, 300);
            let y = world.body(id).position.y;
            // Landing overshoot is at most one step of impact speed.
            let impact_speed = (2.0 * 9.81_f32 * 4.0).sqrt();
            assert!(
                y >= 1.0 - impact_speed * FIXED_DT - 1e-3,
                "floor_first={floor_first}: ball fell through, y = {y}"
            );
            assert!(y <= 1.0 + 1e-3, "floor_first={floor_first}: ball rose, y = {y}");
        }
    }

    #[test]
    fn head_on_dynamic_bodies_conserve_momentum_along_normal() {
        let mut world = World::new();
        let mut a = RigidBody::new_dynamic(Vec2::new(0.0, 100.0), 1.0, Shape::circle(1.0));
        let mut b = RigidBody::new_dynamic(Vec2::new(1.9, 100.0), 3.0, Shape::circle(1.0));
        a.velocity = Vec2::new(2.0, 0.0);
        b.velocity = Vec2::new(-2.0, 0.0);
        let ida = world.add_body(a);
        let idb = world.add_body(b);

        world.step(FIXED_DT);

        let (va, vb) = (world.body(ida).velocity.x, world.body(idb).velocity.x);
        let momentum = 1.0 * va + 3.0 * vb;
        assert!((momentum - (1.0 * 2.0 + 3.0 * -2.0)).abs() < 1e-4, "momentum {momentum}");
        assert!((va - 2.0).abs() > (vb - -2.0).abs(), "lighter body should change more");
    }

    #[test]
    fn separating_contact_is_left_alone() {
        let mut a = RigidBody::new_dynamic(Vec2::new(0.0, 0.0), 1.0, Shape::circle(1.0));
        let mut b = RigidBody::new_dynamic(Vec2::new(1.9, 0.0), 1.0, Shape::circle(1.0));
        a.velocity = Vec2::new(-1.0, 0.0);
        b.velocity = Vec2::new(1.0, 0.0);
        let mut bodies = vec![a, b];
        let manifolds = detect_contacts(&bodies);
        assert!(!manifolds.is_empty());

        resolve(&mut bodies, &manifolds, Vec2::ZERO, FIXED_DT);

        assert_eq!(bodies[0].velocity, Vec2::new(-1.0, 0.0));
        assert_eq!(bodies[1].velocity, Vec2::new(1.0, 0.0));
        assert_eq!(bodies[0].angular_velocity, 0.0);
        assert_eq!(bodies[1].angular_velocity, 0.0);
    }

    #[test]
    fn static_bodies_never_change_and_static_pair_is_ignored() {
        for floor_first in [true, false] {
            let (mut world, _, f) = scene(ball(5.0), floor_first);
            let before = world.body(f).clone();
            run(&mut world, 300);
            assert_eq!(*world.body(f), before);
        }

        let mut world = World::new();
        let s1 = world.add_body(RigidBody::new_static(Vec2::new(0.0, 0.0), Shape::circle(1.0)));
        let s2 = world.add_body(RigidBody::new_static(Vec2::new(1.0, 0.0), Shape::circle(1.0)));
        let (b1, b2) = (world.body(s1).clone(), world.body(s2).clone());
        run(&mut world, 10);
        assert_eq!(*world.body(s1), b1);
        assert_eq!(*world.body(s2), b2);
    }

    #[test]
    fn accumulated_normal_impulse_is_never_negative() {
        // Static floor (body 0) under a dynamic box (body 1) that is tilting:
        // the box's right contact point is separating, its left one closing.
        let mut box_body = RigidBody::new_dynamic(Vec2::new(0.0, 1.0), 1.0, square());
        box_body.velocity = Vec2::new(0.0, -1.0);
        box_body.angular_velocity = 2.0;
        let mut bodies = vec![floor(), box_body];
        let n = Vec2::new(0.0, 1.0);
        let manifold = Manifold {
            body_a: BodyId(0),
            body_b: BodyId(1),
            points: vec![
                Contact { point: Vec2::new(-1.0, 0.0), normal: n, penetration: 0.0 },
                Contact { point: Vec2::new(1.0, 0.0), normal: n, penetration: 0.0 },
            ],
        };

        resolve(&mut bodies, &[manifold], Vec2::ZERO, FIXED_DT);

        // Right point started separating (vn = +1). A pulling impulse would
        // drag it toward 0; the clamp must leave it separating.
        let b = &bodies[1];
        let r_right = Vec2::new(1.0, 0.0) - b.position;
        let vn_right = (b.velocity + Vec2::cross_sv(b.angular_velocity, r_right)).y;
        assert!(vn_right > 0.1, "separating point was pulled back: vn = {vn_right}");
        // The floor can only push the box up.
        assert!(b.velocity.y >= -1.0);
    }

    #[test]
    fn off_center_polygon_hit_spins_the_body() {
        // Rotated +30° (CCW): the lowest corner lies left of center, so the
        // floor's upward impulse there torques the box clockwise (ω < 0).
        let mut tilted = RigidBody::new_dynamic(Vec2::new(0.0, 3.0), 1.0, square());
        tilted.orientation = Rot2::new(30.0_f32.to_radians());
        for floor_first in [true, false] {
            let (mut world, id, _) = scene(tilted.clone(), floor_first);
            let mut min_w = 0.0_f32;
            for _ in 0..120 {
                world.step(FIXED_DT);
                min_w = min_w.min(world.body(id).angular_velocity);
            }
            assert!(min_w < -0.1, "floor_first={floor_first}: min ω = {min_w}");
        }
    }

    #[test]
    fn box_landing_flat_does_not_tip() {
        for floor_first in [true, false] {
            let flat = RigidBody::new_dynamic(Vec2::new(0.0, 3.0), 1.0, square());
            let (mut world, id, _) = scene(flat, floor_first);
            run(&mut world, 120);
            let body = world.body(id);
            assert!(
                body.orientation.angle().abs() < 0.02,
                "floor_first={floor_first}: tipped to {}",
                body.orientation.angle()
            );
        }
    }

    #[test]
    fn identical_scenes_are_bit_identical() {
        fn build() -> World {
            let mut world = World::new();
            world.add_body(floor());
            world.add_body(ball(4.0));
            world.add_body(RigidBody::new_dynamic(Vec2::new(0.5, 8.0), 2.0, square()));
            let mut tilted = RigidBody::new_dynamic(Vec2::new(-3.0, 6.0), 1.5, square());
            tilted.orientation = Rot2::new(0.4);
            world.add_body(tilted);
            world
        }
        let (mut w1, mut w2) = (build(), build());
        run(&mut w1, 300);
        run(&mut w2, 300);
        for i in 0..4 {
            let id = BodyId(i);
            assert_eq!(w1.body(id), w2.body(id), "body {i} diverged");
        }
    }

    /// Drops `body` from rest onto the floor. Returns the speed the ball was
    /// falling at just before its first contact step, the vertical velocity
    /// right after that step, and the peak center height of the first
    /// rebound.
    fn first_bounce(body: RigidBody, floor_first: bool) -> (f32, f32, f32) {
        let (mut world, id, _) = scene(body, floor_first);

        let mut prev_vy = 0.0;
        for _ in 0..2000 {
            world.step(FIXED_DT);
            if !world.contacts().is_empty() {
                // Speed the ball was falling at before the contact step.
                let closing_speed = -prev_vy;
                let rebound_vy = world.body(id).velocity.y;

                let mut peak = world.body(id).position.y;
                for _ in 0..2000 {
                    world.step(FIXED_DT);
                    let y = world.body(id).position.y;
                    if y > peak {
                        peak = y;
                    } else if world.body(id).velocity.y <= 0.0 {
                        break;
                    }
                }
                return (closing_speed, rebound_vy, peak);
            }
            prev_vy = world.body(id).velocity.y;
        }
        panic!("ball never touched the floor");
    }

    #[test]
    fn e1_ball_returns_to_drop_height() {
        for floor_first in [true, false] {
            let drop_y = 20.0;
            let (_, _, peak) = first_bounce(ball(drop_y).with_restitution(1.0), floor_first);
            // Heights above the ball's resting center height (its radius, 1.0).
            let (drop_h, peak_h) = (drop_y - 1.0, peak - 1.0);
            assert!(
                (peak_h - drop_h).abs() / drop_h < 0.05,
                "floor_first={floor_first}: dropped from {drop_h}, rebounded to {peak_h}"
            );
        }
    }

    #[test]
    fn e_half_ball_rebounds_at_half_speed() {
        for floor_first in [true, false] {
            let drop_y = 20.0;
            let (closing, rebound_vy, peak) =
                first_bounce(ball(drop_y).with_restitution(0.5), floor_first);
            assert!(
                (rebound_vy - 0.5 * closing).abs() / (0.5 * closing) < 0.05,
                "floor_first={floor_first}: closed at {closing}, rebounded at {rebound_vy}"
            );
            let expected_peak = 1.0 + 0.25 * (drop_y - 1.0);
            assert!(
                (peak - expected_peak).abs() / expected_peak < 0.10,
                "floor_first={floor_first}: peak {peak}, expected ~{expected_peak}"
            );
        }
    }

    #[test]
    fn restitution_combine_is_order_independent() {
        let closing_pair = |first_e: f32, second_e: f32| {
            let mut world = World::new();
            let mut a = RigidBody::new_dynamic(Vec2::new(0.0, 100.0), 1.0, Shape::circle(1.0))
                .with_restitution(first_e);
            let mut b = RigidBody::new_dynamic(Vec2::new(1.9, 100.0), 1.0, Shape::circle(1.0))
                .with_restitution(second_e);
            a.velocity = Vec2::new(3.0, 0.0);
            b.velocity = Vec2::new(-3.0, 0.0);
            let ida = world.add_body(a);
            let idb = world.add_body(b);
            world.step(FIXED_DT);
            (world.body(ida).velocity.x, world.body(idb).velocity.x)
        };
        let (a_vx, b_vx) = closing_pair(0.2, 0.9);
        // With max(e) = 0.9 both bodies rebound at ~0.9 of their approach speed.
        assert!((a_vx - -2.7).abs() < 1e-3 && (b_vx - 2.7).abs() < 1e-3, "{a_vx}, {b_vx}");
        // Swapping which body carries which restitution changes nothing.
        let (a_vx_swapped, b_vx_swapped) = closing_pair(0.9, 0.2);
        assert!((a_vx_swapped - a_vx).abs() < 1e-5 && (b_vx_swapped - b_vx).abs() < 1e-5);

        // A bouncy ball on a default (e = 0) floor still bounces.
        let (_, _, peak) = first_bounce(ball(20.0).with_restitution(1.0), true);
        assert!(peak > 0.9 * 20.0);
    }

    /// Penetration of two unit circles centered `distance` apart.
    fn circle_pair_penetration(bodies: &[RigidBody]) -> f32 {
        2.0 - (bodies[1].position - bodies[0].position).length()
    }

    #[test]
    fn overlap_shrinks_without_pop() {
        let mut bodies = vec![
            RigidBody::new_dynamic(Vec2::new(0.0, 0.0), 1.0, Shape::circle(1.0)),
            RigidBody::new_dynamic(Vec2::new(1.8, 0.0), 1.0, Shape::circle(1.0)),
        ];
        let before = circle_pair_penetration(&bodies);
        let manifolds = detect_contacts(&bodies);
        resolve(&mut bodies, &manifolds, Vec2::ZERO, FIXED_DT);
        let after = circle_pair_penetration(&bodies);

        assert!(after < before, "overlap did not shrink: {before} -> {after}");
        let max_step = CORRECTION_PERCENT * (before - PENETRATION_SLOP);
        assert!(before - after <= max_step + 1e-5, "popped: removed {}", before - after);

        // A flat box's two contact points are corrected no harder than a
        // circle's single one for the same penetration (0.1).
        let lift = |body: RigidBody| {
            let mut bodies = vec![floor(), body];
            let start_y = bodies[1].position.y;
            let manifolds = detect_contacts(&bodies);
            resolve(&mut bodies, &manifolds, Vec2::ZERO, FIXED_DT);
            bodies[1].position.y - start_y
        };
        let circle_lift = lift(RigidBody::new_dynamic(Vec2::new(0.0, 0.9), 1.0, Shape::circle(1.0)));
        let box_lift = lift(RigidBody::new_dynamic(Vec2::new(0.0, 0.9), 1.0, square()));
        let expected = CORRECTION_PERCENT * (0.1 - PENETRATION_SLOP);
        assert!((circle_lift - expected).abs() < 1e-4, "circle lifted {circle_lift}");
        assert!((box_lift - expected).abs() < 1e-4, "box lifted {box_lift}");
    }

    #[test]
    fn overlap_within_slop_is_not_corrected() {
        let mut bodies = vec![
            RigidBody::new_dynamic(Vec2::new(0.0, 0.0), 1.0, Shape::circle(1.0)),
            RigidBody::new_dynamic(Vec2::new(2.0 - PENETRATION_SLOP / 2.0, 0.0), 1.0, Shape::circle(1.0)),
        ];
        let before = (bodies[0].position, bodies[1].position);
        let manifolds = detect_contacts(&bodies);
        assert!(!manifolds.is_empty(), "overlap must still be reported as a contact");

        resolve(&mut bodies, &manifolds, Vec2::ZERO, FIXED_DT);

        assert_eq!((bodies[0].position, bodies[1].position), before);
    }

    #[test]
    fn box_rests_for_ten_seconds_without_sinking_or_jitter() {
        for floor_first in [true, false] {
            let dropped = RigidBody::new_dynamic(Vec2::new(0.0, 3.0), 1.0, square());
            let (mut world, id, _) = scene(dropped, floor_first);
            run(&mut world, 60);

            let mut prev_y = world.body(id).position.y;
            for step in 0..600 {
                world.step(FIXED_DT);
                let y = world.body(id).position.y;
                let penetration = 1.0 - y;
                assert!(
                    penetration <= 2.0 * PENETRATION_SLOP,
                    "floor_first={floor_first}: step {step}: sunk {penetration}"
                );
                assert!(
                    (y - prev_y).abs() < 1e-3,
                    "floor_first={floor_first}: step {step}: jitter {}",
                    y - prev_y
                );
                prev_y = y;
            }
        }
    }

    #[test]
    fn high_restitution_ball_settles_instead_of_hopping_forever() {
        for floor_first in [true, false] {
            let (mut world, id, _) = scene(ball(5.0).with_restitution(0.8), floor_first);
            let g_dt = 9.81 * FIXED_DT;
            run(&mut world, 18 * 60);
            for step in 0..2 * 60 {
                world.step(FIXED_DT);
                let vy = world.body(id).velocity.y;
                assert!(
                    vy.abs() <= g_dt + 1e-3,
                    "floor_first={floor_first}: still hopping at final step {step}: vy = {vy}"
                );
            }
        }
    }
}
