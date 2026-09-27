//! Free-flight projectile motion — public API only. Validates the claim
//! `examples/parabola.rs` makes visually: that `World::step`'s constant-
//! gravity integration reproduces the parabola a body traces when gravity is
//! the only force acting on it.
//!
//! Semi-implicit ("symplectic") Euler updates velocity from acceleration
//! first, then position from the *new* velocity (`world.rs`'s `step`), so it
//! does not reproduce the textbook `x = x0 + v0*t + 1/2*a*t^2` exactly:
//! sampled every `DT`, position picks up a `+ a*DT*t/2` term relative to the
//! continuous formula. `discrete_position` below is the exact closed form
//! for that stepping scheme, so comparing against it is a floating-point
//! check, not a "within some physics tolerance" one:
//!
//! `v_k = v0 + k*a*dt`
//! `p_n = p0 + dt * sum_{k=1}^{n} v_k = p0 + n*dt*v0 + a*dt^2*n*(n+1)/2`

mod common;

use common::*;
use mirage::{RigidBody, Shape, Vec2, World};

const RADIUS: f32 = 0.35;

/// Exact 1D position after `n` fixed steps of size `dt` under constant
/// acceleration `a`, for the velocity-then-position scheme `World::step`
/// uses (see module docs for the derivation).
fn discrete_position(p0: f32, v0: f32, a: f32, dt: f32, n: u32) -> f32 {
    let n = n as f32;
    p0 + n * dt * v0 + a * dt * dt * n * (n + 1.0) / 2.0
}

/// Exact velocity after `n` fixed steps under constant acceleration `a`.
fn discrete_velocity(v0: f32, a: f32, dt: f32, n: u32) -> f32 {
    v0 + n as f32 * a * dt
}

/// A body in free flight (gravity only, nothing to collide with) should
/// trace exactly the parabola the symplectic-Euler stepping scheme predicts,
/// at every step along the way — not just at the end.
#[test]
fn free_flight_matches_symplectic_euler_closed_form() {
    let mut world = World::new();
    let start = Vec2::new(0.0, 5.0);
    let v0 = Vec2::new(8.0, 10.0);
    let mut body = RigidBody::new_dynamic(start, 1.0, Shape::circle(RADIUS));
    body.velocity = v0;
    let id = world.add_body(body);

    // 1.5 s of free flight; the body climbs, peaks, and falls back well
    // clear of its start height throughout, so nothing here depends on a
    // floor or on collision handling.
    for n in 1..=90u32 {
        world.step(DT);
        let body = world.body(id);

        let expected_x = discrete_position(start.x, v0.x, 0.0, DT, n);
        let expected_y = discrete_position(start.y, v0.y, -G, DT, n);
        let expected_vy = discrete_velocity(v0.y, -G, DT, n);

        assert!((body.position.x - expected_x).abs() < 1e-3, "n={n}: x={}, expected {expected_x}", body.position.x);
        assert!((body.position.y - expected_y).abs() < 1e-3, "n={n}: y={}, expected {expected_y}", body.position.y);
        assert!((body.velocity.y - expected_vy).abs() < 1e-4, "n={n}: vy={}, expected {expected_vy}", body.velocity.y);
        assert_eq!(body.velocity.x, v0.x, "gravity must not touch horizontal velocity");
    }
}

/// End-to-end version of `examples/parabola.rs`'s scene: a circle launched
/// at 45° over a floor should come back down within a couple percent of the
/// ideal projectile range `R = v^2*sin(2θ)/g`.
#[test]
fn projectile_launched_at_45_degrees_lands_near_analytic_range() {
    let angle: f32 = 45.0_f32.to_radians();
    let launch_speed: f32 = 14.0;

    let mut world = World::new();
    world.add_body(floor(0.5));
    let start = Vec2::new(-15.0, RADIUS);
    let v0 = Vec2::new(launch_speed * angle.cos(), launch_speed * angle.sin());
    let mut body = RigidBody::new_dynamic(start, 1.0, Shape::circle(RADIUS)).with_restitution(0.4);
    body.velocity = v0;
    let id = world.add_body(body);

    // Step until the floor first reports a contact with this body, and
    // measure how far it traveled to get there.
    let mut traveled = None;
    for _ in 0..600 {
        // 10 s ceiling; it should land in well under 2 s.
        world.step(DT);
        if world.contacts().iter().any(|m| m.body_a == id || m.body_b == id) {
            traveled = Some(world.body(id).position.x - start.x);
            break;
        }
    }

    let traveled = traveled.expect("projectile never touched the floor within 10 s");
    let expected_range = launch_speed * launch_speed * (2.0 * angle).sin() / G;
    println!("  45°, v={launch_speed}: landed after {traveled:.3} m, expected range {expected_range:.3} m");
    assert!(
        (traveled - expected_range).abs() / expected_range < 0.02,
        "traveled {traveled}, expected {expected_range}"
    );
}
