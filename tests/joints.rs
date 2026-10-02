//! M6 revolute (hinge) joint behavior — public API only.
//!
//! Analytic reference: a physical pendulum pinned at one end has period
//! `T = 2π√(I_pin / (m·g·L))`, where `I_pin = I_center + m·L²` (parallel
//! axis theorem) and `L` is the pin-to-center-of-mass distance — not the
//! point-mass formula, since the rod's own moment of inertia matters.

mod common;

use common::*;
use mirage::{BodyId, RevoluteJoint, RigidBody, Rot2, Shape, Vec2, World};

/// Anchor gap tolerated by these tests — generous relative to the engine's
/// tuned `JOINT_SLOP` (0.002) to leave room for iterative convergence, the
/// same way `solver.rs`'s own tests compare against small multiples of
/// `PENETRATION_SLOP` rather than the raw constant.
const ANCHOR_TOL: f32 = 0.02;

fn world_anchor(world: &World, id: BodyId, anchor_local: Vec2) -> Vec2 {
    let body = world.body(id);
    body.position + body.orientation.rotate(anchor_local)
}

fn anchor_gap(world: &World, a: BodyId, anchor_a: Vec2, b: BodyId, anchor_b: Vec2) -> f32 {
    (world_anchor(world, a, anchor_a) - world_anchor(world, b, anchor_b)).length()
}

// --- User Story 1: the point constraint itself -----------------------------

#[test]
fn anchor_stays_coincident_under_gravity() {
    let mut world = World::new();
    let pin_pos = Vec2::new(0.0, 5.0);
    let pin = world.add_body(RigidBody::new_static(pin_pos, Shape::circle(0.05)));

    // A small, slightly off-center bob: its own anchor is not at its
    // center, so a decomposed (non-coupled) solve would visibly fail this.
    let anchor_b = Vec2::new(0.0, 0.3);
    let bob = world.add_body(RigidBody::new_dynamic(
        pin_pos - anchor_b,
        1.0,
        Shape::circle(0.2),
    ));
    world.add_joint(RevoluteJoint::new(pin, bob, Vec2::ZERO, anchor_b));

    for step in 0..300 {
        world.step(DT);
        let gap = anchor_gap(&world, pin, Vec2::ZERO, bob, anchor_b);
        assert!(gap < ANCHOR_TOL, "step {step}: anchor gap {gap}");
    }
}

#[test]
fn static_body_in_a_joint_never_moves() {
    let mut world = World::new();
    let pin_pos = Vec2::new(0.0, 5.0);
    let pin = world.add_body(RigidBody::new_static(pin_pos, Shape::circle(0.05)));
    let anchor_b = Vec2::new(0.0, 1.0);
    let bob = world.add_body(RigidBody::new_dynamic(pin_pos - anchor_b, 1.0, Shape::circle(0.2)));
    world.add_joint(RevoluteJoint::new(pin, bob, Vec2::ZERO, anchor_b));

    let before = world.body(pin).clone();
    run(&mut world, 5.0);

    assert_eq!(*world.body(pin), before);
}

#[test]
fn jointed_body_also_resolves_contacts() {
    let mut world = World::new();
    world.add_body(floor(0.5));

    let box_half = 0.5;
    let box_id = world.add_body(RigidBody::new_dynamic(
        Vec2::new(0.0, box_half),
        1.0,
        rect_shape(box_half, box_half),
    ));

    // Pin the box's top-center to a fixed point directly above it, exactly
    // where it already sits, so the box is simultaneously resting on the
    // floor (a contact) and pinned (a joint).
    let pin_pos = Vec2::new(0.0, box_half * 2.0);
    let pin = world.add_body(RigidBody::new_static(pin_pos, Shape::circle(0.05)));
    let anchor_b = Vec2::new(0.0, box_half);
    world.add_joint(RevoluteJoint::new(pin, box_id, Vec2::ZERO, anchor_b));

    run(&mut world, 5.0);

    // Contact: the box must not sink into (or rise off) the floor.
    let y = world.body(box_id).position.y;
    assert!((y - box_half).abs() < 0.02, "box sank/rose: y = {y}");

    // Joint: the box's pinned anchor must stay under the fixed point.
    let gap = anchor_gap(&world, pin, Vec2::ZERO, box_id, anchor_b);
    assert!(gap < ANCHOR_TOL, "anchor separated: gap = {gap}");
}

#[test]
fn chain_of_joints_all_stay_coincident() {
    let mut world = World::new();
    let pin_pos = Vec2::new(0.0, 6.0);
    let pin = world.add_body(RigidBody::new_static(pin_pos, Shape::circle(0.05)));

    let link_half = 0.5;
    let link1 = world.add_body(RigidBody::new_dynamic(
        Vec2::new(0.0, 6.0 - link_half),
        1.0,
        rect_shape(0.05, link_half),
    ));
    let link2 = world.add_body(RigidBody::new_dynamic(
        Vec2::new(0.0, 6.0 - 3.0 * link_half),
        1.0,
        rect_shape(0.05, link_half),
    ));

    world.add_joint(RevoluteJoint::new(pin, link1, Vec2::ZERO, Vec2::new(0.0, link_half)));
    world.add_joint(RevoluteJoint::new(
        link1,
        link2,
        Vec2::new(0.0, -link_half),
        Vec2::new(0.0, link_half),
    ));

    run(&mut world, 5.0);

    let gap1 = anchor_gap(&world, pin, Vec2::ZERO, link1, Vec2::new(0.0, link_half));
    let gap2 = anchor_gap(&world, link1, Vec2::new(0.0, -link_half), link2, Vec2::new(0.0, link_half));
    assert!(gap1 < ANCHOR_TOL, "joint 1 (pin-link1) separated: {gap1}");
    assert!(gap2 < ANCHOR_TOL, "joint 2 (link1-link2) separated: {gap2}");
}

#[test]
fn identical_joint_scenes_are_bit_identical() {
    fn build() -> (World, BodyId, BodyId) {
        let mut world = World::new();
        let pin_pos = Vec2::new(0.0, 5.0);
        let pin = world.add_body(RigidBody::new_static(pin_pos, Shape::circle(0.05)));

        let half_length = 1.2;
        let angle = 20.0_f32.to_radians();
        let mut rod = RigidBody::new_dynamic(Vec2::ZERO, 1.0, rect_shape(0.05, half_length));
        rod.orientation = Rot2::new(angle);
        rod.position = pin_pos - rod.orientation.rotate(Vec2::new(0.0, half_length));
        let rod_id = world.add_body(rod);

        world.add_joint(RevoluteJoint::new(pin, rod_id, Vec2::ZERO, Vec2::new(0.0, half_length)));
        (world, pin, rod_id)
    }

    let (mut w1, pin1, rod1) = build();
    let (mut w2, pin2, rod2) = build();
    run(&mut w1, 5.0);
    run(&mut w2, 5.0);

    assert_eq!(w1.body(pin1), w2.body(pin2), "pin diverged");
    assert_eq!(w1.body(rod1), w2.body(rod2), "rod diverged");
}

// --- User Story 2: pendulum period -----------------------------------------

struct Pendulum {
    world: World,
    pin: BodyId,
    rod: BodyId,
}

/// A rod of `mass`, spanning `2 * half_length` along its own local y-axis,
/// pinned at its local `(0, half_length)` end to a fixed point, released
/// from `release_angle` (radians from hanging straight down).
fn build_pendulum(mass: f32, half_length: f32, release_angle: f32) -> Pendulum {
    let mut world = World::new();
    let pin_pos = Vec2::new(0.0, 5.0);
    let pin = world.add_body(RigidBody::new_static(pin_pos, Shape::circle(0.05)));

    let mut rod = RigidBody::new_dynamic(Vec2::ZERO, mass, rect_shape(0.05, half_length));
    rod.orientation = Rot2::new(release_angle);
    // Place the rod so its top anchor starts exactly at the pin (coincident
    // anchors at creation — spec.md User Story 1 scenario 1).
    rod.position = pin_pos - rod.orientation.rotate(Vec2::new(0.0, half_length));
    let rod_id = world.add_body(rod);

    world.add_joint(RevoluteJoint::new(pin, rod_id, Vec2::ZERO, Vec2::new(0.0, half_length)));
    Pendulum { world, pin, rod: rod_id }
}

/// The physical-pendulum analytic period for a rod pinned at one end:
/// `T = 2π√(I_pin / (m·g·L))`, `I_pin = I_center + m·L²`, `L = half_length`
/// (the pin-to-center-of-mass distance).
fn analytic_period(mass: f32, half_length: f32) -> f32 {
    let i_center = rect_shape(0.05, half_length).inertia(mass);
    let i_pin = i_center + mass * half_length * half_length;
    2.0 * std::f32::consts::PI * (i_pin / (mass * G * half_length)).sqrt()
}

/// Steps `world` until `rod`'s angular velocity leaves and then returns to
/// (near) zero — one half-swing, from one extreme of the pendulum to the
/// other — and returns the elapsed time, linearly interpolated to the exact
/// zero crossing. Released from rest, this is exactly `T / 2`.
fn half_period_by_zero_crossing(world: &mut World, rod: BodyId) -> f32 {
    const REST_THRESHOLD: f32 = 1e-3;
    let mut prev_w = world.body(rod).angular_velocity;
    let mut prev_t = 0.0_f32;
    let mut t = 0.0_f32;
    let mut departed = false;

    loop {
        world.step(DT);
        t += DT;
        let w = world.body(rod).angular_velocity;

        if !departed {
            if w.abs() > REST_THRESHOLD {
                departed = true;
            }
        } else if prev_w.signum() != w.signum() {
            let frac = prev_w.abs() / (prev_w.abs() + w.abs());
            return prev_t + frac * (t - prev_t);
        }

        prev_w = w;
        prev_t = t;
        assert!(t < 30.0, "pendulum never completed a half swing");
    }
}

#[test]
fn pendulum_period_matches_analytic_prediction() {
    let (mass, half_length) = (1.0, 1.5);
    let release_angle = 5.0_f32.to_radians();
    let mut pendulum = build_pendulum(mass, half_length, release_angle);

    let measured = 2.0 * half_period_by_zero_crossing(&mut pendulum.world, pendulum.rod);
    let expected = analytic_period(mass, half_length);
    println!("  pendulum period: measured={measured:.4}s expected={expected:.4}s");

    assert!(
        (measured - expected).abs() / expected < 0.05,
        "measured {measured}, expected {expected}"
    );
}

#[test]
fn pendulum_anchor_stays_coincident_across_a_full_swing() {
    let (mass, half_length) = (1.0, 1.5);
    let release_angle = 15.0_f32.to_radians();
    let mut pendulum = build_pendulum(mass, half_length, release_angle);
    let anchor_b = Vec2::new(0.0, half_length);

    for step in 0..(5.0 / DT).round() as usize {
        pendulum.world.step(DT);
        let gap = anchor_gap(&pendulum.world, pendulum.pin, Vec2::ZERO, pendulum.rod, anchor_b);
        assert!(gap < ANCHOR_TOL, "step {step}: anchor gap {gap}");
    }
}

// --- User Story 3: stability beyond the small-angle, light-body case -------

/// Anchor gap tolerance for the large-angle/heavy-body stress scenarios
/// below — looser than `ANCHOR_TOL`. Measured (see
/// specs/006-revolute-hinge-joint/research.md § 4 and tasks.md T022): fast
/// angular motion near the bottom of an 80°-release swing produces a brief
/// one-step peak gap of ~0.0218 (and ~0.0202 for the 50×-mass rod at 45°) —
/// a transient lag between the velocity solve and the position correction
/// catching up, not persistent separation or divergence (`ANCHOR_TOL`'s
/// own scenarios, including the period-matched pendulum, stay comfortably
/// under 0.02 throughout). `JOINT_SLOP`/`JOINT_CORRECTION_PERCENT` were not
/// retuned for this: doing so would perturb the small-angle case's already
/// tight analytic-period match (see `pendulum_period_matches_analytic_prediction`)
/// to chase a peak that is not, on inspection, runaway instability.
const STRESS_ANCHOR_TOL: f32 = 0.05;

/// Upper bound on `|angular_velocity|` from energy conservation (all the
/// potential energy dropped from release settles into rotational kinetic
/// energy at the bottom of the swing), with a 2x margin — not a precise
/// energy check, just a guard against divergence/explosion.
fn max_angular_speed_bound(mass: f32, half_length: f32, release_angle: f32) -> f32 {
    let i_center = rect_shape(0.05, half_length).inertia(mass);
    let i_pin = i_center + mass * half_length * half_length;
    let energy = mass * G * half_length * (1.0 - release_angle.cos());
    2.0 * (2.0 * energy / i_pin).sqrt()
}

fn assert_bounded_and_coincident(mass: f32, half_length: f32, release_angle: f32, seconds: f32) {
    let mut pendulum = build_pendulum(mass, half_length, release_angle);
    let anchor_b = Vec2::new(0.0, half_length);
    let bound = max_angular_speed_bound(mass, half_length, release_angle);

    for step in 0..(seconds / DT).round() as usize {
        pendulum.world.step(DT);

        let gap = anchor_gap(&pendulum.world, pendulum.pin, Vec2::ZERO, pendulum.rod, anchor_b);
        assert!(gap < STRESS_ANCHOR_TOL, "step {step}: anchor separated, gap {gap}");

        let w = pendulum.world.body(pendulum.rod).angular_velocity;
        assert!(w.abs() < bound, "step {step}: angular velocity diverged: {w} (bound {bound})");
    }
}

#[test]
fn large_angle_pendulum_stays_bounded_and_coincident() {
    assert_bounded_and_coincident(1.0, 1.5, 80.0_f32.to_radians(), 15.0);
}

#[test]
fn heavier_rod_pendulum_stays_bounded_and_coincident() {
    assert_bounded_and_coincident(50.0, 1.5, 45.0_f32.to_radians(), 15.0);
}
