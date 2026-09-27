//! Shared scene builders, metrics and acceptance bounds for the M4
//! behavioral tests. Public API only — the same items an example uses.
//!
//! Units are meters and seconds: boxes are 1 m, mass 1, `g = 9.81`.
#![allow(dead_code)]

use mirage::{BodyId, RigidBody, Rot2, Shape, Vec2, World};

pub const DT: f32 = 1.0 / 60.0;
pub const G: f32 = 9.81;

// --- Acceptance bounds (spec.md, incl. its 2026-09-20 Clarifications) -----

/// SC-001: horizontal drift of any tower box, in box widths.
pub const DRIFT_MAX: f32 = 0.05;
/// SC-001: seconds of start-up transient, while contacts are first detected.
pub const SINK_STARTUP_SECONDS: f32 = 1.0;
/// SC-001: top-box compression bound during start-up, in box heights.
pub const SINK_STARTUP_MAX: f32 = 0.20;
/// SC-001: top-box compression bound from `SINK_STARTUP_SECONDS` on.
pub const SINK_MAX: f32 = 0.10;
/// SC-001: top-box compression bound once settled, in box heights.
pub const SINK_SETTLED_MAX: f32 = 0.03;
/// SC-001: seconds after which `SINK_SETTLED_MAX` applies.
pub const SINK_SETTLE_SECONDS: f32 = 10.0;
/// SC-001: allowed growth of compression between the settle time and the end.
pub const CREEP_MAX: f32 = 0.001;
/// SC-002: seconds after which `SPEED_EARLY_MAX` applies.
pub const SPEED_EARLY_SECONDS: f32 = 2.0;
/// SC-002: speed bound from `SPEED_EARLY_SECONDS` on, in m/s.
pub const SPEED_EARLY_MAX: f32 = 0.1;
/// SC-002: seconds after which `SPEED_MAX` applies.
pub const SPEED_SETTLE_SECONDS: f32 = 30.0;
/// SC-002: speed bound from `SPEED_SETTLE_SECONDS` on, in m/s.
pub const SPEED_MAX: f32 = 0.01;
/// US1-1: a held box moves less than this fraction of its width in 10 s.
pub const HOLD_MOVE_MAX: f32 = 0.01;
/// US1-1: a held box's final speed bound, in m/s.
pub const HOLD_SPEED_MAX: f32 = 0.01;
/// SC-004: relative tolerance on sliding / rolling acceleration.
pub const ACCEL_TOL: f32 = 0.05;

// --- Scene builders --------------------------------------------------------

/// Unit box, half-extent 0.5, centered on the origin.
pub fn unit_box() -> Shape {
    box_shape(0.5)
}

pub fn box_shape(half: f32) -> Shape {
    rect_shape(half, half)
}

pub fn rect_shape(half_x: f32, half_y: f32) -> Shape {
    Shape::polygon(vec![
        Vec2::new(-half_x, -half_y),
        Vec2::new(half_x, -half_y),
        Vec2::new(half_x, half_y),
        Vec2::new(-half_x, half_y),
    ])
}

/// Static floor whose top surface is `y = 0`.
pub fn floor(friction: f32) -> RigidBody {
    RigidBody::new_static(Vec2::new(0.0, -10.0), rect_shape(100.0, 10.0)).with_friction(friction)
}

/// Static ramp: a 40 × 1 slab centered on the origin, rotated by `angle`
/// (CCW, so its right end is higher). Its top surface passes through
/// `R(angle)·(x, 0.5)`.
pub fn ramp(angle: f32, friction: f32) -> RigidBody {
    let mut body = RigidBody::new_static(Vec2::ZERO, rect_shape(20.0, 0.5)).with_friction(friction);
    body.orientation = Rot2::new(angle);
    body
}

/// Downhill unit direction along a ramp tilted by `angle`.
pub fn downhill(angle: f32) -> Vec2 {
    Vec2::new(-angle.cos(), -angle.sin())
}

/// Adds a ramp and a 1 m, mass-1 box resting exactly on its surface.
pub fn box_on_ramp(world: &mut World, angle: f32, box_friction: f32, ramp_friction: f32) -> BodyId {
    world.add_body(ramp(angle, ramp_friction));
    let center = Rot2::new(angle).rotate(Vec2::new(0.0, 0.5 + 0.5));
    let mut body = RigidBody::new_dynamic(center, 1.0, unit_box()).with_friction(box_friction);
    body.orientation = Rot2::new(angle);
    world.add_body(body)
}

/// Adds a ramp and a radius-0.5, mass-1 disc resting on its surface, both
/// with the default friction.
pub fn disc_on_ramp(world: &mut World, angle: f32) -> BodyId {
    world.add_body(ramp(angle, 0.5));
    let center = Rot2::new(angle).rotate(Vec2::new(0.0, 0.5 + 0.5));
    world.add_body(RigidBody::new_dynamic(center, 1.0, Shape::circle(0.5)))
}

/// Adds the floor and `n` unit boxes (mass 1) stacked exactly touching.
/// Returns the box ids bottom to top.
pub fn tower(world: &mut World, n: usize) -> Vec<BodyId> {
    world.add_body(floor(0.5));
    (0..n)
        .map(|i| {
            world.add_body(RigidBody::new_dynamic(
                Vec2::new(0.0, 0.5 + i as f32),
                1.0,
                unit_box(),
            ))
        })
        .collect()
}

/// Adds the floor and a pyramid: row `r` (from the bottom) holds
/// `rows − r` unit boxes centered on `x = 0`, exactly touching.
pub fn pyramid(world: &mut World, rows: usize) -> Vec<BodyId> {
    world.add_body(floor(0.5));
    let mut ids = Vec::new();
    for r in 0..rows {
        let count = rows - r;
        for i in 0..count {
            let x = i as f32 - (count as f32 - 1.0) / 2.0;
            ids.push(world.add_body(RigidBody::new_dynamic(
                Vec2::new(x, 0.5 + r as f32),
                1.0,
                unit_box(),
            )));
        }
    }
    ids
}

// --- Metrics ---------------------------------------------------------------

/// SC-002's speed: the larger of linear speed and angular speed times
/// half-width.
pub fn speed(body: &RigidBody, half_extent: f32) -> f32 {
    body.velocity.length().max(body.angular_velocity.abs() * half_extent)
}

/// Downward compression of `top` relative to its ideal touching height.
pub fn sink(world: &World, top: BodyId, ideal_y: f32) -> f32 {
    ideal_y - world.body(top).position.y
}

/// Largest horizontal displacement from `x0` over `ids`.
pub fn max_drift(world: &World, ids: &[BodyId], x0: f32) -> f32 {
    ids.iter().map(|&id| (world.body(id).position.x - x0).abs()).fold(0.0, f32::max)
}

/// Largest speed over `ids` (unit boxes: half-extent 0.5).
pub fn max_speed(world: &World, ids: &[BodyId]) -> f32 {
    ids.iter().map(|&id| speed(world.body(id), 0.5)).fold(0.0, f32::max)
}

/// Largest `|orientation angle|` over `ids`.
pub fn max_tilt(world: &World, ids: &[BodyId]) -> f32 {
    ids.iter().map(|&id| world.body(id).orientation.angle().abs()).fold(0.0, f32::max)
}

/// True if every body's state is finite.
pub fn all_finite(world: &World, ids: &[BodyId]) -> bool {
    ids.iter().all(|&id| {
        let b = world.body(id);
        b.position.x.is_finite()
            && b.position.y.is_finite()
            && b.velocity.x.is_finite()
            && b.velocity.y.is_finite()
            && b.angular_velocity.is_finite()
            && b.orientation.angle().is_finite()
    })
}

/// Prints one metrics line; run with `--nocapture` to paste into research.md.
pub fn report(label: &str, t: f32, sink: f32, drift: f32, speed: f32) {
    println!("  {label}: t={t:>5.1}s sink={sink:+.4} drift={drift:.4} speed={speed:.4}");
}

/// Steps `world` by `seconds` of simulated time (fixed steps).
pub fn run(world: &mut World, seconds: f32) {
    for _ in 0..(seconds / DT).round() as usize {
        world.step(DT);
    }
}
