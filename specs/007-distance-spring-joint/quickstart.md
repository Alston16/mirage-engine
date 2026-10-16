# Quickstart: Distance / Spring Joint

**Feature**: [spec.md](spec.md) | **Contract**: [contracts/distance-joint-api.md](contracts/distance-joint-api.md)

## A box on a critically damped spring (what `examples/spring.rs` shows)

```rust
use mirage::{DistanceJoint, RigidBody, Shape, Vec2, World};

let mut world = World::new();

let mass = 1.0;
let k = 10.0;           // N/m → ω = √(k/m) ≈ 3.16 rad/s, ≈ 0.5 Hz
let rest = 2.0;         // unloaded spring length
let sag = mass * 9.81 / k; // static stretch under the box's weight ≈ 0.98

let hook = world.add_body(RigidBody::new_static(Vec2::new(0.0, 8.0), Shape::circle(0.05)));
// Box's top-centre anchor sits directly below the hook, 1 m past equilibrium.
let half = 0.5;
let box_y = 8.0 - (rest + sag + 1.0) - half;
let bx = world.add_body(RigidBody::new_dynamic(Vec2::new(0.0, box_y), mass, Shape::polygon(vec![
    Vec2::new(-half, -half), Vec2::new(half, -half), Vec2::new(half, half), Vec2::new(-half, half),
])));

world.add_distance_joint(
    DistanceJoint::new(hook, bx, Vec2::ZERO, Vec2::new(0.0, half))
        .with_length(rest)
        .with_spring(k, DistanceJoint::critical_damping(k, mass)),
);

for _ in 0..600 {
    world.step(1.0 / 60.0);
}
// The box rests with its anchor rest + sag below the hook, with no overshoot on the way.
```

(`tests/common::box_shape(half)` wraps that same polygon for the tests.)

## Rigid distance: a pendulum on a massless rod

```rust
let pivot = world.add_body(RigidBody::new_static(Vec2::new(0.0, 5.0), Shape::circle(0.05)));
let bob = world.add_body(RigidBody::new_dynamic(
    Vec2::new(2.0 * 0.1_f32.sin(), 5.0 - 2.0 * 0.1_f32.cos()), 1.0, Shape::circle(0.1)));
// No .with_length: L0 is taken from the current separation (2.0).
world.add_distance_joint(DistanceJoint::new(pivot, bob, Vec2::ZERO, Vec2::ZERO));
// Period ≈ 2π√(2/9.81) ≈ 2.84 s; the bob doesn't spin (anchor at its centre).
```

## Verifying M7

```
cargo test --test distance_joints            # SC-001..SC-004 + edge cases
cargo test                                   # all prior suites unchanged (FR-011)
cargo run --example spring --release         # watch: ringing / no-overshoot / endless
```

The done-when check passes when the critically damped box settles without
visible overshoot and the undamped box keeps the same amplitude for the
whole run, both watched with `--release`.
