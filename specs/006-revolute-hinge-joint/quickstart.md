# Quickstart: Revolute (Hinge) Joint

**Feature**: [spec.md](spec.md) | **Contract**: [contracts/joint-api.md](contracts/joint-api.md)

## Building the pendulum scene (what `examples/hinge.rs` demonstrates)

```rust
use mirage::{RigidBody, RevoluteJoint, Shape, Vec2, World};

let mut world = World::new();

// A fixed pin point in the world, and a rod hanging from it.
let pin = world.add_body(RigidBody::new_static(Vec2::new(0.0, 5.0), Shape::circle(0.05)));
let rod = world.add_body(RigidBody::new_dynamic(
    Vec2::new(0.0, 2.5),   // rod's center, 2.5 units below the pin
    1.0,
    Shape::polygon(vec![
        Vec2::new(-0.1, -2.5), Vec2::new(0.1, -2.5),
        Vec2::new(0.1, 2.5), Vec2::new(-0.1, 2.5),
    ]),
));

// Pin the rod's top end (local +y) to the fixed point (its own local origin).
world.add_joint(RevoluteJoint::new(
    pin, rod,
    Vec2::ZERO,          // anchor on the static pin body, at its own center
    Vec2::new(0.0, 2.5), // anchor on the rod, at its top end (local frame)
));

// Give it a small starting angle so it swings.
// (set rod's initial orientation/angular_velocity here, then loop:)
loop {
    world.step(1.0 / 60.0);
    // read world.body(rod).orientation.angle() to drive rendering / measure period
}
```

## Running it

```
cargo run --example hinge --release
cargo test                                  # unit tests + tests/joints.rs behavioral scenes
cargo test --release --test joints -- --nocapture   # pendulum period measurement
```

`--release` is required for the demo, same as every other example in this
repo (`README.md` § Running it) — debug-build solver iteration is
meaningfully slower.

## What "done" looks like

- The rod's pinned end (its anchor point) never visibly separates from the
  fixed pin as it swings, for the full length of the demo run.
- Released from a small angle, the rod's oscillation period is within a few
  percent of the small-angle pendulum formula `T = 2π√(L/g)`, where `L` is
  the distance from the pin to the rod's center of mass (for a uniform rod
  pinned at one end, the *physical* pendulum period formula — using the
  rod's moment of inertia about the pin, not the point-mass formula — is the
  correct analytic comparison; `tests/joints.rs` computes it from the rod's
  known geometry rather than approximating it as a point mass).
