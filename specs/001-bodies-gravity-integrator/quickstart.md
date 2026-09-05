# Quickstart: Bodies, Gravity & Fixed-Timestep Integrator (M1)

## Run the tests

```
cargo test
```

Covers: gravity-driven velocity/position integration for dynamic bodies,
static bodies remaining unchanged, and fixed-timestep accumulator behavior
under variable input timing (per spec SC-001, SC-002, SC-003, SC-005).

## Run the visual acceptance demo

```
cargo run --example bouncing --release
```

`--release` is required — debug builds are not an accepted way to verify
this milestone's behavioral "done when" criterion (Constitution Principle
IV).

**Expected observation** (spec SC-004): circles fall with visibly
increasing downward speed and exit the window. No circle stops, bounces,
or reacts to any other circle or floor — there is no collision detection
yet (M2).

## Minimal library usage

```rust
use mirage::{RigidBody, Vec2, World};

let mut world = World::new();
let ball = world.add_body(RigidBody::new_dynamic(Vec2::new(0.0, 10.0), 1.0));
let ground = world.add_body(RigidBody::new_static(Vec2::new(0.0, 0.0)));

// Drive the simulation with real elapsed time each frame; the world's
// internal fixed-timestep accumulator handles the rest.
world.step(1.0 / 60.0);

assert!(world.body(ball).velocity.y < 0.0); // fell under gravity
assert_eq!(world.body(ground).position, Vec2::new(0.0, 0.0)); // unmoved
```

## Milestone exit criteria (from README)

- [ ] `cargo test` passes with gravity/integrator/accumulator coverage.
- [ ] `cargo run --example bouncing --release` shows circles falling at
      `9.81 units/s²` with no collision.
