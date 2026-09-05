# Quickstart: Collision Detection (M2)

## Run the tests

```
cargo test
```

Covers: circle–circle, circle–polygon, and polygon–polygon narrowphase
against known geometric fixtures (correct normal/penetration, including the
vertex-region and exact-touching edge cases); broadphase AABB pair rejection
(SC-003); and confirmation that no contact detection alters any body's
velocity or position (SC-004).

## Run the visual acceptance demo

```
cargo run --example ramp --release
```

(or the extended `bouncing` example, if the ramp scene is folded into it —
see plan.md Project Structure). `--release` is required — debug builds are
not an accepted way to verify this milestone's behavioral "done when"
criterion (Constitution Principle IV).

**Expected observation** (spec SC-002, README M2 "done when"): a box resting
on a ramp tilted at an arbitrary angle shows contact point(s) and a normal,
debug-drawn, that visibly coincide with the true overlap between the box
and the ramp surface. Bodies still do **not** react to contact (no bounce,
no stopping) — that's M3.

## Minimal library usage

```rust
use mirage::{RigidBody, Shape, Vec2, World};

let mut world = World::new();

let ball = world.add_body(RigidBody::new_dynamic(
    Vec2::new(0.0, 5.0),
    1.0,
    Shape::circle(0.5),
));
let ground = world.add_body(RigidBody::new_static(
    Vec2::new(0.0, 0.0),
    Shape::polygon(vec![
        Vec2::new(-10.0, -0.5),
        Vec2::new(10.0, -0.5),
        Vec2::new(10.0, 0.5),
        Vec2::new(-10.0, 0.5),
    ]),
));

world.step(1.0 / 60.0);

// Once the ball has fallen far enough to overlap the ground:
for manifold in world.contacts() {
    for contact in &manifold.points {
        println!("contact at {:?}, normal {:?}, depth {}",
            contact.point, contact.normal, contact.penetration);
    }
}
```

## Milestone exit criteria (from README)

- [ ] `cargo test` passes with broadphase and all-three-narrowphase-pair
      coverage.
- [ ] `cargo run --example ramp --release` (or equivalent) shows contact
      points/normals rendering correctly for a box resting on a rotated
      ramp.
