# Interface Contract: Distance Joint Public API

**Feature**: [../spec.md](../spec.md) | **Plan**: [../plan.md](../plan.md)

`mirage` is a library crate. Its interface is the public Rust API. This
contract fixes the new public surface M7 adds. Every item is **additive**:
no existing signature changes.

## New public items

```rust
// src/joint/distance.rs

/// A 2-body constraint on the distance between a local anchor on `body_a`
/// and a local anchor on `body_b`. Rigid by default (the separation is
/// held at `length`); with a `Spring`, a Hookean force pulls the
/// separation toward `length` instead. Never constrains direction or
/// relative rotation.
#[derive(Clone, Debug, PartialEq)]
pub struct DistanceJoint {
    pub body_a: BodyId,
    pub body_b: BodyId,
    /// Anchor on `body_a`, in `body_a`'s local frame.
    pub anchor_a: Vec2,
    /// Anchor on `body_b`, in `body_b`'s local frame.
    pub anchor_b: Vec2,
    /// Target (rest) length `L0`. `None` means "the anchors' separation
    /// when the joint is added"; `World::add_distance_joint` resolves it.
    pub length: Option<f32>,
    /// `None` = rigid; `Some` = spring.
    pub spring: Option<Spring>,
}

impl DistanceJoint {
    /// Rigid joint, length taken from the anchors' current separation
    /// when added to the world.
    pub fn new(body_a: BodyId, body_b: BodyId, anchor_a: Vec2, anchor_b: Vec2) -> Self;

    /// Sets the target / rest length `L0` explicitly.
    pub fn with_length(self, length: f32) -> Self;

    /// Switches to soft mode: `F = −(k·(L − L0) + c·L̇)` along the anchor line.
    pub fn with_spring(self, stiffness: f32, damping: f32) -> Self;

    /// Damping `c = 2·√(k·m)` that makes a spring of `stiffness` critically
    /// damped for one body of `mass` moving along the joint line.
    pub fn critical_damping(stiffness: f32, mass: f32) -> f32;
}

/// Hookean spring parameters for a soft `DistanceJoint`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Spring {
    /// `k`, force per unit of deviation from rest length.
    pub stiffness: f32,
    /// `c`, force per unit of separation speed.
    pub damping: f32,
}

/// Opaque handle to a `DistanceJoint` in the `World` that created it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct DistanceJointId(pub(crate) u32);
```

```rust
// src/world.rs: additions

impl World {
    /// Adds a distance joint, returning a handle to it. Resolves
    /// `length: None` to the anchors' current world-space separation.
    ///
    /// Panics if `body_a == body_b`, if either body is not in this world,
    /// or if `length`, `spring.stiffness` or `spring.damping` is negative
    /// or non-finite.
    pub fn add_distance_joint(&mut self, joint: DistanceJoint) -> DistanceJointId;

    /// The stored joint, with `length` always `Some(L0)`.
    pub fn distance_joint(&self, id: DistanceJointId) -> &DistanceJoint;
}
```

```rust
// src/lib.rs: re-exports (existing line extended)

pub use joint::{DistanceJoint, DistanceJointId, JointId, RevoluteJoint, Spring};
```

## Behavioral guarantees

| Guarantee | Source |
|---|---|
| Rigid: `\|L − L0\|` stays within `ANCHOR_TOL` (0.02) over a sustained run | SC-004 |
| Spring: the force acts only along the anchor line, equal and opposite on both anchors | FR-004 |
| `critical_damping` spring released from rest overshoots ≤ 1% of its displacement | SC-001 |
| Undamped spring: no secular energy drift (≤ 1% period-mean over 60 s) | SC-002 |
| Coincident anchors (`L < 1e-6`): no impulse that step, never NaN | FR-014 |
| Joint between two static bodies: ignored | FR-014 |
| A world with no distance joints steps bit-identically to before | FR-011 |
| Same inputs → same results | FR-013 |

## Unchanged

`RevoluteJoint`, `JointId`, `World::add_joint` and `World::joint` keep
their exact signatures and behaviour.
