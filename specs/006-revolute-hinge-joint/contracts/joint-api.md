# Interface Contract: Revolute Joint Public API

**Feature**: [../spec.md](../spec.md) | **Plan**: [../plan.md](../plan.md)

`mirage` is a library crate with no network/CLI surface — its "interface"
is the public Rust API it exposes to a consumer building a scene. This
contract fixes the new public surface this feature adds, so
`/speckit-tasks` and implementation can target a stable signature set
without re-deriving it from the design docs each time.

## New public items

```rust
// src/joint.rs

/// A 2-body point constraint pinning a local anchor on `body_a` to a local
/// anchor on `body_b`. Bodies remain free to rotate relative to each other;
/// only their anchor points are held coincident.
pub struct RevoluteJoint {
    pub body_a: BodyId,
    pub body_b: BodyId,
    /// Anchor on `body_a`, in `body_a`'s local frame.
    pub anchor_a: Vec2,
    /// Anchor on `body_b`, in `body_b`'s local frame.
    pub anchor_b: Vec2,
}

impl RevoluteJoint {
    /// Builds a joint pinning `anchor_a` (local to `body_a`) to `anchor_b`
    /// (local to `body_b`). The two world-space anchor points do not need
    /// to coincide yet — the solver closes any initial gap gradually.
    pub fn new(body_a: BodyId, body_b: BodyId, anchor_a: Vec2, anchor_b: Vec2) -> Self;
}

/// Opaque handle identifying a `RevoluteJoint` within the `World` that
/// created it. Only ever constructed by `World::add_joint`.
pub struct JointId(pub(crate) u32);
```

```rust
// src/world.rs — additions to the existing `World` impl

impl World {
    /// Adds a joint to the world, returning a handle to it. Panics if
    /// either `joint.body_a` or `joint.body_b` does not refer to a body
    /// already in this world, or if they are equal — the same precondition
    /// class as `World::body` already has for an unknown `BodyId`.
    pub fn add_joint(&mut self, joint: RevoluteJoint) -> JointId;

    /// Looks up a joint's current configuration by id.
    pub fn joint(&self, id: JointId) -> &RevoluteJoint;
}
```

```rust
// src/lib.rs — re-exports

pub use joint::{JointId, RevoluteJoint};
```

## Contract obligations

- **Determinism (FR-010)**: for identical bodies, identical joint
  configuration, and identical call order, `World::step` produces identical
  results across runs — same obligation `World::step` already has for
  contacts.
- **No effect on joint-free scenes (FR-008)**: a `World` with zero joints
  added must behave bit-for-bit identically to the current (pre-M6)
  behavior. This is directly testable: rerun the existing `tests/stacking.rs`
  and `tests/friction.rs` suites unmodified against the post-M6 code and
  confirm they still pass.
- **Static-body pinning (FR-006)**: either `body_a` or `body_b` (or both,
  though research.md § 1 notes this is a degenerate no-op configuration) may
  be a body constructed with `RigidBody::new_static`; the joint must not
  attempt to move a static body's `position`/`velocity`.
- **No new crate dependencies (FR-009)**: this entire surface is
  implementable with only `Vec2`/`Rot2`/`Mat2` (the last one new, see
  data-model.md), matching Principle I.

## Out of contract (explicitly not part of this milestone's surface)

- Joint removal/teardown functions (spec Assumptions: joints are permanent).
- Any accessor for a joint's current constraint error or accumulated
  impulse — not needed by the spec's success criteria, and would expose
  solver-internal state (`JointState`) that data-model.md keeps
  crate-private.
- Angular/motor constraints (`M8`) or distance/spring constraints (`M7`) —
  separate milestones, separate specs.
