# Contract: Public Rust API surface added/changed by M4

The `mirage` library's public surface this milestone adds or changes. Exact
naming may be refined during implementation; the shape and semantics are
binding per the spec's functional requirements.

## `src/body.rs` (CHANGED)

```rust
pub struct RigidBody {
    // ...existing fields unchanged...
    /// Coulomb friction coefficient μ (>= 0). Default 0.5.
    pub friction: f32,
}

impl RigidBody {
    /// Unchanged signatures. `friction` defaults to 0.5.
    pub fn new_dynamic(position: Vec2, mass: f32, shape: Shape) -> Self;
    pub fn new_static(position: Vec2, shape: Shape) -> Self;

    /// NEW. Builder: sets `friction`, clamped to >= 0.
    pub fn with_friction(self, mu: f32) -> Self;
}
```

Existing callers of `new_dynamic` / `new_static` compile unchanged. Code that
builds `RigidBody` with a struct literal (none in this repo outside `body.rs`)
must add the new field.

## `src/collision/manifold.rs` (CHANGED)

```rust
pub struct Contact {
    pub point: Vec2,
    pub normal: Vec2,        // a → b, unchanged
    pub penetration: f32,
    /// NEW. Stable id of the geometric feature that produced this point.
    pub feature: u32,
}
```

`Contact` values constructed by hand (two in the solver's unit tests) need the
extra field. `detect_contacts` guarantees that for a resting pair the same
physical contact keeps the same `feature` from step to step, and that the two
points of a two-point manifold have different `feature` values. Negating the
normal for the `(Circle, Polygon)` ordering preserves `feature`.

## `src/world.rs` (CHANGED — behavior, not signature)

```rust
impl World {
    pub fn step(&mut self, dt_real: f32);   // signature unchanged
    pub fn contacts(&self) -> &[Manifold];  // unchanged
}
```

Behavioral contract of `step`, per fixed substep, in addition to M3's:

- **Friction**: at every contact a tangent impulse acts along `t ⟂ n`, with
  accumulated magnitude at most `μ ·` (accumulated normal impulse), where
  `μ = √(friction_a · friction_b)`. Zero friction on either body ⇒ no
  tangent impulse (M3 behavior).
- **Warm-starting**: impulses from the previous substep are re-applied to
  contacts that persist (same body pair and feature); contacts that end are
  forgotten immediately. Purely an internal accuracy/stability mechanism —
  no new public state.
- Static bodies' position and velocity are never modified.
- Same inputs ⇒ same results, bit for bit.
- `contacts()` semantics unchanged: manifolds detected during the most recent
  substep, before that substep's correction and integration.

## `src/solver.rs` (CHANGED, crate-private)

```rust
pub(crate) fn resolve(
    bodies: &mut [RigidBody],
    manifolds: &[Manifold],
    cache: &mut ImpulseCache,
    gravity: Vec2,
    dt: f32,
);

pub(crate) fn combine_friction(a: f32, b: f32) -> f32;   // (a·b).sqrt()
```

`ImpulseCache` is a crate-private type owned by `World`. Not re-exported from
`lib.rs`; `World::step` is the only production caller.

## Non-changes

- No new crate dependencies.
- `BodyId`, `Manifold`, `Aabb`, `Vec2`, `Rot2`, `Shape` public shapes are
  unchanged (`Manifold` keeps its fields; only `Contact` gains one).
- Restitution behavior, `with_restitution`, and the M3 default (`e = 0`) are
  unchanged.
- Broadphase is unchanged; narrowphase contact thresholds are unchanged.
