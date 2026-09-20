# Contract: Public Rust API surface added/changed by M3

The `mirage` library's public surface this milestone adds or changes.
Exact naming may be refined during implementation; the shape and semantics
are binding per the spec's functional requirements.

## `src/body.rs` (CHANGED)

```rust
pub struct RigidBody {
    // ...existing fields unchanged...
    /// Bounciness `e` in [0, 1]. Default 0.0.
    pub restitution: f32,
    /// `1 / I`, or 0.0 for a static body.
    pub inv_inertia: f32,
}

impl RigidBody {
    /// Unchanged signature. Now also derives `inv_inertia` from `shape`
    /// and `mass`; `restitution` defaults to 0.0.
    pub fn new_dynamic(position: Vec2, mass: f32, shape: Shape) -> Self;

    /// Unchanged signature. `inv_mass == inv_inertia == 0.0`.
    pub fn new_static(position: Vec2, shape: Shape) -> Self;

    /// NEW. Builder: sets `restitution`, clamped to [0, 1].
    pub fn with_restitution(self, e: f32) -> Self;
}
```

Existing callers of `new_dynamic` / `new_static` compile unchanged.

## `src/shape.rs` (CHANGED)

```rust
impl Shape {
    /// NEW. Moment of inertia about the local origin for total `mass`.
    /// Assumes polygon vertices are centered on the center of mass.
    pub fn inertia(&self, mass: f32) -> f32;
}
```

- Circle: `0.5 * mass * radius²`.
- Polygon: triangle-fan formula in research.md; half-extent-1 square gives
  `2 * mass / 3`.

## `src/world.rs` (CHANGED — behavior, not signature)

```rust
impl World {
    pub fn step(&mut self, dt_real: f32); // signature unchanged
    pub fn contacts(&self) -> &[Manifold]; // unchanged
}
```

Behavioral contract of `step`, per fixed substep:

- Contacts are **resolved**: normal impulses change linear and angular
  velocity; overlap beyond slop is projected apart.
- Static bodies' position and velocity are never modified.
- Angular velocity is integrated into orientation.
- No friction: impulses act only along the contact normal.
- `contacts()` returns the manifolds detected during the most recent
  substep (before positional correction and integration of that substep).

## `src/collision/mod.rs` (CHANGED — invariant)

`detect_contacts` guarantees `Contact.normal` points from `Manifold.body_a`
toward `Manifold.body_b` for **every** shape-pair ordering, including
`(Circle, Polygon)`, which M2 returned reversed.

## `src/solver.rs` (NEW, crate-private)

```rust
pub(crate) fn resolve(
    bodies: &mut [RigidBody],
    manifolds: &[Manifold],
    gravity: Vec2,
    dt: f32,
);
```

Runs contact preparation, `VELOCITY_ITERATIONS` velocity iterations, and
positional correction. Not re-exported from `lib.rs`; `World::step` is the
only caller. Position integration remains in `World::step`.

## Non-changes

- No new crate dependencies.
- `BodyId`, `Manifold`, `Contact`, `Aabb`, `Vec2`, `Rot2` public shapes are
  unchanged.
