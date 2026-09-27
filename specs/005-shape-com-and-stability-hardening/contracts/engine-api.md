# Contract: Public Rust API surface changed by M5

Signatures are unchanged; the binding change is a corrected *behavioral*
contract for `Shape::polygon`. No new public types, no new dependencies.

## `src/shape.rs` (CHANGED — behavior, not signature)

```rust
impl Shape {
    /// Signature unchanged.
    ///
    /// CHANGED CONTRACT: vertices no longer need to be pre-centered by the
    /// caller. `Shape::polygon` computes the input's true center of mass
    /// (area-weighted centroid) and stores vertices shifted so local
    /// `(0, 0)` is always that point — for any convex, CCW-wound,
    /// non-degenerate input, not just already-centered input.
    pub fn polygon(vertices: Vec<Vec2>) -> Self;

    /// Signature and formula unchanged (integrates about local `(0, 0)`).
    /// Now always correct, because `polygon`'s stored vertices are always
    /// centered on the true COM — previously correct only if the caller's
    /// input happened to already be centered.
    pub fn inertia(&self, mass: f32) -> f32;
}
```

Every existing caller (`RigidBody::new_dynamic`/`new_static`, all examples,
all tests) already supplies pre-centered vertices, so this is a **no-op
change in stored output for all existing callers** (SC-002) — the
correction is only observable for a caller who was previously (silently)
getting the wrong `inertia()`.

## `src/body.rs` (NO CHANGE)

```rust
impl RigidBody {
    pub fn new_dynamic(position: Vec2, mass: f32, shape: Shape) -> Self;
    pub fn new_static(position: Vec2, shape: Shape) -> Self;
}
```

Unchanged signatures and behavior. `position` now has a strictly *more*
reliable meaning ("the shape's true center of mass") for any `Shape::Polygon`
built from off-center input — previously that meaning silently didn't hold.

## `README.md` (CHANGED — documentation only)

- § "How it works" › narrowphase/shape description: the `Shape::polygon`
  doc-comment language describing the unchecked centering assumption is
  replaced with a description of the corrected, always-centered behavior.
- § Resolution: gains a documented "known trade-off" note for the `μ = 0`
  stacking boundary (issue #11), alongside the existing sequential-impulse
  residual-torque note.

## Non-changes

- No new crate dependencies; engine crate stays zero-dependency.
- No changes to `RigidBody`, `World`, `Contact`, `Manifold`, `Aabb`, `Vec2`,
  `Rot2` — fields and signatures are identical to M4.
- `solver.rs` is untouched: no velocity/speed clamp, no iteration-count,
  warm-starting, or tunable-constant change (FR-010).
- Broadphase and narrowphase dispatch are unchanged.
