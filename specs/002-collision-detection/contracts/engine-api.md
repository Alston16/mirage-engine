# Contract: Public Rust API surface added/changed by M2

This is a library, not a network/CLI service, so the "contract" is the
public `mirage` API surface this milestone adds or changes — the signatures
downstream code (including the examples) can rely on. Exact naming may be
refined during implementation, but the shape and semantics below are
binding per the spec's functional requirements.

## `src/shape.rs` (NEW)

```rust
pub struct Aabb {
    pub min: Vec2,
    pub max: Vec2,
}

impl Aabb {
    pub fn overlaps(&self, other: &Aabb) -> bool;
}

pub enum Shape {
    Circle { radius: f32 },
    Polygon { vertices: Vec<Vec2>, normals: Vec<Vec2> },
}

impl Shape {
    /// Convenience constructor validating winding/convexity is the
    /// caller's responsibility (see data-model.md).
    pub fn circle(radius: f32) -> Self;
    pub fn polygon(vertices: Vec<Vec2>) -> Self; // derives `normals`

    /// World-space AABB for this shape at the given transform.
    pub fn aabb(&self, position: Vec2, orientation: Rot2) -> Aabb;
}
```

- No area/inertia methods this milestone (see research.md).

## `src/body.rs` (CHANGED)

```rust
pub struct RigidBody {
    pub position: Vec2,
    pub velocity: Vec2,
    pub orientation: Rot2,
    pub angular_velocity: f32,
    pub inv_mass: f32,
    pub is_static: bool,
    pub shape: Shape, // NEW field
}

impl RigidBody {
    /// Dynamic body with the given mass (must be > 0.0) and shape.
    pub fn new_dynamic(position: Vec2, mass: f32, shape: Shape) -> Self;

    /// Static (immovable) body with the given shape — inv_mass is 0.0.
    pub fn new_static(position: Vec2, shape: Shape) -> Self;
}
```

- **Breaking change from M1**: both constructors gain a `shape` parameter.
  Existing call sites (`examples/bouncing.rs`) MUST be updated in the same
  change (e.g. `Shape::circle(radius)`).

## `src/broadphase.rs` (NEW)

```rust
/// All body-index pairs (i, j) with i < j whose current AABBs overlap.
/// Internal-facing (crate-visible), consumed by `collision::detect_contacts`;
/// exposed `pub(crate)` unless a consumer outside the crate needs direct
/// access to candidate pairs (not required by this milestone's spec).
pub(crate) fn candidate_pairs(bodies: &[RigidBody]) -> Vec<(usize, usize)>;
```

- Semantics: O(n²) over all body pairs, AABB `overlaps` check per pair, per
  README's explicit non-goal boundary (no spatial index).

## `src/collision/mod.rs` (NEW)

```rust
/// Runs broadphase then narrowphase over the given bodies, returning every
/// pair currently in contact as a Manifold. Pure function — no mutation of
/// `bodies`.
pub(crate) fn detect_contacts(bodies: &[RigidBody]) -> Vec<Manifold>;
```

- Dispatches each candidate pair to `collision::circle` or
  `collision::polygon` based on the pair's `Shape` variants.

## `src/collision/manifold.rs` (NEW)

```rust
pub struct Contact {
    pub point: Vec2,
    pub normal: Vec2,
    pub penetration: f32,
}

pub struct Manifold {
    pub body_a: BodyId,
    pub body_b: BodyId,
    pub points: Vec<Contact>, // 1 for circle pairs, 1-2 for polygon pairs
}
```

## `src/collision/circle.rs` / `src/collision/polygon.rs` (NEW, crate-internal)

```rust
// circle.rs
pub(crate) fn circle_vs_circle(a: &RigidBody, b: &RigidBody) -> Option<Contact>;
pub(crate) fn circle_vs_polygon(circle: &RigidBody, polygon: &RigidBody) -> Option<Contact>;

// polygon.rs
pub(crate) fn polygon_vs_polygon(a: &RigidBody, b: &RigidBody) -> Option<Vec<Contact>>;
```

- Not part of the crate's *public* API (no external consumer needs to call
  narrowphase directly this milestone) — listed here because their
  signatures and return semantics are exactly what the spec's Functional
  Requirements (FR-002 through FR-006) describe and what unit tests exercise
  directly via `#[cfg(test)]` in the same file.

## `src/world.rs` (CHANGED)

```rust
impl World {
    pub fn new() -> Self;                              // unchanged
    pub fn add_body(&mut self, body: RigidBody) -> BodyId; // unchanged
    pub fn body(&self, id: BodyId) -> &RigidBody;       // unchanged

    /// Unchanged signature/integration behavior from M1, plus: each fixed
    /// substep now also runs broadphase + narrowphase and refreshes the
    /// contact list (see below). Still applies no collision response.
    pub fn step(&mut self, dt_real: f32);

    /// NEW: the contacts found during the most recently completed fixed
    /// substep (empty before the first substep ever runs).
    pub fn contacts(&self) -> &[Manifold];
}
```

- **Postcondition on `step` (new, in addition to M1's)**: after `step`
  returns, `contacts()` reflects exactly the pairs whose shapes overlap at
  the *final* fixed substep's body positions — never a pair whose AABBs
  didn't overlap (SC-003), never an empty `Manifold`.

## `src/lib.rs` (CHANGED)

```rust
pub mod body;
pub mod broadphase; // pub(crate) internals, module itself may stay private
pub mod collision;
pub mod math;
pub mod shape;
pub mod world;

pub use body::{BodyId, RigidBody};
pub use collision::manifold::{Contact, Manifold};
pub use math::{Rot2, Vec2};
pub use shape::{Aabb, Shape};
pub use world::World;
```

Consistent with the existing re-export pattern from M0/M1. `broadphase` and
the narrowphase functions inside `collision::{circle, polygon}` stay
`pub(crate)` — only `Shape`, `Aabb`, `Contact`, and `Manifold` need to be
public, since those are the types example code and future milestones (M3's
solver) consume.

## `examples/*.rs` (consumer, not part of the crate's public API)

The box-on-ramp scene constructs a static polygon "ramp" body tilted via
`Rot2::new(angle)`, a dynamic box (or circle, for the simpler bouncing
scene) body, steps the world each frame, and for each `Manifold` in
`world.contacts()` draws every `Contact`'s `point` (e.g. a small circle
marker) and `normal` (e.g. a short line segment from `point` along
`normal`), per spec User Story 3 / FR-009.
