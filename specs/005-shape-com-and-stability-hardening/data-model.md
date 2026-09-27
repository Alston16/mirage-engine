# Phase 1 Data Model: Shape COM Correction & Frictionless-Stack Limits (M5)

No new persistent entities or storage — this feature corrects an existing
computation (`Shape::polygon`'s stored vertices) and adds test/documentation
coverage. This document records the one changed invariant and the new test
constants, in place of a traditional data model.

## `Shape::Polygon` (invariant corrected, fields unchanged)

```rust
Shape::Polygon {
    vertices: Vec<Vec2>,  // CCW, local space — now GUARANTEED centered on
                           // the true center of mass (was: assumed, unchecked)
    normals: Vec<Vec2>,   // outward unit normal per edge — unchanged shape,
                           // computed from the now-recentered vertices
}
```

- **Invariant before this feature**: local `(0, 0)` is the polygon's center
  of mass *only if the caller supplied already-centered vertices*; nothing
  computed or enforced it.
- **Invariant after this feature**: local `(0, 0)` is the polygon's true
  center of mass, always, computed from whatever convex CCW vertices the
  caller supplies (§ Q1 in `research.md`).
- **No new fields.** The correction is a constructor-time transform of
  `vertices` before it's stored; `normals` is derived from the corrected
  vertices exactly as before.
- **Validation rules** (unchanged from today, not expanded by this feature):
  vertices must be convex, CCW-wound, and non-degenerate (≥ 3 vertices,
  positive signed area). `Shape::polygon` does not check winding or
  convexity today and still doesn't after this feature — only the *centering*
  assumption changes from "caller's responsibility, unchecked" to "always
  correct, computed."

## `RigidBody::position` (semantics clarified, field unchanged)

- **Before**: documented as the body's position/COM, but only true for
  polygons if the caller had pre-centered their vertices.
- **After**: always the shape's true center of mass, for both `Circle`
  (already true — a circle's local origin is its center by construction)
  and `Polygon` (now enforced by the corrected `Shape::polygon`).
- No change to `RigidBody`'s fields, constructors, or builder methods
  (`new_dynamic`, `new_static`, `with_restitution`, `with_friction`).

## Test fixtures (new, in `tests/common/mod.rs` and `tests/stacking.rs`)

- **Off-center polygon pair** (User Story 1 / SC-001, SC-002): two
  `Shape::polygon` constructions describing the same physical rectangle —
  one with vertices already centered on `(0, 0)`, one with the identical
  rectangle's vertices shifted by an arbitrary offset (e.g., one corner at
  the local origin) — used to assert identical stored vertices, identical
  `inertia()`, and identical simulated trajectory once both are placed via
  `RigidBody::new_dynamic` at the same world position.
- **Near-degenerate polygon** (FR-005): a valid (positive-area, convex, CCW)
  triangle or quad with a very small area relative to the engine's 1 m body
  scale, used to assert `Shape::polygon`'s centroid computation and
  `Shape::inertia` both stay finite.
- **Frictionless-tower scene builder**: a new `tests/common/mod.rs` helper
  (parallel to the existing `tower()`) that builds an `n`-box tower with
  `μ = 0` on both the floor and every box, for the User Story 2 / FR-008
  boundary test. Does not modify the existing `tower()` (used by every other
  stacking test, which stays at the default `μ = 0.5`).
