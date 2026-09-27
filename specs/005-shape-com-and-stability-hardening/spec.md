# Feature Specification: Shape COM Correction & Frictionless-Stack Limits (M5)

**Feature Branch**: `005-shape-com-and-stability-hardening`

**Created**: 2026-09-27

**Status**: Draft

**Input**: User description: "M5 — Correctness hardening: shape centroid correction and frictionless-stacking limits." Closes GitHub issues #7 (`Shape::inertia` assumes local origin is the polygon's center of mass, unchecked) and #11 (frictionless (μ = 0) box stacks are unstable above ~3 boxes; behavior untested and undocumented). Scope: `Shape::polygon` auto-recenters any convex CCW input to its true center of mass; the frictionless-stack limitation is documented and pinned by a regression test with no change to the solver's math or tunables (explicitly no velocity/speed clamp, no CCD).

## User Scenarios & Testing *(mandatory)*

### User Story 1 - A polygon built from off-center vertices behaves correctly (Priority: P1)

A developer using the engine constructs a `Shape::polygon` from a list of convex, counter-clockwise vertices that are *not* centered on the shape's center of mass — for example, a rectangle authored with one corner at the local origin instead of its middle, or any asymmetric convex hull built the way a caller naturally would (from a mesh, a level-editor export, or hand-typed coordinates around a corner or an edge rather than a computed centroid). Today, `Shape::inertia` silently integrates about local `(0, 0)` regardless of where the true center of mass is, so the body's angular response (spin under torque or impulse) is physically wrong whenever that assumption doesn't hold — with no panic, no warning, and no way for the caller to know.

**Why this priority**: This is a correctness trap in the public shape-construction API that produces wrong physics silently. It is the higher-priority half of M5 because it's a latent bug that could affect any future caller, not just a documentation gap.

**Independent Test**: Build the same logical rectangle two ways — once with vertices already centered on `(0, 0)`, once with the identical rectangle shifted so a corner sits at the local origin — and confirm both produce the same mass, the same inertia, and the same simulated behavior (falling, rotating under an off-center impulse) once each body's position is read back relative to its own true center of mass.

**Acceptance Scenarios**:

1. **Given** a convex, CCW-wound vertex list that is already centered on its centroid, **When** `Shape::polygon` builds it, **Then** the stored vertices and computed inertia are unchanged from today's behavior (no regression for every shape already used in the engine, examples, and tests).
2. **Given** a convex, CCW-wound vertex list whose centroid is *not* at `(0, 0)` (e.g., a rectangle with a corner at the origin), **When** `Shape::polygon` builds it, **Then** the stored vertices are shifted so local `(0, 0)` is the true centroid, and `Shape::inertia` returns the same value it would for the equivalent pre-centered shape.
3. **Given** a body built from an off-center vertex list, **When** the body is placed in the world at a given `position`, **Then** `position` is interpreted as (and continues to track) the shape's true center of mass, so the body renders, rotates, and collides as if it had been authored pre-centered at that same world location.
4. **Given** an off-center polygon body under an impulse applied away from its center of mass, **When** the simulation steps, **Then** the resulting spin matches the equivalent pre-centered body receiving the same impulse at the same world point (within floating-point tolerance).

---

### User Story 2 - The frictionless-stacking limitation is documented and pinned (Priority: P2)

A developer reads the README to understand what the engine can and cannot do, and relies on `cargo test` to catch regressions. Today, stacks of frictionless (`μ = 0`) boxes are only neutrally stable and collapse into non-physical high-speed motion above roughly three boxes — this is expected behavior for a sequential-impulse solver with nothing to damp its residual per-step torque when there's no friction to resist it, but it is currently unmeasured by any test and unmentioned in the README, so a future change could silently make it worse (or better) without anyone noticing.

**Why this priority**: This closes a known-behavior gap with documentation and a regression test; it does not change simulated physics, so it is lower priority than the correctness fix in User Story 1.

**Independent Test**: Run a 2-box frictionless stack and a 5-or-10-box frictionless stack for the same duration used elsewhere in the test suite, and confirm the small stack holds within a measured drift bound while the taller one is allowed to exceed it — pinning today's documented boundary rather than asserting stability that doesn't exist for `μ = 0`.

**Acceptance Scenarios**:

1. **Given** a 2-box frictionless (`μ = 0`) stack placed just touching on a static floor, **When** the simulation runs for the same duration as the existing stacking tests, **Then** horizontal drift stays within a documented bound and the stack does not collapse.
2. **Given** a taller frictionless stack (5 or 10 boxes) under the same conditions, **When** the simulation runs, **Then** the test records the expected outcome (collapse into large drift/speed) as a pinned, passing assertion rather than a failure — the boundary between "holds" and "collapses" is what the test protects, not universal stability.
3. **Given** the README's Resolution section, **When** a developer reads it, **Then** it states that `μ = 0` stacks are neutrally stable and collapse above roughly three boxes, and that post-collapse tunneling through the floor is an expected downstream symptom of no continuous collision detection (an existing, named non-goal), not a new defect.
4. **Given** the same stacking scenarios with non-zero friction (μ = 0.5, as used elsewhere in the test suite), **When** the simulation runs, **Then** behavior is unchanged from today (no regression to the friction-on stacking guarantees from M4).

---

### Edge Cases

- A polygon whose vertices are already centered (shift is exactly zero): must produce bit-identical stored vertices and inertia to today's behavior.
- A near-degenerate polygon (very small area, close to collinear vertices) approaching the zero-area denominator in both the centroid and inertia formulas: must not divide by zero or produce `NaN`/`Inf`; behavior for this input is explicitly decided in Requirements below.
- A body with near-zero mass combined with an off-center shape: inertia and the recentering shift must both remain finite and well-defined.
- A shape provided in clockwise winding order (violates the documented CCW contract): out of scope for this feature — existing behavior for wrong winding is unchanged; only the *centering* assumption is fixed here.
- A frictionless stack of exactly the size at the documented holds/collapses boundary: the regression test pins today's measured boundary, not a newly chosen one — see Assumptions for how that boundary is set.
- Non-zero friction stacks and all other existing behavioral tests (ramp, bouncing, pyramid, mixed shapes, determinism): must continue to pass unchanged.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: `Shape::polygon` MUST compute the true center of mass (centroid) of the convex, CCW-wound vertex list it is given, using the standard area-weighted (shoelace) centroid formula.
- **FR-002**: `Shape::polygon` MUST shift the stored vertices so that local `(0, 0)` coincides with that computed centroid, regardless of where the caller's original vertices were positioned, so `Shape::inertia`'s existing origin-centered integral is always correct.
- **FR-003**: For a vertex list already centered on its centroid, the recentering MUST be a no-op (zero shift), so all shapes already built in the engine, its examples, and its tests are bit-identical in stored geometry and computed inertia to today's behavior.
- **FR-004**: A body's `position` MUST continue to represent the shape's true center of mass after recentering — i.e., recentering is invisible from the public `RigidBody`/`World` API: a body built from off-center vertices at a given world position behaves exactly as the equivalent pre-centered shape would at that same position.
- **FR-005**: `Shape::inertia` and the new centroid computation MUST NOT panic or produce `NaN`/`Inf` for a near-degenerate (very small area) convex polygon within the range of inputs already accepted by the engine (i.e., non-self-intersecting, non-collinear, at least 3 vertices, positive signed area) — the denominator guard added here documents and tests the existing formula's boundary rather than newly restricting valid input.
- **FR-006**: The engine MUST NOT change behavior for `RigidBody`s with near-zero mass beyond what already exists today; the recentering computation itself must remain well-defined for such bodies.
- **FR-007**: The frictionless-stacking limitation MUST be documented in the README's Resolution section (alongside the existing sequential-impulse residual-torque trade-off note): `μ = 0` stacks are neutrally stable and collapse above roughly three boxes, and post-collapse tunneling through the floor is an expected symptom given no continuous collision detection (an existing non-goal), not a new defect.
- **FR-008**: A new or extended behavioral test MUST pin the frictionless-stacking boundary: a small stack (2 boxes) holds within a measured drift bound, and a taller stack (5 or 10 boxes) is asserted to collapse (recorded as the expected, passing outcome) rather than silently left untested.
- **FR-009**: Existing friction-on (`μ = 0.5`) stacking guarantees, the M3 restitution behaviors, the M4 ramp/tower/pyramid criteria, and determinism (bit-identical repeat runs) MUST remain unchanged and their tests MUST continue to pass.
- **FR-010**: This feature MUST NOT modify `solver.rs`'s physics (no velocity/speed clamp, no change to iteration count, warm-starting, or any documented constant) — the frictionless-stack behavior is documented and tested, not altered.
- **FR-011**: The engine crate MUST keep zero dependencies; no new test-only dependency may be added beyond what the existing `tests/` suite already uses.
- **FR-012**: Non-goals MUST remain out of scope: no joints, continuous collision detection, sleeping, concave/compound shapes, spatial partitioning, serialization, parallelism, or 3D.
- **FR-013**: Code and documentation for the centroid correction MUST match the derivation style already used in the README and in `shape.rs`'s doc comments (the shoelace-weighted formula, using consistent variable names), and the doc comments on `Shape::polygon`/`Shape::inertia` that currently describe the *unchecked* assumption MUST be updated to describe the corrected behavior.

### Key Entities

- **Polygon centroid**: The area-weighted center of mass of a convex vertex list, computed once at construction time from the shoelace formula; used to recenter stored vertices so the existing inertia integral (about local origin) is always correct.
- **Shape (Polygon variant)**: Unchanged public shape, except its stored `vertices` are now guaranteed centered on the true center of mass rather than merely assumed to be.
- **Frictionless-stack boundary**: The documented, tested threshold (measured stack height) below which a `μ = 0` stack holds and above which it is expected to collapse; a fact about the existing solver's behavior being pinned, not a new mechanism.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: A polygon built from off-center vertices and the same polygon built from pre-centered vertices produce identical mass, identical inertia (within floating-point tolerance), and identical simulated trajectories (position and orientation over time, within floating-point tolerance) when placed at the same world position and subjected to the same forces/impulses.
- **SC-002**: Every polygon shape already constructed in the engine's source, examples, and tests today has zero recentering shift applied (bit-identical stored vertices and inertia before and after this change).
- **SC-003**: `cargo test` passes with new coverage for: off-center polygon construction (SC-001's equivalence), a near-degenerate polygon (no panic, no `NaN`/`Inf`), and the frictionless-stacking boundary (2-box holds, 5-or-10-box collapse is the pinned, expected, passing outcome).
- **SC-004**: The README documents the `μ = 0` stacking limitation in the Resolution section, in language consistent with its existing "known trade-off" note.
- **SC-005**: All four existing example demos (`bouncing`, `ramp`, `stack`, `pyramid`) build and run correctly with `--release` after this change, confirming the centroid correction is behaviorally invisible for every shape already in use.
- **SC-006**: All previously passing tests (unit and behavioral) continue to pass unchanged.

## Assumptions

- "Off-center" test input in User Story 1 is limited to convex, CCW-wound, non-degenerate polygons — the same input contract `Shape::polygon` already documents; this feature does not add new validation for clockwise winding, self-intersection, or concavity, which remain the caller's responsibility as today.
- The near-degenerate-polygon guard (FR-005) is a defensive test-and-document pass over the existing formula's boundary (do the current formulas already stay finite for very small areas, and if not, add the minimal guard needed), not a new tolerance or error-reporting API; if the existing formula already behaves safely, this requirement is satisfied by adding regression test coverage alone.
- The frictionless-stacking boundary pinned in User Story 2 uses the same scene-building and measurement conventions already established in `tests/common/mod.rs` and `tests/stacking.rs` (unit boxes, same floor, same `dt`, same measurement duration as the existing 60-second tower test), so the new test slots in next to the existing ones rather than introducing new conventions.
- "Collapse above roughly three boxes" and the exact holds/collapses split (2 holds, 5-or-10 collapses) are taken from the measurements already recorded in issue #11; the new test formalizes those specific, already-measured numbers rather than re-deriving new thresholds.
- No velocity/speed clamp and no other solver-level mitigation for the frictionless collapse is in scope for this feature — this was an explicit decision (see issue #11's discussion): the collapse is real behavior for an under-constrained frictionless stack, and masking it would cut against the project's "no black boxes" design principle.
- This feature depends on M0–M4 being complete (they are); it touches `src/shape.rs`, `README.md`, and `tests/` only — `solver.rs`'s physics is explicitly untouched.
