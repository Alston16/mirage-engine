# Phase 1 Data Model: Collision Detection (M2)

## Shape

The geometric form attached to a body. New this milestone.

| Variant | Field | Type | Notes |
|---|---|---|---|
| `Circle` | `radius` | `f32` | Must be `> 0.0`; centered on the owning body's `position`. |
| `Polygon` | `vertices` | `Vec<Vec2>` (or fixed-size array) | Ordered counter-clockwise, in the body's local space (relative to `position`, before `orientation` is applied). Must describe a convex polygon (non-goal: concave shapes). |
| `Polygon` | `normals` | `Vec<Vec2>` (or derived on demand) | Outward-facing unit normal per edge, local space, parallel to `vertices` (edge `i` is between `vertices[i]` and `vertices[(i+1) % n]`). |

- **Validation rules**: A `Polygon` MUST have at least 3 vertices, wound
  counter-clockwise, describing a convex shape — constructing an invalid
  polygon is a programmer error this milestone does not defend against at
  runtime (no user-facing validation API requested by the spec, consistent
  with M1's precedent for `RigidBody` mass validation).
- **Relationships**: Owned by exactly one `RigidBody` via its `shape` field.
- **Derived queries** (methods, not stored fields):
  - `aabb(position: Vec2, orientation: Rot2) -> Aabb` — world-space bounding
    box at the body's current transform.
  - Circle: trivial (`position ± radius` on both axes).
  - Polygon: transform each vertex by `orientation`/`position`, take the
    min/max extent.
  - Support/closest-point helpers used internally by narrowphase (not part
    of the public contract unless a consumer outside `collision/` needs
    them — see `contracts/engine-api.md`).

## Aabb

The axis-aligned bounding box broadphase uses to reject non-overlapping
pairs. New this milestone.

| Field | Type | Notes |
|---|---|---|
| `min` | `Vec2` | Lower corner. |
| `max` | `Vec2` | Upper corner. |

- **Validation rules**: `min.x <= max.x` and `min.y <= max.y` always (an
  inverted box is a programmer error, never constructed by `Shape::aabb`).
- **Relationships**: Computed fresh from a `RigidBody`'s `shape` + current
  `position`/`orientation` each time broadphase runs — not cached/stored on
  `RigidBody` (Constitution Principle II: recomputed deterministically each
  step, no stale cached state to invalidate).
- **Operations**: `overlaps(other: &Aabb) -> bool` — the standard
  interval-overlap-on-both-axes test broadphase calls per candidate pair.

## RigidBody (changed from M1)

| Field | Type | Notes |
|---|---|---|
| `position` | `Vec2` | unchanged |
| `velocity` | `Vec2` | unchanged |
| `orientation` | `Rot2` | unchanged (now actually consumed, by polygon narrowphase) |
| `angular_velocity` | `f32` | unchanged |
| `inv_mass` | `f32` | unchanged |
| `is_static` | `bool` | unchanged |
| `shape` | `Shape` | **NEW.** The body's geometry, in local space. |

- **Validation rules**: unchanged from M1, plus: `shape` must be a validly
  constructed `Shape` per above (caller responsibility).
- **State transitions**: None new — `shape` is fixed at construction, same
  as M1's other fields.

## Contact

A single point of overlap between two bodies. New this milestone.

| Field | Type | Notes |
|---|---|---|
| `point` | `Vec2` | World-space contact position. |
| `normal` | `Vec2` | Unit vector, pointing from the first body toward the second (per spec FR-002/Acceptance Scenario 1). |
| `penetration` | `f32` | `>= 0.0`, depth of overlap along `normal`. |

- **Validation rules**: `normal` MUST be unit length (or `Vec2::ZERO` only
  in a genuinely degenerate case narrowphase should avoid producing);
  `penetration >= 0.0` always — a negative value would mean "not actually
  touching" and such contacts must not be constructed at all (see research
  epsilon/slop decision).

## Manifold

The full set of contacts between one pair of bodies this step. New this
milestone.

| Field | Type | Notes |
|---|---|---|
| `body_a` | `BodyId` | First body in the pair. |
| `body_b` | `BodyId` | Second body in the pair. |
| `points` | `Vec<Contact>` (or a fixed-capacity `[Option<Contact>; 2]`) | 1 point for circle pairs, 1–2 points for polygon pairs, per README. |

- **Validation rules**: `points` MUST be non-empty — broadphase/narrowphase
  only ever produce a `Manifold` for a pair actually found to overlap; a
  pair with no overlap simply produces no `Manifold` at all (not an empty
  one).
- **Relationships**: `body_a`/`body_b` reference `BodyId`s issued by the
  same `World` that owns the manifold list (mirrors the M1 `BodyId`
  contract).

## CandidatePair (internal, `broadphase.rs`)

Not part of the public API — the intermediate output of broadphase before
narrowphase runs.

| Field | Type | Notes |
|---|---|---|
| (pair) | `(BodyId, BodyId)` | A body pair whose AABBs overlap. |

- **Relationships**: Produced by `broadphase::candidate_pairs`, consumed by
  the narrowphase dispatch in `collision::mod`.

## World (changed from M1)

| Field | Type | Notes |
|---|---|---|
| `bodies` | `Vec<RigidBody>` | unchanged |
| `gravity` | `Vec2` | unchanged |
| `accumulator` | `f32` | unchanged |
| `fixed_dt` | `f32` | unchanged |
| `contacts` | `Vec<Manifold>` | **NEW.** The full set of manifolds found during the most recently completed fixed substep. |

- **State transitions**: `World::step(dt_real)` — unchanged accumulator
  behavior from M1 (integrate every dynamic body under gravity), plus: each
  fixed substep, after integration, `contacts` is cleared and repopulated by
  running broadphase → narrowphase over the current body set. If `step`
  drains more than one fixed substep in a single call, only the *last*
  substep's contacts remain (see research.md rationale).
- **Validation rules**: unchanged M1 accumulator invariant, plus: `contacts`
  never contains a `Manifold` for a pair whose AABBs did not overlap
  (SC-003), and never contains a `Manifold` with zero points (see Manifold
  above).

## Notes on scope

Area and moment-of-inertia computation from `Shape` are explicitly **not**
part of this milestone's data model (see research.md) — `RigidBody.inv_mass`
continues to be supplied directly at construction, as in M1. Impulse
resolution fields (accumulated normal/tangent impulse for warm-starting,
restitution/friction coefficients) are **not** part of `Contact`/`Manifold`
this milestone — those belong to M3's solver.
