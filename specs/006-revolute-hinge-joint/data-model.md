# Data Model: Revolute (Hinge) Joint

**Feature**: [spec.md](spec.md) | **Plan**: [plan.md](plan.md) | **Research**: [research.md](research.md)

This feature adds one new user-facing entity (`RevoluteJoint`) and the
solver-internal state needed to resolve it. It does not modify
`RigidBody`, `Shape`, or any existing entity's fields.

## `RevoluteJoint` (public, `src/joint.rs`)

The user-facing description of a hinge connecting two bodies.

| Field | Type | Notes |
|---|---|---|
| `body_a` | `BodyId` | First connected body; may be static or dynamic. |
| `body_b` | `BodyId` | Second connected body; may be static or dynamic. |
| `anchor_a` | `Vec2` | Pin point, in `body_a`'s **local** frame (relative to its center, unrotated). |
| `anchor_b` | `Vec2` | Pin point, in `body_b`'s **local** frame. |

**Validation rules**:
- `body_a != body_b` — a joint connects two distinct bodies (a body cannot
  be joined to itself; this is a programming error, not a recoverable
  runtime state, so it is checked the same way existing `BodyId` misuse
  would be — see the API contract for how this surfaces).
- Both bodies must already exist in the `World` (same precondition
  `World::body(id)` already has for any `BodyId`).
- No constraint on whether `anchor_a`'s and `anchor_b`'s *world* positions
  coincide at creation time — see spec Assumptions and research.md § 4
  (Baumgarte correction closes any initial gap gradually).

**Relationships**: A `RevoluteJoint` references two `RigidBody`s by `BodyId`
but does not own or duplicate their state. A body may participate in more
than one joint (edge case in spec.md — e.g. a chain of hinged rods); each
joint is an independent entry, resolved independently in the shared
iteration loop (research.md § 3).

**State transitions**: None beyond creation. Per spec Assumptions, joints
are permanent for the lifetime of the two bodies — no removal/teardown API
in this milestone.

## `JointId` (public, `src/joint.rs`)

Opaque handle for a joint added to a `World`, mirroring `BodyId`'s shape
(`BodyId(pub(crate) u32)`): `JointId(pub(crate) u32)`. Only ever constructed
by `World::add_joint`.

## `JointState` (crate-internal, `src/joint.rs`)

Per-joint solver working state, built fresh once per fixed step and
discarded — the joint-side analog of `solver::ContactState`.

| Field | Type | Notes |
|---|---|---|
| `a`, `b` | `usize` | Body indices into the `World`'s body array (not `BodyId`s — matches `ContactState`'s convention). |
| `r_a`, `r_b` | `Vec2` | World-space offset from each body's center to its anchor point (`R * anchor_local`). |
| `k` | `Mat2` | The 2×2 effective mass matrix from research.md § 1. |
| `k_inv` | `Mat2` | `k` inverted once per step (bodies don't move mid-iteration in a way that changes `k` meaningfully within a single step, matching how a contact's scalar `k` is also computed once). |

No accumulated-impulse field is needed for M6 (research.md § 5: warm-starting
deferred) — each velocity iteration computes and applies a fresh correcting
impulse directly, the same shape as a plain (non-warm-started) Gauss-Seidel
solve.

## `Mat2` (crate-internal, `src/math.rs`)

New minimal 2×2 matrix primitive (research.md § 2).

| Field | Type | Notes |
|---|---|---|
| `m11`, `m12`, `m21`, `m22` | `f32` | Row-major: `[[m11, m12], [m21, m22]]`. |

Operations: `Mat2 * Vec2 -> Vec2` (matrix-vector product), `Mat2::invert() -> Mat2`
(via `1/det * adjugate`; the joint solve only ever calls this when `det != 0`,
guaranteed by at least one of the two bodies having nonzero inverse mass —
the same "drop degenerate constraints" guard `solver.rs` already applies
when a contact's scalar `k == 0.0`).

## No changes to existing entities

- `RigidBody`: untouched. A joint reads `position`, `orientation`, `velocity`,
  `angular_velocity`, `inv_mass`, `inv_inertia`, `is_static` — all already
  public/`pub(crate)` fields — and writes `velocity`/`angular_velocity`
  (during iteration) and `position` (during correction), the same fields
  `solver.rs` already writes.
- `World`: gains `joints: Vec<RevoluteJoint>` storage and the `add_joint`/
  `joint` accessor pair, parallel to its existing `bodies`/`add_body`/`body`.
  `step()`'s internal structure changes per research.md § 3; its public
  signature (`step(&mut self, dt_real: f32)`) does not.
