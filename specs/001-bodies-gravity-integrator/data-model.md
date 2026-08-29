# Phase 1 Data Model: Bodies, Gravity & Fixed-Timestep Integrator

## BodyId

Opaque handle identifying a `RigidBody` within a `World`.

| Field | Type | Notes |
|---|---|---|
| (inner index) | `u32` (or `usize`) | Newtype wrapper, e.g. `pub struct BodyId(u32);`. Not constructible outside `world.rs`/`body.rs` — obtained only from `World::add_body`. |

- **Validation rules**: N/A — assigned internally by `World` when a body is
  added; never user-supplied.
- **Relationships**: A `BodyId` indexes into `World`'s body storage.
  Referencing a `BodyId` that has never been returned by
  `World::add_body` is a programmer error (out of scope for this milestone
  to guard against, since there is no removal API yet either).

## RigidBody

The single physical object simulated by the world.

| Field | Type | Notes |
|---|---|---|
| `position` | `Vec2` | World-space position of the body's origin. |
| `velocity` | `Vec2` | Linear velocity, units/s. Zero and unchanging for static bodies. |
| `orientation` | `Rot2` (or `f32` angle, see contracts) | Present per architecture; not driven by anything this milestone (no torque source yet — spec Assumptions). |
| `angular_velocity` | `f32` | Present per architecture; remains `0.0` this milestone. |
| `inv_mass` | `f32` | `0.0` for static bodies; `1.0 / mass` for dynamic bodies. See `research.md` decision. |
| `is_static` | `bool` | Explicit classification (see `research.md`); when `true`, `World::step` skips gravity/integration for this body entirely. |

- **Validation rules**:
  - A dynamic body MUST have `inv_mass > 0.0` (i.e. finite positive mass);
    constructing a dynamic body with zero/negative mass is a programmer
    error this milestone does not need to defend against defensively
    (no user-facing validation API requested by the spec).
  - A static body's `inv_mass` MUST be `0.0`.
- **State transitions**: None — a body's static/dynamic classification is
  fixed at construction for this milestone (no requirement to toggle it).

## World

The simulation container.

| Field | Type | Notes |
|---|---|---|
| `bodies` | `Vec<RigidBody>` (or equivalent indexable store keyed by `BodyId`) | Owns all bodies. |
| `gravity` | `Vec2` | Constant, applied to every dynamic body each fixed step. Default matches README (`9.81` units/s², downward). |
| `accumulator` | `f32` | Leftover real time not yet consumed by a fixed step. Starts at `0.0`. |
| `fixed_dt` | `f32` (or associated const) | `1.0 / 60.0`, per README/constitution. |

- **Validation rules**: `accumulator` MUST never be observed holding a full
  `fixed_dt` or more after `step` returns — it always drains down to a
  remainder `< fixed_dt` (this is the core accumulator invariant FR-006/
  FR-007 depend on).
- **Relationships**: `World` owns all `RigidBody` instances; `BodyId`
  values are only meaningful with respect to the `World` that issued them.
- **State transitions**: `World::step(dt_real)` is the only state-mutating
  operation this milestone adds. Internally: `accumulator += dt_real`, then
  while `accumulator >= fixed_dt`: integrate every dynamic body one
  `fixed_dt` step, `accumulator -= fixed_dt`.

## Notes on scope

`Shape` (circle radius, polygon vertices, area/inertia/AABB) is
architecturally a separate module (`shape.rs`) and is **not** introduced by
this milestone's data model — per spec Assumptions, `examples/bouncing.rs`
only needs enough shape data (e.g. a bare `radius: f32` local to the
example, or a minimal field on `RigidBody`) to draw a circle. Full
`Shape`/mass-from-geometry computation is M2 scope.
