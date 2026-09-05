# Phase 0 Research: Bodies, Gravity & Fixed-Timestep Integrator

No `NEEDS CLARIFICATION` markers remained in the Technical Context — the
README and constitution already pin language, dependency, testing, and
integration-method choices. This document records the design decisions
that still had more than one reasonable implementation, so the rationale
is on record before Phase 1 design.

## Decision: Mass stored as inverse mass (`inv_mass: f32`) on `RigidBody`

- **Decision**: `RigidBody` stores `inv_mass: f32` (0.0 for static bodies)
  rather than `mass: f32` plus a separate "is static" bool derived from
  `mass == 0.0` or `mass == f32::INFINITY`.
- **Rationale**: The solver's impulse formula (README § Resolution) already
  divides by mass in the denominator (`1/m_a + 1/m_b + ...`); storing the
  inverse up front means static bodies are simply `inv_mass = 0.0` with no
  branch or division-by-zero guard needed anywhere in integration or (later)
  the solver. This is the standard Box2D-Lite/Randy Gaul convention this
  project's References section already points to.
- **Alternatives considered**:
  - *Store `mass` and branch on a `BodyKind::Static` enum everywhere mass is
    used* — rejected: pushes a branch into every future call site (M3/M4
    solver) instead of encoding "infinite mass" once, in the data.
  - *Store `mass: f32::INFINITY` for static bodies* — rejected: multiplying
    or dividing by `INFINITY`/`NAN` in later solver math is a correctness
    trap; `inv_mass = 0.0` is exact and safe in every arithmetic position it
    will be used.

## Decision: Static vs. dynamic is a stored classification, not inferred solely from `inv_mass == 0.0`

- **Decision**: `RigidBody` still carries an explicit `is_static: bool` (or
  equivalent) alongside `inv_mass`, rather than treating `inv_mass == 0.0`
  as the sole source of truth.
- **Rationale**: `inv_mass == 0.0` is a floating-point comparison; an
  explicit flag documents intent for readers (README's "readable over
  clever" principle) and matches how `body.rs` will be described in
  `README.md`/`CLAUDE.md` ("dynamic bodies (affected by forces and
  gravity)" vs. "static bodies (immovable)").
- **Alternatives considered**:
  - *Infer purely from `inv_mass`* — rejected: correct in practice but
    relies on exact float equality to `0.0`, which is fragile style for a
    project whose stated goal is readability.

## Decision: `World` owns a single constant gravity vector, applied uniformly

- **Decision**: `World` stores `gravity: Vec2` (default `(0.0, -9.81)`,
  with the demo's "down" being whichever screen-space convention
  `examples/bouncing.rs` renders with) and applies it to every dynamic
  body's velocity each fixed step, per the README's `v += (F/m + g) * dt`.
- **Rationale**: Matches spec FR-005 and the README's integration formula
  exactly, in the simplest form that supports the milestone. No per-body
  gravity scale is in scope (spec Assumptions).
- **Alternatives considered**:
  - *Per-body gravity scale factor* — rejected as premature: not requested
    by the spec or README, adds an unused field before any consumer needs
    it (violates the "don't design for hypothetical future requirements"
    guidance).

## Decision: Fixed-timestep accumulator has no maximum-substep clamp

- **Decision**: `World::step(&mut self, dt_real: f32)` accumulates
  `dt_real` and drains it in whole `1/60` increments with no upper bound on
  how many steps run in one call.
- **Rationale**: The spec's Assumptions section explicitly defers a
  "spiral of death" guard as out of scope for M1 — the README does not
  call for one, and `bouncing.rs` runs at normal frame rates where the gap
  will not matter. Adding a clamp now would be an uncalled-for design
  decision layered onto a milestone whose job is just to get integration
  correct and visible.
- **Alternatives considered**:
  - *Clamp accumulated time per call (e.g., max 5 steps/frame)* — deferred,
    not rejected outright: worth revisiting if/when a milestone's demo
    exhibits real stutter-death; tracked as a possible follow-up, not
    introduced speculatively here.

## Decision: `examples/bouncing.rs` uses macroquad's own frame-delta as the real-time input to `World::step`

- **Decision**: The example calls `world.step(macroquad::time::get_frame_time())`
  once per rendered frame, letting the engine's accumulator turn that into
  zero or more fixed physics steps before drawing.
- **Rationale**: This is the direct, standard way to drive a fixed-timestep
  simulation from a variable-framerate render loop, and is exactly the
  scenario User Story 2 / FR-006 / FR-007 describe.
- **Alternatives considered**:
  - *Fix the example's loop to a hard-coded frame time* — rejected: would
    not exercise or demonstrate framerate-independence at all, defeating
    the purpose of the acceptance demo.

## Output

All Technical Context fields are resolved (none were `NEEDS
CLARIFICATION`); the decisions above are the only open design choices
Phase 1 needed before writing `data-model.md` and the API contract.
