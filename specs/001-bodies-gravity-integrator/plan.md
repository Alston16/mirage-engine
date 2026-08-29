# Implementation Plan: Bodies, Gravity & Fixed-Timestep Integrator

**Branch**: `001-bodies-gravity-integrator` | **Date**: 2026-08-29 | **Spec**: [spec.md](./spec.md)

**Input**: Feature specification from `/specs/001-bodies-gravity-integrator/spec.md`

## Summary

Add `RigidBody`/`BodyId` (`body.rs`) and `World` (`world.rs`) to the `mirage`
engine crate: a dynamic/static rigid body representation, a fixed-timestep
(1/60s) accumulator, and semi-implicit Euler integration of gravity
(9.81 units/s², downward) with zero collision detection. Wire the result
into a new `examples/bouncing.rs` (macroquad dev-dependency only) that
renders falling circles to visually confirm the milestone's "done when"
criterion. No new engine dependencies; `math.rs` (`Vec2`/`Rot2`, from M0) is
the only internal dependency this milestone builds on.

## Technical Context

**Language/Version**: Rust, 2024 edition (`Cargo.toml` already pins
`edition = "2024"` from M0)

**Primary Dependencies**: None in the `mirage` library crate (zero-dependency
constraint). `macroquad` added under `[dev-dependencies]` only, scoped to
`examples/bouncing.rs`.

**Storage**: N/A — in-memory simulation state only, no persistence.

**Testing**: `cargo test`, using the existing per-module `#[cfg(test)] mod
tests` convention established in `src/math.rs` (no separate `tests/`
directory).

**Target Platform**: Native desktop (Windows/Linux/macOS), wherever
`macroquad` runs the example window; the engine crate itself (`body.rs`,
`world.rs`) is platform-independent `std`-only Rust.

**Project Type**: Single library crate (`mirage`) with a `examples/`
directory for runnable demos — no workspace split, per README architecture
and Constitution "Technical Constraints".

**Performance Goals**: None specific to this milestone — body counts in
`bouncing.rs` are small (a handful of circles); the O(n²) broadphase and
solver performance concerns belong to M2–M4.

**Constraints**: Zero dependencies in the engine crate; no `unsafe`;
semi-implicit Euler only (not explicit Euler); fixed `dt = 1/60`, accumulator
pattern, decoupled from render framerate; no collision detection or
response of any kind this milestone (explicit non-goal boundary — that is
M2).

**Scale/Scope**: Two new engine modules (`body.rs`, `world.rs`) plus one new
example (`examples/bouncing.rs`). No changes to `math.rs`'s public API
expected; `lib.rs` gains re-exports for the new public types.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Check | Status |
|---|---|---|
| I. Zero-Dependency Engine Core | `body.rs`/`world.rs` use only `std` and `math.rs`; `macroquad` is added strictly as `[dev-dependencies]`, consumed only by `examples/bouncing.rs`. No `unsafe`. | PASS |
| II. Deterministic Fixed-Timestep Simulation | `World::step` takes real elapsed time, accumulates it, and drains it in fixed `1/60` increments; integration is semi-implicit Euler (velocity before position), per README exactly. | PASS |
| III. Spec-Faithful Math | Integration uses the README's own formula (`v += (F/m + g) * dt; x += v * dt`) with matching structure; no alternate integrator or renamed terms introduced. | PASS |
| IV. Milestone-Gated, Behaviorally-Verified Development | M0 (`math.rs` + tests) is complete and merged; this plan only adds M1 scope (bodies + integrator + `bouncing.rs`) and explicitly excludes M2+ work. Verification includes actually running `cargo run --example bouncing --release`, not just `cargo test`. | PASS |
| V. Explicit Non-Goals (Scope Discipline) | No collision/AABB/manifold logic, no sleeping, no joints, and no floor body are introduced — circles fall through the window bounds with nothing to collide against, matching the spec's edge cases. | PASS |

No violations. Complexity Tracking table is not needed.

## Project Structure

### Documentation (this feature)

```text
specs/001-bodies-gravity-integrator/
├── plan.md              # This file (/speckit-plan command output)
├── research.md          # Phase 0 output (/speckit-plan command)
├── data-model.md        # Phase 1 output (/speckit-plan command)
├── quickstart.md        # Phase 1 output (/speckit-plan command)
├── contracts/           # Phase 1 output (/speckit-plan command)
│   └── engine-api.md    # Public Rust API surface added by this milestone
└── tasks.md             # Phase 2 output (/speckit-tasks command - NOT created by /speckit-plan)
```

### Source Code (repository root)

```text
mirage-engine/
├── Cargo.toml            # macroquad added under [dev-dependencies] only
├── src/
│   ├── lib.rs             # gains: pub mod body; pub mod world; + re-exports
│   ├── math.rs            # unchanged (M0)
│   ├── body.rs            # NEW: RigidBody, BodyId, dynamic/static classification
│   └── world.rs           # NEW: World, fixed-timestep accumulator, gravity, step(dt)
└── examples/
    └── bouncing.rs        # NEW: macroquad demo — falling circles, no collision
```

**Structure Decision**: Single project (matches the existing M0 layout — no
workspace split, no `tests/` directory). Unit tests live inline in
`body.rs` and `world.rs` via `#[cfg(test)] mod tests`, following the
precedent set in `src/math.rs`. `examples/bouncing.rs` is the only place
`macroquad` is referenced, consistent with the "engine crate has no
rendering code" architecture rule.

## Complexity Tracking

*No Constitution Check violations — table not needed.*
