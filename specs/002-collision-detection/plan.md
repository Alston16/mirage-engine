# Implementation Plan: Collision Detection (M2)

**Branch**: `002-collision-detection` | **Date**: 2026-09-05 | **Spec**: [spec.md](./spec.md)

**Input**: Feature specification from `/specs/002-collision-detection/spec.md`

## Summary

Add shape geometry (`shape.rs`), AABB broadphase (`broadphase.rs`), and
narrowphase collision detection (`collision/{mod,circle,polygon,manifold}.rs`)
to the `mirage` engine crate, per README § "How it works" → Broadphase /
Narrowphase and M2's milestone entry. `RigidBody` (M1) gains a `shape: Shape`
field. Every fixed `World::step` runs broadphase (O(n²) pair loop with AABB
rejection) then narrowphase (circle–circle, circle–polygon, polygon–polygon
via SAT + face clipping) and stores the resulting `Contact`/`Manifold` list
for the step — no velocity/position response yet (that's M3). Extend
`examples/bouncing.rs` (or add a new example) to debug-draw each contact's
point and normal, verified against the milestone's "done when" criterion: a
box resting on a rotated ramp shows correct contact geometry. No new engine
dependencies.

## Technical Context

**Language/Version**: Rust, 2024 edition (unchanged from M0/M1).

**Primary Dependencies**: None in the `mirage` library crate (zero-dependency
constraint, Constitution Principle I). `macroquad` remains a
`[dev-dependencies]`-only addition for debug-drawing contacts in the
example(s).

**Storage**: N/A — in-memory simulation state only.

**Testing**: `cargo test`, following the existing per-module `#[cfg(test)]
mod tests` convention (`math.rs`, `body.rs`, `world.rs`). Collision math gets
the heaviest new coverage: known geometric fixtures for each of the three
narrowphase pairs (circle–circle, circle–polygon, polygon–polygon), plus
broadphase pair-rejection coverage.

**Target Platform**: Native desktop (Windows/Linux/macOS) — unchanged from
M1; the engine crate itself remains platform-independent `std`-only Rust.

**Project Type**: Single library crate (`mirage`) with `examples/` demos —
unchanged from M0/M1, per README architecture and Constitution "Technical
Constraints".

**Performance Goals**: None beyond the README's explicit acceptance of an
O(n²) broadphase at MVP scale (hundreds of bodies) — a spatial index is a
non-goal (Constitution Principle V).

**Constraints**: Zero dependencies in the engine crate; no `unsafe`;
broadphase MUST reject non-overlapping AABB pairs before narrowphase runs;
narrowphase MUST NOT mutate any body's velocity or position (FR-007 — that's
M3's job); polygon narrowphase MUST work at arbitrary orientation, not just
axis-aligned.

**Scale/Scope**: Three new engine modules (`shape.rs`, `broadphase.rs`,
`collision/` with `mod.rs`, `circle.rs`, `polygon.rs`, `manifold.rs`).
`body.rs` gains a `shape` field on `RigidBody`. `world.rs` gains a
broadphase+narrowphase pass in `step` and a way to read back the current
contact list. `lib.rs` gains re-exports. One updated or new example for
debug-drawn contacts.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Check | Status |
|---|---|---|
| I. Zero-Dependency Engine Core | `shape.rs`/`broadphase.rs`/`collision/*` use only `std` and the crate's own `math.rs`/`body.rs`; `macroquad` stays a `[dev-dependencies]`-only addition, consumed only by example code. No `unsafe`. | PASS |
| II. Deterministic Fixed-Timestep Simulation | Broadphase+narrowphase run once per fixed `1/60` step inside `World::step`'s existing accumulator loop, in a fixed iteration order (body storage order), so results are reproducible given identical inputs. No change to the M1 integrator. | PASS |
| III. Spec-Faithful Math | Circle–circle overlap (`\|d\| < r1+r2`), circle–polygon closest-point test, and polygon–polygon SAT + reference/incident face clipping are implemented exactly as derived in README § Narrowphase, with matching terminology (reference face, incident face, penetration). | PASS |
| IV. Milestone-Gated, Behaviorally-Verified Development | M1 (bodies/gravity/integrator) is complete and merged. This plan is scoped to M2 only — no impulse resolution, restitution, or friction (that's M3). Verification includes actually running the updated example with `--release` to visually confirm contact points/normals on a box-on-ramp scene, per the milestone's own "done when" criterion, not just `cargo test`. | PASS |
| V. Explicit Non-Goals (Scope Discipline) | No spatial index (broadphase stays O(n²) with AABB rejection); no concave/compound shapes; no CCD — fast bodies may tunnel in a single step, and that's accepted per spec Assumptions, not fixed here. | PASS |

No violations. Complexity Tracking table is not needed.

## Project Structure

### Documentation (this feature)

```text
specs/002-collision-detection/
├── plan.md              # This file (/speckit-plan command output)
├── research.md          # Phase 0 output (/speckit-plan command)
├── data-model.md         # Phase 1 output (/speckit-plan command)
├── quickstart.md         # Phase 1 output (/speckit-plan command)
├── contracts/
│   └── engine-api.md    # Public Rust API surface added/changed by this milestone
└── tasks.md              # Phase 2 output (/speckit-tasks command - NOT created by /speckit-plan)
```

### Source Code (repository root)

```text
mirage-engine/
├── Cargo.toml            # unchanged (macroquad already a dev-dependency)
├── src/
│   ├── lib.rs             # gains: pub mod shape; pub mod broadphase; pub mod collision; + re-exports
│   ├── math.rs            # unchanged (M0)
│   ├── body.rs             # CHANGED: RigidBody gains `shape: Shape`; constructors take a Shape
│   ├── world.rs            # CHANGED: World::step runs broadphase+narrowphase each fixed substep;
│   │                       #          gains a way to read back the current contact list
│   ├── shape.rs            # NEW: Shape { Circle { radius }, Polygon { vertices, normals } }, AABB
│   ├── broadphase.rs       # NEW: O(n²) candidate-pair generation with AABB overlap rejection
│   └── collision/
│       ├── mod.rs          # NEW: dispatch narrowphase by shape-pair type
│       ├── circle.rs       # NEW: circle–circle, circle–polygon narrowphase
│       ├── polygon.rs      # NEW: polygon–polygon SAT, reference/incident face selection
│       └── manifold.rs     # NEW: Contact, Manifold, incident-face clipping
└── examples/
    ├── bouncing.rs         # unchanged, or minimally extended
    └── ramp.rs             # NEW (or folded into bouncing.rs): box-on-rotated-ramp scene used to
                             #      verify M2's "done when" criterion, debug-drawing contacts
```

**Structure Decision**: Single project, no workspace split — matches the
existing M0/M1 layout and README's module tree exactly. Unit tests live
inline in each new module via `#[cfg(test)] mod tests`, following the
`math.rs`/`body.rs`/`world.rs` precedent. `collision/` is a directory module
per the README's own architecture diagram, not a flattened set of files.
`examples/` gains whatever scene is needed to visually verify the box-on-
ramp acceptance criterion; the engine crate itself still contains no
rendering code — all macroquad calls stay in `examples/*.rs`.

## Complexity Tracking

*No Constitution Check violations — table not needed.*
