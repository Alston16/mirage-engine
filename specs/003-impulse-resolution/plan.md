# Implementation Plan: Impulse Resolution (M3)

**Branch**: `003-impulse-resolution` | **Date**: 2026-09-19 | **Spec**: [spec.md](./spec.md)

**Input**: Feature specification from `/specs/003-impulse-resolution/spec.md`

## Summary

Add `solver.rs` to the `mirage` engine crate: a sequential-impulse solver
that consumes the `Manifold`s M2 already produces and applies normal
impulses with restitution (README § Resolution: `vr`, `e`, `j`) plus a
slop-gated positional correction. `RigidBody` gains `restitution` and
`inv_inertia` (inertia derived from `Shape` + mass). `World::step` is
reordered to the README/CLAUDE.md data flow — gravity → broadphase +
narrowphase → velocity iterations (~8) → position correction → position
integration — and now also integrates angular velocity into orientation,
which M1 left out. `examples/bouncing.rs` gains a static floor and balls of
differing restitution, and the milestone's "done when" criterion (`e = 1`
returns to within a few percent of drop height, `e = 0` stops dead) is
verified by running it with `--release`. No friction (M4), no new
dependencies.

Two pre-existing gaps in M1/M2 must be closed for the solver to be correct;
both are recorded in [research.md](./research.md):

1. `detect_contacts` returns a `b → a` normal for `(Circle, Polygon)` pairs,
   contradicting the documented and README-required `a → b` convention.
2. `World::step` never integrates `angular_velocity` into `orientation`.

## Technical Context

**Language/Version**: Rust, 2024 edition (unchanged).

**Primary Dependencies**: None in the `mirage` library crate (Constitution
Principle I). `macroquad` stays `[dev-dependencies]`-only; only
`examples/bouncing.rs` changes.

**Storage**: N/A — in-memory simulation state only.

**Testing**: `cargo test`, following the existing inline `#[cfg(test)] mod
tests` convention. Solver tests are behavioral: drop-height ratio for
`e ∈ {0, 0.5, 1}`, momentum conservation, static-body immutability,
separating-contact no-op, determinism, 10 s resting stability, off-center
polygon hit producing angular velocity.

**Target Platform**: Native desktop — unchanged; engine is `std`-only.

**Project Type**: Single library crate (`mirage`) with `examples/`.

**Performance Goals**: None beyond the existing O(n²) broadphase scale
(hundreds of bodies). ~8 velocity iterations per step per README.

**Constraints**: Zero dependencies; no `unsafe`; deterministic (fixed
iteration order = manifold order × point order, no hashing/randomness);
normal impulses only (FR-010); static bodies never mutated (FR-007);
accumulated normal impulse clamped ≥ 0 (FR-005).

**Scale/Scope**: One new module (`solver.rs`); edits to `body.rs`,
`shape.rs`, `world.rs`, `collision/mod.rs`, `lib.rs`; one updated example.
README § Resolution gets a short clarification (see Constitution Check III).

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Check | Status |
|---|---|---|
| I. Zero-Dependency Engine Core | `solver.rs` and the additions use only `std` and the crate's own math. No `unsafe`. | PASS |
| II. Deterministic Fixed-Timestep Simulation | Resolution runs inside the existing fixed `1/60` accumulator loop in manifold order; no data-dependent ordering. Integration stays semi-implicit Euler: velocity (gravity + impulses) is final before position is advanced. | PASS |
| III. Spec-Faithful Math | Code uses README names `e`, `j`, `vr`, `n`, `r_a`, `r_b`, `m_a`, `I_a`. Two points go beyond the README's one-line derivation: (a) iterated form of `j` (target velocity `−e·vr₀·n` fixed before iterating, impulses accumulated and clamped), and (b) positional correction as a linear projection with `percent` and `slop`. Both are refinements of the README text, not contradictions; per Principle III the README § Resolution is updated first, in the same change (task T014, sequenced before any solver code). | PASS (with README update) |
| IV. Milestone-Gated, Behaviorally-Verified Development | M2's "done when" (box on ramp shows correct contacts) was met and merged. M3 is verified by running `examples/bouncing.rs --release` and observing `e = 0` stops and `e = 1` returns to ≈ drop height, not by `cargo test` alone. | PASS |
| V. Explicit Non-Goals (Scope Discipline) | No friction/warm-starting (M4), joints, CCD, sleeping, spatial index. Angular integration and inertia are required by README § Integration and FR-003, not new scope. | PASS |

No violations. Complexity Tracking table not needed.

**Post-design re-check**: unchanged — PASS.

## Project Structure

### Documentation (this feature)

```text
specs/003-impulse-resolution/
├── plan.md              # This file
├── research.md          # Phase 0: solver formulation decisions
├── data-model.md        # Phase 1: new body fields, solver-internal state
├── quickstart.md        # Phase 1: how to verify M3 behaviorally
├── contracts/
│   └── engine-api.md    # Public API added/changed by this milestone
├── checklists/
│   └── requirements.md
└── tasks.md             # Phase 2 (/speckit-tasks — NOT created here)
```

### Source Code (repository root)

```text
mirage-engine/
├── src/
│   ├── lib.rs               # CHANGED: pub mod solver (crate-private items only)
│   ├── body.rs              # CHANGED: + restitution, inv_inertia; with_restitution()
│   ├── shape.rs             # CHANGED: + Shape::inertia(mass) (about local origin)
│   ├── world.rs             # CHANGED: step reordered; angular integration; solver call;
│   │                        #          replaces the M2 "contact doesn't alter velocity" test
│   ├── solver.rs            # NEW: velocity iterations, restitution, position correction
│   └── collision/
│       └── mod.rs           # CHANGED: negate normal for (Circle, Polygon) so it is a → b
├── examples/
│   ├── bouncing.rs          # CHANGED: static floor, balls with e ∈ {0 … 1}
│   └── ramp.rs              # unchanged (box will now slide — no friction until M4)
└── README.md                # CHANGED: § Resolution clarifications (Principle III)
```

**Structure Decision**: Single project, unchanged layout; `solver.rs` is
already named in the README architecture tree. Solver items are
`pub(crate)` — the public surface grows only by the two body fields, one
builder, and `Shape::inertia`.

## Step Order (per fixed substep)

Steps 3–5 are phases inside the single crate-private `solver::resolve` call
(see contracts/engine-api.md); steps 1, 2, 6 and 7 live in `World::step`.

```text
1. for dynamic bodies: v += g·dt ; ω unchanged (no torque source yet)
2. manifolds = detect_contacts(bodies)              # positions from previous step
3. solver::prepare(manifolds)                       # r_a, r_b, K⁻¹, bounce target per point
4. repeat 8×: for manifold, for point: apply Δj      # accumulated j clamped ≥ 0
5. solver::correct_positions(manifolds)             # slop-gated projection
6. for dynamic bodies: x += v·dt ; θ += ω·dt        # semi-implicit Euler
7. self.contacts = manifolds                        # exposed via World::contacts()
```

## Complexity Tracking

*No Constitution Check violations — table not needed.*
