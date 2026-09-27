# Implementation Plan: Shape COM Correction & Frictionless-Stack Limits (M5)

**Branch**: `005-shape-com-and-stability-hardening` | **Date**: 2026-09-27 | **Spec**: [spec.md](./spec.md)

**Input**: Feature specification from `/specs/005-shape-com-and-stability-hardening/spec.md`

## Summary

Closes two hardening issues filed after the MVP (M0–M4) was complete, with no
new physics capability and no change to `solver.rs`:

1. **Issue #7** — `Shape::polygon` computes the input vertex list's true
   center of mass (area-weighted centroid, same cross-product convention
   `Shape::inertia` already uses) and stores vertices shifted so local
   `(0, 0)` is always that point, regardless of where the caller's original
   vertices sat. Every shape already built in the engine, its examples, and
   its tests is already centered, so this is a no-op for all existing
   callers and only changes behavior for the (previously silently broken)
   off-center case.
2. **Issue #11** — the frictionless (`μ = 0`) stacking limitation (neutral
   stability, collapse above ~3 boxes) is documented in the README's
   Resolution section and pinned by a new regression test in
   `tests/stacking.rs`, reusing the exact measurements already recorded in
   the issue. No solver/velocity change — an explicit scope decision made
   before this spec was written.

Deliverables: a corrected `Shape::polygon`/`Shape::inertia` with updated doc
comments, new unit tests for off-center and near-degenerate polygon input, a
new frictionless-tower scene builder and boundary test in `tests/`, and a
README § Resolution addition. No new dependencies, no non-goals touched, no
milestone-order violation (M5 is post-MVP hardening, not a new milestone
inserted ahead of M0–M4, all of which are already complete).

## Technical Context

**Language/Version**: Rust, 2024 edition (unchanged).

**Primary Dependencies**: None in the `mirage` library crate (Constitution
I). `macroquad` stays `[dev-dependencies]`-only; unaffected — no example
changes are needed since existing demos already use pre-centered shapes.

**Storage**: N/A — in-memory simulation state, unchanged.

**Testing**: `cargo test`. The centroid correction gets inline
`#[cfg(test)]` unit tests in `src/shape.rs`, in the existing style
(off-center-vs-centered equivalence, near-degenerate finiteness). The
frictionless-stacking boundary gets a new behavioral test in
`tests/stacking.rs`, using a new scene builder in `tests/common/mod.rs`
alongside the existing `tower()` (which stays at its current `μ = 0.5`
default and is unchanged, since every other stacking test depends on it).

**Target Platform**: Native desktop — unchanged; engine is `std`-only.

**Project Type**: Single library crate (`mirage`) with `examples/` and
`tests/` — unchanged structure, no new project type.

**Performance Goals**: N/A beyond the existing SC-008 (60 FPS stack demo)
from M4, which this feature doesn't touch. The centroid computation is a
one-time cost at `Shape::polygon` construction, not per-step.

**Constraints**: Zero dependencies; no `unsafe`; deterministic (the
centroid computation is a pure function of the input vertices, evaluated
once at construction — no iteration-order or floating-point-summation-order
change to anything that already runs per step); static bodies never
mutated; `solver.rs`'s existing invariants (`j ≥ 0`, `|jt| ≤ μ·j`) and
tunables are untouched (FR-010).

**Scale/Scope**: Edits to `src/shape.rs` (centroid computation, recentering,
updated doc comments, new unit tests) and `README.md` (Resolution section);
new coverage in `tests/stacking.rs` and a new builder in
`tests/common/mod.rs`. No edits to `src/solver.rs`, `src/world.rs`,
`src/body.rs`, `src/broadphase.rs`, `src/collision/*`, or any `examples/*`.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Check | Status |
|---|---|---|
| I. Zero-Dependency Engine Core | Only `std` and the crate's existing math (`Vec2::cross`, arithmetic) are used for the centroid. No `unsafe`. No new dependency, dev or otherwise. | PASS |
| II. Deterministic Fixed-Timestep Simulation | The centroid computation runs once at `Shape::polygon` construction (not per step) and is a pure, order-independent function of the input vertex list. No change to `World::step`, the accumulator, or integration order. The frictionless-boundary test reuses the same fixed-`dt` stepping as the rest of `tests/stacking.rs`. | PASS |
| III. Spec-Faithful Math | README gains the centroid derivation (same cross-product convention as the existing `inertia()` derivation it sits beside) and the `μ = 0` known-trade-off note, in the same change as the code (task sequenced first, per precedent). Doc comments in `shape.rs` describing the *unchecked* assumption are replaced with the corrected, always-centered description. | PASS (with README update) |
| IV. Milestone-Gated, Behaviorally-Verified | M0–M4 are complete and verified (README `## Status`). M5 is post-MVP hardening closing filed issues, not a new milestone inserted ahead of or interleaved with M0–M4 — it doesn't reorder or reopen any of them. Gate 0 (quickstart.md) re-confirms `cargo test` and `stack --release` pass before M5 code. Done-when is verified by running all four existing example demos with `--release` (expected to be behaviorally invisible, SC-005) plus `cargo test`. | PASS |
| V. Explicit Non-Goals | No joints/CCD/sleeping/concave/spatial-index/serialization/parallelism/3D touched. The frictionless-collapse fix explicitly excludes a velocity/speed clamp (FR-010) — a deliberate decision recorded in the spec's Assumptions, made precisely so this feature doesn't quietly expand scope into solver changes. | PASS |

No violations. Complexity Tracking table not needed.

## Project Structure

### Documentation (this feature)

```text
specs/005-shape-com-and-stability-hardening/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md         # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/
│   └── engine-api.md    # Phase 1 output
├── checklists/
│   └── requirements.md
└── tasks.md             # Phase 2 output (/speckit-tasks — not this command)
```

### Source Code (repository root)

```text
mirage-engine/
├── src/
│   ├── shape.rs          # CHANGED: centroid computation + recentering in
│   │                      #   Shape::polygon; updated doc comments; new
│   │                      #   #[cfg(test)] coverage (off-center equivalence,
│   │                      #   near-degenerate finiteness)
│   ├── body.rs            # UNCHANGED
│   ├── world.rs           # UNCHANGED
│   ├── broadphase.rs      # UNCHANGED
│   ├── solver.rs          # UNCHANGED (explicitly out of scope, FR-010)
│   └── collision/         # UNCHANGED
├── tests/
│   ├── common/mod.rs      # CHANGED: new frictionless-tower scene builder,
│   │                      #   added alongside the existing tower()
│   └── stacking.rs        # CHANGED: new frictionless-boundary test
├── examples/               # UNCHANGED (no new demo; existing four are the
│                           #   regression check per quickstart.md §2)
└── README.md               # CHANGED: Resolution section gains the μ = 0
                            #   known-trade-off note; shape/inertia derivation
                            #   text updated for the corrected behavior
```

**Structure Decision**: Single lib crate, unchanged (no workspace split, no
new project type). This feature is scoped to one module (`src/shape.rs`) plus
test-only additions — the smallest structural footprint of any milestone so
far, matching its hardening (not new-capability) nature.

## Complexity Tracking

Not applicable — no Constitution Check violations.
