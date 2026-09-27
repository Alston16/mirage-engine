# Quickstart: verifying M5

## 0. Gate — the MVP baseline is unaffected

Before any M5 code:

```
cargo test                              # baseline: confirm the current passing count
cargo run --example stack --release     # M4 acceptance demo still holds
```

M5 does not touch `solver.rs`; the gate here is that the *current* suite and
demos pass before any change, so a later failure is attributable to this
feature.

## 1. Unit + behavioral tests

```
cargo test
cargo test --release stacking -- --nocapture   # prints tower/frictionless-tower metrics
```

Expect passing tests covering, at minimum:

| Behavior | Spec ref |
|---|---|
| Off-center polygon (e.g. rectangle with a corner at the local origin) yields the same stored vertices, mass, and `inertia()` as the equivalent pre-centered polygon | US1-1, US1-2, FR-001, FR-002, SC-001 |
| Every polygon already built in `src/shape.rs`, examples, and tests is unchanged (zero recentering shift) | US1-1, FR-003, SC-002 |
| A body built from off-center vertices, placed at a world position, behaves identically (position, orientation, response to an off-center impulse) to the equivalent pre-centered body at the same position | US1-3, US1-4, FR-004, SC-001 |
| Near-degenerate (small-area, still valid) polygon: centroid and `inertia()` both stay finite | FR-005, SC-003 |
| Near-zero-mass body with an off-center shape: recentering stays well-defined | FR-006 |
| 2-box frictionless (`μ = 0`) tower holds within a documented drift bound over 60 s | US2-1, FR-008, SC-003 |
| 5-or-10-box frictionless tower collapses (large drift/speed) — pinned as the expected, passing outcome | US2-2, FR-008, SC-003 |
| Same scenes with `μ = 0.5` (existing `tower()` builder): unchanged from M4 | US2-4, FR-009 |
| Existing M3/M4 suite (restitution, ramp, 10-box tower at `μ = 0.5`, pyramid, mixed shapes, determinism) | FR-009, SC-006 |

## 2. Behavioral checks (required by Constitution IV)

All with `--release`:

```
cargo run --example bouncing --release
cargo run --example ramp --release
cargo run --example stack --release
cargo run --example pyramid --release
```

Observe: all four demos look exactly as they did before this feature — this
change is expected to be **invisible** in every existing demo (SC-005), since
every shape they build was already centered. There is no new demo for this
milestone; the centroid correction is verified by the off-center-vs-centered
equivalence tests (§1), not by a new visual.

## 3. Documentation

`README.md` § Resolution gains the `μ = 0` known-trade-off note (FR-007,
SC-004); `Shape::polygon`/`Shape::inertia`'s doc comments are updated to
describe the corrected (always-centered) behavior in place of the current
"caller's responsibility, unchecked" language.

## 4. Expected non-behavior

- Frictionless stacks above ~3 boxes still collapse — this feature documents
  and pins that behavior, it does not fix it (explicit scope decision, see
  `research.md` § Q3 and `spec.md` Assumptions).
- No velocity/speed clamp, no CCD, no change to `solver.rs`'s tunables.
