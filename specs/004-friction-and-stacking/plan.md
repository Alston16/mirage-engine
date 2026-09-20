# Implementation Plan: Friction & Stable Stacking (M4)

**Branch**: `004-friction-and-stacking` | **Date**: 2026-09-20 | **Spec**: [spec.md](./spec.md)

**Input**: Feature specification from `/specs/004-friction-and-stacking/spec.md`

## Summary

Extend `solver.rs` with Coulomb friction (README § Resolution, the bullet marked
"M4 — not yet implemented"): a tangent impulse per contact point, clamped to
`±μ·j` against the *accumulated* normal impulse, solved inside the same
iterated loop as the normal impulse. `RigidBody` gains a `friction` material
property; a pair uses the symmetric combine `μ = √(μ_a·μ_b)`. Then make the
10-box tower and the pyramid actually stand, using the levers the README
milestone names: iteration-count tuning and warm-starting.

Phase 0 measured (in a throwaway copy of the crate, see
[research.md](./research.md) § Prototype evidence) that **iteration tuning
alone cannot hold the tower**: the M3 solver collapses a 10-box tower at every
count from 8 to 30, and even with friction added it still ends as a heap at 8
and 16. Friction **plus warm-starting** stands it. FR-007's gate is therefore
met, and warm-starting is in scope: contacts carry a stable `feature` id from
the narrowphase, `World` keeps a per-step impulse cache keyed by
`(body_a, body_b, feature)`, and the solver seeds each matched contact with
last step's accumulated `j`/`jt` before iterating. Contacts that vanish drop
out of the cache the same step (FR-008).

The same prototype showed the spec's original SC-001/SC-002 numbers were
tighter than this solver class delivers while the tower settles from its "just
touching" spawn (at slop 0.002 the top box is 6.7% low at t = 2 s, 2.4% at
10 s, 1.8% at 60 s; combined linear/angular speed last exceeds 0.01 at
~22 s). Those criteria were re-based on the measured data in the spec's
2026-09-20 Clarifications (10% always, 3% from 10 s, no creep; speed < 0.1
from 2 s and < 0.01 from 30 s). If the tuning tasks still miss the re-based
bounds, the fix is another data-backed spec amendment, not a silent
threshold change. See research.md § Spec-threshold risk.

Deliverables: `friction` on bodies, tangent solve + warm-start in the solver,
`feature` ids on contacts, `examples/stack.rs` and `examples/pyramid.rs`, a
two-ramp `examples/ramp.rs`, behavioral integration tests, and a README §
Resolution rewrite (done first, per Constitution III). No new dependencies, no
non-goals touched.

## Technical Context

**Language/Version**: Rust, 2024 edition (unchanged).

**Primary Dependencies**: None in the `mirage` library crate (Constitution
Principle I). `macroquad` stays `[dev-dependencies]`-only; used by the
examples only.

**Storage**: N/A — in-memory simulation state; the impulse cache lives in
`World` and is rebuilt every substep.

**Testing**: `cargo test`. Solver internals (friction clamp, combine rule,
cache matching/discard) get inline `#[cfg(test)]` unit tests in the existing
style. The behavioral M4 criteria (ramp thresholds and acceleration, the
±5%-of-`atan μ` boundary, rolling disc, 60 s tower, pyramid, mixed shapes,
15-box tower and 1000:1 mass ratio stability, bit-identical determinism) go in a new
`tests/` directory that uses only the public API, the way an engine user or
example does; shared scene builders live in `tests/common/mod.rs`. Baseline
before this feature: 69 unit tests passing.

**Target Platform**: Native desktop — unchanged; engine is `std`-only.

**Project Type**: Single library crate (`mirage`) with `examples/` and, new
this milestone, `tests/`.

**Performance Goals**: ≥ 60 FPS for the stack demo in `--release` (SC-008).
Cost is trivial at this scale: ~10–15 bodies, O(n²) broadphase unchanged.

**Constraints**: Zero dependencies; no `unsafe`; deterministic (fixed
iteration order = manifold order × point order; cache is a `Vec` scanned in
order — no `HashMap`, no thread-locals, no randomness); static bodies never
mutated; accumulated `j ≥ 0` and `|jt| ≤ μ·j` invariants; M3 behavior
preserved (FR-012).

**Scale/Scope**: Edits to `body.rs`, `solver.rs`, `world.rs`, `collision/`
(`manifold.rs`, `circle.rs`, `polygon.rs`, `mod.rs`), `lib.rs`, `README.md`;
new `tests/`, `examples/common/`, `examples/stack.rs`, `examples/pyramid.rs`;
`examples/ramp.rs` reworked. Tunable constants are in world units and are
calibrated for ~1 m bodies (the new demos use meters and a pixels-per-meter
draw scale).

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Check | Status |
|---|---|---|
| I. Zero-Dependency Engine Core | Only `std` and the crate's own math added. No `unsafe`. Cache is a plain `Vec` owned by `World`. | PASS |
| II. Deterministic Fixed-Timestep Simulation | Tangent solve and cache seeding run inside the fixed `1/60` substep in manifold × point order; cache lookup is an ordered scan. Integration stays semi-implicit Euler (impulses finalize velocity before position advances). A bit-identical 60 s tower test is included. | PASS |
| III. Spec-Faithful Math | README § Resolution **and § Narrowphase** are updated **before** any solver or narrowphase code (task sequenced first): § Narrowphase gains the contact `feature` id; § Resolution adds `t`, `vt`, `K_t`, `Δjt`, the `±μ·j` clamp, the combine rule, the iteration count, and the warm-start description. The tuned iteration count, `slop` and `percent` are written into the README when tuning finishes. Code uses README names `μ`, `j`, `jt`, `vr`, `n`, `t`, `r_a`, `r_b`. | PASS (with README update) |
| IV. Milestone-Gated, Behaviorally-Verified | M3's "done when" is marked met in the README and its example was run. Gate 0 of the quickstart re-runs `cargo test` (69 pass) and `bouncing --release` before M4 code. M4's done-when is verified by running `stack` and `ramp` with `--release`, not by `cargo test` alone. Note: this branch was cut from `003-impulse-resolution`; confirm whether M3's PR has merged before opening M4's. | PASS |
| V. Explicit Non-Goals | Warm-starting is named in the README's M4 text, so it is in scope; sleeping is **not** used to achieve stability (FR-016). `Contact.feature` is narrowphase bookkeeping, not a spatial index. No joints/CCD/concave/serialization/parallelism/3D. | PASS |

No violations. Complexity Tracking table not needed.

**Post-design re-check**: unchanged — PASS. The one design choice that touches
M2 code (`Contact.feature`) is additive, justified by measured evidence, and
covered by a narrowphase test.

## Project Structure

### Documentation (this feature)

```text
specs/004-friction-and-stacking/
├── plan.md              # This file
├── research.md          # Phase 0: prototype evidence + decisions
├── data-model.md        # Phase 1: body/contact/cache state, constants
├── quickstart.md        # Phase 1: how to verify M4 behaviorally
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
│   ├── lib.rs               # unchanged exports (solver items stay crate-private)
│   ├── body.rs              # CHANGED: + friction (μ), with_friction(); default 0.5
│   ├── world.rs             # CHANGED: owns impulse cache; step docs (friction, warm-start)
│   ├── solver.rs            # CHANGED: tangent solve, combine_friction, warm-start,
│   │                        #          ImpulseCache, retuned constants
│   └── collision/
│       ├── manifold.rs      # CHANGED: Contact + feature: u32
│       ├── circle.rs        # CHANGED: feature = 0
│       ├── polygon.rs       # CHANGED: feature from (ref face, incident face, endpoint, flip)
│       └── mod.rs           # CHANGED: preserve feature when negating the normal
├── tests/                   # NEW (public-API behavioral tests)
│   ├── common/mod.rs        #   ramp / tower / pyramid scene builders + metrics
│   ├── friction.rs          #   ramp hold/slide, sliding a, rolling disc, μ = 0
│   └── stacking.rs          #   10-box tower 60 s, pyramid 30 s, mixed shapes, determinism
├── examples/
│   ├── common/mod.rs        # NEW: shared macroquad drawing helpers (stack, pyramid)
│   ├── stack.rs             # NEW: 10-box tower — the MVP acceptance demo
│   ├── pyramid.rs           # NEW: stress case
│   ├── ramp.rs              # CHANGED: shallow + steep ramp, one box on each
│   └── bouncing.rs          # unchanged
└── README.md                # CHANGED: § Resolution + § Narrowphase first; Status/M4 checkbox last
```

**Structure Decision**: Single project, unchanged module layout; `solver.rs`
already exists in the README architecture tree, and the cache is a
crate-private type inside it. The one structural addition is `tests/`, chosen
because the M4 criteria are end-to-end behaviors on the public API (60 s
scenes) rather than solver internals; the README architecture tree gets `tests/`
and the two new examples listed. `examples/common/mod.rs` is a non-example
module (Cargo only treats `examples/<name>.rs` and `examples/<name>/main.rs`
as examples), used by the two new demos; the existing examples are not
refactored.

## Step Order (per fixed substep)

Steps 3–6 are phases inside the single crate-private `solver::resolve` call;
steps 1, 2, 7, 8 live in `World::step`. Changes from M3 are marked ◆.

```text
1. for dynamic bodies: v += g·dt
2. manifolds = detect_contacts(bodies)                  # positions from previous step
3. solver::prepare(manifolds)                           # r_a, r_b, K, K_t, t, μ, bounce
   ◆ then seed from cache: for each contact matching (a, b, feature) in the previous
     step's cache, set j_acc/jt_acc and apply P = j·n + jt·t to both bodies
     (bounce is computed BEFORE seeding, from the true approach velocity)
4. repeat N×: for manifold, for point:
   ◆ tangent:  Δjt = −vt / K_t ; jt_acc ← clamp(jt_acc + Δjt, −μ·j_acc, +μ·j_acc) ; apply
     normal:   Δj  = −(vn − bounce) / K ; j_acc ← max(j_acc + Δj, 0) ; apply
5. ◆ write cache ← (a, b, feature, j_acc, jt_acc) for every contact of THIS step
6. solver::correct_positions(manifolds)                 # slop-gated projection
7. for dynamic bodies: x += v·dt ; θ += ω·dt            # semi-implicit Euler
8. self.contacts = manifolds
```

The tangent-then-normal order lets the normal constraint have the final say
each iteration. The prototype only compared the two orders *without*
warm-starting (both collapsed the tower), so the order is a starting choice
with an A/B task under warm-starting; the result is recorded in research.md.

## Complexity Tracking

*No Constitution Check violations — table not needed.*
