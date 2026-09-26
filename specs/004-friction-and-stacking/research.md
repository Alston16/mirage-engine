# Phase 0 Research: Friction & Stable Stacking (M4)

No `NEEDS CLARIFICATION` markers remained in the Technical Context — README §
Resolution and the M4 milestone pin the approach (tangent impulses, iteration
tuning, warm-starting "if needed"). "If needed" is the one open question, so
it was settled with an experiment rather than by opinion, then the remaining
decisions were recorded.

## Prototype evidence (throwaway, not in the repo)

A copy of the crate was built in the session scratchpad and driven by a small
probe: N unit boxes (half-extent 0.5, mass 1) stacked touching on a static
floor, `World::step(1/60)` for 3600 steps, release build. Prototype changes
were made only in the copy: hard-coded `μ = 0.5`, tangent solve, and a
warm-start cache keyed by `(a, b, point index)`. "Sink" is
`ideal_top_y − top.y`; "speed" is the max over boxes of `|v|` and `|ω|·0.5`.

**Q1 — Does iteration count alone hold the tower? (M3 solver, no friction)**

| Iterations | Result |
|---|---|
| 8, 12, 16, 20, 30 | Collapses: horizontal drift of 2+ units by t = 2 s, bodies flung to speeds > 100 |

Smallest failing case: **2 boxes** (drift 0.034 and rising by t = 2 s; 1 box is
rock solid). Two-point manifolds solved one point at a time inject a small
torque each pass; nothing damps the resulting rock-and-slide.

**Q2 — Does friction fix it?** Tangent solve added, both orderings, 8 and 16
iterations: sliding runaway stops (bodies come to rest) but the tower still
**ends as a heap** (top sink ≈ 9.0 = fully fallen, angles ≈ π).

**Q3 — Friction + warm-starting**, 10 boxes:

| Iterations | Slop | Sink @ 2 s | @ 10 s | @ 60 s | Drift @ 60 s | Speed last > 0.01 |
|---|---|---|---|---|---|---|
| 8 | 0.01 | 0.106 | 0.099 | 0.096 | 0.032 | — |
| 12 | 0.01 | 0.106 | 0.099 | 0.098 | 0.014 | — |
| 16 | 0.01 | 0.106 | 0.100 | 0.098 | 0.008 | — |
| 12 | 0.005 | 0.082 | 0.053 | 0.048 | 0.015 | 22.3 s |
| 12 | 0.002 | 0.067 | 0.024 | 0.018 | 0.015 | 22.3 s |
| 12 | 0.001 | 0.062 | 0.015 | 0.033 | 0.073 | 60 s (jitter grows) |

(Sink and drift in world units = box heights, since boxes are 1 m.)

**Q4 — Does friction + warm-start break M3?** The prototype's full `cargo test`
run: 69/69 pass (with `PENETRATION_SLOP = 0.002`, `μ = 0.5`).

### Conclusions drawn

1. **Warm-starting is needed** (FR-007's gate). Neither iterations (Q1) nor
   friction (Q2) stands the tower; friction + warm-start does (Q3). The
   in-repo tasks re-confirm Q1/Q2 on the real implementation before adding the
   cache, sweeping iterations {8, 12, 16, 20, 30} with friction only, so the
   evidence for FR-007's "iteration count alone" is recorded against the
   shipped code.
2. **Steady-state compression ≈ slop × contacts.** With slop 0.01 the tower
   settles ~0.098 low (≈ ten interfaces resting at slop); position correction
   only acts *above* slop, so slop is the floor on resting penetration.
3. **Slop has a lower stability limit near 0.002** for this scene: 0.001
   made the tower jitter and drift. It must stay comfortably above the
   narrowphase's own contact threshold (`SLOP = 1e-4`), otherwise resting
   contacts flicker in and out of existence.
4. **Settling is slow but small.** Speed stays above 0.01 for ~22 s at
   0.05 m/s or less — at 40 px/m that is under 2 px/s, invisible, but above
   the spec's numeric jitter threshold.

## Spec-threshold risk (resolved by amending the spec; residual risk remains)

The spec's original numbers (top box ≤ 2% low for the whole 60 s; speed
< 0.01 m/s after 2 s) were written before this evidence: the prototype was
6.7% low at t = 2 s while the tower settled from its "just touching" spawn
(top boxes fall onto boxes already stopped below; unavoidable with no contact
margin) and needed ~22 s to get under 0.01.

Resolution (spec.md § Clarifications, 2026-09-20): compression ≤ 10% from
1 s (originally "at all times" — see § Start-up transient below), ≤ 3% from 10 s, creep ≤ 0.1% between 10 s and 60 s; speed < 0.1 m/s
from 2 s and < 0.01 m/s from 30 s. Each bound has margin against the
prototype (6.7% / 2.4% / 0.0% creep; last speed > 0.01 at 22 s), and 0.1 m/s
is about 4 px/s at the demo scale — not visible as motion.

Residual risk: the prototype used a hard-coded key and one parameter point.
If the real implementation cannot meet the re-based bounds after the tuning
sweep, the acceptable resolutions are (a) another data-backed amendment via
`/speckit-clarify`, or (b) a solver feature outside M4's stated levers (e.g., a
contact margin or split impulses) — which would need a scope decision first.
The test harness prints the measured values either way.

## Decision: friction is a per-body property, combined as `μ = √(μ_a · μ_b)`

- **Decision**: `RigidBody.friction: f32`, default `0.5`, builder
  `with_friction(μ)` clamped to `≥ 0`. The pair coefficient is
  `combine_friction(a, b) = (a·b).sqrt()` — commutative, so body order never
  matters (FR-002).
- **Rationale**: FR-011 requires that setting friction to zero reproduces the
  frictionless M3 behavior. `max` (the restitution rule) would break that: a
  `μ = 0` block on a default `μ = 0.5` floor would still stick. The geometric
  mean gives 0 whenever either surface is 0 and returns `μ` when both match.
  Same rule as Box2D.
- **Default 0.5** puts the slide/hold threshold `θ = atan(μ)` at 26.6°,
  between the demo ramps (15° holds, 35° slides), and lets a default floor +
  default box hold a stack without configuration.
- **Alternatives considered**: `max` — rejected (FR-011); `min` — order
  independent but a rough floor could never grip a slick body; arithmetic mean
  — rejected for the same FR-011 reason; default `0.0` — rejected: the
  ramp/stack criteria would fail out of the box.

## Decision: tangent impulse formulation (README-name derivation)

- `t = (−n_y, n_x)` (n rotated +90°). The sign is irrelevant: the clamp is
  symmetric.
- `vt = vr · t` with the same `vr` as the normal solve (FR-003).
- `K_t = 1/m_a + 1/m_b + (r_a×t)²/I_a + (r_b×t)²/I_b`.
- `Δjt = −vt / K_t` (drive tangential relative velocity to zero — no
  "restitution" for friction).
- `jt_acc ← clamp(jt_acc + Δjt, −μ·j_acc, +μ·j_acc)` and only the change is
  applied, using the contact's **accumulated** normal impulse `j_acc` (FR-005),
  so friction capacity grows as load builds up through a stack.
- Contacts where `K_t == 0` (both static) are already dropped.
- **Alternatives considered**: clamp against the per-iteration `Δj` —
  rejected (FR-005; friction capacity would collapse to ≈ 0 once the normal
  solve converges); friction as a velocity-level Coulomb *cone* (2-D
  coupling) — unnecessary in 2-D, where the tangent space is one-dimensional.

## Decision: per-iteration order is tangent, then normal

- **Decision**: within each contact visit, apply the tangent impulse, then the
  normal impulse.
- **Rationale**: non-penetration matters more than sticking; ending each
  visit on the normal solve leaves that constraint freshest. With
  warm-starting `j_acc` is already non-zero on the first iteration, so
  ordering doesn't starve friction of capacity.
- **Status**: a starting choice. The prototype only compared orders without
  warm-starting (both collapsed). A tuning task runs the real solver both ways
  on tower + pyramid and records the winner here; the alternative
  (normal-then-tangent, Box2D-Lite's order) is a one-line swap.

## Decision: warm-start with narrowphase feature ids

- **Decision**: add `pub feature: u32` to `Contact`. Circle contacts use `0`.
  Polygon contacts pack `(flip, reference face, incident face, which clipped
  endpoint)`. The solver's cache entry is
  `(body_a, body_b, feature, j_acc, jt_acc)`. On the next step each new contact
  looks up its key in the previous step's entries (ordered scan; ≤ a few dozen
  entries); a hit seeds `j_acc`/`jt_acc` and applies `P = j·n + jt·t` to both
  bodies before iterating. The cache is then **replaced wholesale** by this
  step's contacts, so anything not touching this step disappears (FR-008).
- **Rationale**: Box2D-Lite's feature pairs are the README-referenced
  construction. Matching by feature is exact, deterministic and needs no
  tolerance; matching by point proximity would need a length scale the engine
  doesn't have (bodies may be pixels or meters). The prototype's
  index-based key proves the mechanism works but would mis-pair contacts when
  the clipped point order flips (e.g., the reference face changes), giving
  exactly the "ghost impulses" FR-008 forbids.
- **Bounce ordering**: `bounce = −e·(vr·n)₀` is computed in `prepare`,
  **before** seeding, from the true approach velocity; seeding first would
  read a velocity already altered by last step's impulse and corrupt the
  restitution target. A fresh (unmatched) impact starts at `j_acc = 0`.
- **Cost**: additive field on a public struct. Struct-literal users (the two
  in-crate solver tests) get a mechanical update; the crate is pre-1.0 with
  no external consumers. Covered by a narrowphase test that features persist
  across a small motion of a resting box and differ between its two points.
- **Alternatives considered**: proximity matching — rejected (above);
  index-only key — rejected (above); a `HashMap` cache — rejected: iteration
  order is not deterministic and lookup cost is irrelevant at this size.
  Not warm-starting — rejected on the Q1–Q3 evidence.

## Decision: tuning parameters and how they get chosen

| Constant | M3 | M4 starting point | How chosen |
|---|---|---|---|
| `VELOCITY_ITERATIONS` | 8 | 12 | Sweep {8, 10, 12, 16} on tower + pyramid; smallest that meets targets with margin; recorded in README (FR-006) |
| `PENETRATION_SLOP` | 0.01 | 0.002–0.005 | Q3: 0.01 gives ~10% tower compression; 0.001 destabilises; sweep {0.002, 0.003, 0.005} |
| `CORRECTION_PERCENT` | 0.4 | 0.4 | Unchanged unless the sweep shows a reason |

All values are in world units, calibrated for ~1 m bodies. The demos therefore
use meters and a pixels-per-meter draw scale; the existing pixel-scale
`bouncing` example is unaffected (its bodies are large, so a smaller slop is
harmless).

## Decision: verification lives in `tests/` and examples, measured on the public API

- Behavioral M4 criteria are end-to-end scenes, so they are integration tests
  using only public items (`World`, `RigidBody`, `Shape`, `Vec2`, `Rot2`).
  This also checks that a user can build these scenes without crate
  internals. Solver-internal invariants stay inline.
- Metrics (defined once in `tests/common/mod.rs`): **sink** = ideal touching
  top height − actual; **drift** = max |x − x₀|; **speed** = max over bodies of
  `|v|` and `|ω|·half_extent`; **jitter** = max frame-to-frame position change
  after the settle window. The harness prints them under `--nocapture` so
  tuning is data-driven.
- Analytic references: hold/slide threshold `atan(μ)`; sliding acceleration
  `a = g(sin θ − μ cos θ)`; frictionless block `a = g sin θ`; rolling solid
  disc `a = (2/3) g sin θ` (holds when `μ ≥ tan θ / 3`).
- `cargo test` alone does not satisfy M4 (Constitution IV): `stack` and `ramp`
  must be run with `--release` and watched.

## Decision: examples

- `stack.rs`: 10 boxes, 1 m, mass 1, floor top at y = 0, drawn at a fixed
  pixels-per-meter scale; HUD shows elapsed time and the two headline metrics.
- `pyramid.rs`: 5 rows (15 boxes) — the README's stress case.
- `ramp.rs`: a shallow (15°) and a steep (35°) ramp with a box on each,
  contact debug-drawing kept. At the default `μ = 0.5` the shallow box holds
  and the steep box slides — the M4 criterion made visible.
- Shared draw helpers in `examples/common/mod.rs` for the two new demos; the
  existing examples are left as they are (no unrelated refactor).

## Decision: README first, status last

Per Constitution III the README § Resolution is rewritten before solver code:
tangent derivation, combine rule, warm-start, chosen iteration count.
Status text and the M4 checkbox are updated only after the examples have been
run with `--release` and the criteria observed.

## Note: current narrowphase contact threshold

`circle.rs`/`polygon.rs` report a contact only when penetration `> 1e-4`, and
the solver's slop leaves resting bodies penetrating by roughly `slop`, so
resting contacts persist. Any slop retune must stay well above `1e-4`
(Q3, slop 0.001 already misbehaved). The narrowphase threshold itself is not
changed by this feature.

## In-repo notes

Measurements and observations recorded while implementing (newest last).

### T001 — Gate 0 (M3 baseline), 2026-09-20

- `cargo test`: 69 passed, 0 failed.
- `cargo build --examples --release` succeeds; `bouncing.exe` launches and
  runs without panicking (killed by a 5 s timeout, exit 124).
- The M3 behaviors themselves (`e = 0` stops dead, `e = 1` returns within 5%
  of drop height) are covered headlessly by `e0_ball_stops_dead_on_floor` and
  `e1_ball_returns_to_drop_height`, both passing. The visual check of the
  bouncing window needs a person watching it; it is left to the user.

### T018 — US1 friction results (no warm-starting yet)

Measured on the tangent-solve implementation, before any cache:

| Test | Measured | Reference |
|---|---|---|
| Box, 35° ramp, μ = 0.5 | a = 1.596 | 1.609 (0.8% off) |
| Frictionless box, 30° | a = 4.905 | 4.905 (exact) |
| Disc, 20°, rolling | a = 2.229, \|ω\|·r = 4.481 vs v = 4.458 | 2.237 (0.3% off); ω·r within 0.5% of v |
| 30° ramp, distance in 3 s | μ = 0.3: 10.66 m; μ = 0.9: 0.009 m | higher μ slides less |
| Box, 15° ramp (hold) | moved 0.0024 in 10 s, speed 0.0002 | < 0.01 / < 0.01 — passes |
| Box, 1.05·atan μ (27.9°) | slides | passes |
| Box, 0.95·atan μ (25.2°) (hold) | moved **0.038** in 10 s, speed 0.0023 | limit 0.01 — **misses** |

Only the last hold case misses, and only because friction at 95% of its
limit creeps when each step's impulses restart from zero. Per the closing-out
rule that one test (`just_below_threshold_holds`) is `#[ignore]`d with its
bound unchanged, and is un-ignored once warm-starting lands.

Solver invariant note: the tangent solve runs before the normal solve in each
contact visit, so a later normal update can lower `j_acc` slightly after `jt`
was clamped against the earlier value. The clamp is exact when applied; the
final `|jt|` can exceed `μ·j` by a fraction of a percent (0.04% observed) and
the residual shrinks as iterations converge. The unit test allows 1% slack.
The normal-then-tangent order (the A/B in the tuning tasks) would make the
final state exact instead.

### T019 — `ramp` demo

`cargo build --examples --release` is clean; `ramp.exe` launches and runs
without panicking (5 s timeout). The two-ramp scene is the same geometry the
headless tests use (15° holds, 35° slides). Watching it for ~15 s is left to
the user.

### T021 — FR-007 gate, in-repo (friction only, no cache, slop 0.01)

`cargo test --release --test stacking tower -- --nocapture`, 10 boxes, 60 s:

| Iterations | sink @ 2 / 10 / 60 s | max drift | max tilt | result |
|---|---|---|---|---|
| 8 | 2.18 / 9.01 / 9.01 | 6.53 | 3.14 | FAIL (heap) |
| 12 | 1.43 / 9.01 / 9.01 | 7.48 | 3.13 | FAIL |
| 16 | 1.08 / 9.01 / 9.01 | 7.81 | 3.11 | FAIL |
| 20 | 0.87 / 9.01 / 9.01 | 8.28 | 3.13 | FAIL |
| 30 | 0.57 / 9.01 / 9.01 | 8.71 | 3.14 | FAIL |

The tower collapses at every iteration count (sink ≈ 9.0 = fallen; tilt ≈ π).
Iteration tuning alone, even with friction, does not hold it, so FR-007's
gate is met on the shipped code and warm-starting proceeds.

### T031 — warm-starting lands (slop 0.01, 8 iterations)

With the cache the real implementation reproduces the prototype: the 10-box
tower stands (max tilt 0.011, max drift 0.07, sink 0.096 settled). The
25.2° hold case that crept 0.038 m without it now moves 0.00000 (un-ignored,
passes), confirming the closing-out hypothesis from T018.

## Tuning results

### Sweep: iterations × slop (T036)

`cargo test --release --test stacking -- --nocapture`, friction + warm-start,
tangent-then-normal. "Others" = heavy-on-light, mixed, pyramid (all pass in
every row). Tall = 15-box tower. Peak sink is the largest top-box compression
at any time.

| It. | Slop | Peak sink (t) | Sink @ 2 / 10 / 60 s | Max drift | Speed > 0.01 last at | Tall | Tower test |
|---|---|---|---|---|---|---|---|
| 8 | 0.002 | 9.06 (33.9 s, collapsed) | 0.067 / 0.024 / 9.00 | 9.94 | 34.3 s | FAIL | FAIL (collapse) |
| 8 | 0.003 | 0.153 (0.18 s) | 0.072 / 0.033 / 0.026 | 0.071 | 41.2 s | FAIL | FAIL |
| 8 | 0.005 | 0.153 (0.18 s) | 0.082 / 0.052 / 0.046 | 0.071 | 41.2 s | FAIL | FAIL |
| 10 | 0.002 | 0.151 (0.18 s) | 0.067 / 0.024 / 0.017 | 0.049 | 29.8 s | FAIL | FAIL |
| 10 | 0.003 | 0.151 (0.18 s) | 0.072 / 0.034 / 0.027 | 0.048 | 29.7 s | FAIL | FAIL |
| 10 | 0.005 | 0.152 (0.18 s) | 0.082 / 0.053 / 0.047 | 0.048 | 29.7 s | ok | FAIL |
| 12 | 0.002 | 0.150 (0.17 s) | 0.067 / 0.024 / 0.018 | 0.036 | 22.3 s | FAIL | FAIL |
| 12 | 0.003 | 0.150 (0.17 s) | 0.072 / 0.034 / 0.028 | 0.036 | 22.3 s | ok | FAIL |
| 12 | 0.005 | 0.150 (0.18 s) | 0.082 / 0.053 / 0.048 | 0.036 | 22.3 s | ok | FAIL |
| **16** | **0.002** | 0.150 (0.17 s) | 0.067 / **0.024** / 0.018 | **0.022** | **15.1 s** | ok | FAIL |
| 16 | 0.003 | 0.150 (0.17 s) | 0.072 / 0.034 / 0.028 | 0.022 | 15.1 s | ok | FAIL |
| 16 | 0.005 | 0.150 (0.17 s) | 0.082 / 0.053 / 0.048 | 0.022 | 15.1 s | ok | FAIL |

Reading it:

- **Slop sets settled compression** (≈ slop × interfaces): only 0.002 keeps
  the 10 s value (0.024) within the spec's 3%; 0.003 gives 3.4%.
- **Iterations set drift and settling time**: 8 is unstable at slop 0.002 and
  slow to settle at any slop; 10 is marginal (the 15-box tower fails at
  0.002–0.003); 12 stands but has only ~28% drift margin (0.036 vs 0.05); 16
  has ~56% margin and settles by 15 s.
- **Chosen: 16 iterations, slop 0.002, `CORRECTION_PERCENT` 0.4 unchanged.**
- For that row, every bound passes **except the start-up transient** below:
  speed never exceeds 0.1 m/s after 2 s or 0.01 m/s after 30 s; compression is
  0.085 at 1 s, 0.067 at 2 s, 0.024 at 10 s, 0.018 at 60 s (falling, so no
  creep); drift 0.022; tilt 0.004.

### Start-up transient: the one bound tuning cannot meet

Peak compression is ≈ 0.150 at t ≈ 0.17 s in **every** configuration
(0.1499–0.1528), and stays above 0.10 until t ≈ 0.5 s (0.121 at 0.25 s, 0.102
at 0.5 s, 0.085 at 1 s). It does not depend on iterations or slop.

Why: the tower spawns exactly touching, gravity kicks every box at once, and a
contact is only reported once penetration exceeds the narrowphase threshold
(1e-4). Step 1 lets everything fall together (0.0027); only the floor
interface penetrates. Each interface above is detected one step after the box
below it has been stopped, having accumulated one more step of relative
motion. Interface *i* therefore arrives with penetration ≈ i·g·dt², and the
top box's total compression at peak is ≈ g·dt²·n(n+1)/2 = 0.0027 × 55 ≈ 0.149
for n = 10 — matching the measured 0.150. The solver then removes it (40% of
the excess per step), which is why it is gone by ~1 s.

That is a property of zero-margin contact at 60 Hz with an exactly-touching
spawn, not of the solver's tuning. It would only shrink with something
outside M4's stated levers (contacts that exist *before* penetration —
speculative margin — or a pre-settled spawn). SC-001 as amended ("≤ 10% at
all times") therefore cannot hold in its first ~0.5 s. Per the Stop rule this
was taken to `/speckit-clarify` rather than loosened in the test. **Decision
(user, 2026-09-20): amend SC-001** with a start-up window — ≤ 20% before 1 s
(measured peak 0.150), ≤ 10% from 1 s (0.085 measured), ≤ 3% from 10 s,
creep ≤ 0.1%. Alternatives declined: a contact margin (beyond M4's levers)
and a pre-settled spawn (contradicts "just touching").

### Step order A/B (T037), it = 16, slop 0.002

| Order | Tower: last speed > 0.01 | Tower: max drift | Heavy-on-light | Notes |
|---|---|---|---|---|
| **tangent → normal** (kept) | 15.1 s | 0.022 | ok | |
| normal → tangent | 12.5 s | 0.019 | **FAIL** | slightly calmer tower, but breaks the 1000:1 case |

Tangent-then-normal is kept: the 1000:1 mass-ratio stack is an explicit edge
case, and normal-first loses it. (Normal-first would make the final
`|jt| ≤ μ·j` exact rather than within ~0.04%; not worth the regression.)

### Stress tests at the chosen constants

- Pyramid (5 rows, 30 s): max drift 0.0004–0.0011, max sink 0.041, tilt
  0.0005 — passes with wide margin.
- Mixed boxes and circles: at rest (speed 0, per-step motion 0) by 15 s.
- 15-box tower: stands but sways slowly (top drifts up to ~1 m at ≤ 5° tilt
  with a ~10 s period at slop 0.01); worst speed after 1 s 0.12–0.33 m/s.
  "Soft under load", as the README says, not unstable.
- 1000:1 stack: finite and bounded, but the light box is squeezed out and the
  heavy box ends on the floor (y = 0.494). Stable, not stacked.

### Implementation close-out (2026-09-20)

- **Amendment applied**: SC-001 start-up window (≤ 20% before 1 s, ≤ 10% from
  1 s) — see § Start-up transient. At the chosen constants (16 iterations,
  slop 0.002) `tower_stands_for_sixty_seconds` has zero violations: peak
  0.150 (t = 0.17 s), 0.085 at 1 s, 0.024 at 10 s, 0.018 at 60 s, drift 0.022,
  tilt 0.004, speed ≤ 0.01 from 15.1 s.
- **Frame-time independence**: with a mix of 144/75/30/60 Hz frames for 60 s
  the tower ends at sink 0.018, drift 0.008, speed 0
  (`tower_is_stable_under_variable_frame_times`).
- **Determinism**: two 60 s tower runs are bit-identical.
- **Suite**: 97 tests pass in debug and release (83 unit, 7 friction, 7
  stacking).
- **M3 regression review (T045)**: every line removed from `src/` since
  `30318b7` is production refactoring or a mechanical test edit; no M3
  assertion or tolerance changed.
- **Constitution checks (T049)**: zero dependencies, no `unsafe`, no
  `HashMap`/`thread_local`, no rendering code in `src/`, no non-goal
  machinery.
- **Clippy**: three warnings remain, all in M1–M3 code this milestone did not
  touch (`broadphase.rs:15`, `collision/mod.rs:51`, `examples/bouncing.rs:88`);
  the one in `ramp.rs` was fixed.
- **Not verified by a person watching**: `ramp`, `stack`, `pyramid` and
  `bouncing` all build cleanly and launch without panicking, and every
  behavior they show is covered by a passing headless test, but the
  Constitution IV "watch it with `--release`" check is left to the user.
  Until then the README's M4 checkbox and "MVP is complete" line are not
  applied.

### Visual confirmation (2026-09-20)

The user watched `ramp`, `stack`, `pyramid` and `bouncing` with `--release`
and reported they look good. That closes T019, T041, T043 and T046, and the
Constitution IV check for M4's "done when". The README's M4 checkbox and
"MVP complete" status are now applied.
