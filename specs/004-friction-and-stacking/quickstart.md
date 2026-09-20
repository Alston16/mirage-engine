# Quickstart: verifying M4

## 0. Gate — M3 is actually done (Constitution IV)

Before any M4 code:

```
cargo test                              # baseline: 69 passed
cargo run --example bouncing --release  # e = 1 returns to ~drop height; e = 0 stops dead
```

Also confirm whether M3's PR has merged; this branch was cut from
`003-impulse-resolution`, so open M4's PR against that branch if it has not.

## 1. Unit + behavioral tests

```
cargo test
cargo test --release stacking -- --nocapture   # prints the tower/pyramid metrics
```

Expect passing tests covering, at minimum:

| Behavior | Spec ref |
|---|---|
| Box on 15° ramp (μ = 0.5, below `atan μ = 26.6°`): moves < 1% of its width in 10 s, speed < 0.01 | US1-1, FR-009, SC-003 |
| Box on 35° ramp: slides with `a = g(sin θ − μ cos θ)` within 5% | US1-2, FR-010, SC-004 |
| Higher-μ setup slides less than lower-μ setup | US1-3 |
| `friction = 0` on the box reproduces frictionless `a = g sin θ`; M3 tests unchanged | US1-4, FR-011, US4 |
| Solid disc on a ramp rolls: `a = (2/3) g sin θ` within 5% | US1-5 |
| `combine_friction(a, b) == combine_friction(b, a)`; `0` if either is `0` | FR-002 |
| Accumulated `|jt| ≤ μ·j` and `j ≥ 0` after every solve | FR-001, FR-005 |
| 10-box tower, 60 s: drift ≤ 5%; sink ≤ 10% always, ≤ 3% from 10 s, creep ≤ 0.1%; speed < 0.1 from 2 s and < 0.01 from 30 s; does not topple | US2-1..3, SC-001, SC-002 |
| Tower, same scene twice: bit-identical state | US2-5, SC-007, FR-013 |
| Pyramid (5 rows), 30 s: drift ≤ 0.1, sink ≤ 0.1, tilt ≤ 0.05 rad, speed ≤ 0.05 after 10 s | US3-1, SC-005 |
| 15-box tower: values stay finite, speed stays bounded (may be soft, must not explode) | Edge cases |
| 1000:1 mass ratio stack: finite and bounded | Edge cases |
| Threshold within 5% of `atan μ`: holds at 0.95·θc, slides at 1.05·θc | FR-009 |
| Mixed boxes and circles landing on a floor: speed < 0.02, per-step motion < 0.001 over the last 5 s | US3-2 |
| Cache: matched contact is seeded; vanished contact is forgotten; unmatched new contact starts at 0 | FR-008 |
| Contact `feature` is stable for a resting box and differs between its two points | FR-008 |
| Existing 69 tests (restitution, correction, determinism, …) | US4-3, FR-012, SC-006 |

## 2. Behavioral checks (required by Constitution IV)

All with `--release`:

```
cargo run --example ramp --release      # shallow-ramp box stays put; steep-ramp box slides
cargo run --example stack --release     # 10-box tower, watch a full 60 s
cargo run --example pyramid --release   # 15 boxes settle and stay
cargo run --example bouncing --release  # regression: M3 behavior unchanged
```

Observe:

- **ramp**: left (15°) box holds; right (35°) box slides off. Contact debug
  drawing still works.
- **stack**: tower stands; no visible jitter, creep or sinking over 60 s
  (SC-009). HUD sink/drift values stay within the bounds in the spec once the
  initial settle is over.
- **pyramid**: rows stay in place.
- **bouncing**: `e = 0` ball stops dead; `e = 1` ball returns near its drop
  height.

This — not `cargo test` alone — satisfies M4's "done when" criterion and the
MVP's.

## 3. Tuning record

The chosen `VELOCITY_ITERATIONS`, `PENETRATION_SLOP` and step order, with the
sweep data behind them, are written into research.md and the README §
Resolution when tuning is finished. If the (re-based) SC-001/SC-002 numbers or the pyramid bounds cannot be met,
stop and take the evidence to `/speckit-clarify` rather than loosening tests
in place.

## 4. Expected non-behavior

- Very tall stacks (well beyond 10) stay slightly soft — the README's stated
  trade-off for sequential impulses.
- Fast bodies still tunnel (no CCD) and resting bodies never sleep: both are
  non-goals.
