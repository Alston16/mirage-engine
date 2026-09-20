# Quickstart: verifying M3

## 1. Unit tests

```
cargo test
```

Expect passing tests covering, at minimum:

| Behavior | Spec ref |
|---|---|
| Ball on static floor, `e = 0`: normal velocity zero after landing, no rebound | US1-1, SC-002 |
| Ball rests 10 s: penetration ≤ tolerance, no frame-to-frame drift | US1-2, US3-2, SC-004 |
| Two dynamic bodies head-on: momentum along `n` conserved, lighter body changes more | US1-3, SC-005 |
| Separating contact: velocities unchanged | US1-4 |
| Static body: position/velocity bit-identical after every step | US1-5, SC-007 |
| `e = 1` drop: first rebound within 5% of drop height | US2-1, SC-001 |
| `e = 0.5` drop: rebound speed ≈ `0.5 ×` impact speed (±5%) | US2-3, SC-003 |
| Restitution combine is order-independent | US2-4 |
| Small overlap shrinks, no pop; overlap ≤ slop untouched | US3-1, US3-3 |
| Off-center polygon hit changes angular velocity correctly | Edge cases |
| Two-point polygon manifold: box lands flat without tipping | Edge cases |
| Same scene twice → bit-identical state | SC-006 |
| Every shape-pair ordering yields an `a → b` normal | Research finding |

## 2. Behavioral check (required by Constitution IV)

```
cargo run --example bouncing --release
```

Observe, with balls of increasing restitution dropped onto the floor:

- No ball falls through the floor (SC-008).
- The `e = 0` ball stops dead on landing.
- The `e = 1` ball returns to within a few percent of its drop height on
  the first bounce.
- Intermediate balls rebound proportionally lower.
- Nothing jitters while resting.

This — not `cargo test` alone — satisfies M3's "done when" criterion.

## 3. Expected non-behavior

`cargo run --example ramp --release`: the box now reacts to the ramp but
**slides** down it, because friction is M4. This is expected, not a bug.
A 10-box tower (M4's `stack`) is not expected to hold yet.
