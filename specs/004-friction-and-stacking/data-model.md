# Data Model: Friction & Stable Stacking (M4)

## `RigidBody` (CHANGED — `src/body.rs`)

| Field | Type | Meaning | Validation |
|---|---|---|---|
| `friction` | `f32` | Coulomb friction coefficient `μ` of this body's surface | `with_friction` clamps to `≥ 0.0`; default `0.5` for both dynamic and static bodies |

All other fields unchanged. A body with `friction == 0.0` produces a pair
coefficient of `0.0` against anything (see below), reproducing M3 behavior
(FR-011).

## Combined friction (derived, not stored)

`μ = √(friction_a · friction_b)` per contact. Commutative, so it is identical
for `(a, b)` and `(b, a)`; `0` if either body is `0`; equals the shared value
when both bodies match.

## `Contact` (CHANGED — `src/collision/manifold.rs`)

| Field | Type | Meaning |
|---|---|---|
| `point` | `Vec2` | unchanged |
| `normal` | `Vec2` | unchanged; always `a → b` |
| `penetration` | `f32` | unchanged |
| `feature` | `u32` | NEW. Identifies which geometric feature produced the point, stable while the same feature stays in contact. Circle contacts: `0`. Polygon contacts: packs flip flag, reference-face index, incident-face index and clipped-endpoint index. Used only as part of the warm-start cache key. |

`Manifold` is unchanged in shape.

## Solver-internal contact state (`src/solver.rs`, crate-private)

Built once per substep from each `Manifold`/`Contact`, discarded afterwards.
Fields added by M4 are marked ◆.

| Field | Meaning |
|---|---|
| `a`, `b` | Indices into the body slice |
| `n` | Contact normal, unit, `a → b` |
| ◆ `t` | Tangent `(−n_y, n_x)` |
| `r_a`, `r_b` | `point − position` for each body |
| `k` | `K = 1/m_a + 1/m_b + (r_a×n)²/I_a + (r_b×n)²/I_b` |
| ◆ `k_t` | `K_t`, the same with `t` in place of `n` |
| ◆ `mu` | Combined `μ` for the pair |
| `bounce` | Target normal velocity `−e·(vr·n)₀`, computed **before** cache seeding |
| `j_acc` | Accumulated normal impulse; invariant `j_acc ≥ 0`. ◆ Seeded from the cache. |
| ◆ `jt_acc` | Accumulated tangent impulse; clamped to `|jt_acc| ≤ mu · j_acc` when applied. Because the normal solve follows in the same visit, the final value can exceed that by a fraction of a percent (0.04% observed). Seeded from the cache. |
| ◆ `j_seed`, `jt_seed` | The impulses the contact was seeded with (`0` if new); read only by tests |
| ◆ `feature` | Copied from the `Contact`; cache key part |
| `penetration`, `share` | Unchanged |

## Impulse cache (◆ NEW — `src/solver.rs`, owned by `World`)

```text
struct CachedImpulse { body_a: u32, body_b: u32, feature: u32, j: f32, jt: f32 }
cache: Vec<CachedImpulse>      // order = manifold order × point order of the last step
```

| Property | Rule |
|---|---|
| Lookup | Ordered linear scan for `(body_a, body_b, feature)`; first match wins |
| Seeding | Hit → `j_acc = j`, `jt_acc = jt`, apply `P = j·n + jt·t` (`−P` to `a`, `+P` to `b`) |
| Miss | `j_acc = jt_acc = 0` (new contact) |
| Replacement | After the iterations, the cache is **replaced** with this step's `(a, b, feature, j_acc, jt_acc)`; unmatched old entries vanish (FR-008) |
| Determinism | `Vec`, ordered scan, no hashing — same inputs, same iteration order, same result |
| Static pairs | Dropped with the contact when `K == 0`, so never cached |

## Constants (`src/solver.rs`)

| Name | M3 | M4 final | Purpose |
|---|---|---|---|
| `VELOCITY_ITERATIONS` | `8` | **`16`** (tuned; research.md § Tuning results) | Solver passes per step |
| `PENETRATION_SLOP` | `0.01` | **`0.002`** (tuned) | Penetration tolerated before correction; sets steady-state stack compression |
| `CORRECTION_PERCENT` | `0.4` | `0.4` | Fraction of excess penetration removed per step |
| `MIN_BOUNCE_SPEED` | `1e-4` | `1e-4` | Float-noise floor for restitution |

## State transitions per fixed step

```text
v ← v + g·dt
manifolds ← detect_contacts(bodies)                     # contacts carry `feature`
contact state ← build(manifolds)                        # r, K, K_t, t, μ, bounce
seed from cache; apply P = j·n + jt·t                   # warm start
N × { for each contact:
        Δjt = −vt/K_t ; jt_acc ← clamp(jt_acc+Δjt, ±μ·j_acc) ; apply Δjt·t
        Δj  = −(vn − bounce)/K ; j_acc ← max(j_acc+Δj, 0)     ; apply Δj·n }
cache ← this step's (a, b, feature, j_acc, jt_acc)
positions ← project(manifolds)                          # slop-gated
x ← x + v·dt ; θ ← θ + ω·dt
```
