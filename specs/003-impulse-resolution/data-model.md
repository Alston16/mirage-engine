# Data Model: Impulse Resolution (M3)

## `RigidBody` (CHANGED — `src/body.rs`)

| Field | Type | Meaning | Validation |
|---|---|---|---|
| `restitution` | `f32` | Bounciness `e` of this body's material | Clamped to `[0.0, 1.0]` by `with_restitution`; default `0.0` |
| `inv_inertia` | `f32` | `1 / I` about the body's origin; `0.0` for static bodies | Derived from `Shape::inertia(mass)`; `> 0` for dynamic bodies |

Existing fields (`position`, `velocity`, `orientation`, `angular_velocity`,
`inv_mass`, `is_static`, `shape`) are unchanged. A static body has
`inv_mass == 0.0` and `inv_inertia == 0.0`, so every impulse term for it
vanishes and no special-casing beyond skipping the both-static pair is
needed.

## Combined restitution (derived, not stored)

`e = max(a.restitution, b.restitution)` per contact; symmetric in `a`/`b`.

## Solver-internal contact state (`src/solver.rs`, crate-private)

Built once per step from each `Manifold`/`Contact`, discarded afterwards.
No state persists between steps (no warm-starting until M4).

| Field | Meaning |
|---|---|
| `a`, `b` | Indices into the body slice |
| `n` | Contact normal, unit, `a → b` |
| `r_a`, `r_b` | `point − position` for each body |
| `k` | Effective mass denominator `K = 1/m_a + 1/m_b + (r_a×n)²/I_a + (r_b×n)²/I_b` |
| `bounce` | Target normal velocity `−e·vn₀`, where `vn₀` is the approach velocity before this step's gravity kick; `0` if not approaching |
| `j_acc` | Accumulated normal impulse; invariant `j_acc ≥ 0` |
| `penetration` | Depth from the manifold, used by position correction |
| `share` | `1 / points in the source manifold`; scales position correction so a two-point manifold is not corrected twice as hard as a one-point one |

Contacts where `k == 0` (both bodies static) are dropped when building this
state (FR-007).

## Constants (`src/solver.rs`)

| Name | Value | Purpose |
|---|---|---|
| `VELOCITY_ITERATIONS` | `8` | README § Resolution |
| `PENETRATION_SLOP` | `0.01` | Penetration tolerated before correction |
| `CORRECTION_PERCENT` | `0.4` | Fraction of excess penetration removed per step, split across a manifold's points |

## `Manifold` / `Contact` (M2, consumed)

Unchanged in shape. Invariant strengthened: `normal` **always** points from
`body_a` toward `body_b`, for every shape-pair ordering.

## State transitions per fixed step

```text
v ← v + g·dt
manifolds ← detect_contacts(bodies)
contact state ← build(manifolds)              # r, K, bounce
8 × { for each contact: Δj = −(vn − bounce)/K ; j_acc ← max(j_acc+Δj, 0) ; apply }
positions ← project(manifolds)                # slop-gated
x ← x + v·dt ; θ ← θ + ω·dt
```
