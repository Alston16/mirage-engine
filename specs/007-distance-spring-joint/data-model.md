# Data Model: Distance / Spring Joint

**Feature**: [spec.md](spec.md) | **Plan**: [plan.md](plan.md) | **Research**: [research.md](research.md)

## Public entities

### `DistanceJoint` (new, `src/joint/distance.rs`)

| Field | Type | Meaning |
|---|---|---|
| `body_a` | `BodyId` | First connected body (may be static). |
| `body_b` | `BodyId` | Second connected body (may be static; must differ from `body_a`). |
| `anchor_a` | `Vec2` | Anchor in `body_a`'s local frame. |
| `anchor_b` | `Vec2` | Anchor in `body_b`'s local frame. |
| `length` | `Option<f32>` | Target / rest length `L0`. `None` until added; `World::add_distance_joint` replaces it with `Some(current anchor separation)`. |
| `spring` | `Option<Spring>` | `None` = rigid mode; `Some` = soft mode. |

**Validation** (enforced by `World::add_distance_joint`, which panics on
violation; research § 6):
- `body_a != body_b`, and both are in the world (the same asserts as `add_joint`).
- `length`, if `Some`, is finite and `≥ 0`.
- `spring.stiffness` and `spring.damping`, if present, are finite and `≥ 0`.

**Lifecycle**: Built, then added once, then permanent. There is no removal
API (spec Assumptions). After it is added, the stored copy is immutable
from outside and can be read through `World::distance_joint(id)`.

### `Spring` (new, `src/joint/distance.rs`)

| Field | Type | Meaning |
|---|---|---|
| `stiffness` | `f32` | `k`, N/m: force per metre of deviation from `L0`. |
| `damping` | `f32` | `c`, N·s/m: force per m/s of separation rate `L̇`. |

`DistanceJoint::critical_damping(stiffness, mass) = 2·√(stiffness · mass)`
gives `c` for `ζ = 1` when a single body of `mass` moves along the joint
line (FR-005).

### `DistanceJointId` (new)

Opaque `u32` index into `World::distance_joints`. It is constructed only
by `World::add_distance_joint`, and it is a separate type from `JointId`
so the two lists can't be cross-indexed.

### Existing entities, unchanged

- `RevoluteJoint`, `JointId`: no field or signature changes.
- `RigidBody`: no new fields. Spring forces are written straight into
  `velocity` / `angular_velocity` as impulses (research § 2), so there is
  no force accumulator.
- `World`: gains `distance_joints: Vec<DistanceJoint>`.

## Internal (per-step) entities

### `DistanceState` (`pub(crate)`)

This is rebuilt each fixed step from `DistanceJoint` plus body state, then
discarded. It is the distance-joint counterpart of `JointState` /
`ContactState`.

| Field | Meaning |
|---|---|
| `a`, `b` | Body indices. |
| `r_a`, `r_b` | World-space centre-to-anchor offsets (`R · anchor`), fixed for the step. |
| `n` | Unit vector `(p_b − p_a) / L`, fixed for the step. |
| `c` | Constraint value `C = L − L0` at step start. It feeds the spring force. Rigid positional correction does **not** use it: it recomputes `L` from current positions, because earlier corrections may have moved the bodies. |
| `length` | Resolved `L0`, needed for that recomputation. |
| `k_inv` | `1 / K`, `K = 1/m_a + 1/m_b + (r_a×n)²/I_a + (r_b×n)²/I_b`. |
| `spring` | Copy of the joint's `Option<Spring>` (selects the phase that uses it). |

**Built only when** at least one body is dynamic and `L ≥ DISTANCE_EPSILON`
(1e-6). Otherwise the joint is skipped for this step (research § 6).

## Per-step state transitions

```text
start of step
  │  spring joints:  λ = −(k·C + c·Cdot)·dt;  P = λ·n;  −P → a, +P → b
  ▼
gravity:  v += g·dt
  ▼
contacts + revolute + rigid distance built
  ▼
16 × [ contacts → revolute → rigid distance: λ = −Cdot·k_inv; P = λ·n ]
  ▼
correct positions: contacts → revolute → rigid distance (recomputes L, n, C from current positions)
  ▼
integrate: x += v·dt, θ += ω·dt
```

Spring-mode joints never enter the iteration loop or positional correction.
Rigid-mode joints never receive the spring impulse.
