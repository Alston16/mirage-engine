# Research: Distance / Spring Joint

**Feature**: [spec.md](spec.md) | **Plan**: [plan.md](plan.md)

Phase 0 output. Each subsection resolves one open technical question from the
plan's Technical Context, in the Decision / Rationale / Alternatives format.
Numbers quoted as "measured" come from a 1D scratch simulation of the
engine's exact integration order (semi-implicit Euler, `dt = 1/60`) for a
1 kg box on a `k = 10 N/m`, `L0 = 2 m` spring under `g = 9.81`, released
1 m beyond equilibrium. They are design evidence only. The real acceptance
numbers come from `tests/distance_joints.rs` once the code exists.

## 1. Soft mode: explicit spring force vs. softened constraint

**Decision**: Apply soft mode as an **explicit Hookean force** along the
anchor-to-anchor line, `F = −(k·(L − L0) + c·L̇)`, turned into an impulse
`F·dt` once per fixed step. It does not take part in the velocity
iterations.

**Rationale**:
- The README names it literally: "soft (Hookean spring force with
  damping)". Principle III says the code should read like that derivation.
- M7's second done-when criterion is that an **undamped** spring conserves
  energy. Semi-implicit Euler is symplectic, so an explicit undamped spring
  has no secular energy drift, only bounded `O(ω·dt)` wobble.
  Measured: instantaneous energy stays within **3.0%** of the oscillation
  energy `½·k·A²`, and drift between the first and last period over 60 s
  is **0.002%**.
- With `c = 2√(k·m)` the discrete system stays non-oscillatory. For the
  step map `[x, v]`, the eigenvalue discriminant at `ζ = 1` is
  `4a³ + a⁴ > 0` with `a = ω·dt`, so both eigenvalues are real and
  positive while `a < 0.5`. Measured overshoot past equilibrium is
  **0.0** (to f32 precision).
- Measured under-damped period (`ζ = 0.1`) is **1.990 s**. The analytic
  `2π/ω_d` is 1.997 s, so the difference is **0.3%**.

**Alternatives considered**:
- *Soft constraint* (Box2D v2.4 / v3 style). The spring becomes a
  constraint with `γ`/`β` softness terms derived from frequency and damping
  ratio, and is solved inside the velocity iterations. Its upside is
  unconditional stability for arbitrarily stiff springs. Rejected because
  it is effectively implicit Euler on the spring, which **dissipates
  energy** even at `ζ = 0`. That fails M7's "undamped spring conserves
  energy" criterion by construction. It also parameterizes by
  frequency/damping-ratio, which the spec's Assumptions rejected in favour
  of physical `k`/`c`.
- *Implicit damping, explicit stiffness* (solve only the `c·L̇` term
  implicitly). Its upside is unconditional stability for large `c`.
  Rejected: plain explicit damping already gives zero overshoot at
  critical damping for the demo's parameters, so the extra algebra buys
  nothing M7 needs. Revisit only if a test shows instability.

**Known limitation (documented, not guarded)**: an explicit spring is only
stable while `ω·dt < 2`, where `ω = √(k/m_eff)`, and explicit damping only
while `c·dt/m_eff` stays well below 1. For `dt = 1/60` that means springs
slower than about 19 Hz. A user who wants "infinitely stiff" uses rigid
mode (spec Edge Cases).

## 2. When the spring force is evaluated within a step

**Decision**: Compute every spring impulse from the bodies' state at the
**start** of the fixed step, *before* gravity's `v += g·dt` kick. It is
the same "forces" phase as gravity: gravity and springs both read
start-of-step state, and both write velocity before collision detection.

**Rationale**: The scratch sim first evaluated the spring after gravity.
There the damper reads `L̇` that already contains this step's `g·dt` kick,
so at rest it sees a phantom velocity `g·dt` every step. It pushes back
with a constant `c·g·dt`, and the box settled **10.3 cm** away from
equilibrium (`c·g·dt / k` with the critical `c ≈ 6.32`). Evaluating
before gravity removed the offset completely (final error 0.0). This is
the same lesson the contact solver already encodes for restitution
(README § Resolution: "the contact step's own `g·dt` kick is subtracted
out"). Forces in one step must all see the same state.

**Alternatives considered**:
- *Subtract `g·dt` from `L̇`* (mirror the restitution fix). Rejected: it
  only works for the gravity component along `n` and is easy to get wrong
  for two dynamic bodies. Ordering the phases correctly is simpler and
  exact.
- *A general force accumulator on `RigidBody`* (`F` in README's
  `v += (F/m + g)·dt`). Rejected for M7: it adds a public field and a
  clear-every-step contract for a single caller. The ordering gives the
  same result.

## 3. Rigid mode formulation

**Decision**: Rigid mode is a **scalar** sequential-impulse equality
constraint along `n`, solved in the shared velocity-iteration loop after
the revolute joints:

```
d    = p_b − p_a,   L = |d|,   n = d / L      (fixed for the step)
C    = L − L0
Cdot = n · ((v_b + ω_b × r_b) − (v_a + ω_a × r_a))
K    = 1/m_a + 1/m_b + (r_a×n)²/I_a + (r_b×n)²/I_b
λ    = −Cdot / K                              (no clamp, equality constraint)
P    = λ · n,   applied −P to a, +P to b at r_a / r_b
```

**Rationale**: A distance constraint removes exactly one degree of freedom,
the separation, so unlike the revolute joint it has no 2×2 coupling to
respect (research for M6, § 1). `K` has exactly the contact normal's form
(README § Resolution), and `λ` is the contact's `Δj` without the `≥ 0`
clamp. That is what the README means by solving an equality constraint
"the same way a contact is solved as an inequality one". `n`, `r_a` and
`r_b` stay fixed through the step for the same reason `JointState::r_a`
does: orientation only integrates after resolution.

**Alternatives considered**: *Use the revolute joint's 2×2 solve with a
target offset*. Rejected because it over-constrains: it would also pin
the direction, which FR-006 forbids.

## 4. Rigid-mode positional correction

**Decision**: Mirror `joint::correct_positions`. After the velocity
iterations, move each body along `n` by
`JOINT_CORRECTION_PERCENT · (|C| − JOINT_SLOP) · sign(C) / (1/m_a + 1/m_b)`,
split by inverse mass. `L`, `n` and `C` are recomputed from current
positions, not taken from step start, exactly as the revolute joint
recomputes its gap. Reuse the revolute joint's existing `JOINT_SLOP`
(0.002) and `JOINT_CORRECTION_PERCENT` (0.4).

**Rationale**: FR-007 asks for "the same style as the revolute joint".
The revolute constants were tuned for a 1 m-scale pendulum, the same
scale as M7's scenes, and M6 found they needed no change from the contact
values. A third pair of identical constants would be noise. If measurement
shows rigid distance needs different values, split them then.

**Alternatives considered**: *Velocity-bias (Baumgarte in the velocity
solve)*. Rejected for the same reason M6 and the contact solver rejected
it: it injects energy into the swing.

## 5. Where the code lives, and the public API shape

**Decision**:
- New module `src/joint/distance.rs` as a submodule of the existing
  `src/joint.rs`. Rust 2018+ allows `joint.rs` plus a `joint/` directory.
  `RevoluteJoint` code stays untouched in `joint.rs`, and M8's motor (a
  revolute variant) can land beside it later.
- A separate public type `DistanceJoint` with its own handle
  `DistanceJointId`, stored in its own `World::distance_joints` vec and
  added through `World::add_distance_joint` / read through
  `World::distance_joint`.
- `DistanceJoint` follows `RigidBody`'s builder style: `DistanceJoint::new(a,
  b, anchor_a, anchor_b)` is rigid with the length taken at add time,
  `.with_length(L0)` sets the target length, and `.with_spring(k, c)`
  switches to soft mode. The public fields mirror `RevoluteJoint`, plus
  `length: Option<f32>` and `spring: Option<Spring>`.
- `World::add_distance_joint` resolves `length: None` into
  `Some(current anchor separation)` (FR-002), so `distance_joint(id)`
  always reports the length actually in use.
- `DistanceJoint::critical_damping(stiffness, mass) -> f32` returns
  `2·√(k·m)`, which is FR-005's "documented, no reverse-engineering".

**Rationale**: This is purely additive. `add_joint(RevoluteJoint)`,
`joint(id) -> &RevoluteJoint` and `JointId` keep their exact signatures,
so FR-011 (no behaviour change) extends to API compatibility. A separate
id type stops a `JointId` from one list indexing the other.

**Alternatives considered**:
- *A `Joint` enum (`Revolute | Distance`) behind `add_joint`*. That
  changes `joint(id)`'s return type, a breaking change for one extra
  variant. Revisit when M8 lands, if three joint kinds make a common enum
  worth it.
- *Two types, `DistanceJoint` and `SpringJoint`*. Rejected: the README
  describes one joint "rigid or soft". A single type keeps anchors and
  length handling in one place.
- *Moving `joint.rs` to `joint/mod.rs` + `joint/revolute.rs`*. Rejected
  because it churns M6's freshly merged code for no behavioural gain.

## 6. Degenerate configurations

**Decision**:
- **Separation below `DISTANCE_EPSILON` (1e-6)**: the joint is skipped for
  that step in both modes (no `n` exists). It is never normalized, so no
  NaN can appear.
- **Both bodies static**: dropped from the step's working set, exactly as
  `joint::build_joints` drops a two-static revolute joint.
- **Invalid parameters**: `World::add_distance_joint` panics on a
  negative or non-finite length, stiffness or damping, alongside the
  existing "cannot connect a body to itself" / unknown-body asserts. A
  resolved length of `0` is allowed (spec Edge Cases). Rigid mode at
  `L0 = 0` degenerates to a revolute-like pull with an undefined
  direction at the target; the docs point users to `RevoluteJoint` for
  that.

**Rationale**: This matches the existing precondition style (panic at
add time for programmer error) and the existing "drop what can't move"
rule. FR-014 requires no non-finite state.

## 7. Warm-starting

**Decision**: Neither mode warm-starts. Spring mode isn't iterated at
all, and rigid mode cold-starts like the revolute joint.

**Rationale**: Same argument as M6 research § 5. `K` depends only on
geometry, and a single-DOF constraint converges within the shared 16
iterations. Revisit only if a rigid-chain test shows a startup transient.

## 8. Acceptance tolerances (making SC-001 to SC-004 concrete)

| Criterion | Scene | Measured quantity | Bound |
|---|---|---|---|
| SC-001 | 1 kg box below static anchor, `k = 10`, `c = critical_damping(k, 1)`, released 1 m past equilibrium | max overshoot past equilibrium; error after 10 s | ≤ 1% of 1 m; ≤ 1% thereafter |
| SC-002 | same, `c = 0`, 60 s | total mechanical energy (½mv² + ½Iω² + m·g·y + ½k(L−L0)²) relative to `½·k·A²` | mean over last period within 1% of mean over first; instantaneous within 5% |
| SC-003 | same, `ζ = 0.1` | period from successive upward zero crossings | within 2% of `2π / (ω·√(1−ζ²))` |
| SC-004 | 0.1 m disc on rigid `L = 2` from static anchor at its center, released at 0.1 rad | `|L − L0|` every step; period | ≤ `ANCHOR_TOL` (0.02, same as `tests/joints.rs`); within 2% of `2π√(L/g)` |

Energy is normalized by the oscillation energy, not the total, because
the large `m·g·y` offset would otherwise hide a real drift. The 5%
instantaneous bound leaves headroom over the measured 3.0%. Anchoring the
disc at its own center means the rigid joint applies no torque, so the bob
doesn't spin and the simple-pendulum formula is exact.

## 9. Known trade-off: restitution and spring kicks

A contact's restitution target subtracts the step's `g·dt` kick from the
approach velocity (README § Resolution). It does **not** subtract a
spring's kick from the same step, so a body pressed into a bouncy (`e > 0`)
surface by a spring sees that kick as approach velocity and bounces
slightly more than it should. With the default `e = 0` this has no effect,
and no M7 scene combines springs with restitution. It is documented in the
README beside the other solver trade-offs, not fixed here.
