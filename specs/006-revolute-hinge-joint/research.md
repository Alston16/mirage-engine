# Research: Revolute (Hinge) Joint

**Feature**: [spec.md](spec.md) | **Plan**: [plan.md](plan.md)

Phase 0 output. Each subsection resolves one open technical question from the
plan's Technical Context, in the Decision / Rationale / Alternatives format.

## 1. Constraint formulation for the point constraint

**Decision**: Solve the 2-body point constraint as a single coupled 2×2
linear system per velocity iteration — the classic Box2D-Lite `Joint`
formulation — rather than decomposing it into two independent scalar solves
(one for each world axis, the way contacts decompose into a normal solve and
a tangent solve).

```
C = p_b - p_a                     (p = body position + R * local anchor)
Cdot = (v_b + ω_b × r_b) - (v_a + ω_a × r_a)

K = (invMass_a + invMass_b) * I2
  + invI_a * [[ r_a.y², -r_a.x*r_a.y], [-r_a.x*r_a.y,  r_a.x²]]
  + invI_b * [[ r_b.y², -r_b.x*r_b.y], [-r_b.x*r_b.y,  r_b.x²]]

P = K⁻¹ * (-Cdot + bias)          (P is a 2D impulse vector, applied ± at r_a/r_b)
```

**Rationale**: A revolute joint constrains *both* world axes at once — unlike
a contact, which only ever pushes along one normal per point. Decomposing
into two independent scalar solves (solve x, then solve y, Gauss-Seidel
style) ignores the off-diagonal coupling term (`-r.x*r.y`) whenever a body's
anchor isn't at its center of mass, which is exactly the pendulum case (the
rod's pin is at one end, not its centroid). Erin Catto's GDC talks and
Box2D Lite — already cited in `README.md` § References — use the coupled
2×2 solve for this reason, and the README's Principle III (spec-faithful
math) points toward the same well-established derivation rather than an
approximation invented for this repo.

**Alternatives considered**:
- *Decomposed scalar (x then y) solve*: reuses the existing per-axis impulse
  pattern from `solver.rs` with no new math primitive. Rejected: measurably
  wrong for an off-center anchor (the pendulum's whole premise), and
  "readable over clever" (Principle III) argues for the textbook derivation,
  not a shortcut that happens to compile.
- *Soft constraint (spring-like) point joint*: avoids a matrix inverse by
  treating the joint as a very stiff spring. Rejected: M7 (distance/spring
  joint) already owns the soft-constraint case per README § Post-MVP
  milestones; M6 is explicitly the rigid equality constraint.

## 2. New math primitive: `Mat2`

**Decision**: Add a small `Mat2` (2×2 matrix) type to `math.rs`, alongside
`Vec2`/`Rot2`, with just what the joint solve needs: construction from four
scalars, `Mat2 * Vec2`, and `invert() -> Mat2` (via the determinant). Keep it
`pub(crate)` — like `solver::ContactState`/`ImpulseCache`, it's solver
plumbing, not part of the engine's public math API.

**Rationale**: `math.rs` is already the single home for "`Vec2`, `Rot2`,
cross products" per `README.md` § Architecture; a 2×2 matrix is the same
kind of primitive, not a new module boundary. Keeping it crate-private
matches how `ContactState` is internal today — consumers of `mirage` build
scenes and read back `RigidBody` state; they don't need to construct
matrices.

**Alternatives considered**:
- *Inline the 2×2 solve by hand inside `joint.rs`* (no reusable type, just
  four `f32`s and manual algebra at each call site). Rejected: less
  readable than naming the matrix, and M7/M8 (motor, spring) are likely to
  want a 2×2 or 1×1 effective-mass solve of their own — a tiny, well-named
  primitive now is cheaper than re-deriving the algebra per joint type.
- *Depend on a matrix/linear-algebra crate*. Rejected outright by
  Principle I (zero dependencies) — not a real option.

## 3. Where the joint's velocity-iteration loop lives

**Decision**: Keep every contact formula in `solver.rs` byte-for-byte
unchanged, but split its current single `resolve()` entry point into the
four phases it already implicitly has — `build_contacts`, `warm_start`,
`iterate_once` (one pass over all contacts), `correct_positions` — each
`pub(crate)`. `World::step` then owns one shared loop:

```
let mut contacts = solver::build_contacts(...);
let mut joints    = joint::build_joints(...);
solver::warm_start(&mut contacts, &cache);
for _ in 0..VELOCITY_ITERATIONS {
    solver::iterate_once(&mut bodies, &mut contacts);
    joint::iterate_once(&mut bodies, &mut joints);
}
solver::correct_positions(&mut bodies, &contacts);
joint::correct_positions(&mut bodies, &joints);
```

**Rationale**: `README.md` § Post-MVP milestones is explicit that a joint is
"added to `World::step`'s existing velocity-iteration loop alongside
contacts" and that this requires "no change to `solver.rs`'s contact path."
Read literally, "contact path" means the contact *math* (the friction/normal
impulse formulas, the clamps, the warm-start cache) — none of that changes.
What does move is *where the loop lives*: today it's a private detail inside
`solver::resolve()`; after M6 it's shared between contacts and joints so a
body that is both jointed and touching a contact (User Story 1, scenario 3)
gets both constraints solved together each iteration, not as two
independently-converged full passes. That coupling matters physically
(a joint solved to convergence, then a contact solved to convergence
afterward, does not equal the two solved together) even though the M6 demo
itself (an isolated pendulum) never exercises it.

**Alternatives considered**:
- *Two sequential full passes*: call `solver::resolve()` (all 16 iterations,
  unchanged) and then a brand-new `joint::resolve()` (its own 16 iterations)
  right after it in `World::step`, with zero changes to `solver.rs`'s public
  surface. Simpler and lower-risk, but does not deliver the interleaved
  convergence the README's architecture section describes, and would need
  revisiting the moment a joint+contact scene (already promised as a P1
  acceptance scenario) shows visibly wrong coupling. Rejected as
  short-sighted given the spec already commits to that scenario.

## 4. Positional (Baumgarte) correction for the joint

**Decision**: Mirror the contact solver's approach — correct position, not
velocity, after the velocity iterations — but drive it by the vector gap
`C = p_b - p_a` instead of a scalar penetration depth:

```
if |C| > JOINT_SLOP:
    correction = JOINT_CORRECTION_PERCENT * (|C| - JOINT_SLOP) / (invMass_a + invMass_b)
    shift = correction * normalize(C)
    body_a.position -= shift * invMass_a
    body_b.position += shift * invMass_b
```

with `JOINT_SLOP` and `JOINT_CORRECTION_PERCENT` as new constants in
`joint.rs`, tuned empirically against the pendulum demo the same way M4
tuned `PENETRATION_SLOP`/`CORRECTION_PERCENT` against the stacking demos
(see `solver.rs`'s doc comments and
`specs/004-friction-and-stacking/research.md`) — a starting point of
`percent = 0.4` mirrors the contact solver's tuned value, but the actual
number is a Phase 2/implementation task, not a planning decision.

**Rationale**: `README.md`'s own note on why contacts correct position
rather than bias velocity — "keeps the restitution solve energy-clean" —
applies just as well here: the joint has no restitution concept at all, so
biasing velocity would just add spurious energy into the pendulum's swing
with no equivalent benefit. Reusing the existing contact constants directly
would be wrong: a joint's "gap" isn't a compressive penetration between two
solid bodies, so it deserves its own tuned slop rather than silently
inheriting a value calibrated for 1 m stacked boxes.

**Alternatives considered**:
- *Velocity-bias (Baumgarte term added directly into `Cdot`'s target each
  iteration)*: the more common textbook presentation. Rejected for the same
  energy-injection reason the README already gives for contacts — consistent
  with Principle III (match the README's established rationale, don't
  reintroduce a problem the codebase already solved once).

## 5. Warm-starting

**Decision**: Out of scope for M6's acceptance bar; left as a
straightforward follow-up once the joint is working, using the same
`(body_a, body_b)` keyed cache pattern `ImpulseCache` already established
for contacts. Not required because a revolute joint's `K` matrix — unlike a
contact's scalar `k` — depends only on body geometry (masses, `r_a`, `r_b`),
not on which iteration converged last time, so cold-starting at zero impulse
converges within `VELOCITY_ITERATIONS` for a single joint without visible
startup transient in the pendulum demo.

**Rationale**: matches the spec's own Assumptions section — the README's
"done when" bar for M6 (anchor stays coincident, period matches analytic
prediction) does not depend on warm-starting, and M4's warm-starting was
motivated by a specific failure mode (a two-point manifold rocking a tall
stack apart) that a single two-body point joint doesn't have.

**Alternatives considered**:
- *Warm-start joints from day one*: more future-proof for M8 (motor, which
  is a revolute-joint variant) but adds cache-plumbing complexity with no
  M6 acceptance criterion motivating it. Deferred, not rejected — worth
  revisiting when M8 is planned.
