# Feature Specification: Distance / Spring Joint

**Feature Branch**: `007-distance-spring-joint`

**Created**: 2026-10-16

**Status**: Draft

**Input**: User description: "@README.md M7" — README.md § Post-MVP milestones, M7: "A 2-body constraint holding two anchors at a target distance — rigid (equality constraint) or soft (Hookean spring force with damping). `examples/spring.rs`: a box suspended from a fixed point oscillates and settles. Done when: a critically-damped spring settles to rest length without visible overshoot, and an undamped spring conserves energy within the solver's iterative tolerance over a fixed run length."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Hang a body from a point on a spring (Priority: P1)

As someone building a scene with `mirage`, I want to connect two bodies with a spring between an anchor point on each, so that the bodies are pulled toward a chosen rest distance apart with a configurable stiffness and damping — a box bobbing on a spring, a soft suspension, a bungee.

**Why this priority**: The soft (spring) mode is what the milestone's demo and both "done when" criteria are about. Without it there is no `examples/spring.rs` and nothing to measure M7 against.

**Independent Test**: Hang a box from a static body by a spring, release it away from equilibrium, step the simulation, and confirm the box oscillates along the spring's axis and — with damping — comes to rest at the length where the spring force balances gravity.

**Acceptance Scenarios**:

1. **Given** a box hanging from a static point by a spring with some damping, released stretched beyond its equilibrium length, **When** the simulation steps forward under gravity, **Then** the box oscillates about the equilibrium length with decreasing amplitude and comes to rest there.
2. **Given** the same scene with the damping set to exactly the critical value for that spring and box, **When** the box is released, **Then** it returns to the equilibrium length without overshooting it and settles there.
3. **Given** the same scene with zero damping, **When** the simulation runs for a fixed duration, **Then** the box keeps oscillating with no systematic growth or decay in total mechanical energy (kinetic + gravitational + spring), beyond small bounded fluctuation from the fixed-timestep integrator.
4. **Given** a spring between two dynamic bodies in free space (no gravity), **When** they are released stretched apart, **Then** they oscillate about the rest length symmetrically about their shared center of mass, and their total momentum is unchanged.

---

### User Story 2 - Hold two bodies at a fixed distance (Priority: P2)

As someone building a scene with `mirage`, I want to connect two bodies with a rigid distance constraint, so that the anchor points stay a fixed distance apart while the bodies remain otherwise free to swing and rotate — a weightless rod between two points, a chain link, a bob on a stiff string that can't stretch.

**Why this priority**: The README lists rigid mode as part of M7, and it is the equality-constraint counterpart to the revolute joint, but neither "done when" criterion depends on it — the spring mode alone delivers the milestone's demo.

**Independent Test**: Connect a body to a static point with a rigid distance constraint at some length, release it to one side under gravity, and confirm the anchor separation stays at that length for the whole run while the body swings like a pendulum on a massless rod.

**Acceptance Scenarios**:

1. **Given** a body connected to a static point by a rigid distance constraint and released off to one side, **When** it swings under gravity, **Then** the distance between the anchors stays at the target length (within the engine's positional-slop tolerance) at every step.
2. **Given** a small body on a rigid distance constraint of length `L`, released from a small angle, **When** it swings, **Then** its period matches the pendulum prediction for that length within a few percent.
3. **Given** two bodies connected by a rigid distance constraint whose anchors start closer or farther apart than the target length, **When** the simulation steps, **Then** the separation is corrected gradually toward the target length rather than snapped instantly.

---

### User Story 3 - Spring demo watched end to end (Priority: P3)

As someone verifying the engine behaves correctly, I want to run `examples/spring.rs` and watch a box suspended from a fixed point oscillate and settle, so I can confirm by eye — the way every earlier milestone was gated — that the spring looks physically right.

**Why this priority**: The behavioral checks in P1 can be measured headlessly; the demo is the project's established visual acceptance gate and depends on P1 being correct first.

**Independent Test**: Run `cargo run --example spring --release` and watch for the full run.

**Acceptance Scenarios**:

1. **Given** the demo running in `--release`, **When** watched for its full run, **Then** the suspended box visibly oscillates, its oscillation dies away, and it comes to rest at its equilibrium length with no jitter or creeping.
2. **Given** the demo, **When** it shows more than one spring configuration side by side (e.g., under-damped, critically damped, undamped), **Then** each behaves as its configuration predicts: ringing down, settling without overshoot, and oscillating indefinitely, respectively.

---

### Edge Cases

- **Anchors coincide (current distance is zero).** The direction along which the constraint or spring acts is undefined. The joint applies nothing for that step rather than producing invalid (non-finite) motion; the bodies' other motion carries them out of the degenerate configuration.
- **Rest length of zero.** A spring with a zero rest length pulls the anchors together; it is valid configuration and behaves as a spring pulling toward a point, subject to the zero-distance rule above once the anchors actually meet.
- **Both bodies static.** Nothing can move; the joint is ignored, the same posture the revolute joint and contacts take toward two immovable bodies.
- **Invalid spring parameters** (negative stiffness, negative damping, negative rest length). Rejected when the joint is created, the same way connecting a body to itself is rejected today.
- **Very stiff springs.** A spring stiff enough that its natural oscillation is fast relative to the fixed timestep may become inaccurate or unstable. This is a known limitation of a fixed-timestep engine, documented rather than guarded against; users who want "infinitely stiff" use the rigid mode instead.
- **Jointed body also touching contacts, or connected by several joints.** Distance/spring joints are resolved alongside contacts and revolute joints in the same per-step loop; none disables the others.
- **A body that is both on a spring and pinned by a revolute joint** (e.g., a hinged lid held open by a spring). Both joints act on it together; no special handling is required.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST provide a way to connect two bodies with a distance joint by specifying a local anchor point on each body (in that body's own frame) and a target (rest) length.
- **FR-002**: If the caller does not specify a target length, System MUST use the anchors' world-space separation at the moment the joint is added to the world.
- **FR-003**: A distance joint MUST be configurable as either **rigid** — the anchor separation is held at the target length — or **soft** — a Hookean spring with a caller-chosen stiffness and damping pulls the separation toward the target length.
- **FR-004**: In soft mode, the force along the anchor-to-anchor line MUST be proportional to the deviation from rest length (stiffness) plus a term proportional to the rate of change of that separation (damping), acting equally and oppositely on the two anchor points.
- **FR-005**: Callers MUST be able to configure a spring's damping so that it is exactly critically damped for a given connected mass and stiffness, without having to reverse-engineer the engine's internal conventions (the relationship between stiffness, mass, and critical damping is documented alongside the joint).
- **FR-006**: A distance joint, rigid or soft, MUST constrain only the separation between the two anchors — never their direction and never the bodies' relative orientation.
- **FR-007**: In rigid mode, positional drift in the anchor separation MUST be corrected gradually, in the same style as the revolute joint's and contacts' positional correction, rather than snapped instantly.
- **FR-008**: Distance joints MUST be resolved within the same fixed step as contacts and revolute joints, so a body subject to several of these converges under all of them together: rigid distance constraints are solved in the shared per-step resolution loop, and spring forces are applied in that same step, before constraint resolution, so contacts and other joints respond to them.
- **FR-009**: A distance joint MUST support one of its two bodies being static, so a body can be suspended from a fixed point in the world.
- **FR-010**: System MUST include a runnable demo, `examples/spring.rs`, showing a box suspended from a fixed point by a spring that oscillates and settles, watchable with `--release`.
- **FR-011**: Adding this feature MUST NOT change the outcome of any scene that contains no distance joints — the existing bouncing, ramp, stack, pyramid, and hinge demos and their behavioral tests produce the same results as before.
- **FR-012**: The engine crate MUST remain free of external dependencies.
- **FR-013**: Simulations with distance joints MUST remain deterministic — the same scene and joint configuration produce the same result on every run.
- **FR-014**: The degenerate cases in § Edge Cases (zero separation, two static bodies, invalid parameters) MUST be handled as described there; in particular, no configuration may produce non-finite body state.
- **FR-015**: README § How it works MUST gain the derivation for the distance/spring joint in the same style as the contact and revolute-joint math once implemented, per README § Post-MVP milestones.

### Key Entities

- **Distance Joint**: A constraint connecting exactly two bodies (either may be static). Holds a local anchor on each body, a target length, and a mode: rigid, or soft with a stiffness and a damping value. Its role is to keep — or pull — the anchors' separation to the target length, leaving every other degree of freedom free.
- **Revolute Joint** *(existing, M6)*: Coexists with distance joints in the same world and the same per-step resolution; its behavior is unchanged.
- **Rigid Body** *(existing)*: A distance joint references two existing bodies; it changes their state only through the forces/impulses it applies.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: A box hung from a fixed point by a critically damped spring and released from a displacement away from equilibrium settles to its equilibrium length with any overshoot past equilibrium no larger than 1% of the initial displacement, and stays within 1% of the initial displacement of equilibrium from then on.
- **SC-002**: A box on an undamped spring, run for a fixed 60 simulated seconds, shows no systematic drift in total mechanical energy: energy averaged over the final oscillation period is within 1% of energy averaged over the first, and instantaneous energy never departs from its initial value by more than 5% of the oscillation energy (the spring energy at the release displacement). Total energy is the wrong yardstick here, because it is dominated by the gravitational term and would hide real drift.
- **SC-003**: An under-damped spring's measured oscillation period matches the analytic mass–spring prediction for its stiffness, damping, and mass within a few percent.
- **SC-004**: A body on a rigid distance constraint keeps its anchor separation within the engine's positional-slop tolerance of the target length for a full sustained run, and, released from a small angle, swings with a period within a few percent of the corresponding pendulum prediction.
- **SC-005**: `examples/spring.rs`, watched for its full run in `--release`, shows a suspended box that oscillates and settles with no visible jitter, sinking, or energy gain.
- **SC-006**: All pre-existing behavioral tests and demos produce the same outcomes as before the feature was added.

## Assumptions

- "Settles to rest length" in the README's done-when criterion is read as settling to the **equilibrium** length — for a box hanging under gravity, the rest length plus the static stretch from the box's weight — since a hanging spring never returns to its unloaded rest length.
- "Critically damped" is defined for the demo's configuration, where the box hangs directly below its anchor and moves along a single line, so the classic single-mass critical damping value applies. Off-axis or two-dynamic-body configurations are not required to be exactly critically damped by the same value.
- "Within the solver's iterative tolerance" is made concrete as SC-002's 1% drift bound and 5% instantaneous bound, both relative to the oscillation energy; the exact bounds may be tightened during planning once measured, as M6's "few percent" period bound was.
- Spring parameters are expressed as physical stiffness and damping coefficients (the textbook Hookean form the README names), not as a frequency/damping-ratio pair; whether the spring is applied as an explicit force or as a softened constraint impulse is an implementation choice for planning, provided the success criteria hold.
- Rigid mode is a two-sided equality constraint (a rod: it resists both stretching and compression). A one-sided "rope" (resists stretching only), length limits, and breakable joints are out of scope.
- Joints are permanent for the life of the scene; runtime removal is out of scope, as for M6.
- M8 (motor) is not started as part of this work.
