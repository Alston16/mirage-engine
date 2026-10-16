# Feature Specification: Revolute (Hinge) Joint

**Feature Branch**: `006-revolute-hinge-joint`

**Created**: 2026-09-27

**Status**: Draft

**Input**: User description: "M6 as per README.md" — README.md § Post-MVP milestones, M6: "A 2-body point constraint pinning a local anchor on body A to a local anchor on body B, Baumgarte-stabilized the same way contact penetration is. `examples/hinge.rs`: a rod pinned at one end swings like a pendulum under gravity. Done when: the anchor stays coincident within positional-slop tolerance for the full run, and the pendulum's period matches the small-angle analytic prediction within a few percent."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Connect two bodies at a shared point (Priority: P1)

As someone building a scene with `mirage`, I want to connect two rigid bodies at a point so that they stay attached to each other while remaining free to rotate relative to one another, the way a door stays on its hinge.

**Why this priority**: This is the core capability of the milestone — without a working point constraint, there is no hinge joint and nothing else in this feature is meaningful.

**Independent Test**: Create two bodies (one of them may be static), connect them with a revolute joint at chosen anchor points, run the simulation for a sustained period under gravity, and confirm the two anchor points never drift apart beyond the engine's existing positional-slop tolerance.

**Acceptance Scenarios**:

1. **Given** two bodies whose local anchor points coincide in world space at creation time, **When** the simulation steps forward under gravity, **Then** the anchor points remain coincident (within positional-slop tolerance) at every subsequent step.
2. **Given** a joint between a dynamic body and a static (infinite-mass) body, **When** the simulation steps forward, **Then** only the dynamic body moves to satisfy the constraint, and the static body stays fixed.
3. **Given** a joint that also has other forces or contacts acting on either body, **When** the simulation steps forward, **Then** the joint constraint and any contact constraints are both satisfied within their respective tolerances (the joint does not need to disable or override contact handling).

---

### User Story 2 - Pendulum demo validates the joint under gravity (Priority: P2)

As someone verifying the engine behaves correctly, I want to watch a rod pinned at one end swing under gravity like a real pendulum, so I can confirm the joint produces physically believable motion, not just a point that technically stays coincident.

**Why this priority**: The milestone's own "done when" criterion is behavioral and demo-driven — matching a known analytic result is the strongest available evidence that the constraint math is correct, beyond just "the anchor didn't separate."

**Independent Test**: Run the pendulum demo, release the rod from a small angle, and measure its oscillation period against the small-angle pendulum period formula.

**Acceptance Scenarios**:

1. **Given** a rod pinned at one end and released from a small angle from vertical, **When** it swings freely under gravity, **Then** its measured oscillation period matches the analytic small-angle pendulum prediction within a few percent.
2. **Given** the demo running continuously, **When** watched for its full run, **Then** the pendulum shows no visible jitter, no visible stretching of the joint, and no energy gain over time.

---

### User Story 3 - Joint holds up under a longer, less ideal run (Priority: P3)

As someone stress-testing the engine the way the existing stacking demos do, I want the joint to stay stable well past the first few swings — including scenarios like a larger release angle or a heavier attached body — so I can trust the joint outside the single "small angle" happy path.

**Why this priority**: Nice-to-have robustness beyond the milestone's minimum bar; valuable but not blocking if the P1/P2 behavior is solid.

**Independent Test**: Run the pendulum demo (or a variant scene) with a larger release angle and/or a heavier rod for an extended duration and confirm the anchor stays coincident and motion stays bounded (no runaway energy gain).

**Acceptance Scenarios**:

1. **Given** a rod released from a large angle (well outside the small-angle regime), **When** it swings under gravity for an extended run, **Then** the anchor stays coincident within tolerance and the motion remains bounded (no divergence or explosion).

---

### Edge Cases

- What happens when a joint's two anchor points do **not** coincide in world space at creation time (the bodies are placed with some initial separation between anchors)? The joint pulls them together gradually via the same positional-correction mechanism used for contact penetration, rather than snapping them together instantly.
- What happens when both bodies in a joint are static? The constraint has nothing to move; this is a degenerate, physically meaningless configuration and is out of scope to guard against explicitly (same posture as other physically nonsensical inputs elsewhere in the engine).
- What happens when a body has more than one joint attached to it (e.g., a chain of hinged rods)? Each joint is solved as its own sequential-impulse constraint within the same per-step velocity-iteration loop, the same way a body touching multiple contacts is handled today.
- What happens when a jointed body is also resting in a stack or colliding with the floor? Joint and contact constraints are solved side by side in the same iteration loop; neither mechanism is aware of or disables the other.
- What happens over a very long run (well beyond the demo's watched duration)? Not covered by this feature's acceptance bar; the milestone's "done when" criterion is a single full demo run, consistent with how M4's 60-second stacking bar was defined.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST provide a way to connect two bodies with a revolute joint by specifying a local anchor point on each body (a point expressed in that body's own local frame, not world space).
- **FR-002**: System MUST enforce, every simulation step, that a joint's two anchor points stay coincident in world space within the engine's existing positional-slop tolerance, regardless of gravity or other forces acting on the connected bodies.
- **FR-003**: System MUST allow the two bodies connected by a joint to rotate freely relative to one another — the joint constrains position only, never relative orientation.
- **FR-004**: System MUST correct any positional drift between a joint's anchor points using a Baumgarte-style bias, the same style of correction already used for contact penetration, rather than an instantaneous snap.
- **FR-005**: System MUST solve joint constraints as sequential impulses within the same per-step velocity-iteration loop that already resolves contacts, so joints and contacts on a shared body are resolved together, not in separate passes.
- **FR-006**: System MUST support a joint where one of the two connected bodies is static (infinite mass, immovable), so a body can be "pinned" to a fixed point in the world.
- **FR-007**: System MUST include a runnable demo (`examples/hinge.rs`) showing a rod pinned at one end swinging under gravity like a pendulum, watchable with `--release`.
- **FR-008**: System MUST NOT require any changes to how existing contact resolution (normal impulses, friction, restitution) behaves — adding a joint to a scene must not alter outcomes for scenes that contain no joints.
- **FR-009**: System MUST keep the engine crate free of external dependencies; the joint feature introduces no new crate dependencies.
- **FR-010**: System MUST behave deterministically — given the same bodies, joint configuration, and iteration order, repeated runs produce the same result.

### Key Entities

- **Revolute Joint**: A constraint connecting exactly two bodies (either may be static). Holds a local anchor point for each of the two bodies. Its role is to keep those two anchor points coincident in world space every step while leaving relative rotation between the bodies unconstrained.
- **Rigid Body** *(existing entity, referenced not redefined)*: A joint attaches to two already-existing bodies by reference; it does not create, own, or alter body state beyond applying constraint impulses to it.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: In a joint connecting two bodies, the distance between the two anchor points stays within the engine's existing positional-slop tolerance for the full duration of a sustained simulation run.
- **SC-002**: A pendulum released from a small angle completes its swing with a measured oscillation period within a few percent of the analytically predicted small-angle pendulum period.
- **SC-003**: The pendulum demo, watched for its full run in `--release`, shows no visible jitter, no visible joint stretching, and no visible energy gain (the swing does not grow larger over time).
- **SC-004**: Scenes with no joints in them (the existing bouncing, ramp, stack, and pyramid demos and their behavioral tests) continue to produce the same outcomes as before this feature was added.

## Assumptions

- A "static" body for pinning purposes reuses the engine's existing static/infinite-mass body representation rather than introducing a separate "anchor to world" concept.
- Joint anchors are not required to coincide at scene-creation time; if they start apart, the same gradual positional correction used for contact penetration pulls them together rather than snapping instantly.
- Joints are permanent for the lifetime of the two bodies they connect — runtime joint removal or "breakable" joints are out of scope for this milestone, consistent with the README not mentioning joint teardown.
- Warm-starting of joint impulses across steps (as already done for contacts) is left as an implementation detail to decide during planning, not a user-facing requirement, since the README's "done when" bar for M6 does not depend on it.
- The pendulum demo's rod is a single rigid body (e.g., a thin box or capsule-like polygon) pinned at one end to a fixed point; no additional bodies are required to demonstrate the milestone.
