# Feature Specification: Friction & Stable Stacking (M4)

**Feature Branch**: `004-friction-and-stacking`

**Created**: 2026-09-20

**Status**: Draft

**Input**: User description: "M4 as per README.md" — Milestone M4, *Friction & stacking*: tangent impulses, iteration count tuning, warm-starting if needed. Done when the 10-box stack demo holds stable for 60 seconds without visible jitter or sinking, and a box on a shallow ramp stays put while one on a steep ramp slides.

## Clarifications

### Session 2026-09-20

Recorded after a plan-time prototype (see `research.md` § Prototype evidence) showed the original tower numbers were tighter than a sequential-impulse solver delivers while a tower is still settling from its "just touching" spawn.

- Q: What is the tower's "rest position" for the sinking bound, and how long may settling take? → A: The reference is the ideal pose (boxes exactly touching). Compression is bounded at 10% of a box height at all times (settling transient), 3% from 10 s onward, and must not creep by more than 0.1% between 10 s and 60 s. (Prototype at the intended tuning: about 6.7% at 2 s, 2.4% at 10 s, 1.8% at 60 s.)
- Q: What does "no visible jitter" mean numerically? → A: Speed below 0.1 m/s from 2 s (under about 4 px/s at the demo's 40 px/m, not visible as motion) and below 0.01 m/s from 30 s. "Speed" is the larger of a box's linear speed and its angular speed times its half-width. (Prototype: the last time speed exceeded 0.01 was about 22 s.)
- Q: What does "within a few percent" mean? → A: 5%, everywhere.
- Q: What does "pyramid remains standing" and "settle without persistent jitter" mean numerically? → A: See SC-005 and User Story 3 scenario 2.

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Friction holds or releases bodies on slopes (Priority: P1)

A developer using the engine places a box on an inclined surface. Whether the box stays put or slides must depend on the surface's friction and the slope angle, as in real life: a shallow slope holds the box, a steep slope lets it slide. Today (after M3) contacts only push along the contact normal, so every box slides on every ramp regardless of friction.

**Why this priority**: Friction is the missing physical behavior that defines M4, and it is the prerequisite for stable stacking (stacks of boxes rely on friction to resist sideways drift). It is also independently demonstrable and testable with a single box and a single ramp.

**Independent Test**: Drop a box on a static ramp at a shallow angle and another at a steep angle (same materials) and observe them for several seconds. Delivers a working Coulomb friction model even if stacking is not yet tuned.

**Acceptance Scenarios**:

1. **Given** a box resting on a ramp whose angle is below the friction limit for the pair's friction coefficient, **When** the simulation runs for 10 seconds, **Then** the box moves by less than 1% of its width and its speed stays below 0.01 m/s.
2. **Given** a box resting on a ramp whose angle is above the friction limit for the pair's friction coefficient, **When** the simulation runs, **Then** the box slides down the ramp with an acceleration consistent with the classical result for sliding under Coulomb friction (within 5%).
3. **Given** two identical setups differing only in the friction coefficient, **When** both run, **Then** the higher-friction body slides less (or not at all) compared to the lower-friction one.
4. **Given** a body with zero friction on any slope, **When** the simulation runs, **Then** it slides as it does today (no regression from M3 behavior).
5. **Given** a circle resting or rolling against a surface, **When** friction is applied, **Then** the contact can transfer linear motion into rotation and back (a ball on a slope rolls rather than only sliding).

---

### User Story 2 - A 10-box tower stands stably (Priority: P1)

A developer runs the stack demo: ten boxes stacked vertically on a static floor. The tower must stand still, without visible jitter, drift, or sinking, for a full 60 seconds. This is the MVP acceptance demo of the entire engine.

**Why this priority**: This is the headline "done when" criterion of M4 and of the whole MVP; the project is not complete until it passes.

**Independent Test**: Run the 10-box stack scenario for 60 simulated seconds and measure per-box displacement, velocity, and vertical compression; also run it visually as a demo.

**Acceptance Scenarios**:

1. **Given** a tower of 10 boxes placed just touching on a static floor, **When** the simulation runs for 60 simulated seconds, **Then** no box's horizontal drift exceeds 5% of a box width, and the tower does not topple.
2. **Given** the same tower, **When** the simulation runs for 60 seconds, **Then** the top box's downward compression (relative to the ideal touching pose) is at most 10% of a box height at all times, at most 3% from 10 seconds onward, and grows by no more than 0.1% of a box height between 10 and 60 seconds.
3. **Given** the same tower, **When** any box's speed is sampled, **Then** it stays below 0.1 m/s from 2 seconds onward and below 0.01 m/s from 30 seconds onward (no visible jitter).
4. **Given** the tower is running in the visual demo, **When** a person watches it for 60 seconds, **Then** no visible jitter, sinking, or creeping is apparent.
5. **Given** the same scenario run twice with identical inputs, **When** the results are compared, **Then** they are identical (determinism is preserved).

---

### User Story 3 - Pyramid and general resting-contact stress cases (Priority: P2)

A developer runs a pyramid of boxes (wider base, decreasing rows) and other multi-body resting arrangements. These should settle and stay settled, exercising contacts where several bodies interact at once.

**Why this priority**: The pyramid is the README's stress case; it validates that stacking stability is not an artifact of a single vertical column. It is lower priority because the MVP is complete at the tower and ramp criteria.

**Independent Test**: Run the pyramid scenario and confirm it settles into a stable resting configuration without collapse.

**Acceptance Scenarios**:

1. **Given** a pyramid of boxes on a static floor, **When** the simulation runs for 30 simulated seconds, **Then** the pyramid remains standing: every box stays within 10% of a box width of its starting horizontal position and no more than 10% of a box height below its starting height, tilts by no more than 0.05 rad, and moves slower than 0.05 m/s after 10 seconds.
2. **Given** a mix of boxes and circles landing on a floor, **When** the simulation runs for 20 seconds, **Then** each body's speed is below 0.02 m/s and it moves less than 0.001 m per step over the last 5 seconds, and none sits more than 2% of its size below its resting height.

---

### User Story 4 - Existing behavior is preserved (Priority: P2)

A developer who already relies on M3 behavior (restitution, bouncing, positional correction) sees no regression. A ball dropped with restitution 1.0 still returns near its drop height, and with restitution 0 still stops dead; the new friction and stacking work must not break these.

**Why this priority**: Regressions in earlier milestones undermine confidence in the engine; the earlier "done when" criteria must remain true.

**Independent Test**: Re-run the M3 acceptance scenarios (bouncing ball at e = 1.0 and e = 0) and confirm the same outcomes.

**Acceptance Scenarios**:

1. **Given** a ball with restitution 1.0 dropped onto a static floor, **When** it bounces, **Then** it returns to within 5% of its drop height (unchanged from M3).
2. **Given** a ball with restitution 0 dropped onto a static floor, **When** it lands, **Then** it stops dead (unchanged from M3).
3. **Given** the full existing test suite, **When** it runs, **Then** all previously passing tests still pass.

---

### Edge Cases

- A body with zero friction (μ = 0) in contact with any surface: no tangential force is applied and it slides freely.
- A body in contact with a static (infinite-mass) body: friction applies to the dynamic body only; the static body does not move.
- A contact with zero relative tangential velocity at the contact point (sticking): friction must hold it without introducing jitter or slow creep.
- A two-point contact manifold (e.g., a flat box face on the floor): friction and normal impulses must be distributed so the box does not rock or spin spuriously.
- Contacts that are separating: no friction impulse is applied (contacts push and resist, never pull).
- A circle in contact with a surface: relative tangential velocity at the contact point includes the contribution of spin, so rolling without slipping is representable.
- Very heavy body on very light body (large mass ratios): the solver must remain stable (no explosion), even if convergence is slower.
- A stack taller than 10 boxes: behavior may degrade gracefully (soft under load, per the README's known trade-off) but must not become unstable or explode.
- Body pairs with different friction coefficients: a single combined coefficient is used, consistently in both directions.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: The engine MUST apply Coulomb friction at every contact as a tangential impulse whose accumulated magnitude is limited to `μ` times the accumulated normal impulse `j` of that contact.
- **FR-002**: The engine MUST compute a single combined friction coefficient `μ` for each body pair, from the two bodies' individual friction properties, in a way that is symmetric (the order of the bodies does not matter) and documented.
- **FR-003**: The tangent direction MUST be derived from the contact normal (perpendicular to `n`), and relative velocity at the contact point MUST include the angular contribution of both bodies (`vr = (v_b + ω_b × r_b) − (v_a + ω_a × r_a)`).
- **FR-004**: Friction MUST be resolved together with normal impulses in the iterated solve, so that stacks converge rather than being resolved once per contact in isolation.
- **FR-005**: The friction clamp MUST use the accumulated normal impulse for the contact (not a per-iteration increment), so that friction capacity grows as load builds up through a stack.
- **FR-006**: The number of velocity iterations MUST be tuned so the 10-box tower and the pyramid meet their stability criteria; the chosen value MUST be documented along with the reasoning.
- **FR-007**: The engine MUST carry accumulated impulses from the previous step forward to the next step for persistent contacts (warm-starting) if, and only if, tuning iteration count alone does not achieve the stacking criteria. The decision and evidence MUST be recorded.
- **FR-008**: If warm-starting is used, contacts MUST be matched between consecutive steps in a deterministic way, and stale impulses MUST be discarded when a contact disappears, so that no ghost impulses are applied to new contacts.
- **FR-009**: A box on a ramp MUST remain at rest when the ramp angle is below the friction limit for the pair, and MUST slide when the angle is above it, within 5% of the classical threshold angle.
- **FR-010**: A sliding body's acceleration down a ramp MUST match the classical Coulomb friction result within 5%.
- **FR-011**: Setting friction to zero on a body MUST reproduce the frictionless M3 behavior.
- **FR-012**: The existing normal-impulse behavior (restitution, "contacts push, never pull", positional correction with slop) MUST be preserved and its tests MUST continue to pass.
- **FR-013**: Simulation results MUST remain deterministic: identical inputs and iteration order produce identical results across runs.
- **FR-014**: The stack example MUST build and run a 10-box tower on a static floor, and the pyramid example MUST build and run a pyramid of boxes, both drawing the scene and (optionally) contact debug information.
- **FR-015**: The engine crate MUST keep zero dependencies; the visual examples may depend only on the already-permitted demo dependency.
- **FR-016**: Non-goals MUST remain out of scope: no joints, continuous collision detection, sleeping, concave shapes, spatial partitioning, serialization, parallelism, or 3D. In particular, achieving stability MUST NOT rely on putting resting bodies to sleep.
- **FR-017**: Code and documentation for friction and any warm-starting MUST match the derivations in the README's "How it works" section (variable names included), and the README MUST be updated to describe the friction model, the tuned iteration count, and to mark M4 complete.

### Key Entities

- **Contact**: A single point of contact between two bodies, with a point, a normal (pointing from body A to body B), a penetration depth, and an identity for the geometric feature that produced it, so the same physical contact can be recognized from one step to the next.
- **Manifold**: The one or two contacts between a body pair for a single step; the unit over which impulses are solved and, if warm-starting is used, over which contacts are matched to the previous step.
- **Contact impulses**: The accumulated normal and tangent impulses of each contact, carried across solver iterations and, for persistent contacts, across steps.
- **Material properties**: Per-body restitution and friction. Friction is combined per body pair into one coefficient `μ`.
- **Tangent impulse**: The friction impulse applied along the direction perpendicular to the contact normal, limited by `μ` times the normal impulse.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: A 10-box tower stays standing for 60 simulated seconds. Against the ideal pose (boxes exactly touching), no box drifts horizontally by more than 5% of a box width at any time; the top box's downward compression is at most 10% of a box height at all times and at most 3% from 10 seconds onward; and it grows by no more than 0.1% of a box height between 10 and 60 seconds.
- **SC-002**: For every box in the tower, speed (the larger of linear speed and angular speed times half-width) is below 0.1 m/s from 2 simulated seconds onward and below 0.01 m/s from 30 seconds onward, through 60 seconds, for 1 m boxes.
- **SC-003**: A box on a ramp shallower than the friction limit moves less than 1% of its width over 10 simulated seconds, while a box on a ramp steeper than the limit slides down it.
- **SC-004**: The sliding acceleration of a box on a steep ramp matches the classical prediction to within 5%.
- **SC-005**: A 5-row pyramid of boxes remains standing for 30 simulated seconds, meeting the bounds in User Story 3 scenario 1.
- **SC-006**: The M3 acceptance results are unchanged: a restitution-1.0 ball returns to within 5% of its drop height and a restitution-0 ball stops dead; all previously passing tests still pass.
- **SC-007**: Two runs of the same scenario produce bit-identical body states after 60 simulated seconds.
- **SC-008**: The stack demo runs at an interactive frame rate (at least 60 frames per second) in release mode on a typical developer machine, with the tower stepping at the fixed timestep.
- **SC-009**: A person watching the stack demo for 60 seconds sees a tower that stays stable, with no visible jitter or sinking.

## Assumptions

- The engine's users are developers reading and running this repository; "visible jitter" and "visible sinking" are judged by watching the demo and also confirmed with numeric thresholds in automated tests.
- The tests can run without a display; the simulation can be stepped headless for 60 simulated seconds, and visual confirmation of the examples is a separate manual step that MUST be performed by running the examples.
- Friction is a per-body material property (with a default of a moderate, non-zero value); the default for a new body is chosen so the ramp and stack criteria work out of the box. If the existing body type already has a friction field, it is reused.
- The combined friction coefficient for a pair is derived from the two bodies' values in a symmetric way (for example, geometric mean); the exact combination rule is a design decision for planning and does not change the user-facing behavior beyond documented values.
- Only dynamic-versus-static and dynamic-versus-dynamic contacts are needed; static-versus-static contacts are ignored as before.
- Rolling resistance and torsional (spinning) friction are out of scope; only sliding friction at the contact point is modeled.
- The velocity iteration count starts at the README's approximately 8 and may be raised as part of tuning; the final number is a planning/implementation decision recorded in the README.
- Warm-starting is conditional: it is only added if iteration-count tuning alone cannot meet the tower and pyramid criteria, per the README milestone wording.
- Dependencies: M0–M3 (math, bodies and integrator, collision detection with manifolds, and normal-impulse resolution) are complete and are the foundation for this work; plan and design for this feature live in `specs/004-friction-and-stacking/` alongside this spec.
- A shallow-ramp box holding perfectly still may depend on the cross-step impulse carry-over delivered with User Story 2. If User Story 1 alone cannot meet its hold bounds, that is recorded and the hold acceptance is closed out under User Story 2 rather than loosened.
- Numeric bounds in the success criteria are for 1 m bodies in meters and seconds, matching the demos; they are scale-dependent by design.
