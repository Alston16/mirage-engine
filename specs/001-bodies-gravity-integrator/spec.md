# Feature Specification: Bodies, Gravity & Fixed-Timestep Integrator

**Feature Branch**: `001-bodies-gravity-integrator`

**Created**: 2026-08-29

**Status**: Draft

**Input**: User description: "M1 from README"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Bodies fall under gravity with correct physics (Priority: P1)

A developer using the engine creates one or more dynamic rigid bodies with a
starting position and mass, adds them to the simulation world, and advances
the simulation forward in time. Each body accelerates downward exactly as
Newtonian gravity predicts, with velocity updated before position each step
(semi-implicit Euler), and static bodies placed in the world do not move at
all.

**Why this priority**: This is the physical core of the milestone — without
correct gravity integration nothing later (collision, resolution, stacking)
can be verified, because those milestones assume the integrator is already
trustworthy.

**Independent Test**: Create a dynamic body at rest and a static body, step
the world forward by a known amount of simulated time, and confirm the
dynamic body's velocity and position match the closed-form gravity
equations while the static body's position and velocity are unchanged.

**Acceptance Scenarios**:

1. **Given** a dynamic body at rest at a known height, **When** the world is
   stepped forward by a known number of fixed timesteps, **Then** the
   body's downward velocity matches `g * t` (with `g = 9.81 m/s²`) within
   numerical tolerance.
2. **Given** a static body placed anywhere in the world, **When** the world
   is stepped forward any number of times, **Then** the static body's
   position and velocity remain exactly unchanged.
3. **Given** two dynamic bodies with different masses at the same height,
   **When** the world is stepped forward, **Then** both bodies fall with
   identical acceleration (gravity is mass-independent), matching the
   physical expectation.

---

### User Story 2 - Simulation behavior is independent of render framerate (Priority: P2)

A developer runs the simulation inside a render loop that reports variable
frame times (e.g., due to system load). Regardless of how those frame times
vary, the simulation always advances in fixed 1/60-second increments, so the
sequence of physics states produced for a given amount of total elapsed time
is the same no matter how that time was chopped into rendered frames.

**Why this priority**: Determinism and framerate-independence are called
out as core design principles; without a fixed-timestep accumulator, the
same scenario would behave differently on different machines, breaking any
later milestone's ability to verify behavior (e.g., M4's 60-second stable
stack).

**Independent Test**: Step the world using two different sequences of
variable frame-time inputs that sum to the same total elapsed time, and
confirm both sequences produce the same body states (within floating-point
tolerance).

**Acceptance Scenarios**:

1. **Given** a running world, **When** it receives a sequence of small,
   irregular frame-time inputs, **Then** it internally advances physics in
   whole 1/60-second steps only, never a partial step.
2. **Given** two separate runs of the same scenario fed different framing
   of frame times but identical total elapsed time, **When** both runs
   complete, **Then** the resulting body states are equal within tolerance.

---

### User Story 3 - Visual confirmation via the bouncing example (Priority: P3)

A developer runs `examples/bouncing.rs` and visually observes circles
falling under gravity with no collision response yet — the circles pass
through each other and through the floor, continuing to accelerate downward
until they leave the visible window.

**Why this priority**: This is the milestone's own "done when" acceptance
demo — a behavioral, visual check that the integrator is wired up
end-to-end (world, bodies, rendering), not just unit-tested in isolation.

**Independent Test**: Run `cargo run --example bouncing --release` and
observe that circles fall with visibly increasing speed and exit the bottom
of the window without stopping or bouncing.

**Acceptance Scenarios**:

1. **Given** the bouncing example is running, **When** it starts, **Then**
   circles are rendered at their initial positions.
2. **Given** the bouncing example is running, **When** time passes,
   **Then** circles visibly accelerate downward and exit the window without
   colliding with the floor, other circles, or window bounds.

---

### Edge Cases

- What happens when a body has zero or undefined mass? Static bodies are
  treated as having infinite mass (zero inverse mass) and are excluded from
  gravity/integration entirely, not divided-by-zero.
- What happens when a single rendered frame takes much longer than 1/60s
  (e.g., a stutter)? The accumulator MUST still only ever advance physics
  in fixed 1/60s increments, running as many of them as needed to catch up
  before rendering the next frame.
- What happens with a body that starts already in motion (nonzero initial
  velocity)? Gravity accumulates on top of the existing velocity each step,
  per the semi-implicit Euler update.
- What happens when the world contains zero bodies? `World::step` completes
  without error and performs no per-body work.
- What happens when multiple dynamic bodies occupy the same or overlapping
  positions? They fall independently and pass through each other, since no
  collision detection exists at this milestone.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST provide a rigid body representation with a unique
  identity, 2D position, 2D linear velocity, orientation, and angular
  velocity.
- **FR-002**: System MUST support both dynamic bodies (affected by forces
  and gravity) and static bodies (immovable, unaffected by gravity or
  integration).
- **FR-003**: System MUST provide a world container that holds all bodies
  and exposes a single "step the simulation forward by an elapsed amount of
  time" operation.
- **FR-004**: The step operation MUST integrate each dynamic body's motion
  using semi-implicit (symplectic) Euler: velocity is updated from
  acceleration first, then position is updated from the new velocity.
- **FR-005**: The step operation MUST apply a constant downward
  gravitational acceleration of 9.81 units/s² to every dynamic body, every
  fixed step.
- **FR-006**: The step operation MUST run the simulation on a fixed
  timestep of 1/60 second, using an accumulator that converts variable
  elapsed real time into zero or more fixed-size simulation steps per call.
- **FR-007**: Simulation outcomes MUST be deterministic: given the same
  starting body states and the same total sequence of fixed steps, the
  resulting body states MUST be identical (within floating-point
  tolerance) regardless of how real time was divided into calls to the
  step operation.
- **FR-008**: This milestone MUST NOT perform any collision detection or
  collision response — bodies move under gravity alone and do not react to
  overlapping other bodies or any ground/floor geometry.
- **FR-009**: System MUST provide a runnable visual example
  (`examples/bouncing.rs`) that creates one or more dynamic circular bodies
  and renders their positions each frame as they fall under gravity, with
  no collision response.
- **FR-010**: Static bodies MUST report zero velocity and unchanged
  position after any number of simulation steps.

### Key Entities

- **RigidBody**: A single physical object in the simulation. Key
  attributes: identity, position, linear velocity, orientation, angular
  velocity, mass (or inverse mass), and a dynamic/static classification.
  Does not yet participate in collision at this milestone.
- **World**: The simulation container. Key attributes: the set of bodies it
  owns, the fixed timestep duration, the accumulated leftover time from the
  last call to step, and the gravity constant applied to dynamic bodies.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: For a dynamic body released from rest, its downward speed
  after any elapsed simulated time matches the closed-form value predicted
  by constant 9.81 units/s² acceleration, within a small numerical
  tolerance, at every fixed step checked.
- **SC-002**: A static body's position never changes across any number of
  simulation steps, in every scenario tested.
- **SC-003**: Feeding the simulation the same total elapsed time split into
  different numbers/sizes of real-time updates produces the same sequence
  of fixed-step body states, within floating-point tolerance.
- **SC-004**: Running the bouncing example with `--release` shows circles
  visibly falling with increasing speed and exiting the window, with zero
  instances of circles stopping, bouncing, or colliding with each other or
  a floor.
- **SC-005**: The full automated test suite (`cargo test`) passes,
  including coverage for gravity-driven velocity/position integration and
  for fixed-timestep accumulator behavior under variable input timing.

## Assumptions

- Gravity is a single constant, world-level value (9.81 units/s², downward)
  applied uniformly to all dynamic bodies; per-body gravity scaling is out
  of scope for this milestone, matching the README's integration model.
- The accumulator has no maximum-substep safety clamp ("spiral of death"
  guard) specified for this milestone; it is assumed to run as many fixed
  steps as have accumulated before rendering the next frame, since no cap
  is called out in the README and adding one would be a design decision
  beyond M1's stated scope.
- Angular velocity and orientation exist on the rigid body type (per the
  architecture) but are not exercised by gravity alone at this milestone —
  no torque source exists yet, so bodies fall without rotating.
- The circles rendered in `examples/bouncing.rs` need only enough shape
  data (e.g., a radius) to be drawn; full shape-derived mass/inertia/AABB
  computation (`shape.rs`'s broader responsibilities) is not required to
  be wired in until collision detection lands in M2.
- "Off-screen" / exiting the visible window in the bouncing example is a
  visual observation only; there is no floor or world-bounds body to
  collide with, since no collision exists yet.
