# Feature Specification: Impulse Resolution (M3)

**Feature Branch**: `003-impulse-resolution`

**Created**: 2026-09-19

**Status**: Draft

**Input**: User description: "M3 as per the README.md"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Bodies stop passing through each other (Priority: P1)

A developer simulating rigid bodies needs the engine to react to the contacts
it already detects (M2): when a falling body lands on the floor or on another
body, it must stop moving into it instead of passing straight through. Today
(post-M2) contacts are reported but bodies still fall through everything.

**Why this priority**: This is the point at which the engine becomes a
physics engine rather than a collision reporter. Every later milestone
(friction, stacking) builds on bodies actually reacting to contact.

**Independent Test**: Drop a dynamic circle onto a static floor with zero
restitution and step the world. Can be fully tested without rendering by
inspecting the body's velocity and position after landing — it stops dead on
the surface and stays there.

**Acceptance Scenarios**:

1. **Given** a dynamic body falling onto a static body with restitution
   `e = 0`, **When** the world steps through the moment of contact, **Then**
   the component of the body's velocity along the contact normal becomes zero
   (within floating-point tolerance) and the body does not rebound.
2. **Given** that same body now resting on the static body, **When** the
   world continues to step under gravity, **Then** the body stays at the
   surface and does not sink through it or drift away over time.
3. **Given** two dynamic bodies colliding head-on, **When** the world steps,
   **Then** both bodies' velocities change, with the lighter body changing
   more than the heavier one, and total momentum along the contact normal is
   conserved.
4. **Given** two bodies that are already moving apart at a contact,
   **When** the world steps, **Then** no impulse is applied and their
   velocities are unchanged.
5. **Given** a static body (zero inverse mass) in any contact, **When** the
   world steps, **Then** its velocity and position never change.

---

### User Story 2 - Bounce according to restitution (Priority: P2)

A developer needs each body to carry a restitution coefficient controlling how
bouncy its collisions are — from perfectly inelastic (`e = 0`, stops dead) to
perfectly elastic (`e = 1`, rebounds with all its speed) — so that a dropped
ball behaves like a rubber ball, a bean bag, or anything in between.

**Why this priority**: Restitution is what makes contact response look
physically right rather than merely non-penetrating, and it is one of the two
measurable "done when" criteria for this milestone. It depends on Story 1's
contact response existing first.

**Independent Test**: Drop a ball from a known height onto a static floor
with `e = 1.0`, record the peak height of the first rebound, and compare it
to the drop height. Repeat with `e = 0`.

**Acceptance Scenarios**:

1. **Given** a ball dropped from a known height onto a static floor with
   `e = 1.0`, **When** it rebounds, **Then** its peak rebound height is
   within a few percent of the drop height.
2. **Given** the same drop with `e = 0`, **When** the ball lands, **Then** it
   stops dead on the floor with no visible rebound.
3. **Given** the same drop with an intermediate value such as `e = 0.5`,
   **When** the ball rebounds, **Then** its rebound speed is approximately
   half its impact speed (rebound height approximately one quarter of the
   drop height).
4. **Given** two bodies with different restitution values in contact,
   **When** the world steps, **Then** the engine applies one well-defined
   combined restitution to that contact (the same result regardless of which
   body is listed first).

---

### User Story 3 - Resolved overlap doesn't linger or jitter (Priority: P3)

A developer needs bodies that have ended up overlapping — from landing at
speed, or from accumulated numerical error — to be gently pushed apart, and
needs resting contacts to stay steady rather than vibrating or slowly sinking.

**Why this priority**: Velocity response alone lets bodies creep into each
other over many steps. Positional correction keeps resting bodies visibly on
the surface, but the engine already produces the milestone's headline
behaviors (Stories 1–2) without it, so it ranks lower.

**Independent Test**: Rest a box on a static floor for several simulated
seconds and measure how far it sits inside the floor and how much its
position changes frame to frame.

**Acceptance Scenarios**:

1. **Given** two bodies overlapping by a small amount, **When** the world
   steps for a few frames, **Then** the overlap shrinks toward zero without
   any visible pop or explosive separation.
2. **Given** a body at rest on a static surface for 10 simulated seconds,
   **When** its position is sampled every frame, **Then** its penetration
   into the surface stays below a small fixed tolerance and it shows no
   visible jitter.
3. **Given** a body overlapping by no more than the allowed tolerance,
   **When** the world steps, **Then** no positional correction is applied,
   so resting contacts do not flicker on and off.

---

### User Story 4 - Watch the behavior in the demo (Priority: P4)

A developer wants to see the resolution working — balls of different
restitution dropped onto a floor, bouncing or stopping accordingly — in the
existing example harness, rather than only trusting unit-test numbers.

**Why this priority**: The milestone's "done when" criterion is behavioral, so
it should be verified by actually running a scene, but the engine works
without the demo. It is a verification aid.

**Independent Test**: Run the bouncing example and observe that circles land
on the floor and bounce or stop according to their restitution.

**Acceptance Scenarios**:

1. **Given** the bouncing example with balls of several restitution values,
   **When** it runs, **Then** each ball lands on the floor instead of
   falling off-screen, and rebounds in proportion to its restitution.

---

### Edge Cases

- A contact whose relative velocity along the normal is zero or positive
  (resting or separating): no impulse is applied, and the body is never
  pulled back toward the contact.
- Two static bodies (or a static body against a body with no mass) in
  contact: no impulse or correction is attempted and nothing divides by zero.
- A polygon–polygon manifold with two contact points: the impulse is shared
  across both points so a resting box does not tip or rock from one contact
  being resolved before the other.
- A contact where the normal impulse would be negative (pulling the bodies
  together): it must be clamped so contacts can push but never pull.
- A very low-speed impact with `e > 0`: the body should settle to rest rather
  than bouncing forever with ever-smaller, jittery hops.
- Fast-moving bodies tunneling through thin geometry within a single step
  remain a known limitation and are not treated as a defect here.
- Contacts from off-center hits on polygons: the impulse must produce the
  correct change in angular velocity as well as linear velocity.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: The engine MUST resolve every contact reported by collision
  detection on each fixed simulation step, after contacts are detected and
  before positions are integrated.
- **FR-002**: For each contact, the engine MUST compute the relative velocity
  `vr` of the two bodies at the contact point, including the contribution of
  each body's angular velocity, and skip the contact if the bodies are
  already separating along the normal.
- **FR-003**: For a non-separating contact, the engine MUST apply an equal
  and opposite normal impulse of magnitude `j`, per the formula in
  README § Resolution (including the `(1 + e)` restitution term and the
  effective-mass denominator built from `1/m_a`, `1/m_b`, `(r_a×n)²/I_a` and
  `(r_b×n)²/I_b`), changing both linear and angular velocity of each body.
- **FR-004**: The engine MUST iterate contact resolution multiple times per
  step (about 8 velocity iterations) so that bodies touching several others
  converge toward a consistent result.
- **FR-005**: Accumulated normal impulse for a contact MUST never be
  negative — contacts push bodies apart but never pull them together.
- **FR-006**: Every body MUST carry a restitution value in `[0, 1]`, and a
  contact between two bodies MUST use a single combined restitution that does
  not depend on the order of the pair.
- **FR-007**: Bodies with infinite mass (static) MUST never have their
  velocity or position altered by resolution, and a contact between two such
  bodies MUST be ignored.
- **FR-008**: The engine MUST correct residual penetration through a
  positional bias (Baumgarte-style) applied only to the portion of
  penetration exceeding a small tolerance ("slop"), so resting contacts
  neither jitter nor sink.
- **FR-009**: Resolution MUST be deterministic: identical inputs and
  iteration order produce identical results on every run.
- **FR-010**: Resolution MUST NOT apply friction (tangent impulses); that
  belongs to M4. Contact response in this milestone acts only along the
  contact normal.
- **FR-011**: The bouncing example MUST be updated so its bodies collide
  with a static floor and visibly bounce or stop according to restitution.

### Key Entities

- **Body material (restitution)**: A per-body value in `[0, 1]` describing
  bounciness; `0` absorbs all approach speed, `1` preserves it. Extends the
  material properties already on the body from M1.
- **Contact impulse**: The scalar impulse `j` applied along a contact's
  normal, equal and opposite on the two bodies, that changes their linear and
  angular velocities.
- **Relative velocity (`vr`)**: The velocity of one body relative to the
  other at a contact point, accounting for both linear and rotational motion.
- **Positional correction**: A small bias that gradually removes penetration
  beyond the allowed slop, distinct from the velocity impulse.
- **Manifold / Contact**: Produced by M2 and consumed unchanged here as the
  input to resolution.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: A ball dropped from a known height onto a static floor with
  `e = 1.0` rebounds to within 5% of its drop height on the first bounce.
- **SC-002**: A ball dropped onto a static floor with `e = 0` has zero
  velocity along the contact normal (to within 1e-3 of its impact speed) on
  the step after landing and never rebounds.
- **SC-003**: For a ball dropped with an intermediate restitution `e`, the
  measured rebound speed is within 5% of `e` times the impact speed.
- **SC-004**: A body resting on a static floor for 10 simulated seconds never
  penetrates more than a small fixed tolerance and never moves visibly from
  frame to frame.
- **SC-005**: In a head-on collision between two dynamic bodies, total
  momentum along the contact normal is conserved to within floating-point
  tolerance.
- **SC-006**: Running the same scene twice produces bit-identical body
  states after the same number of steps.
- **SC-007**: Static bodies have exactly the same position and velocity
  after every step of every test scene as before it.
- **SC-008**: Running the bouncing example shows balls landing on the floor
  and bouncing or stopping according to their restitution, with none falling
  through.

## Assumptions

- "Users" are developers consuming the `mirage` engine crate directly or via
  the bundled examples; there is no end-user UI beyond the existing
  macroquad demos.
- Contact data (point, normal, penetration, one- or two-point manifolds) from
  M2 is correct and consumed as-is; this feature does not change collision
  detection.
- Combining two bodies' restitution into a single value uses a
  conventional order-independent rule (the larger of the two, or the
  product); the exact rule is a planning decision and either satisfies the
  requirements above.
- The fixed timestep stays `1/60` and about 8 velocity iterations per step,
  per README; tuning iteration counts belongs to M4.
- Friction, warm-starting, and stable tall stacks are explicitly M4 and out of
  scope here; the 10-box tower and ramp-sliding behavior are not expected to
  work yet.
- Per the project non-goals, there are no joints, no continuous collision
  detection, no sleeping, and no concave shapes; bodies at very high speed
  may tunnel through thin geometry and that is expected.
- Static bodies (floor, ramp) are representable by the existing body type
  with zero inverse mass, as established in M1/M2.
- Two pre-existing gaps are closed as prerequisites rather than new
  capabilities: contact normals must consistently point from the first body
  toward the second for every shape-pair ordering (M2 reversed one
  ordering), and angular velocity must actually rotate bodies, as README
  § Integration already states (M1 did not integrate it).
- "Small fixed tolerance" and "no visible jitter" (SC-004, User Story 3)
  are concretely tested as: after a one-second settle, penetration no
  greater than twice the positional-correction slop and frame-to-frame
  vertical movement under 1e-3 world units.
