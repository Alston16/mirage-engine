# Feature Specification: Collision Detection (M2)

**Feature Branch**: `002-collision-detection`

**Created**: 2026-09-05

**Status**: Draft

**Input**: User description: "M2 as per README"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Know when two bodies are touching (Priority: P1)

A developer simulating rigid bodies needs the engine to tell them, every
step, which pairs of bodies overlap and exactly how — a point of contact, a
direction to push them apart along, and how deeply they've penetrated. Today
(post-M1) bodies fall under gravity and pass straight through each other and
the ground with no awareness that a collision occurred.

**Why this priority**: Without collision detection there is no physics
engine — gravity and integration alone just produce falling shapes.
Detecting contact is the prerequisite for every later milestone (impulse
resolution in M3, stacking in M4).

**Independent Test**: Drop a circle body onto a static circle body positioned
below it and step the world forward. Can be fully tested by inspecting the
contact data the engine reports once the circles' distance apart is less
than the sum of their radii, and confirming it disappears once they
separate again — delivers the core "the engine knows this happened" value on
its own, with no rendering required.

**Acceptance Scenarios**:

1. **Given** two circle bodies whose centers are closer together than the
   sum of their radii, **When** the world steps, **Then** the engine reports
   exactly one contact for that pair with a normal pointing from the first
   body toward the second and a penetration depth equal to the radii sum
   minus the center distance.
2. **Given** two circle bodies whose centers are farther apart than the sum
   of their radii, **When** the world steps, **Then** the engine reports no
   contact for that pair.
3. **Given** many bodies scattered across the world with only a few pairs
   whose bounding boxes overlap, **When** the world steps, **Then** only
   those overlapping pairs are tested for contact — pairs whose bounding
   boxes don't overlap are never reported as colliding.

---

### User Story 2 - Detect contact between circles and polygons, and between polygons (Priority: P2)

A developer needs collision detection to work for every shape combination
the engine supports, not just circle pairs — a ball landing on a box, or a
box landing on a ramp, must also be recognized as touching, with contact
information accurate enough to react to later.

**Why this priority**: The MVP acceptance demo is a box stack and a box on a
ramp — both depend on polygon–polygon and (for the bouncing-ball demo)
circle–polygon contact. Circle-only detection from Story 1 isn't sufficient
to reach the project's own milestone targets.

**Independent Test**: Can be fully tested by placing a circle body just
above/overlapping a static polygon edge and confirming a contact is
reported with the correct closest-edge normal, then separately placing two
overlapping polygon bodies (one rotated) and confirming a 1–2 point contact
manifold is produced along the correct reference face.

**Acceptance Scenarios**:

1. **Given** a circle body overlapping the flat face of a static polygon
   body, **When** the world steps, **Then** the engine reports one contact
   whose normal points away from that face and whose penetration equals the
   circle's radius minus the distance from its center to the face.
2. **Given** a circle body whose center is nearest to a polygon's corner
   rather than any single face, **When** the world steps, **Then** the
   contact normal points away from that corner rather than from either
   adjacent face.
3. **Given** two overlapping polygon bodies, one of them rotated to an
   arbitrary angle, **When** the world steps, **Then** the engine reports a
   contact manifold of one or two points lying on the shared overlap region,
   with a single normal aligned with the axis of minimum penetration.
4. **Given** a box resting on a ramp tilted at a shallow angle, **When** the
   scenario is rendered, **Then** the reported contact point(s) and normal
   visibly coincide with the true overlap between the box and the ramp
   surface.

---

### User Story 3 - See contact data to verify it's correct (Priority: P3)

A developer building and debugging the collision system needs a way to
visually confirm contact points and normals are being computed correctly,
without reading raw numbers out of a debugger.

**Why this priority**: Collision math is easy to get subtly wrong (sign
flips on normals, off-by-one clipping errors). A visual check is how the
milestone's own "done when" criterion is verified, but the engine works
without it — resolution (M3) will consume the same contact data
programmatically. Lowest priority because it's a diagnostic aid, not a
capability the later milestones depend on.

**Independent Test**: Run the existing example scene with two or more
bodies driven into contact and confirm each active contact's point and
normal are drawn on screen, updating every frame as the bodies move.

**Acceptance Scenarios**:

1. **Given** an example scene where two bodies are in contact, **When** the
   scene renders, **Then** every contact's point is drawn as a marker and
   its normal as a short line/arrow from that point.
2. **Given** a contact that stops existing (bodies separate), **When** the
   next frame renders, **Then** its marker and normal are no longer drawn.

---

### Edge Cases

- Two shapes exactly touching with zero penetration: the pair should not
  flicker between "contact" and "no contact" from floating-point noise at
  the boundary.
- A circle's center lands exactly on a polygon vertex (equidistant from two
  edges): the contact normal must still be well-defined (pointing away from
  that vertex).
- Two polygons deeply overlapping (e.g., one nearly inside the other): the
  reference/incident face selection and clipping must still produce a
  sane 1–2 point manifold rather than degenerating to zero points.
- A polygon body rotated so that none of its edges are axis-aligned: SAT
  axis testing and face clipping must use the body's current orientation,
  not its rest orientation.
- Bodies whose axis-aligned bounding boxes overlap but whose actual shapes
  do not: broadphase must pass these to narrowphase, and narrowphase must
  correctly report no contact.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: The engine MUST compute an axis-aligned bounding box for every
  body's shape and use it to reject non-overlapping body pairs before any
  narrowphase test runs.
- **FR-002**: The engine MUST detect circle–circle contact by comparing the
  distance between centers to the sum of the two radii, producing a single
  contact point, an outward normal, and a penetration depth when they
  overlap.
- **FR-003**: The engine MUST detect circle–polygon contact by finding the
  closest point on the polygon's boundary (edges and vertices) to the
  circle's center, producing a contact when that distance is less than the
  circle's radius.
- **FR-004**: The engine MUST detect polygon–polygon contact using the
  Separating Axis Theorem tested over both bodies' face normals, treating
  any axis with positive separation as proof the bodies don't overlap.
- **FR-005**: For an overlapping polygon pair, the engine MUST select the
  axis of minimum penetration to determine a reference face on one body and
  an incident face on the other, then clip the incident face against the
  reference face's side planes to produce a manifold of one or two contact
  points.
- **FR-006**: Every reported contact MUST carry a contact point, a unit
  normal, and a non-negative penetration depth.
- **FR-007**: Collision detection MUST run every fixed simulation step,
  independent of rendering, and MUST NOT alter any body's velocity or
  position — resolving contacts is out of scope for this milestone.
- **FR-008**: The engine MUST support collision detection between bodies at
  arbitrary orientation (not just axis-aligned), for every shape pair.
- **FR-009**: The example scenes MUST be able to draw each active contact's
  point and normal for visual verification.

### Key Entities

- **Shape**: The geometric form attached to a body — either a Circle
  (center, radius) or a Polygon (ordered vertices, face normals) — used to
  compute bounding boxes and run narrowphase tests. Already defined in M0;
  this feature is the first to consume it for collision math.
- **AABB (bounding box)**: The axis-aligned box enclosing a body's shape at
  its current position/orientation, used only to cheaply reject pairs
  before narrowphase.
- **Contact**: A single point of overlap between two bodies — position,
  outward normal, and penetration depth.
- **Manifold**: The set of one or more Contacts describing the full overlap
  region between one pair of bodies (a single point for circle pairs, up to
  two points for polygon pairs).
- **Candidate pair**: A pair of bodies whose AABBs overlap, queued for a
  narrowphase test; produced by broadphase.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: For each of the three shape-pair combinations (circle–circle,
  circle–polygon, polygon–polygon), a suite of known geometric test cases
  reports the mathematically correct contact normal and penetration depth
  to within 1e-4 units.
- **SC-002**: A box resting on a ramp tilted at an arbitrary angle shows
  contact points and normals, rendered on screen, that visibly coincide
  with the true overlap between the box and ramp — matching this
  milestone's own acceptance criterion.
- **SC-003**: In a scene with bodies scattered such that most pairs' AABBs
  do not overlap, none of those non-overlapping pairs is ever passed to a
  narrowphase test, confirmed by test coverage that counts narrowphase
  invocations.
- **SC-004**: No dropped or resting body's velocity changes as a result of a
  detected contact — collision detection alone produces no visible
  behavior change versus M1's free-fall behavior, other than the debug
  markers.

## Assumptions

- Only the two shapes already defined in M0/M1 (Circle, Polygon) need
  collision support; no other shape types are in scope.
- "Users" of this feature are developers consuming the `mirage` engine
  (directly or via the bundled examples) — there is no end-user-facing UI
  beyond the existing macroquad debug rendering.
- Debug drawing of contacts extends the existing example harness introduced
  in M1 (`examples/bouncing.rs` and friends); it does not require a new
  rendering target.
- Static bodies (e.g., a ramp or floor with infinite/zero inverse mass) are
  representable by the existing body type from M1 and participate in
  collision detection the same as dynamic bodies.
- Per the project's non-goals, collision detection covers only convex
  circle and polygon shapes with no continuous collision detection — fast
  bodies may tunnel through thin geometry in a single step, and that is
  expected, not a defect to fix in this milestone.
- Impulse resolution, restitution, and friction (reacting to a contact) are
  explicitly out of scope — this milestone only detects and reports
  contacts; M3 consumes that data.
