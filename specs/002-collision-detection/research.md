# Phase 0 Research: Collision Detection (M2)

No `NEEDS CLARIFICATION` markers remained in the Technical Context — README
§ "How it works" (Broadphase, Narrowphase) and the constitution already pin
the algorithms this milestone implements. This document records the design
decisions that still had more than one reasonable implementation.

## Decision: `Shape` attaches to `RigidBody` as an owned field, not a parallel store

- **Decision**: `RigidBody` gains `pub shape: Shape`; `new_dynamic`/
  `new_static` take a `Shape` argument alongside position (and mass, for
  `new_dynamic`).
- **Rationale**: `World` already owns and indexes `RigidBody` by `BodyId`
  from M1; a shape is inseparable per-body data (it's *what* the body is),
  not a separate simulation subsystem. Putting it on `RigidBody` keeps
  broadphase/narrowphase as pure functions over `&RigidBody` rather than
  needing a second lookup keyed by the same `BodyId`.
- **Alternatives considered**:
  - *Parallel `Vec<Shape>` in `World`, indexed by the same `BodyId`* —
    rejected: duplicates the indexing/storage `World` already manages for
    bodies, for no benefit at this project's scale, and risks the two
    vectors drifting out of sync (a body with no matching shape entry).

## Decision: `Shape` this milestone provides geometry + AABB + narrowphase support only — no area/inertia yet

- **Decision**: `Shape::Circle { radius }` and `Shape::Polygon { vertices,
  normals }` implement `aabb(position, orientation)` and the geometric
  queries narrowphase needs (closest point on a polygon boundary, support/
  farthest-point-in-direction, face normals). Area and moment-of-inertia
  computation are **not** added this milestone.
- **Rationale**: M1 already established that a dynamic body's mass is
  supplied directly (`new_dynamic(position, mass)`), not derived from
  shape + density; M2's spec (Key Entities, Functional Requirements) only
  calls for AABBs and narrowphase geometry, not mass properties. Adding
  inertia-from-shape now would be unrequested scope with no consumer until
  the solver needs it — README lists "area/inertia/AABB" as `shape.rs`'s
  eventual full responsibility, but nothing in M2's "done when" criterion or
  FRs exercises inertia.
- **Alternatives considered**:
  - *Implement area/inertia now since README's architecture table mentions
    them for `shape.rs`* — rejected: would be built and left unused,
    violating the project's "don't design for hypothetical future
    requirements" guidance; revisit when a milestone (M3/M4) actually
    consumes it.

## Decision: Broadphase returns candidate pairs; it does not itself call narrowphase

- **Decision**: `broadphase.rs` exposes a function that takes the current
  bodies and returns `Vec<(BodyId, BodyId)>` (or an iterator) of pairs whose
  AABBs overlap. `World::step` (or a `collision::detect_contacts` entry
  point it calls) is what feeds those pairs into narrowphase dispatch.
- **Rationale**: Matches the README's own data-flow description ("broadphase
  produces candidate pairs → narrowphase produces Contact/Manifold") and
  keeps `broadphase.rs` a pure, independently testable function per
  Constitution "Technical Constraints" module boundaries — SC-003 (no
  non-overlapping pair ever reaches narrowphase) is directly testable
  against this function alone.
- **Alternatives considered**:
  - *Broadphase directly invokes narrowphase per accepted pair, returning
    contacts* — rejected: collapses two independently-testable concerns
    into one function and makes it harder to unit-test "which pairs were
    even considered" (SC-003) separately from "was the resulting contact
    correct" (SC-001).

## Decision: `World` stores the current step's contacts and exposes them read-only

- **Decision**: `World` gains a `contacts: Vec<Contact>` (or similar) field,
  fully recomputed (cleared and repopulated) once per fixed substep inside
  `step`, and a `World::contacts(&self) -> &[Contact]` accessor. Each
  `Contact` carries the pair's `BodyId`s plus its manifold data.
- **Rationale**: User Story 3 / FR-009 need a way for example code to draw
  the current contacts; a read-only accessor mirrors the existing
  `World::body(id)` pattern from M1 and needs no new dependency. Recomputing
  fully each substep (rather than incrementally diffing) matches
  Constitution Principle II's determinism requirement — the same inputs
  always regenerate the same contact set, with no stale carry-over.
- **Alternatives considered**:
  - *Return contacts from `step` instead of storing them* — rejected: with
    multiple fixed substeps potentially running inside one `step(dt_real)`
    call (M1's accumulator can drain more than one step), only the final
    substep's contacts are meaningful for the current visual frame; storing
    the latest set on `World` and exposing an accessor is simpler than
    threading a `Vec<Vec<Contact>>` back through the call.

## Decision: Polygon narrowphase order — SAT axis test, then reference/incident face selection, then clip

- **Decision**: Polygon–polygon narrowphase (`collision/polygon.rs`) tests
  every face normal of body A, then every face normal of body B, tracking
  the axis of *least* penetration (most positive separation among
  overlapping axes); if any axis shows positive separation, there's no
  contact. The winning axis's body supplies the reference face; the other
  body's face most anti-parallel to that axis's normal is the incident
  face; the incident face's two endpoints are clipped against the reference
  face's two side planes, and any clipped point still behind the reference
  face becomes a manifold contact.
- **Rationale**: This is the exact algorithm README § Narrowphase describes
  for polygon–polygon, and the standard Box2D-Lite/Sutherland–Hodgman-style
  formulation this project's References already point to (Constitution
  Principle III: implement the derivation as written, don't invent a
  different formulation).
- **Alternatives considered**: None considered — the README pins this
  algorithm exactly; Principle III leaves no room for an alternate
  narrowphase formulation.

## Decision: Circle–polygon closest point is computed per-edge, with a vertex-region fallback

- **Decision**: `collision/circle.rs`'s circle–polygon test projects the
  circle center onto each polygon edge (clamped to the edge's segment),
  tracks the closest such point, and if the center lies "outside" every
  edge's region entirely (nearest a vertex rather than any edge's interior),
  the contact normal points away from that vertex directly, per spec Edge
  Cases.
- **Rationale**: Directly satisfies spec Acceptance Scenario 2 (circle
  nearest a polygon corner) and the corresponding edge case; this is the
  standard closest-point-on-convex-polygon construction referenced in the
  project's cited sources (Randy Gaul's impulse-engine writeups).
- **Alternatives considered**:
  - *Only test face normals, ignore vertex regions* — rejected: produces an
    incorrect normal (and possibly a false negative) exactly in the corner
    case the spec calls out as an edge case to get right.

## Decision: A small epsilon/slop guards the "exactly touching" boundary

- **Decision**: Overlap tests treat penetration `<= 0` (or below a tiny
  epsilon, matching the `f32::EPSILON`-style tolerance already used in
  `math.rs`'s `normalize`) as "no contact," so bodies at the exact boundary
  don't flicker between contact/no-contact from floating-point noise.
- **Rationale**: Directly satisfies the spec's first Edge Case (exact-
  touching flicker) and SC-001's stated 1e-4 tolerance; mirrors the existing
  epsilon convention `math.rs` already established for `Vec2::normalize`,
  keeping the codebase's tolerance handling consistent (Principle III's
  "readable over clever" spirit — reuse the established convention rather
  than inventing a new one).
- **Alternatives considered**:
  - *No epsilon, exact `< `/`>` comparisons* — rejected: this is precisely
    what produces contact-state flicker at the boundary, which the spec
    calls out explicitly as an edge case to avoid.

## Decision: Contact normal direction is shape-pair-specific, not a fixed body-index convention

- **Decision**: For circle–circle, `Contact.normal` points from the pair's
  first body toward its second (FR-002, literal). For any pair involving a
  polygon (circle–polygon or polygon–polygon), `Contact.normal` instead
  always points away from the polygon's touching face/vertex toward the
  other shape — independent of which body happens to be first/second by
  `BodyId`/array order.
- **Rationale**: Spec US2 Acceptance Scenario 1 states unconditionally that
  a circle overlapping a polygon's face produces a normal "pointing away
  from that face" — this must hold regardless of which body was added to
  the `World` first. A strict "always first-body-to-second-body" rule
  (as originally sketched in `data-model.md`'s first draft) is
  un-satisfiable simultaneously with that scenario whenever the circle
  happens to be added before the polygon. Making the polygon-involving
  direction depend on the *shape*, not array order, resolves the conflict
  and matches the well-known reference-implementation pattern (Randy
  Gaul's impulse-engine circle/polygon dispatch, README References):
  `PolygonToCircle` collision is computed by calling the same
  `CircleToPolygon` routine with arguments swapped and using its result
  as-is — not by re-deriving a body-index-relative sign.
- **Alternatives considered**:
  - *Strict body_a → body_b normal for every shape pair, flipping when the
    polygon ends up first* — rejected: this is exactly the convention that
    conflicts with the spec's own literal acceptance scenario wording; it
    would make "does the normal point away from the face" depend on
    insertion order, which is not a property the spec describes as
    conditional.
  - *Track reference/incident roles explicitly for circle–polygon (mirroring
    polygon–polygon's reference/incident face selection)* — unnecessary:
    circle–polygon only ever has the polygon supplying the reference face
    (a circle has no faces), so there is no ambiguity to resolve the way
    polygon–polygon's SAT axis choice has.
- **Consequence for polygon–polygon**: by the same reasoning, `Contact.normal`
  for a polygon pair points from the reference face's body toward the
  incident body (the standard SAT/clipping convention), again independent
  of `BodyId` array order — not flipped to force a fixed body_a → body_b
  reading.

## Output

All Technical Context fields are resolved (none were `NEEDS
CLARIFICATION`); the decisions above are the design choices Phase 1 needed
before writing `data-model.md` and the API contract.
