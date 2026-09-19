# Phase 0 Research: Impulse Resolution (M3)

No `NEEDS CLARIFICATION` markers remained in the Technical Context — README
§ Resolution pins the algorithm. This records the decisions that still had
more than one reasonable implementation, plus two defects found in M1/M2
code while reading it for this plan.

## Finding: `(Circle, Polygon)` contact normal points `b → a`

- **Observed**: `circle_vs_polygon` returns a normal pointing from the
  polygon toward the circle. `detect_contacts` passes it through unchanged
  for `(Circle, Polygon)` pairs, where `a` is the circle and `b` the
  polygon — so the normal points from `b` to `a`. The `Manifold`/`Contact`
  docs and README's `vr = v_b … − v_a …` formulation both assume `a → b`.
  `(Polygon, Circle)` and both other pair types are already `a → b`.
  The M2 doc comment on `detect_contacts` claims the polygon case "needs no
  flip", which is only true for one of the two orderings.
- **Decision**: Negate the normal in the `(Circle, Polygon)` arm of
  `detect_contacts`; update its doc comment; add a dispatch test asserting
  `a → b` for every ordering of every pair type.
- **Why it matters**: with the wrong sign the solver would compute a
  closing velocity as separating and skip the contact (ball passes through
  a floor), or worse, pull bodies together.
- **Alternatives considered**: flip inside the solver by checking shape
  types — rejected; the invariant belongs to the contact producer, and the
  debug-draw example (`ramp.rs`) already treats normals uniformly.

## Finding: angular velocity is never integrated

- **Observed**: `World::step` integrates only `velocity → position`.
  `angular_velocity` and `orientation` exist on `RigidBody` but are never
  updated, though README § Integration says "the same pair of updates for
  angular velocity ω and angle θ".
- **Decision**: Integrate `θ += ω·dt` (semi-implicit: after impulses have
  updated `ω`) via `Rot2::new(orientation.angle() + ω·dt)`. Gravity exerts
  no torque, so `ω` only changes through impulses.
- **Alternatives considered**: leave orientation frozen and only track `ω`
  — rejected: FR-003 requires angular response, and a spinning body that
  never visibly rotates would fail the behavioral-verification principle.

## Decision: inertia is derived from `Shape` and mass

- **Decision**: `Shape::inertia(mass)` returns the moment of inertia about
  the local origin. Circle: `½·m·r²`. Polygon:
  `I = m / (6·Σcᵢ) · Σ cᵢ·(pᵢ·pᵢ + pᵢ·pᵢ₊₁ + pᵢ₊₁·pᵢ₊₁)` with
  `cᵢ = pᵢ × pᵢ₊₁`. Verified by hand on the half-extent-1 square:
  `2m/3`. `RigidBody::new_dynamic` stores `inv_inertia = 1 / inertia`;
  static bodies get `0.0`.
- **Assumption**: polygon vertices are authored around the body's center
  of mass (as every example does). Computing about the origin is only
  correct under that assumption; recentring polygons is not done here.
- **Rationale**: M2 deliberately deferred inertia until a consumer
  existed; the solver is that consumer. Mass stays user-supplied (M1
  decision), so no density/area API is added.
- **Alternatives considered**: take inertia as a constructor argument —
  rejected: error-prone for callers and forces every example to hand-derive
  it; README lists inertia under `shape.rs`.

## Decision: restitution combine rule is `max(e_a, e_b)`

- **Decision**: `e = max(e_a, e_b)`. Order-independent (FR-006).
- **Rationale**: a ball with `e = 1` dropped on a default floor
  (`e = 0`) must bounce — the README's "done when" describes the ball's
  restitution only. Product would force every floor to be set to `1.0`.
  Box2D uses the same rule.
- **Alternatives considered**: product — rejected for the reason above;
  average — rejected: a `0.5` floor would spoil a `0.0` bean bag.
- **Default**: bodies are created with `restitution = 0.0`;
  `with_restitution(e)` sets it, clamped to `[0, 1]`.

## Decision: iterated impulse with accumulated clamping and a fixed bounce target

- **Decision**: Per contact point, before iterating, compute
  `vn₀ = vr · n` and the target `bounce = −e·vn₀` (only if `vn₀ < 0`
  and the closing speed exceeds the resting threshold, else `0`).
  Each iteration recomputes `vn = vr · n` and
  `Δj = −(vn − bounce) / K`, where
  `K = 1/m_a + 1/m_b + (r_a×n)²/I_a + (r_b×n)²/I_b`. The running total
  `Σj` is clamped to `≥ 0` and only the change is applied. On the first
  iteration with a single contact this equals README's
  `j = −(1+e)(vr·n)/K` exactly.
- **Rationale**: re-applying `(1+e)` every iteration would over-bounce;
  clamping the accumulated impulse (not each `Δj`) lets later iterations
  undo an earlier over-correction without ever pulling bodies together
  (FR-005). This is the standard Catto formulation from the README's
  references.
- **Alternatives considered**: one-shot `j` per contact with no
  iteration — rejected: fails FR-004 and cannot handle a box's two contact
  points or any stack.

## Decision: restitution resting threshold `|g|·dt`

- **Decision**: if the closing speed `−vn₀ < |g|·dt + ε`, use `e = 0` for
  that contact. The solver receives gravity and `dt` for this.
- **Rationale**: a resting body gains `g·dt` of closing speed every step;
  with `e = 1` an unthresholded solver bounces it back each step, causing
  perpetual micro-hopping. Tying the threshold to gravity keeps it
  scale-independent (demo world units are pixels, not metres). Satisfies
  the "settle to rest" edge case.
- **Alternatives considered**: fixed 1.0 m/s threshold (Box2D) —
  rejected: meaningless when a world unit is a pixel.

## Decision: positional correction is a slop-gated linear projection, separate from velocity

- **Decision**: after velocity iterations, for each contact point move
  bodies apart along `n` by
  `percent · share · max(penetration − slop, 0) / (1/m_a + 1/m_b)`,
  weighted by each body's `inv_mass`, where `share = 1 / (points in the
  manifold)` so a flat box's two contact points together correct it no
  harder than a single point would. `percent = 0.4`, `slop = 0.01` (in world
  units). Static bodies move by `0`.
- **Rationale**: a velocity-bias form of Baumgarte injects energy into the
  normal velocity and inflates `e = 1` rebound height, threatening SC-001.
  Projecting positions keeps the velocity solve energy-clean while still
  being the "Baumgarte-style bias with slop" FR-008 and README describe.
  The slop keeps resting contacts from toggling.
- **Alternatives considered**: velocity-bias Baumgarte (Box2D-Lite) —
  rejected for the energy reason above; full split-impulse — rejected as
  more machinery than M3 needs (revisit in M4 if stacks jitter).
- **Note**: M2's narrowphase already discards penetration below its own
  `SLOP` (`collision/circle.rs`), so correction and detection thresholds
  should be kept consistent; the solver's slop is a separate constant.

## Decision: positional correction uses stored penetration, one pass

- **Decision**: correction runs once per step from the penetrations the
  manifold reported at detection time; no re-detection between iterations.
- **Rationale**: simple, deterministic, and adequate for the M3 criteria;
  tall-stack accuracy is M4's concern.

## Decision: step order follows CLAUDE.md data flow

- **Decision**: apply gravity to velocity → detect contacts → solve →
  correct positions → integrate positions. M1/M2 integrated positions
  first and detected afterwards.
- **Rationale**: FR-001 requires resolution after detection and before
  position integration, and CLAUDE.md states this order. Semi-implicit
  Euler is preserved: position uses the final post-impulse velocity.
- **Consequence**: `World::contacts()` now reports contacts from the
  pre-integration state of the step. The M2 test
  `detecting_a_contact_does_not_alter_body_velocity_or_position` encodes
  M2's behavior and is replaced by resolution tests.
