# Phase 0 Research: Shape COM Correction & Frictionless-Stack Limits (M5)

No `[NEEDS CLARIFICATION]` markers remain in the Technical Context (see
`plan.md`) — both design questions below were closed in the pre-specify
discussion (auto-recenter; document-and-test only, no velocity clamp) and
are recorded here as decisions, not open research.

## Q1: Centroid formula

**Decision**: Compute the polygon's centroid with the standard area-weighted
(shoelace) formula, using the same per-edge cross product `Shape::inertia`
already computes:

```
cᵢ = xᵢ·yᵢ₊₁ − xᵢ₊₁·yᵢ     (= pᵢ × pᵢ₊₁, already computed in inertia())
A  = ½ · Σ cᵢ                (signed area)
Cx = 1/(6A) · Σ (xᵢ + xᵢ₊₁) · cᵢ
Cy = 1/(6A) · Σ (yᵢ + yᵢ₊₁) · cᵢ
```

`Shape::polygon` computes `(Cx, Cy)`, then stores `vertices[i] - (Cx, Cy)`
instead of the raw input vertices. Edge normals are computed from the
(already-shifted) vertices as today — normals are translation-invariant, so
this ordering doesn't change their values, only where the code computes them
relative to the shift.

**Rationale**: This is the textbook centroid formula for a simple polygon,
uses the same cross-product convention (`p.cross(q)`) `inertia()` already
uses so the two formulas read side by side (Constitution III), and requires
no new math primitives beyond what `math.rs` already provides (`Vec2::cross`,
arithmetic).

**Alternatives considered**:
- *Validate-and-panic instead of recenter*: rejected in the pre-specify
  discussion — it closes the same trap for hand-centered input but reopens
  it for any future caller who gets the centering wrong, which is exactly
  the failure mode issue #7 describes. Auto-recentering is strictly more
  robust for the same amount of code.
- *Recenter at `RigidBody` construction time instead of `Shape::polygon`*:
  rejected — `Shape` is already tested and constructed standalone (see
  `src/shape.rs`'s `#[cfg(test)]` module), independent of any `RigidBody`.
  Fixing it in `Shape::polygon` keeps the invariant ("local `(0,0)` is the
  COM") enforced at the one place it's stated today, and keeps `Shape` a
  self-contained, independently correct type.

## Q2: Degenerate-input safety

**Decision**: Add test coverage for a small-but-valid convex polygon
(comparable in area to, e.g., a 1 mm² triangle at the engine's 1 m body
scale) confirming both the new centroid computation and the existing
`inertia()` integral stay finite. No new guard/clamp/error type is added
unless that test finds the existing (unchanged) division-by-`Σcᵢ` formula
already producing `NaN`/`Inf` within that range — which is not expected,
since a valid (non-self-intersecting, non-collinear, positive-area) input
keeps `Σcᵢ` bounded away from zero at the same order as the polygon's area.

**Rationale**: FR-005's contract is "no panic/`NaN`/`Inf` for input already
accepted" (i.e., convex, CCW, ≥ 3 vertices, positive area) — it doesn't
expand what counts as valid input. A genuinely zero-area (fully collinear or
self-intersecting) polygon was already unspecified behavior before this
feature and stays out of scope, matching the Edge Cases section of `spec.md`.

**Alternatives considered**:
- *`Result`-returning fallible constructor*: rejected as scope creep — no
  issue asked for it, and it would change `Shape::polygon`'s signature for
  every existing caller (examples, tests) for a case (degenerate input) the
  engine has never validated against and isn't asked to start validating now.

## Q3: Frictionless-stacking boundary numbers

**Decision**: Reuse the exact measurements already recorded in issue #11's
table (release build, `dt = 1/60`, unit boxes, `μ = 0` on both floor and
boxes, warm-starting on): a 2-box stack holds (max drift 0.0099 box widths
over 60 s, never collapses), a 5-box and a 10-box stack both collapse (drift
exceeds 0.5 box widths well before 60 s). The new test pins loosened versions
of these exact numbers (e.g., drift stays under 0.05 — the same `DRIFT_MAX`
used elsewhere — for the 2-box case, and drift exceeds 1.0 for the taller
case) so implementation-time floating-point noise doesn't make the test
flaky, while still failing if the boundary moves meaningfully (i.e., if a
future change accidentally stabilizes or further destabilizes frictionless
stacks, the pinned test catches it).

**Rationale**: The measurements are already validated evidence (issue #11),
and `tests/stacking.rs`'s existing tower tests establish the convention of
loosened-but-meaningful numeric bounds around measured behavior (e.g.
`SINK_STARTUP_MAX` vs. the measured ~15% peak). Reusing that convention here
keeps this test consistent with the rest of the suite.

**Alternatives considered**:
- *Assert the exact measured numbers*: rejected — over-fits to one measured
  run; the collapse is chaotic (small floating-point differences compound
  once bodies are airborne), so exact-value assertions would be fragile even
  though the *outcome* (holds vs. collapses) is reproducible and, per the
  existing `tower_is_bit_identical_across_runs` test, deterministic given
  identical inputs and iteration order.
