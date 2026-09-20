---
description: "Task list for M3: Impulse Resolution"
---

# Tasks: Impulse Resolution (M3)

**Input**: Design documents from `/specs/003-impulse-resolution/`

**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/engine-api.md, quickstart.md

**Tests**: Included. The spec's Success Criteria (SC-001–SC-007) and the constitution (Principle IV: behaviorally verified; Principle III: spec-faithful math) require `cargo test` coverage of the solver, following the existing inline `#[cfg(test)] mod tests` convention. No separate `tests/` directory is used.

**Organization**: Tasks are grouped by user story (from spec.md) to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies on incomplete tasks)
- **[Story]**: Which user story this task belongs to (US1–US4)
- Exact file paths are included in every task

## Path Conventions

Single project (per plan.md): `src/` and `examples/` at the repository root; unit tests are inline per module. All `cargo` commands run from the repo root. Never push to `main`; all work stays on branch `003-impulse-resolution`.

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Create the empty solver module and confirm the M2 baseline is green.

- [X] T001 Run `cargo test` on branch `003-impulse-resolution` and confirm the M2 baseline passes before any change; record the test count.
- [X] T002 Create `src/solver.rs` containing only a module doc comment (`//! Sequential-impulse solver — README § Resolution.`) and the three constants from data-model.md: `VELOCITY_ITERATIONS: usize = 8`, `PENETRATION_SLOP: f32 = 0.01`, `CORRECTION_PERCENT: f32 = 0.4`, each with a one-line doc comment.
- [X] T003 Add `pub mod solver;` to `src/lib.rs` after `pub mod shape;`, following the existing declaration order. Do NOT re-export anything from it. Depends on T002.

**Checkpoint**: Crate compiles; `cargo test` count unchanged.

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Fix the two M1/M2 defects found in research.md and add the body/shape data every story needs. No story can be correct without these.

**⚠️ CRITICAL**: No user story work can begin until this phase is complete.

- [X] T004 In `src/collision/mod.rs`, negate the contact normal in the `(Shape::Circle, Shape::Polygon)` arm of `detect_contacts` so it points from `body_a` (circle) toward `body_b` (polygon), and rewrite the function's doc comment to state the invariant: `Contact.normal` always points from `body_a` toward `body_b` for every shape-pair ordering. Leave the `(Polygon, Circle)` arm unchanged. (Research: "`(Circle, Polygon)` contact normal points `b → a`".)
- [X] T005 Add tests to the `#[cfg(test)]` module of `src/collision/mod.rs` asserting `(body_b.position − body_a.position) · normal > 0` for overlapping pairs in every ordering: circle–circle, circle-then-polygon, polygon-then-circle, polygon–polygon (one rotated). Confirm the circle-then-polygon test fails without T004's fix. Depends on T004.
- [X] T006 Fix any existing tests or `examples/ramp.rs` code that relied on the reversed `(Circle, Polygon)` normal (grep for `normal` in `src/collision/` tests and `examples/ramp.rs`); `cargo test` must pass. Depends on T004.
- [X] T007 [P] Add `pub fn inertia(&self, mass: f32) -> f32` to `impl Shape` in `src/shape.rs`: circle `0.5 * mass * radius²`; polygon `mass / (6·Σcᵢ) · Σ cᵢ·(pᵢ·pᵢ + pᵢ·pᵢ₊₁ + pᵢ₊₁·pᵢ₊₁)` with `cᵢ = pᵢ.cross(pᵢ₊₁)`, using local-space vertices about the origin. Doc comment states the assumption that polygon vertices are centered on the center of mass.
- [X] T008 Add tests in `src/shape.rs` for `inertia`: unit circle mass 1 → `0.5`; half-extent-1 square mass 3 → `2.0` (`2m/3`); a 2×4 rectangle matches `m·(w²+h²)/12` within 1e-4. Depends on T007.
- [X] T009 Add `pub restitution: f32` and `pub inv_inertia: f32` to `RigidBody` in `src/body.rs`. In `new_dynamic` set `inv_inertia = 1.0 / shape.inertia(mass)` (compute before `shape` is moved) and `restitution = 0.0`; in `new_static` set both `inv_inertia = 0.0` and `restitution = 0.0`. Add `pub fn with_restitution(mut self, e: f32) -> Self` that stores `e.clamp(0.0, 1.0)`. Depends on T007.
- [X] T010 Add tests in `src/body.rs`: `new_dynamic` circle radius 2 mass 4 has `inv_inertia == 1/8`; `new_static` has `inv_mass == 0.0` and `inv_inertia == 0.0`; default `restitution == 0.0`; `with_restitution(1.5)` → `1.0`, `with_restitution(-0.2)` → `0.0`. Depends on T009.
- [X] T011 Add a `pub(crate) fn resolve(bodies: &mut [RigidBody], manifolds: &[Manifold], gravity: Vec2, dt: f32)` stub to `src/solver.rs` (empty body, `let _ = (...)` to silence unused warnings) matching contracts/engine-api.md. Depends on T002.
- [X] T012 In `src/world.rs`, reorder `World::step`'s fixed-substep loop to the plan's Step Order: (1) `v += gravity * FIXED_DT` for dynamic bodies; (2) `let manifolds = collision::detect_contacts(&self.bodies)`; (3) `solver::resolve(&mut self.bodies, &manifolds, self.gravity, FIXED_DT)`; (4) for dynamic bodies `position += velocity * FIXED_DT` and `orientation = Rot2::new(orientation.angle() + angular_velocity * FIXED_DT)`; (5) `self.contacts = manifolds`. Static bodies stay untouched. Update the `step` and `contacts` doc comments (resolution now happens; `contacts()` reflects the pre-integration state of the last substep). Depends on T004, T011.
- [X] T013 In `src/world.rs` tests, replace `detecting_a_contact_does_not_alter_body_velocity_or_position` (an M2-only behavior) with `contacts_are_still_reported_after_resolution_is_wired_in`: an overlapping dynamic/static pair still yields a non-empty `world.contacts()` after one step. Also add `angular_velocity_rotates_orientation`: a body with `angular_velocity = 1.0` after 60 steps has `orientation.angle()` ≈ 1.0 within 1e-3. Update the `use` line for `Rot2` if needed. Depends on T012.
- [X] T014 [P] Update `README.md` § Resolution BEFORE any solver code is written (Constitution III: the README derivation is updated first, in the same change): document the iterated form of `j` (target normal velocity `−e·vr₀·n` fixed before iterating, accumulated impulse clamped ≥ 0, equal to `j = −(1+e)(vr·n)/K` on the first iteration of a single contact), the `max(e_a, e_b)` combine rule, restitution measured from the approach velocity before the step's gravity kick (revised during implementation from a `|g|·dt` resting threshold, which left an e = 0.8 ball in a permanent micro-hop; see research.md), and positional correction as a slop-gated projection with `percent` and `slop`, split evenly across a manifold's points. Keep variable names `e`, `μ`, `j`, `vr`.

**Checkpoint**: `cargo test` passes; normals are `a → b`; README § Resolution matches the planned solver math; bodies carry restitution and inertia; step order matches the plan; no velocity response yet.

---

## Phase 3: User Story 1 - Bodies stop passing through each other (Priority: P1) 🎯 MVP

**Goal**: Contacts produce normal impulses so a falling body stops at a surface, with `e = 0` behavior, correct angular response, and static bodies untouched.

**Independent Test**: Drop a dynamic circle onto a static floor with default restitution; it stops dead on the surface and stays there.

### Tests for User Story 1

- [X] T015 [P] [US1] In `src/solver.rs` tests, add a helper that builds a `World` with a large static polygon floor (top surface at `y = 0`) and a dynamic circle above it, parameterized by body order (`floor_first: bool`) so every floor test runs with the circle added before the polygon AND after it — the `(Circle, Polygon)` ordering is where M2 returned a reversed normal. Also add a `run(world, steps)` helper using `FIXED_DT`-sized `step` calls; make `FIXED_DT` in `src/world.rs` `pub(crate) const` so tests can use it.
- [X] T016 [US1] Add `e0_ball_stops_dead_on_floor` in `src/solver.rs` tests: ball (default `e = 0`) dropped from a height; after landing, `velocity.y.abs() < 1e-3 × impact_speed` on the step after first contact and the ball never rises above its landing height + slop afterwards. Run for both body orders. Covers US1-1, SC-002. Depends on T015.
- [X] T017 [US1] Add `ball_settles_on_floor_without_falling_through` in `src/solver.rs` tests: after landing, run 5 simulated seconds (300 steps); the ball never ends up below the floor surface by more than its landing overshoot (`impact_speed × dt`) plus 1e-3, and never rises. Run for both body orders. Position correction is not required to pass this; the strict 10 s resting bound is T035. Covers US1-2.
- [X] T018 [P] [US1] Add `head_on_dynamic_bodies_conserve_momentum_along_normal` in `src/solver.rs` tests: two dynamic circles, masses 1 and 3, approaching head-on with `e = 0`; total momentum along the normal is unchanged within 1e-4 and the lighter body's velocity changes more. Covers US1-3, SC-005.
- [X] T019 [P] [US1] Add `separating_contact_is_left_alone` in `src/solver.rs` tests: two overlapping bodies already moving apart keep exactly their velocities after `solver::resolve`. Covers US1-4.
- [X] T020 [P] [US1] Add `static_bodies_never_change_and_static_pair_is_ignored` in `src/solver.rs` tests: a static body in contact with a dynamic one has bit-identical position and velocity after many steps; a contact between two static bodies causes no panic and no change (no divide by zero). Covers US1-5, FR-007, SC-007.
- [X] T021 [P] [US1] Add `accumulated_normal_impulse_is_never_negative` in `src/solver.rs` tests: give `resolve` an overlapping pair whose bodies are separating in one contact point of a two-point manifold and closing in the other, then assert (by exposing the total impulse to the test through a `#[cfg(test)]` accessor or by asserting neither body gains velocity toward the other) that no contact ever pulls the bodies together. Confirm the test fails if the `.max(0.0)` clamp in T025 is removed. Covers FR-005.
- [X] T022 [P] [US1] Add `off_center_polygon_hit_spins_the_body` in `src/solver.rs` tests: a dynamic box falling with its corner over a static floor point (rotated 30°) acquires nonzero `angular_velocity` with the sign matching the lever-arm cross product; a box landing flat gains ~0 angular velocity and does not tip. Covers the angular and two-point-manifold edge cases.
- [X] T023 [P] [US1] Add `identical_scenes_are_bit_identical` in `src/solver.rs` tests: run the same multi-body scene twice for 300 steps and assert `position`, `velocity`, `orientation`, `angular_velocity` are equal via `==` for every body. Covers SC-006, FR-009.

### Implementation for User Story 1

- [X] T024 [US1] In `src/solver.rs`, define a private `ContactState` struct with the fields from data-model.md (`a`, `b`, `n`, `r_a`, `r_b`, `k`, `bounce`, `j_acc`, `penetration`) and a private `build_contacts(bodies: &[RigidBody], manifolds: &[Manifold]) -> Vec<ContactState>` that iterates manifolds then points in order, computes `r_a = point − a.position`, `r_b = point − b.position`, and `k = inv_mass_a + inv_mass_b + (r_a×n)²·inv_inertia_a + (r_b×n)²·inv_inertia_b` using `Vec2::cross`, and drops entries with `k == 0.0` (both static). Set `bounce = 0.0` for now. Name locals after the README (`n`, `r_a`, `r_b`, `vr`, `j`). Also record `share = 1.0 / manifold.points.len() as f32` on each entry (used by position correction). Depends on T009, T011, T014 (README updated first, per Constitution III).
- [X] T025 [US1] In `src/solver.rs`, implement the velocity iteration inside `resolve`: repeat `VELOCITY_ITERATIONS` times over `ContactState`s in order; for each compute `vr = (v_b + Vec2::cross_sv(ω_b, r_b)) − (v_a + Vec2::cross_sv(ω_a, r_a))` and `vn = vr.dot(n)`; `Δj = −(vn − bounce) / k`; `new_acc = (j_acc + Δj).max(0.0)`; `j = new_acc − j_acc`; store `j_acc = new_acc`; apply `v_a −= n·j·inv_mass_a`, `ω_a −= r_a.cross(n·j)·inv_inertia_a`, `v_b += n·j·inv_mass_b`, `ω_b += r_b.cross(n·j)·inv_inertia_b`. Static bodies have zero inverse mass/inertia so they are unaffected; additionally skip writes to bodies with `is_static`. Add a comment block quoting the README `vr` and `j` formulas next to the code. Depends on T024.
- [X] T026 [US1] Run `cargo test` and make T016–T023 pass; fix solver bugs, not the tests. (If a test's tolerance is wrong, justify the change in a comment.) Depends on T025.

**Checkpoint**: A ball with default restitution stops on a floor; momentum, static-body and determinism properties hold. US1 is independently demonstrable and testable — this is the MVP.

---

## Phase 4: User Story 2 - Bounce according to restitution (Priority: P2)

**Goal**: Per-body restitution controls rebound from fully inelastic to fully elastic, with an order-independent combine rule and no perpetual micro-hopping at rest.

**Independent Test**: Drop a ball from a known height with `e = 1.0`; first rebound peak is within 5% of the drop height. Repeat with `e = 0`.

### Tests for User Story 2

- [X] T027 [P] [US2] Add `e1_ball_returns_to_drop_height` in `src/solver.rs` tests: ball `with_restitution(1.0)` dropped from a known height onto the static floor; track `position.y` and assert the first rebound peak is within 5% of the drop height (measured center-to-center, accounting for radius). Covers US2-1, SC-001.
- [X] T028 [P] [US2] Add `e_half_ball_rebounds_at_half_speed` in `src/solver.rs` tests: with `e = 0.5`, the velocity immediately after the first contact step is within 5% of `0.5 ×` the pre-contact impact speed, and peak rebound height ≈ one quarter of the drop height (±10%). Covers US2-3, SC-003.
- [X] T029 [P] [US2] Add `restitution_combine_is_order_independent` in `src/solver.rs` tests: for bodies with `e = 0.2` and `e = 0.9`, the post-collision velocities are identical (bit-equal) whichever body is added to the world first; and a ball with `e = 1.0` on a default (`e = 0.0`) floor still bounces. Covers US2-4, FR-006.
- [X] T030 [P] [US2] Add `high_restitution_ball_settles_instead_of_hopping_forever` in `src/solver.rs` tests: an `e = 0.8` ball dropped from a moderate height and run for 20 simulated seconds ends with `|velocity.y|` below `|g|·dt` and stays there for the last 2 seconds. Covers the low-speed-impact edge case.

### Implementation for User Story 2

- [X] T031 [US2] In `src/solver.rs` `build_contacts`, add `gravity: Vec2` and `dt: f32` parameters (update `resolve`'s call). After computing `vr` and `vn₀ = vr·n` from pre-iteration velocities, set `e = a.restitution.max(b.restitution)`; remove this step's own gravity kick from it (`vn_approach = vn₀ − (g·dt)·n · (b_dynamic − a_dynamic)`, research.md "restitution uses the pre-gravity approach velocity"); if `−vn_approach > 1e-4` set `bounce = −e * vn_approach`, else `bounce = 0.0`. Comment: on the first iteration with one contact this equals the README `j = −(1+e)(vr·n)/K`. Depends on T025.
- [X] T032 [US2] Run `cargo test` and make T027–T030 pass. If the `e = 1` rebound is outside 5%, first check the approach-velocity handling and the step order (velocity must be final before position integration) before changing tolerances. Depends on T031.

**Checkpoint**: US1 and US2 both work: `e = 0` stops dead, `e = 1` returns to ≈ drop height, intermediate values scale, resting bodies settle.

---

## Phase 5: User Story 3 - Resolved overlap doesn't linger or jitter (Priority: P3)

**Goal**: Overlap beyond slop is gently projected apart; resting contacts neither sink nor jitter.

**Independent Test**: Rest a box on a static floor for 10 simulated seconds; penetration stays below a small tolerance with no frame-to-frame jitter.

### Tests for User Story 3

- [X] T033 [P] [US3] Add `overlap_shrinks_without_pop` in `src/solver.rs` tests (include a flat box on a floor so the two-point manifold is exercised, asserting its per-step correction is no larger than the one-point case for the same penetration): two bodies overlapping by 0.2 (zero velocity, no gravity influence — use a world/gravity setup that isolates it or call `solver::resolve` directly) end each step with strictly smaller penetration and no step reduces it by more than `CORRECTION_PERCENT × (penetration − slop)` plus tolerance. Covers US3-1.
- [X] T034 [P] [US3] Add `overlap_within_slop_is_not_corrected` in `src/solver.rs` tests: two bodies overlapping by `PENETRATION_SLOP / 2` have bit-identical positions after `resolve`. Covers US3-3.
- [X] T035 [P] [US3] Add `box_rests_for_ten_seconds_without_sinking_or_jitter` in `src/solver.rs` tests: a dynamic box on a static floor for 600 steps; after the first 60 steps, penetration ≤ `2 × PENETRATION_SLOP` and the max per-step change in `position.y` is under 1e-3. Covers US1-2 (strict form), US3-2, SC-004.

### Implementation for User Story 3

- [X] T036 [US3] In `src/solver.rs`, add `fn correct_positions(bodies: &mut [RigidBody], contacts: &[ContactState])` implementing `correction = max(penetration − PENETRATION_SLOP, 0) / (inv_mass_a + inv_mass_b) · CORRECTION_PERCENT · share` (`share = 1/points`, so a two-point manifold is corrected as strongly as a one-point one, not twice as hard), then `position_a −= n · correction · inv_mass_a` and `position_b += n · correction · inv_mass_b`; skip static bodies' writes. Call it at the end of `resolve`, after the velocity iterations. Doc comment explains it is a Baumgarte-style bias applied as a position projection (research.md). Depends on T025.
- [X] T037 [US3] Run `cargo test` and make T033–T035 pass. Depends on T036.

**Checkpoint**: Resting bodies stay steady on the floor; US1–US3 all pass together.

---

## Phase 6: User Story 4 - Watch the behavior in the demo (Priority: P4)

**Goal**: The bouncing example visibly demonstrates resolution and doubles as the Constitution-IV behavioral verification.

**Independent Test**: `cargo run --example bouncing --release` shows balls landing on a floor and bouncing or stopping by restitution.

### Implementation for User Story 4

- [X] T038 [US4] Update `examples/bouncing.rs`: add a wide static polygon floor (e.g. `Shape::polygon` box, half-extents ~300×20, positioned so its top surface is visible near the bottom of the window in the existing `screen_y = screen_height() − y` mapping); give the five circles restitution `[0.0, 0.3, 0.6, 0.85, 1.0]` via `with_restitution`; choose drop heights (~150–250 units) so the first bounce is observable within a few seconds at the default gravity; draw the floor with macroquad; update the file's module doc comment from "no collision yet" to describe M3. Keep all macroquad calls in the example (engine stays render-free).
- [X] T039 [US4] Run `cargo run --example bouncing --release` and verify by observation, per quickstart.md § 2: no ball falls through the floor; the `e = 0` ball stops dead; the `e = 1` ball returns to within a few percent of its drop height; intermediate balls rebound proportionally lower; nothing jitters at rest. Record what was observed in the T044 commit/PR description. Depends on T038 and T037.

**Checkpoint**: Milestone "done when" criterion demonstrated by running the example.

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Keep docs consistent with code (Constitution III) and confirm nothing regressed or crept out of scope.

- [X] T040 [P] In `README.md`, tick the M3 milestone box and update the Status section to reflect that impulse resolution has landed. Keep variable names `e`, `j`, `vr` wherever they appear.
- [X] T041 [P] Update `CLAUDE.md` "Data flow per `World::step`" paragraph only if it no longer matches the implemented step order (gravity → detect → solve → correct → integrate); keep the SPECKIT block untouched.
- [X] T042 Run `cargo test`, `cargo build --release`, and `cargo build --examples --release`; confirm zero warnings introduced and zero new dependencies (`Cargo.toml` `[dependencies]` still empty, no `unsafe` — `grep -rn "unsafe" src` returns nothing).
- [X] T043 Run `cargo run --example ramp --release` and confirm contacts still debug-draw correctly with the corrected normals and that the box now reacts to the ramp (it slides — expected without friction; do NOT add friction). Confirm nothing from the non-goals list or M4 (tangent impulses, warm-starting) was added.
- [ ] T044 Commit on branch `003-impulse-resolution` and open a pull request to `main` (never push to `main` directly), with the T039 observations in the description.

---

## Dependencies & Execution Order

### Phase dependencies

- **Setup (Phase 1)**: no dependencies — start immediately.
- **Foundational (Phase 2)**: depends on Setup; **blocks all user stories**.
- **US1 (Phase 3)**: depends on Foundational. MVP.
- **US2 (Phase 4)**: depends on US1 (extends the same iteration loop with `bounce`).
- **US3 (Phase 5)**: depends on US1 (adds `correct_positions` after the loop); independent of US2 except that both edit `src/solver.rs`.
- **US4 (Phase 6)**: depends on US2 and US3 to show the full behavior.
- **Polish (Phase 7)**: depends on all stories.

### Within Foundational

- T004 → T005, T006, T012
- T007 → T008, T009 → T010
- T002 → T011 → T012 → T013
- T014 (README) has no code dependency but MUST finish before T024 (first solver code)

### Parallel opportunities

- T007 (shape.rs) can run in parallel with T004–T006 (collision/mod.rs).
- US1 tests T018–T023 are independent test functions and can be written in parallel once T015 exists (same file, so coordinate merges).
- US2 tests T027–T030 and US3 tests T033–T035 are all independent functions.
- T040 and T041 touch different files.

### Parallel example: Foundational

```text
Stream A: T004 → T005 → T006          (collision normal fix)
Stream B: T007 → T008 → T009 → T010   (inertia + body fields)
Join:     T011 → T012 → T013          (step reorder needs both streams)
Anytime:  T014                        (README; must precede T024)
```

---

## Implementation Strategy

### MVP first (User Story 1 only)

1. Phase 1 → Phase 2 (fix normals, add inertia/restitution fields, reorder step).
2. Phase 3 (US1): normal impulses with `e = 0`.
3. **STOP and VALIDATE**: a ball lands on a floor and stays there; bodies no longer pass through each other.

### Incremental delivery

1. US1 → bodies stop at surfaces (MVP).
2. US2 → restitution (`e = 1` returns to height) — the milestone's headline criterion.
3. US3 → positional correction — steady resting contacts.
4. US4 → demo verified with `--release`, satisfying Constitution IV.
5. Polish → milestone ticked, docs in sync, PR opened (README § Resolution was already updated in T014).

### Notes

- Tests live inline in `src/solver.rs` (and the other touched modules); most solver-story tasks edit the same file, so `[P]` marks independence of the test *functions*, not of the file.
- When a test fails, suspect the sign of the normal, the step order, and `r × n` handedness before loosening tolerances.
- Do not add friction, warm-starting, or tuning of iteration counts — those are M4.
- Commit after each phase checkpoint.
