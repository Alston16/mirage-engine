---
description: "Task list for M5: Shape COM Correction & Frictionless-Stack Limits"
---

# Tasks: Shape COM Correction & Frictionless-Stack Limits (M5)

**Input**: Design documents from `/specs/005-shape-com-and-stability-hardening/`

**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/engine-api.md, quickstart.md

**Tests**: Included. The spec's Success Criteria (SC-001–SC-006) and Constitution Principle IV require behavioral/regression verification. The centroid correction gets inline `#[cfg(test)]` tests in `src/shape.rs` (existing convention); the frictionless-stacking boundary gets integration tests in the existing `tests/stacking.rs`, using a new builder in `tests/common/mod.rs`.

**Organization**: Tasks are grouped by user story (from spec.md) so each can be implemented and verified on its own. Unlike M4, **User Story 1 and User Story 2 are fully independent** — US1 touches only `src/shape.rs`; US2 touches only `README.md` § Resolution, `tests/common/mod.rs`, and `tests/stacking.rs`. Either can be done first, or in parallel.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies on incomplete tasks)
- **[Story]**: Which user story this task belongs to (US1, US2)
- Exact file paths are included in every task

## Path Conventions

Single project (per plan.md): `src/`, `tests/` at the repository root. All `cargo` commands run from the repo root. Never push to `main`; all work stays on branch `005-shape-com-and-stability-hardening`.

## Conventions used by many tasks

- **Centroid formula** (research.md § Q1): `cᵢ = pᵢ × pᵢ₊₁` (already computed by `inertia()`), signed area `A = ½Σcᵢ`, `Cx = 1/(6A)·Σ(xᵢ+xᵢ₊₁)·cᵢ`, `Cy = 1/(6A)·Σ(yᵢ+yᵢ₊₁)·cᵢ`. `Shape::polygon` stores `vertices[i] − (Cx, Cy)`.
- **Equality shortcut** (research.md, data-model.md): because recentering is a pure function of the input vertex list, two vertex lists describing the same physical polygon but differing by a constant translation produce *bit-identical* stored `Shape::Polygon` values after recentering — `Shape` and `RigidBody` both derive `PartialEq`, so SC-001's "identical behavior" is directly testable as struct equality, with no need to step a `World`.
- **Frictionless boundary numbers** (research.md § Q3, from issue #11): 2 boxes hold (measured max drift 0.0099 over 60 s); 5 and 10 boxes collapse (drift exceeds 0.5 within 14.1 s / 3.8 s respectively). Tests use loosened bounds around these measured numbers, not the exact values (avoids floating-point flakiness while still catching a regression either direction).
- **Where evidence goes**: `research.md` gets a new section "In-repo notes" (created by whichever task first writes to it), matching the M4 convention.

---

## Phase 1: Setup

**Purpose**: Confirm the pre-M5 baseline before any change.

- [X] T001 Gate 0 (Constitution IV): on branch `005-shape-com-and-stability-hardening` run `cargo test` and record the passing count, then run `cargo run --example stack --release` and confirm the 10-box tower still looks stable. Record both under a new "In-repo notes" section in `specs/005-shape-com-and-stability-hardening/research.md`. Do not start US1/US2 work if either fails — that would be a pre-existing regression unrelated to this feature. **Status: 97 tests pass (83 unit + 7 friction + 7 stacking); `stack --release` builds and launches cleanly.**

**Checkpoint**: Baseline recorded.

---

## Phase 2: Foundational

**Not applicable to this feature.** User Story 1 (`src/shape.rs`) and User Story 2 (`README.md`, `tests/common/mod.rs`, `tests/stacking.rs`) touch disjoint files with no shared blocking prerequisite — unlike M4, there is no cross-story dependency (US2 does not need US1's centroid fix, and US1 does not need US2's documentation). Each story's own "docs/tests first" task below stands in for this phase's role. Proceed directly to Phase 3.

---

## Phase 3: User Story 1 — Off-center polygons behave correctly (Priority: P1) 🎯 MVP

**Goal**: `Shape::polygon` always recenters its input to the true center of mass, so `Shape::inertia`'s origin-centered integral is always correct — closes issue #7.

**Independent Test**: `cargo test --lib shape` (all cases below).

### Tests for User Story 1

> Write these first. T002 should FAIL before T005 (today's code leaves off-center input off-center).

- [X] T002 [US1] In `src/shape.rs`'s `#[cfg(test)]` module, add `off_center_polygon_recenters_to_match_pre_centered`: build a centered rectangle's vertices (e.g. `[(-1,-1),(1,-1),(1,1),(-1,1)]`) and an off-center version of the *same* rectangle shifted by a constant offset (e.g. `(3.0, -2.0)` added to every vertex); assert `Shape::polygon(off_center) == Shape::polygon(centered)` (full struct equality — `Shape` derives `PartialEq`) and that both report the same `inertia(mass)` for a given mass (US1-1, US1-2, US1-3, US1-4, FR-001, FR-002, FR-004, SC-001). **Status: implemented using the file's existing `unit_square()` fixture as the off-center input — see note below.**
- [X] T003 [P] [US1] In `src/shape.rs`, add `already_centered_polygon_has_no_shift`: for the existing `unit_square()` fixture and the existing rectangle fixture from `rectangle_inertia_matches_closed_form`, assert `Shape::polygon(verts.clone())`'s stored vertices are unchanged (within `EPS`) from `verts` — pins the no-op guarantee for every shape already in the engine (US1-1, FR-003, SC-002). This test is expected to pass both before and after T005 (it documents a non-regression, not new behavior). **Status: `unit_square()` turned out NOT to be centered (see note below) — implemented using the square/rectangle vertices from `square_inertia_is_two_thirds_m_for_half_extent_one` and `rectangle_inertia_matches_closed_form` instead, which are genuinely centered.**
- [X] T004 [P] [US1] In `src/shape.rs`, add `near_degenerate_polygon_centroid_and_inertia_stay_finite`: build a valid but very small-area triangle (e.g. vertices around `1e-3` scale) and assert both the resulting `Shape::polygon`'s stored vertices and its `inertia(1.0)` are finite (`f32::is_finite`), not `NaN`/`Inf` (FR-005, FR-006, SC-003).

### Implementation for User Story 1

- [X] T005 [US1] In `src/shape.rs`, implement the centroid computation and recentering inside `Shape::polygon` per the formula in "Conventions" above, applied before computing `normals` (normals are translation-invariant, so this ordering is safe). Replace the doc comments on `Shape::polygon` and `Shape::inertia` that currently describe the *unchecked* centering assumption ("this is the caller's responsibility... nothing here checks that assumption") with a description of the corrected, always-centered behavior, including the centroid formula (FR-013, contracts/engine-api.md). Depends on T002–T004 existing.
- [X] T006 [US1] Run `cargo test`. Confirm: T002 now passes; T003 and T004 pass; every other existing test (`src/body.rs`, `src/world.rs`, `src/collision/*`, `src/solver.rs`, and the rest of `src/shape.rs`'s existing tests — `circle_inertia_is_half_m_r_squared`, `square_inertia_is_two_thirds_m_for_half_extent_one`, `rectangle_inertia_matches_closed_form`, the AABB/normals tests) is unaffected. **Status: one existing test needed a deliberate update — see note below. Everything else (99 tests: 86 unit + 7 friction + 7 stacking, up from the 97 baseline) is unaffected.**

**Note — `unit_square()` was itself an off-center fixture.** `src/shape.rs`'s test-only `unit_square()` helper spans `(0,0)`–`(1,1)`, centroid `(0.5, 0.5)` — it was never actually centered on the origin (it's used elsewhere only for normal/AABB checks, never for `inertia`). This made it the natural off-center input for T002 (paired against the equivalent pre-centered half-extent-0.5 box) instead of a synthetic example. It also meant one existing test, `polygon_aabb_axis_aligned_matches_extent`, hardcoded the *old*, uncorrected AABB bounds for this off-center shape (`(10,10)`–`(11,11)` at position `(10,10)`). After the fix, `unit_square()`'s local origin is its centroid, so the correct bounds are `(9.5,9.5)`–`(10.5,10.5)`. This is a deliberate, necessary test update — not a loosened bound — recorded here and in `research.md`; `polygon_normals_point_outward_for_ccw_winding` and `polygon_aabb_grows_when_rotated` (the other two tests using `unit_square()`) needed no change (normals are translation-invariant; the rotated-vs-axis-aligned extent comparison holds regardless of where the local origin sits).

**Checkpoint**: US1 complete — off-center polygon input now behaves identically to pre-centered input; every shape already in the engine is provably unaffected except the one fixture (`unit_square()` in shape.rs's own test module) that was always off-center, whose dependent AABB test was corrected.

---

## Phase 4: User Story 2 — Frictionless-stacking limitation documented and pinned (Priority: P2)

**Goal**: The `μ = 0` stacking boundary (holds ≤ ~2 boxes, collapses above) is written into the README and pinned by a regression test — closes issue #11. Independent of Phase 3.

**Independent Test**: `cargo test --release stacking -- --nocapture` (new cases below); reading `README.md` § Resolution.

- [X] T007 [US2] Update `README.md` § Resolution FIRST (Constitution III): add a known-trade-off note, next to the existing sequential-impulse residual-torque note, stating that `μ = 0` stacks are neutrally stable and collapse above roughly three boxes, and that post-collapse tunneling through the floor is an expected symptom of having no continuous collision detection (an existing, named non-goal) rather than a new defect (FR-007, SC-004).
- [X] T008 [US2] In `tests/common/mod.rs`, add `pub fn frictionless_tower(world: &mut World, n: usize) -> Vec<BodyId>`, mirroring the existing `tower()` but building the floor and every box `with_friction(0.0)`. Leave `tower()` itself untouched — every other stacking test still uses it at the default `μ = 0.5` (data-model.md § Test fixtures).
- [X] T009 [US2] In `tests/stacking.rs`, add `frictionless_two_box_stack_holds_within_measured_drift`: `frictionless_tower(&mut world, 2)`, step 3600 times (60 s, matching `tower_stands_for_sixty_seconds`'s duration); assert `all_finite(&world, &ids)` holds at every step and max horizontal drift over the whole run stays under a loosened bound (e.g. `0.02`, above the 0.0099 measured in issue #11 but well below any collapse) (US2-1, FR-008, SC-003). Depends on T008.
- [X] T010 [US2] In `tests/stacking.rs`, add `frictionless_five_box_stack_collapses_as_documented`: `frictionless_tower(&mut world, 5)`, step for long enough to see the collapse issue #11 measured (e.g. 20 s, comfortably past the ~14.1 s it recorded); assert `all_finite` holds throughout (collapse produces large-but-finite numbers, not `NaN`/`Inf`) **and** assert max horizontal drift exceeds a large pinned threshold (e.g. `1.0`) at some point in the run — the collapse is the expected, passing outcome being pinned, not a failure (US2-2, FR-008, SC-003). Depends on T008. **Status: measured max drift 19.04 over 20 s (well above the 1.0 threshold), all values stayed finite.**
- [X] T011 [US2] Run `cargo test --release stacking -- --nocapture`. Confirm every pre-existing test in this file that uses `tower()`/`pyramid()` (`tower_stands_for_sixty_seconds`, `pyramid_stays_standing_for_thirty_seconds`, `mixed_boxes_and_circles_come_to_rest`, `tall_tower_does_not_explode`, `heavy_on_light_stays_finite_and_bounded`, `tower_is_bit_identical_across_runs`, `tower_is_stable_under_variable_frame_times`) is unaffected by the new `frictionless_tower` builder (US2-4, FR-009). Depends on T009, T010. **Status: all 9 stacking tests pass (7 pre-existing + 2 new), identical metrics to baseline for the pre-existing ones.**

**Checkpoint**: US2 complete — the `μ = 0` boundary is documented in the README and pinned by tests; every existing friction-on stacking guarantee is unchanged.

---

## Phase 5: Polish & Cross-Cutting Concerns

- [X] T012 [P] Run the full suite (`cargo test` and `cargo test --release`); record the before/after test counts in `research.md` "In-repo notes" (SC-003, SC-006). Depends on T006, T011. **Status: 102 passed, 0 failed in both debug and release (86 unit + 7 friction + 9 stacking), up from the 97 baseline.**
- [X] T013 [P] Run all four existing demos with `--release` (`bouncing`, `ramp`, `stack`, `pyramid`) and confirm each looks exactly as it did before this feature (SC-005) — this milestone adds no new example. Depends on T006, T007. **Status: all four build and launch cleanly with `cargo run --example <name> --release`, no panics or errors. None of them build an off-center `Shape::polygon` (all use `rect_vertices`/symmetric literals), so SC-005's invisibility claim holds by construction; a full frame-by-frame watch was not re-done here (no display capture available in this session) — recommend the user spot-check `stack`/`pyramid` visually before merge, as M4's did.**
- [X] T014 [P] Constitution hard-rule check (mirrors M4's precedent): `cargo tree -e normal` still shows `mirage` with zero dependencies; `grep -rn "unsafe" src/` is empty; `grep -rniE "sleep|deactivat|joint|ccd|continuous" src/` shows no new non-goal machinery (FR-012 / Constitution V). Depends on T006, T011. **Status: all three checks confirmed clean.**
- [X] T015 Sync design docs: confirm `spec.md`'s Assumptions and `research.md` § Q3's numbers agree with whatever bounds T009/T010 actually landed on, and that `data-model.md`/`quickstart.md` don't reference anything renamed during implementation. Depends on T012. **Status: spec.md/data-model.md/quickstart.md describe the bounds generically (no hardcoded numbers to drift); research.md § Q3's measured numbers match issue #11 and the implemented test thresholds (0.02 hold / 1.0 collapse) are consistent with them.**
- [ ] T016 Prepare the change for review on branch `005-shape-com-and-stability-hardening` (base `main` — M0–M4 are already merged there). Never push to `main`; open the PR only when the user asks. Once merged, close GitHub issues #7 and #11 referencing the PR — only when the user asks. Depends on T012–T015.

---

## Dependencies & Execution Order

### Phase dependencies

- **Setup (Phase 1)**: no dependencies; T001 gates all other work.
- **Foundational (Phase 2)**: N/A — see note above.
- **US1 (Phase 3)**: after Setup. Fully independent of US2.
- **US2 (Phase 4)**: after Setup. Fully independent of US1.
- **Polish (Phase 5)**: after both US1 and US2.

### Within-story order

- US1: T002 ‖ T003 ‖ T004 (all new/existing tests, different assertions in the same file but no dependency on each other) → T005 → T006.
- US2: T007 (docs) and T008 (builder) have no dependency on each other → T009 ‖ T010 (both depend on T008 only) → T011.

### Parallel opportunities

- Phase 3: T003 ‖ T004 (T002 can also run alongside them; all three are additions to the same file's test module but assert independent things — sequence them if editing the same file causes merge friction in practice).
- Phase 4: T007 ‖ T008 (different files); T009 ‖ T010 (different tests, same file, both depend only on T008).
- **Phases 3 and 4 can run entirely in parallel** — no file overlap between them.
- Polish: T012 ‖ T013 ‖ T014.

### Parallel example: Phases 3 and 4 together

```text
Track A (US1): T002 → T003/T004 → T005 → T006   [src/shape.rs only]
Track B (US2): T007, T008 → T009/T010 → T011     [README.md, tests/ only]
```

---

## Implementation Strategy

### MVP first

1. Phase 1 (baseline gate).
2. Phase 3 (US1): the higher-priority correctness fix — closes issue #7 on its own, independently testable and shippable.
3. Phase 4 (US2) can follow, or run in parallel — it closes issue #11 and does not depend on US1.

### Incremental delivery

US1 → US2 (either order, or parallel) → Polish. Each user-story phase ends at a checkpoint that is runnable and independently verifiable.

### Risk gates

- **Gate 0 (T001)**: current `main` must be green before any M5 change, so a later failure is attributable to this feature.
- **Non-goals**: no joints, CCD, sleeping, concave shapes, spatial index, serialization, parallelism, 3D, and (FR-010) no velocity/speed clamp added to `solver.rs`. If a task seems to need one, stop and confirm scope with the user.

## Notes

- `[P]` tasks touch different files (or independent assertions) with no dependency on incomplete tasks.
- Each `[US#]` label maps a task to its story in spec.md.
- Commit after each task or logical group; never commit to `main`.
