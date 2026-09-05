---
description: "Task list for M2: Collision Detection"
---

# Tasks: Collision Detection (M2)

**Input**: Design documents from `/specs/002-collision-detection/`

**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/engine-api.md, quickstart.md

**Tests**: Included. The spec's Success Criteria (SC-001–SC-004) and the project constitution (Principle IV: milestone-gated, behaviorally-verified development; Principle III: spec-faithful math) require `cargo test` coverage of every narrowphase pair type and broadphase pair rejection, following the existing `#[cfg(test)] mod tests` convention (`math.rs`, `body.rs`, `world.rs`). No separate `tests/` directory is used.

**Organization**: Tasks are grouped by user story (from spec.md) to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3)
- Exact file paths are included in every task

## Path Conventions

Single project (per plan.md): `src/` and `examples/` at the repository root; no `tests/` directory — unit tests are inline per module.

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Create the empty module scaffolding this milestone needs, with no behavior yet.

- [ ] T001 [P] Create stub files `src/shape.rs`, `src/broadphase.rs`, `src/collision/mod.rs`, `src/collision/circle.rs`, `src/collision/polygon.rs`, `src/collision/manifold.rs` (empty, per README's module tree and `plan.md` Project Structure).
- [ ] T002 Wire `pub mod shape;`, `pub mod broadphase;`, `pub mod collision;` declarations into `src/lib.rs`, following the existing `pub mod body; pub mod math; pub mod world;` pattern. Depends on T001 (files must exist to be declared).

**Checkpoint**: Crate compiles with empty modules.

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: The minimum working `Shape`/`Aabb`/`RigidBody.shape`/broadphase/narrowphase-dispatch/`World.contacts` pipeline needed before any user story's actual collision math can be demonstrated or tested — every story exercises this same code path.

**⚠️ CRITICAL**: No user story work can begin until this phase is complete.

- [ ] T003 Implement `Aabb { min: Vec2, max: Vec2 }` with `overlaps(&self, other: &Aabb) -> bool` (interval overlap on both axes), in `src/shape.rs`. Per `data-model.md`/`contracts/engine-api.md`.
- [ ] T004 Implement `Shape` (`Circle { radius: f32 }`, `Polygon { vertices: Vec<Vec2>, normals: Vec<Vec2> }`) with `Shape::circle(radius) -> Self`, `Shape::polygon(vertices: Vec<Vec2>) -> Self` (derives outward CCW edge normals from consecutive vertex pairs), and `Shape::aabb(&self, position: Vec2, orientation: Rot2) -> Aabb` (circle: `position ± radius` box; polygon: transform every vertex by `orientation`/`position`, take min/max extent), in `src/shape.rs`. Depends on T003.
- [ ] T005 Add `pub shape: Shape` field to `RigidBody`; change `new_dynamic(position: Vec2, mass: f32, shape: Shape) -> Self` and `new_static(position: Vec2, shape: Shape) -> Self` accordingly, in `src/body.rs`. Depends on T004. **Breaking change from M1** — see T006.
- [ ] T006 Update `examples/bouncing.rs` call sites to pass `Shape::circle(radius)` (using each circle's existing local radius) to `RigidBody::new_dynamic`/`new_static`, restoring compilation after T005's signature change. Depends on T005.
- [ ] T007 [P] Implement `broadphase::candidate_pairs(bodies: &[RigidBody]) -> Vec<(usize, usize)>` — O(n²) loop over all index pairs `i < j`, accepting a pair only when `bodies[i].shape.aabb(bodies[i].position, bodies[i].orientation)` overlaps the same for `j`, in `src/broadphase.rs`. Depends on T004, T005.
- [ ] T008 [P] Implement `Contact { point: Vec2, normal: Vec2, penetration: f32 }` and `Manifold { body_a: BodyId, body_b: BodyId, points: Vec<Contact> }` in `src/collision/manifold.rs`. Depends on T003 (different file from T007 — can run in parallel with it).
- [ ] T009 Add stub narrowphase functions returning `None` — `pub(crate) fn circle_vs_circle(a: &RigidBody, b: &RigidBody) -> Option<Contact>` and `circle_vs_polygon(circle: &RigidBody, polygon: &RigidBody) -> Option<Contact>` in `src/collision/circle.rs`; `pub(crate) fn polygon_vs_polygon(a: &RigidBody, b: &RigidBody) -> Option<Vec<Contact>>` in `src/collision/polygon.rs` — plus `collision::detect_contacts(bodies: &[RigidBody]) -> Vec<Manifold>` in `src/collision/mod.rs` that runs `broadphase::candidate_pairs`, dispatches each pair to the matching stub by `Shape` variant, and collects any `Some` result into a `Manifold`. Depends on T007, T008.
- [ ] T010 Add `contacts: Vec<Manifold>` field to `World`; in `World::step`, after each fixed substep's integration, replace `contacts` with the result of `collision::detect_contacts(&self.bodies)`; add `World::contacts(&self) -> &[Manifold]` accessor, in `src/world.rs`. Depends on T009.

**Checkpoint**: `cargo build` succeeds; the full broadphase→narrowphase→`World.contacts()` pipeline runs every step but reports no contacts yet (narrowphase is stubbed). Foundation ready for story-specific narrowphase implementations.

---

## Phase 3: User Story 1 - Know when two bodies are touching (Priority: P1) 🎯 MVP

**Goal**: Circle–circle contact detection works end-to-end, and broadphase correctly filters pairs before narrowphase runs.

**Independent Test**: Drop a circle body onto a static circle body positioned below it, step the world, and inspect `world.contacts()` for the expected contact; confirm it's absent once they separate; confirm scattered non-overlapping-AABB pairs are never tested.

### Tests for User Story 1

- [ ] T011 [P] [US1] Add test in `src/collision/circle.rs` `#[cfg(test)] mod tests`: two circles whose centers are closer together than the sum of their radii produce `Some(Contact)` with normal pointing from body A's center toward body B's center and `penetration == r_a + r_b - dist` (within 1e-4) (spec Acceptance Scenario 1, SC-001).
- [ ] T012 [P] [US1] Add test in `src/collision/circle.rs` tests: two circles farther apart than the sum of their radii produce `None` (spec Acceptance Scenario 2).
- [ ] T013 [P] [US1] Add test in `src/broadphase.rs` tests: given bodies scattered so only a few pairs' AABBs overlap, `candidate_pairs` returns exactly those overlapping pairs and no others (spec Acceptance Scenario 3, SC-003).
- [ ] T014 [P] [US1] Add test in `src/collision/circle.rs` tests: two circles exactly touching (`dist == r_a + r_b`) do not flicker into a false contact from float noise — confirm the epsilon/slop guard from `research.md` (spec Edge Cases).

### Implementation for User Story 1

- [ ] T015 [US1] Implement `circle_vs_circle` in `src/collision/circle.rs`: compare center distance to `r_a + r_b`, normal `= (b.position - a.position).normalize()`, `penetration = r_a + r_b - dist`, applying the epsilon guard so `penetration <= 0` (or below tolerance) yields `None` (per `research.md`). Depends on Foundational (T003–T010).
- [ ] T016 [US1] Wire `collision::detect_contacts` (`src/collision/mod.rs`) to call the real `circle_vs_circle` for Circle–Circle pairs in place of the stub, producing a one-point `Manifold` on `Some`. Depends on T015.

**Checkpoint**: `cargo test` passes and independently verifies User Story 1 (circle–circle detection + broadphase filtering) without needing US2 or US3.

---

## Phase 4: User Story 2 - Detect contact between circles and polygons, and between polygons (Priority: P2)

**Goal**: Circle–polygon and polygon–polygon narrowphase work correctly, including rotated polygons and manifold clipping.

**Independent Test**: Overlap a circle with a static polygon's face and separately with its corner and confirm correct normals; overlap two polygons (one rotated) and confirm a correct 1–2 point manifold along the axis of minimum penetration.

### Tests for User Story 2

- [ ] T017 [P] [US2] Add test in `src/collision/circle.rs` tests: a circle overlapping the flat face of a static polygon produces a contact with normal pointing away from that face and `penetration == radius - distance_to_face` (spec Acceptance Scenario 1).
- [ ] T018 [P] [US2] Add test in `src/collision/circle.rs` tests: a circle whose center is nearest a polygon's corner (not any single face) produces a contact whose normal points away from that corner, not from either adjacent face (spec Acceptance Scenario 2 / Edge Case, vertex-region fallback from `research.md`).
- [ ] T019 [P] [US2] Add test in `src/collision/polygon.rs` tests: two overlapping polygons, one rotated to an arbitrary angle via `Rot2::new`, produce a manifold of one or two points on the shared overlap region with a single normal aligned to the axis of minimum penetration (spec Acceptance Scenario 3).
- [ ] T020 [P] [US2] Add test in `src/collision/polygon.rs` tests: a box overlapping a shallow-angle rotated static polygon ("ramp") produces contact point(s) lying on the true overlap region with the correct normal — the numeric groundwork for the milestone's box-on-ramp acceptance criterion (spec Acceptance Scenario 4, SC-002).
- [ ] T021 [P] [US2] Add test in `src/collision/polygon.rs` tests: two deeply overlapping polygons (one nearly contained in the other) still produce a non-degenerate manifold of one or two points, not zero (spec Edge Cases).

### Implementation for User Story 2

- [ ] T022 [US2] Implement `circle_vs_polygon` in `src/collision/circle.rs`: project the circle's (world-space) center onto each polygon edge (segment-clamped), track the globally closest point/edge, and fall back to a vertex normal when the center is outside every edge's clamped region (nearest a corner), per `research.md`. Depends on Foundational; shares file with T015.
- [ ] T023 [US2] Implement `polygon_vs_polygon` in `src/collision/polygon.rs`: SAT over both bodies' world-space face normals (faces transformed by each body's `orientation`), tracking the axis of minimum penetration and returning `None` on any axis with positive separation; on overlap, pick the reference face (winning axis's body) and incident face (most anti-parallel face on the other body), then clip the incident face's two endpoints against the reference face's side planes to produce 1–2 `Contact`s, exactly per README § Narrowphase / `research.md`. Depends on Foundational.
- [ ] T024 [US2] Wire `collision::detect_contacts` to call the real `circle_vs_polygon` and `polygon_vs_polygon` for the remaining shape-pair combinations in place of their stubs. Depends on T022, T023.

**Checkpoint**: `cargo test` passes for all three narrowphase pair types; US1 and US2 are both independently verified via tests.

---

## Phase 5: User Story 3 - See contact data to verify it's correct (Priority: P3)

**Goal**: A running example visually confirms contact points/normals, including the milestone's own box-on-rotated-ramp "done when" criterion.

**Independent Test**: Run the example and observe each active contact's point and normal drawn on screen, updating as bodies move, matching the true overlap geometry.

### Implementation for User Story 3

- [ ] T025 [US3] Create `examples/ramp.rs`: a `#[macroquad::main("ramp")]` scaffold with a static rotated polygon "ramp" body (`RigidBody::new_static` with `Shape::polygon(...)` and a non-identity `orientation` via `Rot2::new(angle)`) and a dynamic box body (`RigidBody::new_dynamic` with `Shape::polygon(...)` box vertices) positioned to fall onto the ramp. Depends on Foundational + US1 + US2 (T003–T024).
- [ ] T026 [US3] In `examples/ramp.rs`'s per-frame loop, call `world.step(macroquad::time::get_frame_time())`, draw the ramp and box shapes for visual context, then for every `Manifold` in `world.contacts()` draw each `Contact`'s `point` (small marker) and `normal` (short line/arrow from that point). Depends on T025.
- [ ] T027 [US3] Run `cargo run --example ramp --release` and visually confirm the box's contact point(s)/normal(s) against the ramp coincide with the true overlap geometry, matching README's M2 "done when" criterion and spec SC-002. Depends on T026.

**Checkpoint**: All three user stories are independently verified — `cargo test` covers US1/US2, the running example covers US3.

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Remaining spec edge cases and milestone close-out.

- [ ] T028 [P] Add test in `src/world.rs` tests: stepping a world containing two overlapping bodies never changes either body's `velocity` or `position` as a result of the detected contact — collision detection alone produces no motion beyond ordinary gravity integration (spec FR-007, SC-004).
- [ ] T029 [P] Add test in `src/broadphase.rs` tests: `candidate_pairs` on zero or one body returns an empty list without panicking (spec Edge Cases groundwork).
- [ ] T030 Run the full `quickstart.md` validation end-to-end: `cargo test` and `cargo run --example ramp --release`, confirming every milestone exit criterion in `quickstart.md` is met.
- [ ] T031 Update `README.md`'s M2 checklist entry from `[ ]` to `[x]` (matching the existing M0/M1 entries' style) once T030's verification passes.

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies — can start immediately.
- **Foundational (Phase 2)**: Depends on Setup completion. BLOCKS all user stories (T004→T003; T005→T004; T006→T005; T007→T004,T005; T008→T003; T009→T007,T008; T010→T009).
- **User Stories (Phase 3–5)**: All depend on Foundational (Phase 2) completion.
  - US1 (Phase 3) needs only Foundational.
  - US2 (Phase 4) needs only Foundational — implemented independently of US1's `circle_vs_circle` work, though both stories' code lives in the shared `collision::detect_contacts` dispatch (coordinate merges if working in parallel).
  - US3 (Phase 5) needs Foundational **and** US1 **and** US2, since the ramp scene's whole point is exercising polygon–polygon (and implicitly the general pipeline) contact detection visually.
- **Polish (Phase 6)**: T028/T029 depend only on Foundational; T030 depends on all of Phases 3–5 being complete; T031 depends on T030.

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational — no dependency on US2 or US3.
- **User Story 2 (P2)**: Can start after Foundational — no dependency on US1 (different narrowphase functions, different files), though both write into the same `collision::detect_contacts` dispatch function.
- **User Story 3 (P3)**: Requires US1 and US2 complete — the acceptance demo needs working polygon–polygon detection (a box on a ramp), so it cannot be meaningfully verified before US2 lands.

### Within Each User Story

- US1: T011–T014 (tests, all `[P]`, different assertions but some share `src/collision/circle.rs`'s test module — coordinate if run in parallel) → T015 → T016.
- US2: T017–T021 (tests, `[P]`, spread across `circle.rs`/`polygon.rs` test modules) → T022, T023 (can run in parallel — different files) → T024 (depends on both).
- US3: T025 → T026 → T027, strictly sequential (each builds on the file the previous task created).

### Parallel Opportunities

- T001 (Setup) creates all stub files at once — inherently one task, not parallelized further.
- T007 and T008 (Foundational) touch different files (`broadphase.rs` vs. `collision/manifold.rs`) and can run in parallel.
- Within US1, T011–T014 are all test additions but T011/T012/T014 share `src/collision/circle.rs` while T013 is in `src/broadphase.rs` — T013 can run fully in parallel with the other three.
- Within US2, T022 and T023 (different files: `circle.rs` vs. `polygon.rs`) can run in parallel once their respective tests (T017/T018 vs. T019/T020/T021) are in place.
- US1 and US2 can be worked on in parallel by different contributors once Foundational is complete, provided merges to `src/collision/mod.rs`'s dispatch function (T016 vs. T024) are coordinated.

---

## Parallel Example: Foundational

```bash
Task: "Implement Contact/Manifold structs in src/collision/manifold.rs"
Task: "Implement broadphase::candidate_pairs in src/broadphase.rs"
```

## Parallel Example: User Story 2

```bash
Task: "Implement circle_vs_polygon in src/collision/circle.rs"
Task: "Implement polygon_vs_polygon in src/collision/polygon.rs"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup.
2. Complete Phase 2: Foundational (`Shape`, `Aabb`, `RigidBody.shape`, broadphase, dispatch skeleton, `World.contacts`).
3. Complete Phase 3: User Story 1 (circle–circle detection).
4. **STOP and VALIDATE**: `cargo test` passes for circle–circle contact detection and broadphase filtering.
5. This alone proves the collision pipeline works end-to-end for the simplest shape pair; US2 adds the other two pairs, US3 adds visual proof.

### Incremental Delivery

1. Setup + Foundational → pipeline runs, reports no contacts (stubbed).
2. Add User Story 1 → `cargo test` proves circle–circle detection + broadphase filtering.
3. Add User Story 2 → `cargo test` proves circle–polygon and polygon–polygon detection, including rotation and clipping.
4. Add User Story 3 → `cargo run --example ramp --release` proves the milestone's box-on-ramp criterion visually.
5. Polish → no-mutation guarantee, degenerate-input safety, full quickstart validation, README milestone checkbox.

---

## Notes

- `[P]` tasks touch different files with no dependency on an incomplete task.
- `[Story]` labels map tasks to spec.md's user stories for traceability.
- Constitution Principle IV requires actually running `cargo run --example ramp --release` (T027, T030) — `cargo test` alone does not satisfy this milestone's "done when" criterion.
- Constitution Principle III requires narrowphase math to match README § Narrowphase's derivation exactly (SAT, reference/incident face, clipping) — do not substitute an alternate formulation even if it seems simpler.
- No impulse resolution, restitution, friction, joints, sleeping, or spatial-index work belongs in any of these tasks (Constitution Principle V / spec non-goals) — if a task seems to need it, stop and reconsider scope rather than adding it.
