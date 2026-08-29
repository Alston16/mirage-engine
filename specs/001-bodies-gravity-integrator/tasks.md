---
description: "Task list for M1: Bodies, Gravity & Fixed-Timestep Integrator"
---

# Tasks: Bodies, Gravity & Fixed-Timestep Integrator

**Input**: Design documents from `/specs/001-bodies-gravity-integrator/`

**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/engine-api.md, quickstart.md

**Tests**: Included. The spec's Success Criteria (SC-001, SC-002, SC-003, SC-005) and the project constitution (Principle IV: milestone-gated, behaviorally-verified development) require `cargo test` coverage of gravity integration and accumulator behavior, following the existing `#[cfg(test)] mod tests` convention from `src/math.rs` (M0). No separate `tests/` directory is used.

**Organization**: Tasks are grouped by user story (from spec.md) to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3)
- Exact file paths are included in every task

## Path Conventions

Single project (per plan.md): `src/` and `examples/` at the repository root; no `tests/` directory — unit tests are inline per module.

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Wire up the crate/example scaffolding this milestone needs, with no behavior yet.

- [X] T001 [P] Add `macroquad` under `[dev-dependencies]` in `Cargo.toml` (engine crate itself gains zero new dependencies, per Constitution Principle I); create the empty `examples/` directory.
- [X] T002 [P] Create stub files `src/body.rs` and `src/world.rs`; add `pub mod body;` and `pub mod world;` plus re-exports `pub use body::{BodyId, RigidBody};` and `pub use world::World;` to `src/lib.rs`, following the existing `pub use math::{Rot2, Vec2};` pattern.

**Checkpoint**: Crate compiles with empty modules; `Cargo.toml` has the dev-dependency.

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: The minimum working `RigidBody`/`World`/`step` needed before any user story can be demonstrated or tested — every story exercises this same code path.

**⚠️ CRITICAL**: No user story work can begin until this phase is complete.

- [X] T003 Implement `BodyId` (opaque newtype, e.g. wrapping `u32`) and `RigidBody` (`position: Vec2`, `velocity: Vec2`, `orientation: Rot2`, `angular_velocity: f32`, `inv_mass: f32`, `is_static: bool`) with constructors `RigidBody::new_dynamic(position: Vec2, mass: f32) -> Self` (sets `inv_mass = 1.0 / mass`, `is_static = false`) and `RigidBody::new_static(position: Vec2) -> Self` (sets `inv_mass = 0.0`, `is_static = true`), in `src/body.rs`. Per `data-model.md` and `contracts/engine-api.md`.
- [X] T004 Implement `World` (`bodies: Vec<RigidBody>`, `gravity: Vec2` defaulting to `Vec2::new(0.0, -9.81)`, a `fixed_dt` constant of `1.0 / 60.0`, `accumulator: f32` starting at `0.0`) with `World::new() -> Self`, `World::add_body(&mut self, body: RigidBody) -> BodyId`, and `World::body(&self, id: BodyId) -> &RigidBody`, in `src/world.rs`. Depends on T003.
- [X] T005 Implement `World::step(&mut self, dt_real: f32)`: add `dt_real` to `accumulator`, then while `accumulator >= fixed_dt`, apply one fixed step of semi-implicit Euler to every body where `is_static == false` — `velocity += gravity * fixed_dt; position += velocity * fixed_dt` (matching README's `v += (F/m + g) * dt; x += v * dt` exactly) — and subtract `fixed_dt` from `accumulator`; bodies where `is_static == true` are left untouched. In `src/world.rs`. Depends on T004.

**Checkpoint**: `cargo build` succeeds; `World::step` is callable and integrates gravity. Foundation ready for story-specific tests and the example.

---

## Phase 3: User Story 1 - Bodies fall under gravity with correct physics (Priority: P1) 🎯 MVP

**Goal**: Confirm dynamic bodies integrate gravity correctly (semi-implicit Euler, mass-independent acceleration) and static bodies never move.

**Independent Test**: Create a dynamic body at rest and a static body, step the world forward by a known amount of simulated time, and confirm the dynamic body's velocity/position match the closed-form gravity equations while the static body is unchanged.

### Tests for User Story 1

- [X] T006 [US1] Add test in `src/world.rs` `#[cfg(test)] mod tests`: a dynamic body released from rest has `velocity.y` matching `-9.81 * t` (within a small float tolerance) after stepping the world by a known elapsed time (spec SC-001, Acceptance Scenario 1).
- [X] T007 [US1] Add test in `src/world.rs` tests: a static body's `position` and `velocity` are exactly unchanged after many `World::step` calls (spec SC-002, Acceptance Scenario 2).
- [X] T008 [US1] Add test in `src/world.rs` tests: two dynamic bodies constructed with different masses at the same starting height have identical velocity and position after the same sequence of steps — gravity acceleration is mass-independent (spec Acceptance Scenario 3).

**Checkpoint**: `cargo test` passes and independently verifies User Story 1 (gravity-correct integration) without needing US2 or US3.

---

## Phase 4: User Story 2 - Simulation behavior is independent of render framerate (Priority: P2)

**Goal**: Confirm the fixed-timestep accumulator always advances physics in whole `1/60` increments and produces identical results regardless of how real time is chopped into calls to `step`.

**Independent Test**: Step the world using two different sequences of variable frame-time inputs that sum to the same total elapsed time, and confirm both sequences produce the same body states (within floating-point tolerance).

### Tests for User Story 2

- [X] T009 [US2] Add test in `src/world.rs` tests: feeding `World::step` many small, irregular `dt_real` values that sum to a known total elapsed time produces the same final body state (within tolerance) as feeding a different partition (e.g. fewer, larger steps) of that same total (spec SC-003, Acceptance Scenario 2).
- [X] T010 [US2] Add test in `src/world.rs` tests: after any `World::step` call, the internal `accumulator` remainder is always `< fixed_dt` — a partial step is never silently applied or dropped (spec Acceptance Scenario 1, FR-006).

**Checkpoint**: `cargo test` passes and independently verifies User Story 2 (framerate-independent determinism) on top of the Foundational phase alone.

---

## Phase 5: User Story 3 - Visual confirmation via the bouncing example (Priority: P3)

**Goal**: `examples/bouncing.rs` visually demonstrates the milestone end-to-end — circles fall under gravity, accelerating, with no collision response.

**Independent Test**: Run `cargo run --example bouncing --release` and observe that circles fall with visibly increasing speed and exit the bottom of the window without stopping or bouncing.

### Implementation for User Story 3

- [X] T011 [US3] Create the macroquad window scaffold in `examples/bouncing.rs`: a `#[macroquad::main("bouncing")]` entry point with an empty per-frame `loop { ... next_frame().await }`.
- [X] T012 [US3] In `examples/bouncing.rs`, construct a `World` and add several dynamic circle bodies at staggered starting positions via `RigidBody::new_dynamic`, tracking each body's render radius in a local `Vec<(BodyId, f32)>` (radius is example-local per spec Assumptions — no `Shape` type yet). Depends on T011 and Foundational (T003–T005).
- [X] T013 [US3] In `examples/bouncing.rs`'s per-frame loop, call `world.step(macroquad::time::get_frame_time())` once, then `draw_circle` for each tracked body at its current `world.body(id).position` and radius, clearing the screen each frame. Depends on T012.
- [X] T014 [US3] Run `cargo run --example bouncing --release` and visually confirm circles fall with visibly increasing downward speed and exit the window, with no circle stopping, bouncing, or reacting to another circle (spec SC-004). Depends on T013. Verified via two screen captures ~0.6s apart: all five circles moved downward, no bounce/collision.

**Checkpoint**: All three user stories are independently verified — `cargo test` covers US1/US2, and the running example covers US3.

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Remaining spec edge cases and milestone close-out.

- [X] T015 Add test in `src/world.rs` tests: `World::step` on an empty world (no bodies added) completes without panicking (spec Edge Cases).
- [X] T016 Add test in `src/world.rs` tests: a dynamic body constructed with a nonzero initial velocity has gravity accumulate on top of that velocity (not overwrite it) after one fixed step (spec Edge Cases).
- [X] T017 Run the full `quickstart.md` validation end-to-end: `cargo test` and `cargo run --example bouncing --release`, confirming every milestone exit criterion in `quickstart.md` is met.
- [X] T018 Update `README.md`'s M1 checklist entry from `[ ]` to `[x]` (matching the existing M0 entry's style) once T017's verification passes.

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies — can start immediately.
- **Foundational (Phase 2)**: Depends on Setup completion. BLOCKS all user stories (T004 depends on T003; T005 depends on T004).
- **User Stories (Phase 3–5)**: All depend on Foundational (Phase 2) completion.
  - US1 (Phase 3) and US2 (Phase 4) both add tests against the same `World::step` built in Foundational — no code dependency between them, but both require T005.
  - US3 (Phase 5) requires Foundational (T003–T005) to construct/step a `World`, but does not depend on US1 or US2's test code.
- **Polish (Phase 6)**: T015/T016 depend only on Foundational; T017 depends on all of Phases 3–5 being complete; T018 depends on T017.

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational — no dependency on US2 or US3.
- **User Story 2 (P2)**: Can start after Foundational — no dependency on US1 or US3.
- **User Story 3 (P3)**: Can start after Foundational — no dependency on US1 or US2 (though running it alongside already-passing US1/US2 tests gives the most confidence).

### Within Each User Story

- US1 and US2: all three/two tests are additions to the same `#[cfg(test)] mod tests` block in `src/world.rs` — implement sequentially within the story to avoid clobbering each other's edits (not marked `[P]`).
- US3: T011 → T012 → T013 → T014 are strictly sequential (each builds on the file the previous task created).

### Parallel Opportunities

- T001 and T002 (Setup) touch different files (`Cargo.toml` vs. `src/lib.rs` + new stub files) and can run in parallel.
- Once Foundational (Phase 2) completes, US1 (Phase 3), US2 (Phase 4), and US3 (Phase 5) can be worked on in parallel by different contributors, since none of their tasks touch the same lines of the same file as another story (US1/US2 both edit `src/world.rs`'s test module, so those two stories should NOT be parallelized by two people at once without coordinating merges — US3 is fully independent in `examples/bouncing.rs`).

---

## Parallel Example: Setup

```bash
Task: "Add macroquad under [dev-dependencies] in Cargo.toml; create examples/ directory"
Task: "Create src/body.rs and src/world.rs stubs; wire pub mod / re-exports into src/lib.rs"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup.
2. Complete Phase 2: Foundational (`BodyId`, `RigidBody`, `World`, `World::step` with gravity integration).
3. Complete Phase 3: User Story 1 tests.
4. **STOP and VALIDATE**: `cargo test` passes for gravity-correct integration.
5. This alone satisfies the physics core of the milestone; US2/US3 add determinism proof and the visual demo.

### Incremental Delivery

1. Setup + Foundational → `World::step` works.
2. Add User Story 1 → `cargo test` proves gravity integration is correct.
3. Add User Story 2 → `cargo test` proves framerate-independent determinism.
4. Add User Story 3 → `cargo run --example bouncing --release` proves it end-to-end, visually.
5. Polish → edge-case tests, full quickstart validation, README milestone checkbox.

---

## Notes

- `[P]` tasks touch different files with no dependency on an incomplete task.
- `[Story]` labels map tasks to spec.md's user stories for traceability.
- Tests for US1/US2 live in the same `src/world.rs` test module as the existing convention (`src/math.rs`) — implement them one at a time, running `cargo test` after each.
- Constitution Principle IV requires actually running `cargo run --example bouncing --release` (T014, T017) — `cargo test` alone does not satisfy this milestone's "done when" criterion.
- No collision, joints, sleeping, or spatial-index work belongs in any of these tasks (Constitution Principle V / spec non-goals) — if a task seems to need it, stop and reconsider scope rather than adding it.
