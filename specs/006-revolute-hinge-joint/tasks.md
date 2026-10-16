# Tasks: Revolute (Hinge) Joint

**Input**: Design documents from `specs/006-revolute-hinge-joint/`
**Prerequisites**: [plan.md](plan.md), [spec.md](spec.md), [research.md](research.md), [data-model.md](data-model.md), [contracts/joint-api.md](contracts/joint-api.md), [quickstart.md](quickstart.md)

**Tests**: Included. `README.md`'s constitution (Principle IV, "Milestone-Gated,
Behaviorally-Verified Development") requires `cargo test` to cover solver
behavior and a milestone's "done when" criterion to be verified by actually
running its example — this repo does not treat tests as optional.

**Organization**: Tasks are grouped by user story (spec.md's P1/P2/P3) so
each can be implemented and verified independently, on top of a shared
Foundational phase that builds the joint engine itself.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependency on an
  incomplete task)
- **[Story]**: Which user story this task belongs to (US1, US2, US3)
- Every task names its exact file path(s)

## Path Conventions

Single crate, matching the existing repository layout — `src/`, `tests/`,
`examples/` at the repository root (see plan.md § Project Structure).

---

## Phase 1: Setup

**Purpose**: Scaffolding so later tasks have somewhere to land — no
behavior yet.

- [X] T001 Add `pub mod joint;` to `src/lib.rs` and create `src/joint.rs`
      with a module doc comment following the style of `src/solver.rs`'s
      header (`//! Sequential-impulse solver — README § Resolution.`),
      e.g. `//! Revolute (hinge) joint — a 2-body point constraint.`
- [X] T002 [P] Create `tests/joints.rs` with a module doc comment and a
      `use mirage::{...}` import block, following the shape of
      `tests/friction.rs`'s header
- [X] T003 [P] Create `examples/hinge.rs` with a `#[macroquad::main("hinge")]
      async fn main()` stub (empty loop) following the top-level shape of
      `examples/bouncing.rs`

**Checkpoint**: New files exist and `cargo build`/`cargo test` still pass
(empty module, no behavior change yet).

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: The joint engine itself — the `Mat2` primitive, the
`RevoluteJoint`/`JointId` types, `World` storage for joints, and the
solver/world plumbing that lets a joint's impulse actually get applied.
**No user story is testable until this phase is done.**

**⚠️ CRITICAL**: Complete this phase before starting any user story below.

- [X] T004 [P] Add `Mat2` to `src/math.rs`: fields `m11, m12, m21, m22: f32`;
      `Mat2::new(m11, m12, m21, m22)`; `impl Mul<Vec2> for Mat2` (matrix ×
      vector); `Mat2::invert(&self) -> Mat2` via `1/det * adjugate`. Add
      unit tests in `src/math.rs`'s existing `#[cfg(test)] mod tests`
      block: `invert` undoes a known matrix (`m * m.invert() ≈ identity`
      acting on a test vector), and a rotation-like matrix's product with
      a vector matches hand-computed values. (data-model.md § `Mat2`,
      research.md § 2)
- [X] T005 In `src/joint.rs`, define `pub struct RevoluteJoint { pub body_a:
      BodyId, pub body_b: BodyId, pub anchor_a: Vec2, pub anchor_b: Vec2 }`
      and `impl RevoluteJoint { pub fn new(body_a: BodyId, body_b: BodyId,
      anchor_a: Vec2, anchor_b: Vec2) -> Self }`, and `pub struct
      JointId(pub(crate) u32);` — exact signatures from
      contracts/joint-api.md. *(depends on T001)*
- [X] T006 In `src/world.rs`, add `joints: Vec<RevoluteJoint>` to `World`,
      initialize it in `World::new()`, and add `pub fn add_joint(&mut self,
      joint: RevoluteJoint) -> JointId` (panics if `body_a == body_b` or
      either `BodyId` is out of range for `self.bodies`) and `pub fn
      joint(&self, id: JointId) -> &RevoluteJoint`, mirroring
      `add_body`/`body`'s existing shape. Add unit tests in `src/world.rs`'s
      `#[cfg(test)] mod tests`: `add_joint` returns a usable `JointId`,
      `joint()` returns what was stored, and a joint with `body_a ==
      body_b` panics. *(depends on T005)*
- [X] T007 [P] Add `pub use joint::{JointId, RevoluteJoint};` to
      `src/lib.rs`, next to the existing `pub use body::{BodyId,
      RigidBody};` line. *(depends on T005)*
- [X] T008 Restructure `src/solver.rs`'s `pub(crate) fn resolve(...)` into
      four `pub(crate)` phases with the same signatures its internals
      already imply: `build_contacts(bodies, manifolds, gravity, dt) ->
      Vec<ContactState>`, `warm_start(bodies, &mut Vec<ContactState>,
      &ImpulseCache)`, `iterate_once(bodies, &mut Vec<ContactState>)` (a
      single pass — the body currently inside the `for _ in 0..
      VELOCITY_ITERATIONS` loop, extracted so the caller controls the
      iteration count), and `correct_positions(bodies, &[ContactState])`
      (already a standalone function — no change needed there beyond
      visibility). Keep `resolve()` itself as a thin wrapper calling all
      four in sequence so every existing caller and every existing test in
      `src/solver.rs`'s `#[cfg(test)] mod tests` keeps passing unmodified.
      Do not change any formula, constant, or clamp — this is a pure
      structural split (research.md § 3).
- [X] T009 In `src/joint.rs`, implement the crate-internal `JointState`
      (fields per data-model.md: `a, b: usize`, `r_a, r_b: Vec2`, `k,
      k_inv: Mat2`), `pub(crate) fn build_joints(bodies: &[RigidBody],
      joints: &[RevoluteJoint]) -> Vec<JointState>` (drop a joint whose
      combined `invMass_a + invMass_b == 0`, mirroring `solver.rs`'s `k ==
      0.0` guard), `pub(crate) fn iterate_once(bodies: &mut [RigidBody],
      joints: &mut [JointState])` (one Gauss-Seidel pass: compute `Cdot`,
      solve `P = k_inv * (-Cdot)`, apply `±P` at `r_a`/`r_b` the same way
      `solver::apply_impulse` does, skipping static bodies), and
      `pub(crate) fn correct_positions(bodies: &mut [RigidBody], joints:
      &[JointState], anchors: &[RevoluteJoint])` implementing the
      Baumgarte position correction from research.md § 4, with new
      `pub(crate) const JOINT_SLOP: f32` and `pub(crate) const
      JOINT_CORRECTION_PERCENT: f32` constants (starting values `0.002`
      and `0.4`, matching `solver.rs`'s tuned contact constants as a
      starting point — see T022 for retuning if needed). *(depends on
      T004, T005, T006)*
- [X] T010 In `src/world.rs`'s `World::step`, replace the current
      `solver::resolve(&mut self.bodies, &manifolds, &mut
      self.impulse_cache, self.gravity, FIXED_DT);` call with the shared
      loop from research.md § 3: build contact and joint states, warm-start
      contacts, run `VELOCITY_ITERATIONS` iterations calling
      `solver::iterate_once` then `joint::iterate_once` each pass, then
      call `solver::correct_positions` and `joint::correct_positions`, then
      update the impulse cache. Import `crate::joint` at the top of
      `src/world.rs`. Add a `World::step` doc-comment note (mirroring its
      existing numbered-list style) describing where joints now fit in the
      per-step sequence. *(depends on T008, T009)*

**Checkpoint**: `cargo test` passes with zero new tests failing and zero
regressions in `src/solver.rs`'s and `src/world.rs`'s existing test
modules. The joint engine exists and is wired into every `World::step`,
but nothing has created a `RevoluteJoint` in a real scene yet — that's
User Story 1.

---

## Phase 3: User Story 1 - Connect two bodies at a shared point (Priority: P1) 🎯 MVP

**Goal**: A joint's two anchor points stay coincident, under gravity, for a
sustained run — including when one body is static, when the jointed body
also touches a contact, and when a body has more than one joint.

**Independent Test**: Create two bodies (one may be static), connect them
with `RevoluteJoint`, run the simulation for a sustained period under
gravity, and confirm the anchor points never drift apart beyond
`JOINT_SLOP`.

### Tests for User Story 1

> Write these first; they should fail against a `RevoluteJoint` that
> exists but whose `iterate_once`/`correct_positions` have a bug, and pass
> once Phase 2 is correct.

- [X] T011 [P] [US1] In `tests/joints.rs`, write
      `anchor_stays_coincident_under_gravity`: a static pin body and a
      dynamic body joined by a `RevoluteJoint` with non-center anchors,
      stepped for a sustained run (e.g. 5 s of `World::step`), asserting
      `(world_anchor_a - world_anchor_b).length() <= JOINT_SLOP` (or a
      small tolerance atop it) at every step, not just at the end.
- [X] T012 [P] [US1] In `tests/joints.rs`, write
      `static_body_in_a_joint_never_moves`: a joint between a static and a
      dynamic body; assert the static body's `position`/`velocity`/
      `angular_velocity` are unchanged after a sustained run, the same
      assertion style as `src/world.rs`'s `static_body_never_moves` test.
- [X] T013 [P] [US1] In `tests/joints.rs`, write
      `jointed_body_also_resolves_contacts`: a dynamic body pinned by a
      joint to a fixed point *and* resting on/falling onto a floor (a
      contact), asserting both that the anchor stays coincident and that
      the body doesn't sink through the floor beyond the existing contact
      tolerance (`solver::PENETRATION_SLOP`) — this is spec.md User Story 1
      scenario 3.
- [X] T014 [P] [US1] In `tests/joints.rs`, write
      `chain_of_joints_all_stay_coincident`: three bodies linked by two
      `RevoluteJoint`s (a short chain), run under gravity, and assert every
      joint's anchor pair stays coincident — spec.md's "more than one joint
      on a body" edge case.
- [X] T015 [P] [US1] In `tests/joints.rs`, write
      `identical_joint_scenes_are_bit_identical`: build the same
      joint+body scene twice, run both for N steps, and assert every
      body's state is bit-identical between the two runs — mirrors
      `src/world.rs`'s `identical_scenes_are_bit_identical` test, for
      FR-010.

### Implementation for User Story 1

- [X] T016 [US1] Run `cargo test`, and fix any bug in `src/joint.rs`'s
      `build_joints`/`iterate_once`/`correct_positions` or `src/world.rs`'s
      wiring (from Phase 2) that T011–T015 surface, until every test in
      `tests/joints.rs` and every existing test in `src/solver.rs` /
      `src/world.rs` passes. *(depends on T011-T015 existing, and on
      Phase 2)*

**Checkpoint**: User Story 1 is done — the point constraint itself is
proven correct, alone, pinned-to-static, alongside a contact, and chained.
This is independently demoable via `cargo test --test joints`.

---

## Phase 4: User Story 2 - Pendulum demo validates the joint under gravity (Priority: P2)

**Goal**: `examples/hinge.rs` shows a rod pinned at one end swinging like a
real pendulum, and its measured period matches the analytic prediction.

**Independent Test**: Run the pendulum demo, release from a small angle,
measure the oscillation period, compare to the physical-pendulum formula.

- [X] T017 [US2] Implement `examples/hinge.rs`: a static pin body (small
      circle, per quickstart.md) and a dynamic rod (a tall, narrow polygon)
      joined via `RevoluteJoint::new` at the rod's local top end, released
      from a small angle (set initial `orientation`), rendered each frame
      with macroquad drawing calls adapted from `examples/common/mod.rs`
      (draw the rod as a rotated rectangle, draw the pin as a small
      circle, draw a line for the joint's two anchor points to visually
      confirm they stay together). *(depends on Phase 2, T007)*
- [X] T018 [P] [US2] In `tests/joints.rs`, write
      `pendulum_period_matches_analytic_prediction`: build the same
      pin+rod scene as `examples/hinge.rs` (rod as a known polygon, so its
      mass/inertia are computable via `Shape::inertia`), release from a
      small angle, measure the oscillation period from simulation (time
      between successive returns to the release angle, or successive
      zero-crossings of angular velocity), and assert it is within a few
      percent of the physical-pendulum analytic period computed from the
      rod's actual moment of inertia about the pin (parallel-axis
      theorem: `I_pin = I_center + m·L²`), `T = 2π√(I_pin / (m·g·L))`
      where `L` is the pin-to-center-of-mass distance — per quickstart.md's
      note that the point-mass formula is the wrong comparison here.
- [X] T019 [US2] Manually run `cargo run --example hinge --release`,
      watch the full run, and confirm (per SC-003): no visible jitter, no
      visible separation between the rod's anchor and the pin, and no
      visible growth in swing amplitude over time. Record the observation
      in the PR description — this is the milestone's own "done when"
      criterion (README § Post-MVP milestones, M6) and per Principle IV
      must be verified by actually running the example with `--release`,
      not inferred from `cargo test` passing.

**Checkpoint**: User Stories 1 AND 2 both work — the joint is correct and
the flagship demo proves it behaves like a real pendulum.

---

## Phase 5: User Story 3 - Joint holds up under a longer, less ideal run (Priority: P3)

**Goal**: The joint stays stable and bounded well outside the small-angle,
light-body happy path.

**Independent Test**: Run the pendulum demo (or a variant scene) with a
larger release angle and/or a heavier rod for an extended duration; confirm
the anchor stays coincident and motion stays bounded.

- [X] T020 [P] [US3] In `tests/joints.rs`, write
      `large_angle_pendulum_stays_bounded_and_coincident`: release the same
      rod-on-pin scene from a large angle (e.g. 80°) for an extended run
      (well beyond one period), asserting the anchor gap stays within
      `JOINT_SLOP`-scale tolerance throughout and that angular velocity
      never exceeds a generous bound derived from energy conservation
      (no runaway/divergence).
- [X] T021 [P] [US3] In `tests/joints.rs`, write
      `heavier_rod_pendulum_stays_bounded_and_coincident`: same scene with
      a substantially heavier rod mass, same bounded-motion and
      anchor-coincidence assertions as T020.
- [X] T022 [US3] If T020/T021 reveal instability (divergence, anchor
      separation beyond tolerance), retune `JOINT_SLOP`/
      `JOINT_CORRECTION_PERCENT` in `src/joint.rs` (or, if that's
      insufficient, `solver::VELOCITY_ITERATIONS`'s shared use — see
      research.md § 3) the same way M4 tuned `PENETRATION_SLOP`/
      `CORRECTION_PERCENT`, and document the chosen values and the
      measurements behind them in a doc comment on the constants,
      following `src/solver.rs`'s existing `PENETRATION_SLOP` comment
      style. If T020/T021 already pass with the Phase 2 starting values,
      this task is a no-op — record that in the doc comment instead of
      inventing a change.

**Checkpoint**: All three user stories are independently functional and
tested.

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Bring the milestone to the same finished state M0–M4 and M5
were left in.

- [X] T023 [P] Add a "Joints" subsection to `README.md` § How it works,
      writing out the revolute joint's derivation (the `Cdot`, `K`, and
      impulse formulas from research.md § 1, and the Baumgarte correction
      from research.md § 4) in the same notation/style as § Resolution's
      contact math, per Principle III ("the derivation lands in the same
      change that implements it"). Flip M6's checkbox to `[x]` in §
      Post-MVP milestones.
- [X] T024 [P] Update `CLAUDE.md`: add `cargo run --example hinge
      --release` to the Commands block, add `joint.rs` to the Architecture
      module list, and note the `World::step` loop now interleaves joints
      and contacts (matching the level of detail already given for
      `solver.rs`).
- [X] T025 Run the full `cargo test` suite and confirm `tests/stacking.rs`
      and `tests/friction.rs` pass unmodified (FR-008 / SC-004 — no
      joint-free scene may change behavior).
- [X] T026 Manually re-run `cargo run --example stack --release`,
      `cargo run --example pyramid --release`, `cargo run --example ramp
      --release`, and `cargo run --example bouncing --release` to confirm
      no visible regression from the `World::step`/`solver.rs` restructure
      (Phase 2, T008/T010).

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies — start immediately.
- **Foundational (Phase 2)**: Depends on Phase 1 (T001 creates the file
  T005/T009 edit). **Blocks every user story** — nothing in Phase 3–5 can
  be tested against a joint engine that doesn't exist yet.
- **User Story 1 (Phase 3)**: Depends on Phase 2 completion.
- **User Story 2 (Phase 4)**: Depends on Phase 2 completion; in practice
  build after US1 so the demo is exercising an already-verified engine,
  though it does not depend on US1's *test files* being merged.
- **User Story 3 (Phase 5)**: Depends on Phase 2 completion; like US2, best
  sequenced after US1 for the same reason.
- **Polish (Phase 6)**: Depends on all desired user stories being complete.

### User Story Dependencies

- **US1 (P1)**: No dependency on US2/US3 — the smallest slice that proves
  the joint works at all.
- **US2 (P2)**: Independently testable (`cargo run --example hinge
  --release` + T018's period test) without US3 existing.
- **US3 (P3)**: Independently testable without US2's demo file existing —
  it only needs the same scene built in test code, not the example binary.

### Within Each Phase

- Phase 2: T004 (Mat2) and T005 (types) have no dependency on each other
  and can run in parallel; T006/T007 depend on T005; T008 (solver
  restructure) has no dependency on T004–T007 and can run in parallel with
  them; T009 depends on T004+T005+T006; T010 depends on T008+T009.
- Phase 3: T011–T015 (all test-writing) can run in parallel with each
  other; T016 depends on all of them existing.
- Phase 4: T017 and T018 can run in parallel (different files); T019
  depends on T017.
- Phase 5: T020 and T021 can run in parallel; T022 depends on both.
- Phase 6: T023 and T024 can run in parallel; T025 and T026 can run after
  everything else, in parallel with each other.

### Parallel Opportunities

- Setup: T002, T003 in parallel (T001 is a quick prerequisite for T005 to
  land in a real file, but doesn't block T002/T003).
- Foundational: T004 and T008 are independent of the rest of Phase 2 and
  of each other.
- US1: all five test tasks (T011–T015) in parallel — five different test
  functions in the same new file, no shared mutable state.
- US2: T017 (demo) and T018 (period test) in parallel.
- US3: T020 and T021 in parallel.
- Polish: T023 and T024 in parallel; T025 and T026 in parallel.

---

## Parallel Example: User Story 1

```bash
# All five belong in tests/joints.rs but touch independent test functions —
# safe to write/assign in parallel, then run together:
Task: "Write anchor_stays_coincident_under_gravity in tests/joints.rs"
Task: "Write static_body_in_a_joint_never_moves in tests/joints.rs"
Task: "Write jointed_body_also_resolves_contacts in tests/joints.rs"
Task: "Write chain_of_joints_all_stay_coincident in tests/joints.rs"
Task: "Write identical_joint_scenes_are_bit_identical in tests/joints.rs"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1 (Setup) and Phase 2 (Foundational) — the joint engine.
2. Complete Phase 3 (User Story 1) — prove the point constraint itself is
   correct, alone, pinned-to-static, alongside a contact, and chained.
3. **STOP and VALIDATE**: `cargo test --test joints` all green, no
   regression in `cargo test` overall.
4. This is the earliest point the constitution's "done when" bar for M6
   (anchor coincidence) is met, though the full milestone (README's own
   "done when," which names the pendulum's period) needs Phase 4 too.

### Incremental Delivery

1. Setup + Foundational → the engine exists, wired into `World::step`.
2. Add User Story 1 → verify independently → the constraint is correct.
3. Add User Story 2 → verify independently (watch the demo with
   `--release`, per Principle IV) → the milestone's own "done when"
   criterion is met.
4. Add User Story 3 → verify independently → robustness margin beyond the
   minimum bar is documented.
5. Polish → README/CLAUDE.md catch up to the code (Principle III), full
   regression pass across every existing demo and test.

---

## Notes

- `[P]` tasks touch different files, or different independent test
  functions in the same new file — no shared mutable state.
- `[Story]` labels map every Phase 3+ task to spec.md's US1/US2/US3 for
  traceability back to the acceptance scenario it proves.
- Per this repo's constitution (Principle IV), a milestone's "done when"
  criterion that is observable via an example (T019 here) must be verified
  by actually watching that example run with `--release` — `cargo test`
  passing is necessary but not sufficient for calling M6 complete.
- Commit after each task or logical group, per the project's git workflow
  (`CLAUDE.md` § Git workflow: feature branch + PR, never direct to
  `main`).
