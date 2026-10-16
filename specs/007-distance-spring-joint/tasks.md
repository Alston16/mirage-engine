# Tasks: Distance / Spring Joint

**Input**: Design documents from `specs/007-distance-spring-joint/`
**Prerequisites**: [plan.md](plan.md), [spec.md](spec.md), [research.md](research.md), [data-model.md](data-model.md), [contracts/distance-joint-api.md](contracts/distance-joint-api.md), [quickstart.md](quickstart.md)

**Tests**: Included. The constitution (Principle IV) requires `cargo test`
to cover solver behavior, and it requires the milestone's done-when to be
verified by running its example with `--release`. Tests are not optional
in this repository.

**Organization**: Tasks are grouped by user story (spec.md P1/P2/P3). They
sit on a shared Foundational phase that adds the joint type, `World`
storage and per-step working state. US1 (spring) and US2 (rigid) each add
one independent solver phase, and US3 (demo) needs only US1.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependency on an
  incomplete task)
- **[Story]**: Which user story this task belongs to (US1, US2, US3)
- Every task names its exact file path(s)

## Path Conventions

This is a single crate: `src/`, `tests/` and `examples/` at the repository
root (plan.md § Project Structure). New engine code goes in
`src/joint/distance.rs`, a submodule of the existing `src/joint.rs`.
`RevoluteJoint`'s code in `src/joint.rs` is **not** edited, apart from the
`mod`/`pub use` lines (research.md § 5).

---

## Phase 1: Setup

**Purpose**: Scaffolding only. No behavior yet.

- [X] T001 Add `pub mod distance;` near the top of `src/joint.rs` (after the
      `use` lines, before `RevoluteJoint`), and create
      `src/joint/distance.rs` with the module doc comment
      `//! Distance / spring joint: a 2-body constraint on the distance
      between two anchors, rigid or soft. See README § Joints.`
      followed by `use crate::{BodyId, RigidBody, Vec2};` and
      `use super::{JOINT_CORRECTION_PERCENT, JOINT_SLOP};`. Add
      `#[allow(unused_imports)]` only if the build needs it until T006/T014
      use them.
- [X] T002 [P] Create `tests/distance_joints.rs` with a `//!` header stating
      the analytic references used (critical damping `c = 2√(k·m)`, damped
      period `T_d = 2π / (ω·√(1−ζ²))` with `ω = √(k/m)`, `ζ = c / (2√(k·m))`,
      and simple pendulum `T = 2π√(L/g)`), then `mod common;`,
      `use common::*;` and
      `use mirage::{BodyId, DistanceJoint, RigidBody, Shape, Vec2, World};`.
      Follow the shape of `tests/joints.rs`'s header. It won't compile until
      T004/T005 land.
- [X] T003 [P] Create `examples/spring.rs` with a `//!` doc comment ("M7
      acceptance demo: boxes hanging on springs …"), `mod common;`,
      `use common::*;`, `use macroquad::prelude::*;` and a
      `#[macroquad::main("spring")] async fn main()` loop that only clears
      the screen and calls `next_frame().await`. Follow the top-level shape
      of `examples/hinge.rs`.

**Checkpoint**: `cargo build` passes and `cargo test` passes for the
existing suites. The two new test/example files may fail to compile until
Phase 2. If so, comment out their `use mirage::…` line temporarily.

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: The public types, `World` storage and validation, and the
per-step working state that both modes share. **No user story is testable
until this phase is done.**

- [X] T004 In `src/joint/distance.rs`, define the public items from
      contracts/distance-joint-api.md with those exact signatures, derives
      and doc comments:
      - `DistanceJoint { body_a, body_b, anchor_a, anchor_b, length: Option<f32>, spring: Option<Spring> }`
      - `DistanceJoint::new(body_a, body_b, anchor_a, anchor_b)`, which
        returns `length: None, spring: None`
      - `with_length(self, length) -> Self`, which sets `Some(length)`
      - `with_spring(self, stiffness, damping) -> Self`, which sets
        `Some(Spring { stiffness, damping })`
      - `critical_damping(stiffness, mass) -> f32`, returning
        `2.0 * (stiffness * mass).sqrt()`
      - `Spring { pub stiffness: f32, pub damping: f32 }`
      - `DistanceJointId(pub(crate) u32)`

      Also add `pub(crate) const DISTANCE_EPSILON: f32 = 1e-6;` with a doc
      comment citing research.md § 6. In `src/joint.rs`, add
      `pub use distance::{DistanceJoint, DistanceJointId, Spring};`.
      *(depends on T001)*
- [X] T005 Extend the re-export in `src/lib.rs` to
      `pub use joint::{DistanceJoint, DistanceJointId, JointId, RevoluteJoint, Spring};`.
      *(depends on T004)*
- [X] T006 In `src/world.rs`:
      - Add `distance_joints: Vec<DistanceJoint>` to `World` and initialize
        it in `World::new()`.
      - Add `pub fn add_distance_joint(&mut self, joint: DistanceJoint) -> DistanceJointId`.
        It asserts the same three preconditions as `add_joint` (reuse the
        same message wording, e.g. "a joint cannot connect a body to
        itself"). It then asserts that `length` (if `Some`),
        `spring.stiffness` and `spring.damping` are finite and `>= 0.0`,
        with messages naming the field.
      - In `add_distance_joint`, if `length` is `None`, set it to
        `Some(|p_b − p_a|)`, where `p = body.position + body.orientation.rotate(anchor)`.
        Then push the joint and return the id.
      - Add `pub fn distance_joint(&self, id: DistanceJointId) -> &DistanceJoint`.

      Add unit tests to `src/world.rs`'s `mod tests`:
      - the id round-trips and the stored fields match;
      - `length: None` resolves to the anchors' separation, including a
        rotated body with an off-centre anchor;
      - an explicit `with_length` is kept as given;
      - `#[should_panic]` for self-connection, negative length, negative
        stiffness, negative damping and NaN length.

      *(depends on T004)*
- [X] T007 In `src/joint/distance.rs`, add `pub(crate) struct DistanceState`
      with fields `a, b: usize`, `r_a, r_b, n: Vec2`, `c: f32` (`C = L − L0`), `length: f32` (resolved `L0`, used by T022),
      `k_inv: f32` and `spring: Option<Spring>`, with field doc comments
      from data-model.md. Then add
      `pub(crate) fn build_distance_joints(bodies: &[RigidBody], joints: &[DistanceJoint]) -> Vec<DistanceState>`:
      - Skip a joint whose `inv_mass_a + inv_mass_b == 0.0`, mirroring
        `joint::build_joints`.
      - Compute `r = orientation.rotate(anchor)`, `d = p_b − p_a` and
        `L = |d|`, and skip if `L < DISTANCE_EPSILON` (never normalize).
      - Compute `n = d / L` and `c = L − joint.length.expect("resolved by World::add_distance_joint")`.
      - Compute `K = inv_mass_a + inv_mass_b + (r_a×n)²·inv_I_a + (r_b×n)²·inv_I_b`
        and store `k_inv = 1/K`.

      Add two private helpers mirroring `joint.rs`:
      - `fn separation_rate(bodies, s) -> f32`, returning
        `Cdot = n · ((v_b + ω_b × r_b) − (v_a + ω_a × r_a))`;
      - `fn apply_impulse(bodies, s, lambda: f32)`, which applies
        `P = λ·n` as `−P` to `a` and `+P` to `b` (linear plus
        `r × P · inv_inertia`) and skips static bodies.

      Unit tests in `distance.rs`'s `#[cfg(test)] mod tests` (using the
      same `dynamic`/`static_body` helpers as `joint.rs`):
      - a joint between two static bodies is dropped;
      - coincident anchors (`L = 0`) are dropped and no state contains NaN;
      - `n` points from `a`'s anchor to `b`'s and `c` has the right sign
        when stretched and when compressed;
      - `k_inv` equals `1/(1/m_a + 1/m_b)` when both anchors lie on the
        line through both centres (so `r × n = 0`).

      *(depends on T004)*

**Checkpoint**: `cargo test` passes. Distance joints can be added and
inspected but have no effect on simulation yet. All pre-existing tests
are unchanged.

---

## Phase 3: User Story 1 - Hang a body from a point on a spring (Priority: P1) 🎯 MVP

**Goal**: Soft mode works. A box on a spring oscillates about its
equilibrium length. Critical damping settles without overshoot. Undamped
motion keeps its energy.

**Independent Test**: `cargo test --test distance_joints spring` passes
SC-001, SC-002 and SC-003, plus the free-space and degenerate-case tests
below, with no rigid-mode code present.

### Tests for User Story 1 (write first; they must fail before T014)

Shared scene helper for this phase, in `tests/distance_joints.rs`:
`fn hanging_box(world, k, c, displacement) -> (BodyId hook, BodyId bx)`
builds:
- a static hook at `(0, 8)`;
- a 1 kg box made with `box_shape(0.5)`, anchored at local `(0, 0.5)`
  (top centre), directly below the hook;
- a spring `DistanceJoint::new(hook, bx, Vec2::ZERO, Vec2::new(0.0, 0.5)).with_length(2.0).with_spring(k, c)`.

The box starts at rest with its anchor `2.0 + 9.81/k + displacement`
below the hook. Also add `fn stretch_from_equilibrium(world, hook, bx, k) -> f32`,
returning `L − (2.0 + 9.81/k)`. Use `k = 10.0` and `displacement = 1.0`
throughout (research.md § 8).

- [X] T008 [P] [US1] `critically_damped_spring_settles_without_overshoot`
      in `tests/distance_joints.rs` (SC-001, US1 #2). Use
      `c = DistanceJoint::critical_damping(10.0, 1.0)` and step 20 s.
      Assert that `stretch_from_equilibrium` never goes below `−0.01`
      (no overshoot beyond 1% of the 1 m displacement). Also assert that
      after 10 s it stays within `±0.01`.
- [X] T009 [P] [US1] `undamped_spring_conserves_energy` in
      `tests/distance_joints.rs` (SC-002, US1 #3). Use `c = 0` and step
      60 s. Each step, compute
      `E = ½m|v|² + ½Iω² + m·g·y + ½k(L − 2.0)²`, where
      `I = box_shape(0.5).inertia(1.0)`. Normalize by `E_osc = ½·k·1.0²`.
      Assert:
      - `max |E − E₀| / E_osc < 0.05`;
      - the mean of `E` over the last period is within `0.01·E_osc` of
        the mean over the first period, where period `T = 2π/√k`,
        rounded to whole steps.

      Print both numbers with `println!` for `--nocapture`.
- [X] T010 [P] [US1] `underdamped_spring_period_matches_analytic` in
      `tests/distance_joints.rs` (SC-003, US1 #1). Use
      `c = 0.1 · critical_damping(10, 1)` and step 30 s. Collect the step
      indices where `stretch_from_equilibrium` crosses zero going upward.
      Assert:
      - the mean spacing × `DT` is within 2% of `2π / (√10 · √(1 − 0.01))`;
      - the amplitude of the last full swing is smaller than the first
        (it rings down).
- [X] T011 [P] [US1] `spring_between_two_free_bodies_conserves_momentum` in
      `tests/distance_joints.rs` (US1 #4). `World` has a fixed gravity and
      no setter, so this test cancels gravity out instead of disabling it.
      Place two equal 1 kg `Shape::circle(0.1)` bodies at `(−1.5, 50)` and
      `(1.5, 50)` (3.0 apart, 1.0 past rest) and join them at their
      centres with `with_length(2.0).with_spring(10.0, 0.0)`. In the same
      world, add an unjointed reference circle at `(10, 50)`. Step 5 s and
      assert every step that:
      - the pair's midpoint x is `0 ± 1e-4`;
      - the sum of their x-velocities is `0 ± 1e-4` (momentum);
      - the midpoint y matches the reference body's y within `1e-3`
        (gravity acts equally, and the spring adds no net force);
      - the separation stays within `[0.9, 3.1]` and reaches below 1.1 at
        least once. Undamped and released 1.0 past rest, it swings evenly
        about 2.0, between 1.0 and 3.0.
- [X] T012 [P] [US1] `spring_with_coincident_anchors_stays_finite` in
      `tests/distance_joints.rs` (FR-014, Edge Cases). Join a dynamic
      circle to a static hook at the same point with
      `with_length(0.0).with_spring(10, 1)`, step 2 s, and assert that
      every component of position and velocity is finite every step. In
      a second case, start 1 m apart with rest length 0 and assert that
      the body is pulled toward the hook (the distance at 0.5 s is less
      than 1.0) and stays finite.
- [X] T013 [P] [US1] `spring_force_acts_only_along_the_anchor_line` in
      `src/joint/distance.rs` unit tests (FR-004, FR-006). Use two dynamic
      bodies with anchors at their centres, separated diagonally and
      stretched. After one `apply_spring_forces` call, assert:
      - the change in each body's velocity is parallel to `n` (the cross
        product with `n` is about 0);
      - the changes are equal and opposite scaled by mass, so momentum is
        unchanged;
      - angular velocity is unchanged.

      Also add `apply_spring_forces_skips_rigid_joints` and
      `apply_spring_forces_never_moves_a_static_body`.

### Implementation for User Story 1

- [X] T014 [US1] In `src/joint/distance.rs`, add
      `pub(crate) fn apply_spring_forces(bodies: &mut [RigidBody], joints: &[DistanceState], dt: f32)`.
      For each state with `spring: Some(Spring { stiffness: k, damping })`:
      - let `cdot = separation_rate(bodies, s)`;
      - let `lambda = -(k * s.c + damping * cdot) * dt`;
      - call `apply_impulse(bodies, s, lambda)`.

      Write the doc comment as the README derivation (`F = −(k·C + c·Cdot)`,
      `λ = F·dt`). It must explain that this runs **before** gravity's kick
      so the damper never sees `g·dt` as separation speed (research.md § 2).
      *(depends on T007)*
- [X] T015 [US1] Wire springs into `World::step` in `src/world.rs`. At the
      top of the `while` loop, *before* the gravity loop, add:

      ```rust
      let mut distance = joint::distance::build_distance_joints(&self.bodies, &self.distance_joints);
      joint::distance::apply_spring_forces(&mut self.bodies, &distance, FIXED_DT);
      ```

      Build the states once per step: positions don't change between the
      spring pass and the solver, because integration happens last, so
      the same `distance` vec is reused by US2. Update `step`'s doc comment
      step list to add a step 0, "spring forces from start-of-step state
      (see `joint::distance`)". Make sure a world with an empty
      `distance_joints` performs no extra work that alters results
      (FR-011). *(depends on T006, T014)*
- [X] T016 [US1] Run `cargo test --test distance_joints -- --nocapture` and
      confirm T008–T013 pass. Record the measured overshoot, energy drift
      and period error in the commit message or PR description. If
      SC-001 fails with explicit damping, stop and revisit research.md § 1
      (the implicit-damping alternative) before loosening any bound.

**Checkpoint**: Soft mode is complete and meets M7's done-when as
measured headlessly.

---

## Phase 4: User Story 2 - Hold two bodies at a fixed distance (Priority: P2)

**Goal**: Rigid mode holds the anchor separation at `L0` and leaves
everything else free.

**Independent Test**: `cargo test --test distance_joints rigid` passes
SC-004 and the gradual-correction scenario. It needs only Phase 2 plus
this phase, and does not depend on US1's spring code.

### Tests for User Story 2 (write first)

- [X] T017 [P] [US2] `rigid_distance_holds_length_under_gravity` in
      `tests/distance_joints.rs` (SC-004, US2 #1). Set up:
      - a static pivot at `(0, 5)`;
      - a 1 kg `Shape::circle(0.1)` bob at `(2·sin 0.5, 5 − 2·cos 0.5)`;
      - `DistanceJoint::new(pivot, bob, Vec2::ZERO, Vec2::ZERO)` with no
        `with_length`.

      Assert that `world.distance_joint(id).length == Some(≈2.0)`. Then,
      for 600 steps, assert that `|L − 2.0| < ANCHOR_TOL`, defining
      `const ANCHOR_TOL: f32 = 0.02;` with the same rationale comment as
      `tests/joints.rs`.
- [X] T018 [P] [US2] `rigid_distance_pendulum_period_matches_simple_pendulum`
      in `tests/distance_joints.rs` (SC-004, US2 #2). Use the same scene
      released at 0.1 rad and step 20 s. Measure the period from
      successive upward zero crossings of the bob's x. Assert that it is
      within 2% of `2π√(2/9.81)`. Also assert that the bob's
      `angular_velocity` stays within `1e-3` of 0 (direction and rotation
      are unconstrained, and an anchor at the centre applies no torque).
- [X] T019 [P] [US2] `rigid_distance_corrects_initial_gap_gradually` in
      `tests/distance_joints.rs` (US2 #3, FR-007). Use two dynamic circles
      in a world far from anything, 3.0 apart, with `with_length(2.0)`.
      After one step, assert that the separation decreased but by no more
      than `0.4·(1.0 − 0.002) + small velocity term` (no snap). After
      120 steps, assert that it is within `ANCHOR_TOL` of 2.0. Repeat
      with a compressed start (1.0 apart, target 2.0), which must grow.
- [X] T020 [P] [US2] Unit tests in `src/joint/distance.rs`:
      - `iterate_once_drives_separation_rate_to_zero`: two bodies moving
        apart along `n`; after 16 iterations
        `|separation_rate| < 1e-4`, and the perpendicular velocity is
        unchanged.
      - `iterate_once_skips_spring_joints`.
      - `correct_positions_moves_along_n_by_sign_of_c`: stretched pulls
        in and compressed pushes out, by at most
        `JOINT_CORRECTION_PERCENT·(|C| − JOINT_SLOP)`.
      - `correct_positions_leaves_error_within_slop_alone`.

### Implementation for User Story 2

- [X] T021 [US2] In `src/joint/distance.rs`, add
      `pub(crate) fn iterate_once(bodies: &mut [RigidBody], joints: &[DistanceState])`.
      For each state with `spring: None`, compute
      `lambda = -separation_rate(bodies, s) * s.k_inv` and call
      `apply_impulse(bodies, s, lambda)`. There is no clamp, because this
      is an equality constraint. The doc comment should give the derivation
      from research.md § 3 and note that it is the contact normal's `Δj`
      without the `≥ 0` clamp. *(depends on T007)*
- [X] T022 [US2] In `src/joint/distance.rs`, add
      `pub(crate) fn correct_positions(bodies: &mut [RigidBody], joints: &[DistanceState])`.
      For each rigid state (`spring: None`), skip if `inv_sum == 0`.
      **Recompute from current positions; do not use the step-start
      `s.c`/`s.n`.** Contact and revolute correction run first and may
      have moved these bodies, and an earlier distance joint in the same
      pass may have too, as in chains. This mirrors
      `joint::correct_positions`, which recomputes its gap the same way.
      - Compute `d = (pos_b + r_b) − (pos_a + r_a)` (keep the step's
        `r_a`/`r_b`, as the revolute joint does), then `L = |d|`.
      - Skip if `L < DISTANCE_EPSILON`. Otherwise set `n = d / L` and
        `c = L − L0`.
      - Skip if `|c| <= JOINT_SLOP`.
      - Compute
        `shift = n * (JOINT_CORRECTION_PERCENT * (|c| − JOINT_SLOP) * c.signum() / inv_sum)`.
      - Move `a.position += shift * inv_a` and
        `b.position -= shift * inv_b`, skipping static bodies.

      `L0` must be available here, so add `length: f32` (the resolved
      `L0`) to `DistanceState` in T007's struct. Mirror
      `joint::correct_positions`'s doc comment and its "correct position,
      not velocity" rationale. T020's
      `correct_positions_moves_along_n_by_sign_of_c` must also cover a
      case where one body was moved after `build_distance_joints`: the
      correction must use the moved position. *(depends on T007)*
- [X] T023 [US2] Wire rigid mode into `World::step` in `src/world.rs`:
      - inside the velocity-iteration `for` loop, after
        `joint::iterate_once`, add
        `joint::distance::iterate_once(&mut self.bodies, &distance);`;
      - after `joint::correct_positions`, add
        `joint::distance::correct_positions(&mut self.bodies, &distance);`.

      If US1 isn't merged yet, add the `build_distance_joints` line from
      T015 here instead (before gravity). Update `step`'s doc comment to
      list rigid distance joints in the iteration and correction steps.
      *(depends on T021, T022; shares the `distance` binding with T015)*
- [X] T024 [US2] Run `cargo test --test distance_joints -- --nocapture` and
      confirm T017–T020 pass along with all of US1's tests.

**Checkpoint**: Both modes work. The milestone's engine work is complete.

---

## Phase 5: User Story 3 - Spring demo watched end to end (Priority: P3)

**Goal**: `examples/spring.rs` shows M7's behavior visually, the same
acceptance gate as M0–M6.

**Independent Test**: `cargo run --example spring --release` shows three
boxes behaving as their configurations predict.

- [X] T025 [US3] Implement `examples/spring.rs`, modelled on
      `examples/hinge.rs`'s `Scene` / `build_scene()` / R-to-reset
      structure:
      - Use constants `K = 10.0`, `MASS = 1.0`, `REST = 2.0`, `HALF = 0.5`
        and `RELEASE = 1.0` (metres past equilibrium).
      - Hang three boxes from static hooks at `x = −3, 0, 3`, `y = 8`,
        built as in the US1 `hanging_box` helper with `c` set to:
        - `0.1·critical_damping(K, MASS)`, labelled "under-damped";
        - `critical_damping(K, MASS)`, labelled "critical";
        - `0.0`, labelled "undamped".
      - Draw each box with `draw_polygon_outline(…, rect_vertices(HALF, HALF), …)`,
        the spring as a `draw_line` from the hook's `to_screen` to the
        box's world anchor, and each hook as a small dot.
      - Draw a thin horizontal tick at each box's equilibrium anchor
        height (`8 − REST − MASS·9.81/K`), so overshoot is visible.
      - Show a `draw_hud` line per box with its label and current
        `L − L_eq`, plus "R: release again".
      - Step with `world.step(get_frame_time())`, as the other demos do.

      *(depends on T015; does not need US2)*
- [ ] T026 [US3] Run `cargo run --example spring --release` and watch for
      at least 30 s (US3 #1, #2, SC-005). Check that:
      - the under-damped box rings down;
      - the critical box reaches its equilibrium tick without crossing it;
      - the undamped box keeps the same amplitude;
      - there is no jitter, sinking or energy gain.

      Press R to confirm the reset reproduces the same motion
      (determinism). Record what was observed in the PR description.

**Checkpoint**: M7's done-when is verified visually with `--release`
(Principle IV).

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Cross-mode edge cases, documentation (Principle III) and
regression checks.

- [X] T027 [P] `spring_and_revolute_on_one_body` in
      `tests/distance_joints.rs` (Edge Cases: "a hinged lid held open by
      a spring"). Pin a rod at one end with a `RevoluteJoint` to a static
      body, and attach a spring from the rod's free end to a static hook
      above it. Step 10 s. Assert that:
      - the revolute anchor gap stays below 0.02 throughout;
      - every value stays finite;
      - with damping, the rod comes to rest (`|ω| < 0.01` after 10 s).
- [X] T028 [P] `distance_joint_scene_is_deterministic` in
      `tests/distance_joints.rs` (FR-013). Build the T010 under-damped
      scene plus a T017 rigid pendulum in one world, twice. Step both
      600 times and assert that all body positions and velocities are
      bit-equal (`assert_eq!`).
- [X] T029 [P] `rigid_distance_joint_with_contact` in
      `tests/distance_joints.rs` (Edge Cases, FR-008). Build the scene so
      that both constraints can hold at once, rather than fighting each
      other:
      - a static floor from `common::floor(0.5)`, with its top at `y = 0`;
      - a static hook at `(0, 3)`;
      - a 1 kg `box_shape(0.5)` centred at `(0, 0.5)`, so its bottom face
        rests exactly on the floor;
      - a rigid joint from the hook to the box's top-centre anchor
        `(0, 0.5)`, with no `with_length`. This resolves to 2.0, the
        separation that is already satisfied while the box rests on the
        floor.

      Run two cases.
      - **At rest**: step 5 s. Assert every step that all values are
        finite, `|L − 2.0| < ANCHOR_TOL` and floor penetration
        (`−(box bottom y)`) `< 0.05`. Also assert speed `< 0.05` after 3 s.
      - **Kicked**: the same scene with the box's initial `velocity.x = 1.0`.
        It swings up off the floor on its rod and comes back down onto
        it. Step 5 s and assert every step only that all values are
        finite and `|L − 2.0| < ANCHOR_TOL`. The contact while it tilts
        is ordinary solver territory, not a joint guarantee.
- [X] T030 [P] Update `README.md` (Principle III; the derivation lands
      with the code):
      - In § How it works › Joints, add a "Distance / spring joint"
        subsection with the rigid derivation from research.md § 3 and § 4
        and the spring formula from research.md § 1. Use the code's
        variable names: `d`, `L`, `n`, `C`, `Cdot`, `K`, `λ`, `P`, `k`,
        `c`, `L0`.
      - Explain why the spring is applied before gravity (research.md
        § 2) and why it is explicit rather than a soft constraint (energy
        conservation).
      - Note `critical_damping`, and add known trade-offs: stability only
        while `ω·dt < 2`, and the restitution/spring-kick interaction
        (research.md § 9).
      - Update § Architecture's tree (`joint.rs` plus
        `joint/distance.rs`, `tests/distance_joints.rs`,
        `examples/spring.rs`) and the § Status paragraph.
      - Flip M7 to `[x]` in § Post-MVP milestones and add
        `cargo run --example spring --release` to § Running it.
- [X] T031 [P] Update `CLAUDE.md`:
      - § Project state: say M7 has landed.
      - § Commands: add `cargo run --example spring --release   # M7 acceptance demo`.
      - § Architecture: add `joint/distance.rs` with a one-line role.
      - The data-flow paragraph: spring forces from start-of-step state
        before gravity, and rigid distance joints iterate after revolute
        joints and correct after them.
      - § Working on this repo: say M7 is done and M8 is not started.
- [X] T032 Run `cargo test` (all suites) and `cargo clippy --all-targets`.
      Confirm `tests/stacking.rs`, `tests/friction.rs`, `tests/joints.rs`
      and `tests/parabola.rs` pass **unmodified** (FR-011, SC-006).
- [ ] T033 Manually re-run `cargo run --example hinge --release` and
      `cargo run --example stack --release` and confirm there is no
      visible regression from the `World::step` changes (SC-006).

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: no dependencies.
- **Foundational (Phase 2)**: depends on Setup. Blocks every user story.
- **US1 (Phase 3)**: depends on Phase 2.
- **US2 (Phase 4)**: depends on Phase 2. Independent of US1, except that
  T023 shares the `distance` binding introduced by T015. Whichever lands
  second reuses it.
- **US3 (Phase 5)**: depends on US1 (T015). Does not need US2.
- **Polish (Phase 6)**: T027 needs US1. T028 and T029 need US1 and US2.
  T030 and T031 should be written once both modes' behavior is final.
  T032 and T033 come last.

### User Story Dependencies

- **US1 (P1)**: Foundational only. It is the MVP and carries M7's
  done-when.
- **US2 (P2)**: Foundational only.
- **US3 (P3)**: US1.

### Within Each Phase

- Tests are written before the implementation they cover and fail first
  (T008–T013 before T014; T017–T020 before T021–T022).
- T004 → T005 / T006 / T007. T014 → T015. T021 + T022 → T023.
- `src/world.rs` is touched by T006, T015 and T023. Those tasks are
  sequential, never [P] with each other.
- `src/joint/distance.rs` is touched by T004, T007, T013, T014, T020,
  T021 and T022. These are sequential within the file.

### Parallel Opportunities

- T002 ∥ T003 (Setup).
- T006 ∥ T007 after T004 (different files: `world.rs` vs `distance.rs`).
- T008–T012 are all [P]: separate test functions in one new file, with no
  ordering between them.
- T017–T019 are [P] in the same way.
- T027–T031 are [P] (tests vs README vs CLAUDE.md).
- US1 and US2 implementation can proceed in parallel after Phase 2, except
  for `distance.rs`/`world.rs` edit conflicts. Serialize those edits.

---

## Parallel Example: User Story 1

```text
# After Phase 2, write all US1 tests together:
T008 critically_damped_spring_settles_without_overshoot   (tests/distance_joints.rs)
T009 undamped_spring_conserves_energy                     (tests/distance_joints.rs)
T010 underdamped_spring_period_matches_analytic           (tests/distance_joints.rs)
T011 spring_between_two_free_bodies_conserves_momentum    (tests/distance_joints.rs)
T012 spring_with_coincident_anchors_stays_finite          (tests/distance_joints.rs)
T013 spring_force_acts_only_along_the_anchor_line         (src/joint/distance.rs)
# Then T014 → T015 → T016.
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Phase 1 → Phase 2.
2. Phase 3 (US1): springs, with SC-001, SC-002 and SC-003 measured.
3. **Stop and validate**: M7's done-when criteria (critical damping with
   no overshoot, undamped energy) are already checkable headlessly here.

### Incremental Delivery

1. US1, the spring mode, carries the done-when.
2. US2, the rigid mode, completes the README's "rigid or soft".
3. US3, the demo, is the visual gate watched with `--release`.
4. Polish: cross-mode tests, README derivation and CLAUDE.md, then a
   regression sweep.

All of it ships in one PR from `007-distance-spring-joint` (CLAUDE.md:
never push to `main`).

---

## Notes

- Do not edit `RevoluteJoint`, `build_joints`, `joint::iterate_once`,
  `joint::correct_positions` or anything in `src/solver.rs`. M7 is
  additive (plan.md Constraints).
- If a measured bound fails, record the number and revisit the research
  decision it came from. Do not silently widen the test tolerance.
- Out of scope (spec Assumptions, Principle V): rope/one-sided mode,
  length limits, breakable joints, joint removal, a force accumulator on
  `RigidBody`, and M8 motor.
