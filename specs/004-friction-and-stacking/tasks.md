---
description: "Task list for M4: Friction & Stable Stacking"
---

# Tasks: Friction & Stable Stacking (M4)

**Input**: Design documents from `/specs/004-friction-and-stacking/`

**Prerequisites**: plan.md, spec.md (including its 2026-09-20 Clarifications), research.md, data-model.md, contracts/engine-api.md, quickstart.md

**Tests**: Included. The spec's Success Criteria (SC-001–SC-009) and Constitution Principle IV require behavioral verification. Solver-internal invariants are inline `#[cfg(test)]` tests (existing convention); end-to-end scenes (ramp, 60 s tower, pyramid, determinism) are integration tests in a new `tests/` directory that uses only the public API (plan.md § Structure Decision).

**Organization**: Tasks are grouped by user story (from spec.md) so each can be implemented and verified on its own. Two deliberate exceptions to strict independence: (1) US2 and US3 depend on the warm-starting built in US2 (research.md § Prototype evidence: friction alone still collapses the tower); (2) the US3 scenes and two stress tests are *authored* in Phase 4, before tuning, because FR-006 requires the iteration count to be tuned against the tower **and** the pyramid.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies on incomplete tasks)
- **[Story]**: Which user story this task belongs to (US1–US4)
- Exact file paths are included in every task

## Path Conventions

Single project (per plan.md): `src/`, `tests/`, `examples/` at the repository root. All `cargo` commands run from the repo root. Never push to `main`; all work stays on branch `004-friction-and-stacking`.

## Conventions used by many tasks

- **World units**: demos and tests use meters (1 m boxes, mass 1, `g = 9.81`). Constants in `solver.rs` are calibrated for this scale.
- **Ramp geometry** (used by tests and `ramp.rs`): a static rectangle centered at `c`, rotated by `θ` (`Rot2::new(θ)`, CCW). Its upward surface normal is `R(θ)·(0, 1)`; a box of half-extent `b` sits exactly touching it at center `c + R(θ)·(x, h_ramp + b)`. Downhill direction is `(−cos θ, −sin θ)`.
- **Analytic references**: hold/slide threshold `θ = atan(μ)`; sliding `a = g(sin θ − μ cos θ)`; frictionless `a = g sin θ`; rolling solid disc `a = (2/3)·g·sin θ`.
- **Speed metric**: `max(|v|, |ω|·half_extent)` — the definition used by SC-002.
- **Stop rule**: if a spec bound cannot be met after the tuning tasks, STOP and take the recorded data to `/speckit-clarify`. Do not loosen a bound in a test to make it pass. (The SC-001/SC-002 bounds were already re-based on prototype data in the spec's 2026-09-20 Clarifications; a second miss means the solver needs something beyond M4's stated levers.)
- **Where evidence goes**: `research.md` gets a new section "In-repo notes" (created by whichever task first writes to it) and § "Tuning results".

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Confirm M3 is really done, and create the shared test/demo scaffolding.

- [X] T001 Gate 0 (Constitution IV): on branch `004-friction-and-stacking` run `cargo test` and confirm the baseline (69 passed), then run `cargo run --example bouncing --release` and observe that the `e = 0` ball stops dead and the `e = 1` ball returns near its drop height. Record both results under "In-repo notes" in `specs/004-friction-and-stacking/research.md`; do not start solver changes if either fails.
- [X] T002 [P] Create `tests/common/mod.rs` (start with `#![allow(dead_code)]`, since each test file uses a subset) containing scene builders that use only the public API: `pub const DT: f32 = 1.0 / 60.0;`, `pub const G: f32 = 9.81;`, `pub fn unit_box() -> Shape` (half-extent 0.5), `pub fn floor(friction: f32) -> RigidBody` (static rectangle, top surface at `y = 0`, half-width 100), `pub fn ramp(angle: f32, friction: f32) -> RigidBody` (static rectangle half-extents `(20, 0.5)` at the origin rotated by `angle`), `pub fn box_on_ramp(world: &mut World, angle: f32, box_friction: f32, ramp_friction: f32) -> BodyId` (1 m box, mass 1, placed exactly touching per the ramp geometry above), `pub fn disc_on_ramp(world: &mut World, angle: f32) -> BodyId` (radius 0.5, mass 1), `pub fn tower(world: &mut World, n: usize) -> Vec<BodyId>` (floor added first, then `n` unit boxes mass 1 at `(0, 0.5 + i)`), and `pub fn pyramid(world: &mut World, rows: usize) -> Vec<BodyId>` (row `r` has `rows − r` boxes at `x = (i − (count−1)/2)`, `y = 0.5 + r`, spacing exactly 1.0).
- [X] T003 In `tests/common/mod.rs` add the metrics and thresholds (depends on T002, same file): `pub fn speed(body: &RigidBody, half_extent: f32) -> f32` = `max(|v|, |ω|·half_extent)`; `pub fn sink(world: &World, top: BodyId, ideal_y: f32) -> f32` = `ideal_y − top.position.y`; `pub fn max_drift(world, ids, x0) -> f32`; and threshold constants copied from the spec with a comment naming the criterion: `DRIFT_MAX = 0.05`, `SINK_ALWAYS_MAX = 0.10`, `SINK_SETTLED_MAX = 0.03`, `SINK_SETTLE_SECONDS = 10.0`, `CREEP_MAX = 0.001` (SC-001); `SPEED_EARLY_SECONDS = 2.0`, `SPEED_EARLY_MAX = 0.1`, `SPEED_SETTLE_SECONDS = 30.0`, `SPEED_MAX = 0.01` (SC-002); `HOLD_MOVE_MAX = 0.01`, `HOLD_SPEED_MAX = 0.01` (US1-1); `ACCEL_TOL = 0.05` (SC-004). Add `pub fn report(label: &str, t: f32, …)` that prints metrics with `println!` so `--nocapture` output can be pasted into research.md.
- [X] T004 [P] Create `examples/common/mod.rs` (a plain module, not an example) with shared macroquad helpers for the new demos: `pub const PPM: f32 = 40.0;` (pixels per meter), `pub fn rect_vertices(half_x, half_y) -> Vec<Vec2>`, `pub fn to_screen(world: Vec2) -> (f32, f32)` (x centered, y flipped: `screen_height() − PPM·y`), `pub fn draw_polygon_outline(position, orientation, local_vertices, color)`, and `pub fn draw_hud(lines: &[String])`. Model them on `examples/ramp.rs`, scaled by `PPM`.

**Checkpoint**: Baseline green; scaffolding compiles (`cargo test --no-run`, `cargo build --examples`).

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: The README derivation (Constitution III: docs first), and the body/pair friction data every story needs.

**⚠️ CRITICAL**: No user story work can begin until this phase is complete.

- [X] T005 Update `README.md` BEFORE any solver or narrowphase code (Constitution III), in two places. (1) § Resolution: replace the bullet "Coulomb friction as a tangent impulse … (M4 — not yet implemented)" with the derivation, keeping the README's variable names: tangent `t = (−n_y, n_x)`; `vt = vr · t` with the same `vr`; `K_t = 1/m_a + 1/m_b + (r_a×t)²/I_a + (r_b×t)²/I_b`; `Δjt = −vt / K_t`; accumulated `jt` clamped to `[−μ·j, +μ·j]` using the contact's *accumulated* normal impulse `j`; applied in the same iterated loop as the normal impulse (tangent then normal each visit). Add the combine rule `μ = √(μ_a·μ_b)` with the reason it is not `max` (a `μ = 0` body must stay frictionless). Add a warm-starting paragraph: impulses `j`, `jt` from the previous step are re-applied to contacts matched by `(body_a, body_b, feature)` before iterating, unmatched contacts start at 0, vanished contacts are forgotten; restitution's target is computed *before* seeding. Mark the iteration count, `slop` and `percent` as "(tuned — final values recorded in T038)" for now. (2) § Narrowphase: change "Every contact carries a `point`, a `normal`, and a `penetration` depth" to also mention the `feature` id and what it is for. Do not change Status or the M4 checkbox yet (T047).
- [X] T006 [P] In `src/body.rs` add `pub friction: f32` to `RigidBody`, set to `0.5` in both `new_dynamic` and `new_static`, and add `pub fn with_friction(mut self, mu: f32) -> Self` storing `mu.max(0.0)` (place it next to `with_restitution`). Doc comments per contracts/engine-api.md. Add tests in `src/body.rs`: default is `0.5` for dynamic and static; `with_friction(-1.0)` → `0.0`; `with_friction(2.0)` keeps `2.0` (no upper clamp).
- [X] T007 [P] In `src/solver.rs` add `pub(crate) fn combine_friction(a: f32, b: f32) -> f32 { (a * b).sqrt() }` with a doc comment explaining why it is not `max`. Add unit tests: symmetric (`combine(0.3, 0.8) == combine(0.8, 0.3)` exactly), `0` if either argument is `0`, `combine(x, x) == x` within 1e-6.

**Checkpoint**: `cargo test` passes (69 + new); README states the M4 math and the `feature` id; bodies carry `friction`; no solver behavior change yet.

---

## Phase 3: User Story 1 — Friction holds or releases bodies on slopes (Priority: P1) 🎯 MVP

**Goal**: Coulomb friction at every contact: a box holds on a shallow ramp and slides on a steep one; a disc rolls; `μ = 0` reproduces M3.

**Independent Test**: `cargo test --test friction` (all cases below), and `cargo run --example ramp --release` shows the shallow box holding and the steep box sliding. Caveat (spec Assumptions): the shallow-hold bounds may only be met once warm-starting exists (T031); if so that one test is closed out under US2, not loosened.

### Tests for User Story 1

> Write these first. T008–T012 should FAIL before T013–T014 (a box currently slides on every ramp).

- [X] T008 [US1] Create `tests/friction.rs` (`mod common;`) with `box_holds_on_shallow_ramp`: 15° ramp, default frictions (`μ_pair = 0.5`, threshold 26.6°); step 1 s to settle, then 10 s; assert the box's displacement over the 10 s is `< HOLD_MOVE_MAX` (1% of its 1 m width) and `speed < HOLD_SPEED_MAX` at the end (US1-1, FR-009, SC-003).
- [X] T009 [US1] In `tests/friction.rs` add `box_slides_on_steep_ramp_with_coulomb_acceleration`: 35° ramp, default frictions; measure speed along the downhill direction at `t = 1 s` and `t = 2 s`; assert `a = Δv/Δt` is within `ACCEL_TOL` (5%) of `G·(sin θ − 0.5·cos θ)` (US1-2, FR-010, SC-004).
- [X] T010 [US1] In `tests/friction.rs` add `higher_friction_slides_less` (same 30° ramp, box friction 0.3 vs 0.9 against a ramp of matching friction: the higher-μ box's downhill distance after 3 s is smaller — US1-3) and `zero_friction_matches_frictionless_slide` (both bodies `with_friction(0.0)`, 30° ramp: `a` within 2% of `G·sin θ`, US1-4, FR-011).
- [X] T011 [US1] In `tests/friction.rs` add `disc_rolls_down_ramp`: `disc_on_ramp` at 20°, default friction; measure center-of-mass acceleration along the slope between `t = 1 s` and `t = 2 s` within 5% of `(2/3)·G·sin θ`, and assert the disc's angular velocity magnitude ≈ `v / r` within 5% (rolling without slipping — US1-5).
- [X] T012 [US1] In `tests/friction.rs` add `threshold_is_within_five_percent_of_atan_mu`: with default frictions (`θc = atan(0.5) = 26.57°`), a box on a ramp at `0.95·θc` (≈ 25.2°) holds (same bounds as T008) and a box on a ramp at `1.05·θc` (≈ 27.9°) slides (downhill speed > 0.1 m/s after 3 s) (FR-009's 5% claim). Same closing-out rule as T018 for the hold half.

### Implementation for User Story 1

- [X] T013 [US1] In `src/solver.rs` extend `ContactState` with `t: Vec2`, `k_t: f32`, `mu: f32` (fields per data-model.md) and compute them in `build_contacts`: `t = Vec2::new(−n.y, n.x)`, `K_t = 1/m_a + 1/m_b + (r_a×t)²/I_a + (r_b×t)²/I_b` (mirror the existing `k` computation), `mu = combine_friction(body_a.friction, body_b.friction)`. Add `jt_acc: f32` initialized to `0.0`. Contacts with `k == 0` are already skipped; no new skip needed. Depends on T006 (the `friction` field) and T007.
- [X] T014 [US1] In `src/solver.rs` `resolve`, inside each contact visit and BEFORE the existing normal-impulse code, add the tangent solve: recompute `vr` (as the normal solve does), `vt = vr.dot(c.t)`, `Δjt = −vt / c.k_t`, `jt_new = (c.jt_acc + Δjt).clamp(−c.mu * c.j_acc, c.mu * c.j_acc)`, apply the change `jt_new − c.jt_acc` as `P = c.t * Δ` (`−P·inv_mass`/`−r_a×P·inv_inertia` on `a`, `+…` on `b`, skipping static bodies exactly as the normal solve does), store `c.jt_acc = jt_new`. Recompute `vr` after applying (the normal solve must see the updated velocity). Use README names in comments (`vt`, `jt`, `μ`). Depends on T013.
- [X] T015 [US1] Add inline unit tests in `src/solver.rs`: (a) `friction_decelerates_by_mu_g_dt`: a box sliding at `vx = 2` on the floor with default friction loses `μ·g·dt` of speed in one step within 5% (Coulomb limit `μ·j` with `j = m·g·dt`); (b) `zero_friction_leaves_tangential_velocity_alone` (`with_friction(0.0)` on the box, `vx` unchanged over a step); (c) `friction_never_exceeds_coulomb_limit` — after solving a tilted box with two contact points and a lateral velocity, assert the accumulated impulses satisfy `|jt| ≤ μ·j` and `j ≥ 0`. For (c) split out a `#[cfg(test)]`-visible inner `fn solve(bodies, manifolds, gravity, dt) -> Vec<ContactState>` that runs build + iterations and returns the final contact states; `resolve` calls it and then applies position correction. (d) a static body is unmodified by friction. Depends on T014.
- [X] T016 [US1] Run `cargo test`. Every M3 test must still pass unchanged (FR-012). If one fails (candidates: `off_center_polygon_hit_spins_the_body`, `box_landing_flat_does_not_tip`), diagnose the physics before touching the test and note the cause; never loosen an M3 assertion to hide a regression. Depends on T015.
- [X] T017 [P] [US1] Update the doc comment of `World::step` in `src/world.rs`: remove "There is no friction yet (M4); impulses act only along contact normals", and describe the tangent impulse and the `±μ·j` clamp in the step list.
- [X] T018 [US1] Run `cargo test --test friction`. T008–T012 should now pass. If `box_holds_on_shallow_ramp` or the hold half of T012 misses its 1%/0.01 bounds ONLY because there is no warm-starting yet, record the measured numbers in `research.md` "In-repo notes" and mark just those tests `#[ignore = "needs warm-start (T031)"]` — do NOT loosen the bounds. All other US1 tests must pass now. Depends on T014, T016.
- [X] T019 [US1] Rework `examples/ramp.rs` (keep its own helpers) to show two static ramps side by side in meters with a 40 px/m draw scale: a 15° ramp and a 35° ramp, one 1 m box on each placed touching per the ramp geometry, contact points/normals still debug-drawn, and a HUD line stating each ramp angle and `atan(μ)`. Update the file's module doc (friction now exists: the shallow box holds, the steep box slides). Run `cargo run --example ramp --release`, watch ~15 s: the shallow box must stay put and the steep box must slide off (US1's Independent Test). **Status: builds and launches without panicking; the ~15 s visual confirmation (shallow box holds, steep box slides) is left to the user. The behavior is covered headlessly by `tests/friction.rs`.**

**Checkpoint**: US1 complete and demonstrable: friction works on ramps, discs roll, `μ = 0` matches M3, M3 tests green. (The shallow-hold tests may be `#[ignore]`d pending T031; if so, "US1 complete" applies once T031 un-ignores them.)

---

## Phase 4: User Story 2 — A 10-box tower stands stably (Priority: P1)

**Goal**: The 10-box tower holds for 60 simulated seconds without toppling, jitter, or sinking, per the re-based SC-001/SC-002 — via warm-starting plus tuning (FR-006, FR-007, FR-008). Also home to the stress tests that the tuning sweep must satisfy (US3 pyramid/mixed scenes, tall tower, mass ratio), authored here so tuning sees them.

**Independent Test**: `cargo test --release tower -- --nocapture` (metrics printed), and `cargo run --example stack --release` watched for 60 s.

### Tests and evidence for User Story 2

- [ ] T020 [US2] Create `tests/stacking.rs` (`mod common;`) with `tower_stands_for_sixty_seconds`: `tower(&mut world, 10)`, step 3600 times; record per-second `sink`, `drift`, `speed` via `report`; assert every step: (i) no box's `|x|` exceeds `DRIFT_MAX` and no box's `|angle|` exceeds 0.05 (does not topple); (ii) top-box `sink ≤ SINK_ALWAYS_MAX`; (iii) for `t ≥ SINK_SETTLE_SECONDS`, `sink ≤ SINK_SETTLED_MAX`, and `sink(t) − sink(10 s) ≤ CREEP_MAX` (no steady creep); (iv) every box's `speed ≤ SPEED_EARLY_MAX` for `t ≥ SPEED_EARLY_SECONDS` and `speed ≤ SPEED_MAX` for `t ≥ SPEED_SETTLE_SECONDS`. Bounds come from `tests/common/mod.rs`; do not inline numbers. (US2-1..3, SC-001, SC-002.)
- [ ] T021 [US2] FR-007 gate, in-repo: with the friction-only solver (no cache yet) run `cargo test --release tower -- --nocapture` for `VELOCITY_ITERATIONS` ∈ {8, 12, 16, 20, 30} (edit the constant, one run each) and confirm it FAILS at every value. Paste the metrics into `research.md` "In-repo notes" as the in-repo confirmation that iteration tuning alone (with friction) does not hold the tower; restore the constant to 8 afterwards. If it PASSES at any value: FR-007's gate is not met, warm-starting is unnecessary, tasks T022–T031 must be dropped after checking with the user, and the warm-starting paragraph written by T005 must be reverted from `README.md`. Depends on T020.

### Warm-starting: contact features (narrowphase)

- [ ] T022 [US2] In `src/collision/manifold.rs` add `pub feature: u32` to `Contact` with a doc comment ("Stable id of the geometric feature that produced this point; circle contacts use 0"). Expect compile errors at every `Contact { … }` construction; the following tasks fix them. Depends on T021.
- [ ] T023 [P] [US2] In `src/collision/circle.rs` set `feature: 0` at every `Contact` construction (`grep -n "Contact {" src/collision/circle.rs`).
- [ ] T024 [P] [US2] In `src/collision/polygon.rs` add `fn pack_feature(flip: bool, ref_face: usize, incident_face: usize, endpoint: usize) -> u32` (bit 24 = flip, bits 16–23 = `ref_face`, bits 8–15 = `incident_face`, bits 0–7 = `endpoint`, 0 for `inc_p1`, 1 for `inc_p2`), and set `feature` in the contact-building loop of `polygon_vs_polygon` — the loop iterates `[inc_p1, inc_p2]`, so use its index for `endpoint`. Note this file's `flip`, `ref_face`, and `incident_face` locals.
- [ ] T025 [US2] In `src/collision/mod.rs` confirm the `(Circle, Polygon)` arm's `Contact { normal: -c.normal, ..c }` preserves `feature` (struct-update syntax does), and add a note to the `detect_contacts` doc comment that `feature` is preserved. Fix the two hand-built `Contact` literals in `src/solver.rs` tests (`accumulated_normal_impulse_is_never_negative`) by adding `feature: 0` / `feature: 1`. Depends on T023, T024.
- [ ] T026 [US2] Add collision tests: in `src/collision/polygon.rs` (or `mod.rs`) `resting_box_features_are_stable_and_distinct` — a box resting on a floor polygon, moved by 1e-3 between two `detect_contacts` calls, yields the same two `feature` values in both calls and the two values differ from each other; in `src/collision/mod.rs` `circle_then_polygon_negation_keeps_feature`. Depends on T025.

### Warm-starting: impulse cache (solver)

- [ ] T027 [US2] In `src/solver.rs` add `#[derive(Default)] pub(crate) struct ImpulseCache { entries: Vec<CachedImpulse> }` and `struct CachedImpulse { body_a: u32, body_b: u32, feature: u32, j: f32, jt: f32 }` per data-model.md, with `fn find(&self, a: u32, b: u32, feature: u32) -> Option<(f32, f32)>` (ordered scan, first match) and `fn replace(&mut self, entries: Vec<CachedImpulse>)`. Depends on T025.
- [ ] T028 [US2] In `src/solver.rs` add `feature: u32` (from the source `Contact`) to `ContactState`; change `solve` (from T015) and `resolve` to take `cache: &mut ImpulseCache` after `manifolds`; after `build_contacts` (so `bounce` is already computed from the true approach velocity — do NOT reorder this) seed each contact: on a cache hit set `j_acc`/`jt_acc` and apply `P = j_acc·n + jt_acc·t` (`−P` to `a`, `+P` to `b`, static guard as elsewhere); after the velocity iterations, `cache.replace(...)` with this step's `(a, b, feature, j_acc, jt_acc)` for every contact, in contact order. The `#[cfg(test)]` `solve` helper must still return the final `ContactState`s (now including seeded starting values, exposed as `j_seed`/`jt_seed` fields for tests). Update every `resolve(...)` call in the solver's tests to pass `&mut ImpulseCache::default()`. Depends on T027, T015.
- [ ] T029 [US2] In `src/world.rs` add `impulse_cache: ImpulseCache` to `World`, initialize it in `new()` with `ImpulseCache::default()`, pass `&mut self.impulse_cache` in the `step` call to `solver::resolve`, and update the `step` doc comment (warm-starting: persistent contacts are seeded from the previous step). `use crate::solver::{self, ImpulseCache}`. Depends on T028.
- [ ] T030 [US2] Add inline cache tests in `src/solver.rs`: (a) `matched_contact_is_seeded` — after one `solve` on a resting box the cache has entries; a second `solve` with the same cache returns contacts whose `j_seed > 0`; (b) `vanished_contact_is_forgotten` — solve a touching pair, then solve with the bodies far apart (empty manifolds) and assert the cache is empty (FR-008); (c) `new_contact_starts_at_zero` — a cache holding a different `feature` does not seed (`j_seed == 0`); (d) `seeding_does_not_change_the_bounce_target` — an `e = 1` ball dropped onto the floor still returns within 5% of its drop height with the cache active (the existing `e1_ball_returns_to_drop_height` must also still pass). Depends on T029.
- [ ] T031 [US2] Run `cargo test` (everything) and `cargo test --release tower -- --nocapture`. Remove any `#[ignore]` added in T018 and confirm the shallow-hold tests pass. Record the tower metrics in `research.md` "In-repo notes". Depends on T030.

### Stress scenes authored before tuning (FR-006, edge cases)

- [ ] T032 [US3] In `tests/stacking.rs` add `pyramid_stays_standing_for_thirty_seconds`: `pyramid(&mut world, 5)`, step 1800; assert for every box `|x − x₀| ≤ 0.1`, `y ≥ y₀ − 0.1`, `|angle| ≤ 0.05`, and `speed ≤ 0.05` after 10 s (US3-1, SC-005 — these are the spec's bounds). If neighbor boxes touching at spacing exactly 1.0 cause spurious edge contacts, use spacing 1.01 and note the change in `research.md`. Depends on T031.
- [ ] T033 [US3] In `tests/stacking.rs` add `mixed_boxes_and_circles_come_to_rest`: on a floor, drop three unit boxes and three circles (r = 0.5) from staggered heights at x positions ≥ 3 m apart; run 20 s; assert every body's `speed ≤ 0.02` and frame-to-frame position change `< 1e-3` for the last 5 s, and none has sunk more than 2% of its size below its resting height (US3-2, spec bounds). Depends on T031.
- [ ] T034 [US2] In `tests/stacking.rs` add `tall_tower_does_not_explode`: `tower(&mut world, 15)`, step 1800; assert every value stays finite (`is_finite`) and every box's speed stays below 1.0 m/s at every step; it need NOT meet SC-001's bounds — the spec allows a tall stack to be soft, not unstable (edge case: stack taller than 10). Depends on T031.
- [ ] T035 [US3] In `tests/stacking.rs` add `heavy_on_light_stays_finite_and_bounded`: a floor, a light unit box (mass 1) and a heavy unit box (mass 1000) stacked on it, run 10 s; assert all values finite and every box's speed below 1.0 m/s at every step after 2 s (edge case: very heavy body on very light body; the solver may converge slowly but must not explode). Depends on T031.

### Tuning (FR-006) — decisions must be recorded, not guessed

- [ ] T036 [US2] Sweep `VELOCITY_ITERATIONS` ∈ {8, 10, 12, 16} × `PENETRATION_SLOP` ∈ {0.002, 0.003, 0.005} in `src/solver.rs` (edit the constants, run `cargo test --release stacking -- --nocapture`, one run per combination). The runs must cover the tower (T020), pyramid (T032), mixed (T033), tall tower (T034) and heavy-on-light (T035). Record a table in `research.md` § "Tuning results": per combination, tower sink at t = 2/10/60 s, max drift, last time `speed > SPEED_MAX`, and pass/fail for each of the five tests. Keep `CORRECTION_PERCENT = 0.4` unless the table shows a reason. Depends on T031, T032, T033, T034, T035.
- [ ] T037 [US2] A/B the per-visit order (tangent-then-normal vs normal-then-tangent) at the best combination from T036, on the tower and the pyramid. Record the winner and the numbers in `research.md` (resolving plan.md's "starting choice") and keep the winning order. Depends on T036.
- [ ] T038 [US2] Set the final `VELOCITY_ITERATIONS` and `PENETRATION_SLOP` (and `CORRECTION_PERCENT` if changed) in `src/solver.rs` with doc comments giving the reasoning and pointing to research.md § Tuning results; replace every "(tuned — final values recorded in T038)" placeholder in `README.md` § Resolution with the final iteration count, `slop` and `percent` (FR-006, FR-017), and correct the stale "~8" / `slop = 0.01` wording there. If NO combination meets T020's bounds (or T032's): STOP per the Stop rule above; commit the data, do not weaken the tests. Depends on T037.

### Determinism, demo, and behavioral verification

- [ ] T039 [US2] In `tests/stacking.rs` add `tower_is_bit_identical_across_runs`: build and run two towers for 3600 steps; assert every body compares `==` (RigidBody derives `PartialEq`) (US2-5, SC-007, FR-013). Depends on T031.
- [ ] T040 [US2] Create `examples/stack.rs` (`mod common;` from T004): world with the floor and a 10-box tower via the same layout as `tests/common::tower`, boxes drawn with `draw_polygon_outline`, `world.step(get_frame_time())` each frame, a HUD via `draw_hud` showing elapsed sim time, top-box sink, max drift, and FPS (`get_fps()`), and contact debug-drawing toggled by a key. Depends on T038, T004.
- [ ] T041 [US2] Run `cargo run --example stack --release` and watch the full 60 s (Constitution IV: this, not `cargo test`, is the acceptance check). Record in `research.md`: observed jitter/sinking/creep (SC-009), HUD values at 60 s, and FPS ≥ 60 (SC-008). If anything visible or out of bounds remains, return to T036–T038 rather than accepting it. Depends on T040.

**Checkpoint**: US2 complete: the MVP's headline demo passes. The 10-box tower is stable for 60 s in both the test and the demo, and the stress tests pass with the same constants.

---

## Phase 5: User Story 3 — Pyramid and general resting-contact stress cases (Priority: P2)

**Goal**: A 15-box pyramid and mixed shapes settle and stay settled.

**Independent Test**: `cargo test --release pyramid mixed heavy` and `cargo run --example pyramid --release`. The tests (T032, T033, T035) were authored in Phase 4 so tuning could include them.

- [ ] T042 [P] [US3] Create `examples/pyramid.rs` (`mod common;`): the same layout as `tests/common::pyramid` with 5 rows, HUD and contact toggle as in `stack.rs`. Depends on T038, T004.
- [ ] T043 [US3] Run `cargo run --example pyramid --release` for ≥ 30 s and confirm the pyramid stays standing with no row collapsing. Record the observation in `research.md`. If the pyramid fails while its test passes, treat it as a test-gap: tighten the test to reproduce it, then return to T036–T038. Depends on T032, T042.

**Checkpoint**: US3 complete.

---

## Phase 6: User Story 4 — Existing behavior is preserved (Priority: P2)

**Goal**: No M3 regression from friction, warm-starting, or retuned constants.

**Independent Test**: full suite plus the bouncing example.

- [ ] T044 [US4] Run `cargo test` and `cargo test --release`; the count must be ≥ 69 + the new tests, with zero failures (US4-3, FR-012, SC-006). Depends on T038.
- [ ] T045 [US4] Review `git diff 30318b7..HEAD -- src/` restricted to pre-existing `#[cfg(test)]` modules (`30318b7` is M3's last commit and this branch's base; do NOT diff against `main`, which is stale and would show M3's own edits): the only allowed edits are mechanical (the new `feature` field in `Contact` literals, the new `resolve` argument). Any changed assertion or tolerance in an M3 test is a red flag — justify it in `research.md` or revert it. Depends on T044.
- [ ] T046 [US4] Re-run `cargo run --example bouncing --release` and compare with T001: `e = 0` ball still stops dead, `e = 1` ball still returns to within 5% of its drop height (US4-1, US4-2). Depends on T038.

**Checkpoint**: US4 complete: earlier milestones' "done when" still hold.

---

## Phase 7: Polish & Cross-Cutting Concerns

- [ ] T047 Update `README.md` after T041, T043 and T046 have actually been observed: Status paragraph (M0–M4 landed, MVP complete; friction and stable stacking; run commands for `stack`, `pyramid`, `ramp`), tick the M4 checkbox, list `tests/`, `examples/common/`, `examples/stack.rs`, `examples/pyramid.rs` in § Architecture, confirm § Resolution's iteration count, `slop` and `percent` match `src/solver.rs` and § Narrowphase matches the `Contact` struct, and keep the "tall stacks stay slightly soft" note. Constitution III: no behavior in code that the README derivation doesn't describe.
- [ ] T048 [P] Sync design docs with what was built: final constants and step order in `specs/004-friction-and-stacking/plan.md` and `data-model.md`; make sure `spec.md`, the `tests/common/mod.rs` constants, and `quickstart.md` agree on every numeric bound.
- [ ] T049 [P] Confirm the Constitution's hard rules: `cargo tree -e normal` shows `mirage` with no dependencies (macroquad only under dev-dependencies); `grep -rn "unsafe" src/` returns nothing; no `HashMap`/`thread_local` in `src/`; no rendering code in `src/`; and (FR-016) `grep -rniE "sleep|deactivat|joint|ccd|continuous" src/` shows no sleeping/deactivation or other non-goal machinery.
- [ ] T050 [P] Refresh `CLAUDE.md`: its "Project state" and "Commands" sections still say no crate exists; update them to reflect the current state (crate exists, M0–M4 landed, the commands now run), and update its data-flow paragraph's "~8 velocity iterations" and "semi-implicit Euler" sentence to match the tuned iteration count and the friction/warm-start steps. Keep its plan reference (managed by the plan step) untouched.
- [ ] T051 Run `cargo build --examples --release` (no new warnings) and walk through every step of `specs/004-friction-and-stacking/quickstart.md`, ticking each row of its behavior table against a real test name.
- [ ] T052 Prepare the change for review on branch `004-friction-and-stacking`: this branch was cut from `003-impulse-resolution`, so check whether M3's PR has merged; if not, base the M4 PR on the 003 branch. Never push to `main`. Open the PR only when the user asks.

---

## Dependencies & Execution Order

### Phase dependencies

- **Setup (Phase 1)**: no dependencies; T001 gates all code changes.
- **Foundational (Phase 2)**: after Setup. T005 (README) precedes every solver and narrowphase change (T013+, T022+).
- **US1 (Phase 3)**: after Foundational.
- **US2 (Phase 4)**: after US1 (needs the tangent solve and friction tests; the shallow-hold tests are un-ignored here). Within the phase, the stress-scene tests (T032, T033, T034, T035) come after T031 and before the sweep T036.
- **US3 (Phase 5)**: after US2's T038 (needs warm-start and final constants); its tests already exist.
- **US4 (Phase 6)**: after US2 (T038); can overlap US3.
- **Polish (Phase 7)**: after US2–US4; T047 needs the observations from T041, T043 and T046.

### Within-story order

- Tests before implementation where they are new behavior (US1: T008–T012 then T013–T014; US2: T020–T021 then the cache).
- US1: T013 → T014 → T015 → T016 → T018; T017 and T019 are independent of the solver internals.
- US2: T022 → {T023, T024} → T025 → T026; T027 → T028 → T029 → T030 → T031 → {T032, T033, T034, T035} → T036 → T037 → T038 → {T039, T040} → T041.

### Parallel opportunities

- Phase 1: T002 ‖ T004 (T003 follows T002).
- Phase 2: T006 ‖ T007 (different files; T005 first).
- US1: T017 ‖ T013–T015 (different files); T019 ‖ T018 once T014 is done.
- US2: T023 ‖ T024.
- US3: T042 ‖ the Phase 4 stress tests (different file).
- Polish: T048 ‖ T049 ‖ T050.

### Parallel example: Phase 1

```text
Task T002: tests/common/mod.rs      scene builders
Task T004: examples/common/mod.rs   macroquad helpers
```

---

## Implementation Strategy

### MVP first

1. Phases 1–2 (scaffold, README, `friction`, `combine_friction`).
2. Phase 3 (US1): friction on ramps — the smallest slice that delivers Coulomb friction, verified by analytic tests and the two-ramp demo. Stop and validate. (If the shallow-hold tests had to be ignored, friction on slopes is delivered but the "holds perfectly still" acceptance closes with warm-starting in Phase 4.)
3. Phase 4 (US2): warm-start + tuning → the 10-box tower. This completes the MVP (README: "The MVP is complete at M4").

### Incremental delivery

US1 → US2 (MVP) → US3 (pyramid stress) → US4 (regression proof) → Polish. Each phase ends at a checkpoint that is runnable on its own.

### Risk gates

- **Gate 0 (T001)**: M3 must be green before M4 work.
- **Gate FR-007 (T021)**: tower must fail without the cache at every tested iteration count; otherwise stop.
- **Gate thresholds (T038)**: if the re-based SC-001/SC-002 or the pyramid bounds are unmet after the sweep, stop and use `/speckit-clarify` — do not loosen tests.
- **Non-goals**: no sleeping, joints, CCD, spatial index, concave shapes, serialization, parallelism, 3D. If a task seems to need one, stop and confirm scope with the user.

## Notes

- `[P]` tasks touch different files with no dependency on incomplete tasks.
- Each `[US#]` label maps a task to its story in spec.md.
- Commit after each task or logical group; never commit to `main`.
