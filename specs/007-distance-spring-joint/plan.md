# Implementation Plan: Distance / Spring Joint

**Branch**: `007-distance-spring-joint` | **Date**: 2026-10-16 | **Spec**: [spec.md](spec.md)

**Input**: Feature specification from `/specs/007-distance-spring-joint/spec.md`

**Note**: This template is filled in by the `/speckit-plan` command. See `.specify/templates/plan-template.md` for the execution workflow.

## Summary

This plan covers M7 from `README.md` § Post-MVP milestones: a 2-body
`DistanceJoint` that holds two local anchors at a target length, in one of
two modes.

- **Rigid mode** is a scalar sequential-impulse equality constraint along
  the anchor-to-anchor line `n`. It is solved in the shared
  velocity-iteration loop after the revolute joints and uses the same `K`
  as a contact normal, with no clamp. Positional correction mirrors M6's,
  using the same constants.
- **Soft mode** is an explicit Hookean force `F = −(k·(L − L0) + c·L̇)`.
  It is evaluated from start-of-step state in the same "forces" phase as
  gravity, before gravity's kick. A scratch simulation showed that
  evaluating it after gravity makes the damper settle the box 10 cm off
  equilibrium.

Research measured this explicit form under the engine's semi-implicit
Euler. Critical damping gives zero overshoot. An undamped spring drifts
0.002% in energy over 60 s, with 3% bounded wobble. The under-damped
period is within 0.3% of analytic. The rejected soft-constraint
formulation dissipates energy at `ζ = 0`, which fails M7's own done-when
criterion.

The feature is validated by `tests/distance_joints.rs` (SC-001 to SC-004
with the concrete bounds in [research.md](research.md) § 8) and by
`examples/spring.rs`, watched with `--release`. That demo shows
under-damped, critically damped and undamped boxes side by side. All of
it is additive: `RevoluteJoint`'s API and code are untouched, and scenes
without distance joints run bit-identically.

## Technical Context

**Language/Version**: Rust, edition 2024 (per `Cargo.toml`, unchanged).

**Primary Dependencies**: None in the engine crate (Principle I).
`macroquad 0.4.16` stays a `dev-dependency`, used by `examples/spring.rs`.

**Storage**: N/A. All state is in-memory simulation state.

**Testing**: `cargo test`, covering:
- unit tests in `src/joint/distance.rs` for build, degenerate cases,
  impulse direction and the static-body guard;
- `World` unit tests for `add_distance_joint` validation and length
  resolution;
- a new behavioral suite `tests/distance_joints.rs`, shaped like
  `tests/joints.rs`.

**Target Platform**: Desktop via `macroquad` for the demo. The engine
crate itself is platform-agnostic.

**Project Type**: Single library crate (`mirage`), no workspace split.

**Performance Goals**: Real-time at the fixed `1/60` step for a handful of
springs. This adds one O(joints) pass per step plus O(joints) work per
iteration, which is negligible next to contacts.

**Constraints**:
- Zero engine dependencies.
- Deterministic fixed timestep.
- No formula changes in `solver.rs` or the revolute path in `joint.rs`.
- Explicit springs are stable only while `ω·dt < 2`, documented as a known
  limitation (research.md § 1).

**Scale/Scope**: One joint type with two modes, one demo, one behavioral
test file, and a README § How it works subsection. Out of scope: rope
(one-sided) mode, length limits, breakable joints, joint removal, M8
motor, and a general force accumulator.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Check | Result |
|---|---|---|
| I. Zero-Dependency Engine Core | All new math is scalar `Vec2` work: `√`, dot, cross. No `Mat2` is needed, no crate is added, and there is no `unsafe`. | **PASS** |
| II. Deterministic Fixed-Timestep Simulation | Springs are applied in a fixed phase of the fixed-`dt` loop. Rigid joints iterate in insertion order after revolute joints. Integration stays semi-implicit Euler, and research § 1 relies on its symplecticity. | **PASS** |
| III. Spec-Faithful Math | README § How it works gets a "Distance / spring joint" derivation (research § 1–4: `C`, `Cdot`, `K`, `λ`, `P`, `F`, `k`, `c`, `L0`) in the same change as the code. The README's closing note on Post-MVP milestones says derivations land with the implementation, not ahead of it. Code uses those names. | **PASS** (written with the code) |
| IV. Milestone-Gated, Behaviorally-Verified | M0–M4 and M6 are complete and merged (PR #16). M7 is next in README order. Its done-when will be checked by running `examples/spring.rs --release`, not only by `cargo test`. | **PASS** |
| V. Explicit Non-Goals | Joints were promoted to Post-MVP milestones. This plan stays inside M7: no motor, no rope or limit mode, and no force-accumulator API (research § 2 rejected it explicitly). | **PASS** |

No violations, so Complexity Tracking is not needed.

**Post-design re-check (after Phase 1)**: still PASS. The contract
([contracts/distance-joint-api.md](contracts/distance-joint-api.md)) is
purely additive. The only change to `World::step` adds two phases: the
spring pass before gravity, and a rigid-distance pass inside and after the
iteration loop. Both are no-ops when `distance_joints` is empty, so FR-011
holds by construction. Existing tests are the regression check.

## Project Structure

### Documentation (this feature)

```text
specs/007-distance-spring-joint/
├── plan.md              # This file (/speckit-plan command output)
├── research.md          # Phase 0 output (/speckit-plan command)
├── data-model.md        # Phase 1 output (/speckit-plan command)
├── quickstart.md        # Phase 1 output (/speckit-plan command)
├── contracts/
│   └── distance-joint-api.md   # Phase 1 output (/speckit-plan command)
└── tasks.md             # Phase 2 output (/speckit-tasks command - NOT created by /speckit-plan)
```

### Source Code (repository root)

```text
mirage-engine/
├── src/
│   ├── joint.rs          # + `pub mod distance;` and re-exports. RevoluteJoint
│   │                     #   code and JOINT_SLOP / JOINT_CORRECTION_PERCENT
│   │                     #   unchanged (shared with distance.rs via `super::`).
│   ├── joint/
│   │   └── distance.rs   # NEW: DistanceJoint, Spring, DistanceJointId,
│   │                     #   DistanceState, build_distance_joints,
│   │                     #   apply_spring_forces, iterate_once,
│   │                     #   correct_positions, + unit tests
│   ├── world.rs          # + distance_joints vec, add_distance_joint /
│   │                     #   distance_joint (with validation and length
│   │                     #   resolution); step(): spring pass before gravity,
│   │                     #   rigid-distance iterate after revolute, rigid-
│   │                     #   distance correct_positions after revolute's
│   └── lib.rs            # + pub use joint::{DistanceJoint, DistanceJointId, Spring};
├── tests/
│   └── distance_joints.rs  # NEW: SC-001 critical damping, SC-002 undamped
│                           #   energy, SC-003 under-damped period, SC-004
│                           #   rigid length + pendulum period, free-space
│                           #   momentum (US1 #4), initial-gap correction
│                           #   (US2 #3), zero separation stays finite,
│                           #   spring + revolute on one body, determinism
├── examples/
│   └── spring.rs         # NEW: M7 acceptance demo. Three boxes hang from
│                         #   fixed points (under-damped, critical, undamped),
│                         #   spring drawn as a line, watched with --release
└── README.md             # § How it works: "Distance / spring joint"
                          #   derivation + trade-offs (stiffness bound,
                          #   restitution/spring kick); § Architecture tree;
                          #   § Status; M7 [x]; § Running it. Same change as
                          #   the code (Principle III).
```

**Structure Decision**: Single project, matching the repository. The new
code goes in a `joint/distance.rs` submodule of the existing `joint.rs`
(research § 5), so M6's revolute code is not moved or edited. `World::step`
gains these phases:

```text
apply_spring_forces → gravity → detect → build (contacts, revolute, distance)
→ warm_start → [contacts, revolute, distance] × 16
→ correct_positions (contacts, revolute, distance) → store_impulses → integrate
```

`CLAUDE.md`'s data-flow paragraph and Architecture list are updated in the
same change to match.

## Complexity Tracking

*No entries. The Constitution Check reported no violations.*
