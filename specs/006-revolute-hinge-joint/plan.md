# Implementation Plan: Revolute (Hinge) Joint

**Branch**: `006-revolute-hinge-joint` | **Date**: 2026-09-27 | **Spec**: [spec.md](spec.md)

**Input**: Feature specification from `/specs/006-revolute-hinge-joint/spec.md`

**Note**: This template is filled in by the `/speckit-plan` command. See `.specify/templates/plan-template.md` for the execution workflow.

## Summary

M6 from `README.md` § Post-MVP milestones: a 2-body point constraint
("revolute joint") pinning a local anchor on one body to a local anchor on
another, solved as a sequential-impulse equality constraint alongside
existing contacts. Technical approach (full derivation in
[research.md](research.md)): a coupled 2×2 effective-mass solve per the
Box2D-Lite formulation (not a per-axis decomposition, which would be wrong
the moment an anchor isn't at a body's center of mass — exactly the
pendulum case), backed by a new small `Mat2` primitive in `math.rs`; Baumgarte
position correction on the anchor gap, mirroring the contact solver's
"correct position, not velocity" rationale; and a shared velocity-iteration
loop moved up into `World::step` so joints and contacts on the same body
converge together, without touching any contact formula in `solver.rs`.
Validated by `examples/hinge.rs` (a pendulum, checked against the physical
pendulum's analytic period) and `tests/joints.rs` (anchor-coincidence and
joint+contact-coexistence behavioral tests).

## Technical Context

**Language/Version**: Rust, edition 2024 (per `Cargo.toml`; no version
change needed for this feature — `Mat2` needs nothing edition-2024-specific).

**Primary Dependencies**: None in the engine crate (Principle I). `macroquad
0.4.16` remains a `dev-dependency` for `examples/hinge.rs`, matching the
existing demos.

**Storage**: N/A — in-memory simulation state only, matching the rest of
the engine.

**Testing**: `cargo test` (unit tests inside `src/joint.rs` and `src/math.rs`
for `Mat2`; a new `tests/joints.rs` behavioral suite, following the shape of
`tests/friction.rs`/`tests/stacking.rs`).

**Target Platform**: Whatever the existing examples already target — a
desktop build via `macroquad` for `examples/hinge.rs`; the engine crate
itself is platform-agnostic (no OS APIs, no `unsafe`).

**Project Type**: Single library crate (`mirage`), no workspace split —
unchanged from the existing project shape.

**Performance Goals**: Real-time at the existing fixed `1/60` timestep for a
single-pendulum scene — no new performance goal; this is a small addition
to a solver that already handles a 10-box tower and a 5-row pyramid at
`--release` speed.

**Constraints**: Zero dependencies in the engine crate (Principle I); the
existing contact-resolution formulas in `solver.rs` must not change
(README § Post-MVP milestones, quoted in research.md § 3); deterministic,
fixed-timestep simulation (Principle II) must be preserved.

**Scale/Scope**: One joint type (revolute/point constraint), one demo scene
(a single pendulum), plus enough behavioral test coverage to exercise a
body with more than one joint and a jointed body that also touches a
contact (spec.md Edge Cases / User Story 1 scenario 3). Not in scope:
motors (M8) or springs (M7) — separate future milestones.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Check | Result |
|---|---|---|
| I. Zero-Dependency Engine Core | New `Mat2` primitive is hand-written in `math.rs`; no crate added; `unsafe` not needed for a 2×2 matrix inverse. | **PASS** |
| II. Deterministic Fixed-Timestep Simulation | Joint solve runs inside the same fixed-`dt` accumulator loop, same iteration order every run (research.md § 3); no RNG, no wall-clock dependence. | **PASS** |
| III. Spec-Faithful Math | `README.md` § How it works has no joint section yet — by design (Post-MVP milestones' closing note: "Full derivations... land in § How it works once implemented... not written speculatively ahead of the code"). This plan's job is the *design decision*, not the README prose; the derivation lands in the same change that implements it, per that note. | **PASS** (deferred write, not skipped) |
| IV. Milestone-Gated, Behaviorally-Verified Development | M0–M4 (MVP) and the M5 hardening pass are already complete and merged (git history: PRs #13–#15). M6 is the correct next milestone per README ordering. Its own "done when" criterion (anchor coincidence + analytic period match) will be verified by actually running `examples/hinge.rs --release`, not just `cargo test`. | **PASS** |
| V. Explicit Non-Goals (Scope Discipline) | Joints were promoted out of the non-goals list into § Post-MVP milestones in a prior, deliberate README change (already merged, per the "docs: plan joints & constraints" PR). This feature stays inside M6's own boundary — no motor, no spring, no joint removal API. | **PASS** |

No violations; Complexity Tracking is not needed.

## Project Structure

### Documentation (this feature)

```text
specs/006-revolute-hinge-joint/
├── plan.md              # This file (/speckit-plan command output)
├── research.md          # Phase 0 output (/speckit-plan command)
├── data-model.md        # Phase 1 output (/speckit-plan command)
├── quickstart.md        # Phase 1 output (/speckit-plan command)
├── contracts/
│   └── joint-api.md     # Phase 1 output (/speckit-plan command)
└── tasks.md              # Phase 2 output (/speckit-tasks command - NOT created by /speckit-plan)
```

### Source Code (repository root)

```text
mirage-engine/
├── src/
│   ├── math.rs          # + Mat2 (pub(crate)): construction, Mat2 * Vec2, invert()
│   ├── joint.rs          # NEW — RevoluteJoint, JointId, JointState, build_joints,
│   │                     #   iterate_once, correct_positions (mirrors solver.rs's shape)
│   ├── world.rs          # + joints: Vec<RevoluteJoint> storage, add_joint/joint;
│   │                     #   step() restructured per research.md §3 to share one
│   │                     #   velocity-iteration loop between solver:: and joint::
│   ├── solver.rs          # Restructured (not rewritten): resolve() split into
│   │                     #   build_contacts / warm_start / iterate_once /
│   │                     #   correct_positions, each pub(crate). Every existing
│   │                     #   formula, constant, and test stays as-is.
│   └── lib.rs            # + pub use joint::{JointId, RevoluteJoint};
├── tests/
│   └── joints.rs          # NEW — anchor-coincidence over a sustained run,
│                          #   pendulum period vs. analytic prediction,
│                          #   joint + contact coexistence, static-body pinning,
│                          #   multi-joint body (chain), determinism
├── examples/
│   ├── common/mod.rs      # Reused as-is for drawing helpers if applicable
│   └── hinge.rs            # NEW — the M6 acceptance demo: a rod pinned at one
│                          #   end swinging under gravity, watched with --release
└── README.md              # § How it works gains a "Joints" subsection and M6's
                           #   checkbox flips to [x], written alongside the code
                           #   that implements it (Principle III) — not part of
                           #   this plan's own file list, but a required part of
                           #   the implementation change.
```

**Structure Decision**: Single project (matches the existing repository —
one lib crate, `examples/` and `tests/` at the root, no workspace split).
The only structural change beyond "add a new module" is splitting
`solver::resolve()` into four `pub(crate)` phases so `World::step` can
interleave them with the new `joint::` phases (research.md § 3) — every
existing public item, formula, and test in `solver.rs` is preserved
unchanged; only their grouping into one vs. several functions changes.

## Complexity Tracking

*No entries — Constitution Check reported no violations.*
