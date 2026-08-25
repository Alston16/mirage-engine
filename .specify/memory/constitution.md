<!--
Sync Impact Report
==================
Version change: [TEMPLATE] → 1.0.0 (initial ratification)
Modified principles: n/a (first fill of template placeholders)
Added sections:
  - I. Zero-Dependency Engine Core
  - II. Deterministic Fixed-Timestep Simulation
  - III. Spec-Faithful Math (README Is the Source of Truth)
  - IV. Milestone-Gated, Behaviorally-Verified Development
  - V. Explicit Non-Goals (Scope Discipline)
  - Technical Constraints
  - Development Workflow
  - Governance
Removed sections: none
Templates requiring updates:
  - .specify/templates/plan-template.md ✅ no changes needed (Constitution Check gate is generic, reads from this file)
  - .specify/templates/spec-template.md ✅ no changes needed (technology-agnostic template)
  - .specify/templates/tasks-template.md ✅ no changes needed (generic task structure; milestone-specific ordering applied per-feature)
  - .specify/templates/checklist-template.md ✅ no changes needed
  - .specify/templates/commands/*.md — not present in this repo, nothing to update
Follow-up TODOs: none
-->

# mirage-engine Constitution

## Core Principles

### I. Zero-Dependency Engine Core
The `mirage` library crate MUST have zero external dependencies. `macroquad`
is permitted **only** as a `dev-dependency` for `examples/*.rs`; it MUST
NOT appear as a runtime dependency of the engine. No physics, math,
linear-algebra, or collision crates may be added to work around implementing
`Vec2`, `Rot2`, broadphase, narrowphase, or the solver from scratch. `unsafe`
is disallowed.

Rationale: the entire premise of this project is a physics engine that can
be read start to finish and understood — reaching for an existing crate at
any point (`nalgebra`, `parry`, `rapier`, etc.) defeats the purpose, and
`unsafe` undermines the "readable and correct first" posture the project is
built on.

### II. Deterministic Fixed-Timestep Simulation
`World::step` MUST run on a fixed `dt` (1/60) driven by an accumulator.
Given identical inputs and identical iteration order, a simulation MUST
reproduce identical results across runs. Rendering framerate MUST NOT
influence simulation behavior. Integration MUST use semi-implicit
(symplectic) Euler — velocity updated before position — not explicit Euler.

Rationale: this is a physics engine whose correctness is judged by
behavior (stacks staying stable, balls returning to consistent heights).
Non-determinism or framerate-coupling makes that behavior unverifiable and
unreproducible.

### III. Spec-Faithful Math (README Is the Source of Truth)
`README.md` § "How it works" is the specification, not a paraphrase. Code
MUST implement the derivations there exactly, including variable names
(`e`, `μ`, `j`, `vr`, etc.) so the code and the written derivation can be
read side by side. Any deviation (different formulation, different
variable naming convention, different clamping strategy) requires updating
the README derivation first, in the same change, so the two never drift
apart.

Rationale: the project's stated goal is a codebase where every integrator
step, contact manifold, and impulse equation is traceable to a formula a
reader can check — that guarantee breaks the moment code and doc diverge.

### IV. Milestone-Gated, Behaviorally-Verified Development
Development MUST follow the milestone order defined in `README.md` §
"MVP milestones" (M0 → M4); a later milestone MUST NOT be started until
the "done when" criterion of every earlier milestone is met. When a
milestone's "done when" criterion is observable via an example (e.g. M4's
60-second stable 10-box stack, M3's restitution/height check), it MUST be
verified by actually running that example with `--release` — passing
`cargo test` alone does not satisfy the criterion. `--release` is required
for every example run; debug-build solver behavior does not count as
verification.

Rationale: each milestone's math assumes the previous milestone is
correct (the solver assumes a correct integrator and narrowphase); skipping
ahead compounds unverified assumptions, and this is a domain where
"compiles and passes unit tests" and "behaves correctly" are reliably
different things.

### V. Explicit Non-Goals (Scope Discipline)
The following are out of scope for the MVP and MUST NOT be added even when
they appear to be natural extensions of work already in progress: joints
and constraints (hinges, springs, motors); continuous collision detection;
sleeping/deactivation of resting bodies; concave or compound shapes;
spatial partitioning (grid/BVH) for broadphase; serialization/save-load;
parallelism; 3D. Any of these MUST be raised as a deliberate, separate
decision — via a README milestone update or a new spec — rather than
introduced as a side effect of implementing something else.

Rationale: physics engines invite scope creep (every feature suggests the
next one); the README's non-goals list is a deliberate MVP boundary, and
silently expanding it undermines the milestone-gated plan in Principle IV.

## Technical Constraints

- Single lib crate, no workspace split; module boundaries follow
  `README.md` § Architecture (`math.rs`, `shape.rs`, `body.rs`, `world.rs`,
  `broadphase.rs`, `collision/{mod,circle,polygon,manifold}.rs`,
  `solver.rs`).
- The engine crate produces geometry and body state only; it contains no
  rendering code. All macroquad calls live in `examples/*.rs`.
- Broadphase for the MVP is intentionally O(n²) with AABB rejection — this
  is not a performance bug to "fix" ahead of schedule; see Principle V.
- Tests (`cargo test`) cover math, collision, and solver behavior and MUST
  pass before a milestone is considered complete.

## Development Workflow

- Read `README.md` in full before writing code in this repository; it is
  authoritative over this file for architecture and math. `CLAUDE.md`
  adds process guidance and defers to the README on substance.
- Work proceeds milestone by milestone (Principle IV). Before starting a
  milestone's tasks, confirm the prior milestone's "done when" criterion
  actually passed, not just that its tasks are checked off.
- When a change would touch something on the non-goals list (Principle V),
  stop and confirm scope with the user before proceeding rather than
  assuming it's an acceptable extension.

## Governance

This constitution supersedes ad hoc practice for this repository. Any
amendment MUST update this file, record a Sync Impact Report at the top of
the diff/commit, and follow semantic versioning for the constitution
itself:

- MAJOR: backward-incompatible principle removals or redefinitions (e.g.
  dropping the zero-dependency rule, changing the milestone order's
  binding force).
- MINOR: a new principle or materially expanded section added.
- PATCH: wording, clarification, or typo fixes with no semantic change.

All work in this repository — code, examples, and generated specs/plans —
MUST be checked against the Core Principles above; conflicts are resolved
in favor of this constitution, with any necessary exception documented and
justified in the relevant plan's Complexity Tracking section. `README.md`
remains the authoritative technical spec (architecture and math);
`CLAUDE.md` carries supplementary runtime guidance for Claude Code and
MUST be kept consistent with this constitution.

**Version**: 1.0.0 | **Ratified**: 2026-08-25 | **Last Amended**: 2026-08-25
