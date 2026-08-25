# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project state

Pre-MVP: no `Cargo.toml` or `src/` exist yet. `README.md` is the authoritative
spec — it defines the architecture, the math each module must implement, and
the milestone order (M0–M4) that development follows. Read `README.md` in
full before writing code here; this file only adds what the README doesn't
already say.

## What this project is

A 2D rigid-body physics engine (`mirage`) written from scratch in Rust —
custom `Vec2`/`Rot2` math, AABB broadphase, SAT narrowphase with clipped
contact manifolds, and a sequential-impulse solver. No physics or math
crates. The engine crate must have **zero dependencies**; `macroquad` is
permitted only as a `dev-dependency` for the `examples/` demos.

## Commands

Not yet runnable — no crate exists. Once scaffolded (M0), the standard
workflow will be:

```
cargo test                            # unit tests (math, collision, solver)
cargo test <test_name>                # run a single test
cargo run --example stack --release   # MVP acceptance demo: 10-box tower
cargo run --example bouncing --release
cargo run --example pyramid --release
```

`--release` is not optional for the examples — debug builds of the solver
are meaningfully slower, especially with velocity iterations turned up.

## Architecture

Single lib crate, no workspace split. Module responsibilities (see
`README.md` § Architecture for the full tree):

- `math.rs` — `Vec2`, `Rot2`, cross products. Everything else depends on this.
- `shape.rs` — `Shape::{Circle, Polygon}` with area/inertia/AABB.
- `body.rs` — `RigidBody`, `BodyId`, material properties (mass, restitution, friction).
- `world.rs` — `World::step(dt)`, body storage, the fixed-timestep accumulator.
- `broadphase.rs` — O(n²) candidate-pair generation with AABB rejection (intentionally naive for the MVP — see README non-goals).
- `collision/` — narrowphase, split by shape pair (`circle.rs`, `polygon.rs`) plus manifold construction (`manifold.rs`). `mod.rs` dispatches by shape-pair type.
- `solver.rs` — sequential-impulse resolution (normal + friction + Baumgarte positional correction).

Data flow per `World::step`: broadphase produces candidate pairs →
narrowphase produces `Contact`/`Manifold` (point, normal, penetration) →
solver iterates impulses over manifolds (~8 velocity iterations) → integrator
applies the resulting velocities to position (semi-implicit Euler).

The engine has no rendering code. `examples/*.rs` own all macroquad calls;
`mirage` itself only ever produces geometry and body state.

## Working on this repo

- Follow milestone order (M0 → M4 in `README.md`) — later milestones assume
  earlier ones are solid (e.g. don't build the solver before the integrator
  and narrowphase are correct and tested).
- Match the math in code to the derivations written out in README § How it
  works exactly (variable names like `e`, `μ`, `j`, `vr` included) — the
  README's equations are the spec, not a paraphrase.
- Keep the non-goals (README § Explicit non-goals) out of scope even if they
  seem like natural extensions: no joints, no CCD, no sleeping, no concave
  shapes, no spatial index, no serialization, no parallelism, no 3D.
- When a milestone's "done when" criterion is observable via an example
  (e.g. M4's 60-second stable stack), verify it by actually running that
  example — this is a physics engine, correctness is behavioral, not just
  compile-clean.

<!-- SPECKIT START -->
For additional context about technologies to be used, project structure,
shell commands, and other important information, read the current plan
<!-- SPECKIT END -->
