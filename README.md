# mirage-engine

A 2D rigid-body physics engine written from scratch in Rust. Gravity, collision
detection, and impulse-based resolution, with no physics dependencies and no
black boxes.

The engine crate (`mirage`) has zero dependencies. Every integrator step,
contact manifold, and impulse equation is derived in this repository and
commented with the formula it implements — not wrapped around Box2D, Rapier,
or any other existing solver. The goal is a physics engine you can read start
to finish and understand *why* a box falls, hits the ground, and stops.

## Status

Pre-MVP, one confirmation away. M0–M3 have landed: math, bodies and the
fixed-timestep integrator, collision detection, and normal-impulse resolution
with restitution (`cargo run --example bouncing --release`). M4 is implemented
and verified headlessly: Coulomb friction, warm-started stacking, a 10-box
tower that stands for 60 s, a 5-row pyramid, and a box that holds on a
shallow ramp and slides on a steep one (`cargo test`). The M4 checkbox below
stays open until `stack`, `ramp` and `pyramid` have been watched with
`--release`, per the milestone's "done when". The milestones below (§ MVP
milestones) are the source of truth for progress; this section gets updated
as they land.

## Design principles

- **Zero dependencies in the engine crate.** `macroquad` is used only as a
  `dev-dependency` for the visual examples — `mirage` itself pulls in nothing.
- **Fixed timestep, deterministic.** Given the same inputs and the same
  iteration order, a simulation reproduces the same result every run.
- **Readable over clever.** Prefer the formulation that matches the textbook
  derivation over a micro-optimized rewrite.
- **No `unsafe`, no premature spatial-index optimization.** Correctness and
  clarity first; broadphase acceleration structures are a post-MVP concern.

## Architecture

```
mirage-engine/
├── Cargo.toml           # lib `mirage`; macroquad only under [dev-dependencies]
├── src/
│   ├── lib.rs           # public API re-exports
│   ├── math.rs          # Vec2, Rot2, cross products, clamp helpers
│   ├── shape.rs         # Shape { Circle, Polygon }, area / inertia / AABB
│   ├── body.rs          # RigidBody, BodyId, material properties
│   ├── world.rs         # World: body storage, step(dt), accumulator
│   ├── broadphase.rs    # candidate pair generation
│   ├── collision/
│   │   ├── mod.rs       # dispatch by shape pair
│   │   ├── circle.rs    # circle–circle, circle–polygon
│   │   ├── polygon.rs   # SAT axis test, reference/incident face
│   │   └── manifold.rs  # Contact, Manifold, face clipping
│   └── solver.rs        # sequential impulses: normal, restitution, friction,
│                        #   warm-starting
├── tests/               # behavioral scenes on the public API
│   ├── common/mod.rs    #   scene builders, metrics, acceptance bounds
│   ├── friction.rs      #   ramps: hold/slide, Coulomb acceleration, rolling disc
│   └── stacking.rs      #   10-box tower, pyramid, mixed shapes, determinism
└── examples/
    ├── common/mod.rs    # shared drawing helpers for stack and pyramid
    ├── bouncing.rs      # circles falling onto a static floor
    ├── ramp.rs          # shallow ramp holds, steep ramp slides
    ├── stack.rs         # 10-box tower — the MVP acceptance demo
    └── pyramid.rs       # stress case
```

One lib crate keeps the dependency story clean: `macroquad` is a
dev-dependency, so anything depending on `mirage` inherits nothing but the
math and the solver. `cargo run --example stack` is the entire build story
for seeing it work.

## How it works

### Integration

Semi-implicit (symplectic) Euler:

```
v += (F / m + g) * dt
x += v * dt
```

with the same pair of updates for angular velocity `ω` and angle `θ`.
Velocity is updated before position because semi-implicit Euler is stable for
oscillatory systems, where explicit Euler steadily injects energy and blows
up. `World::step` runs on a fixed `dt` of `1/60` driven by an accumulator, so
simulation behavior never changes with rendering framerate.

### Broadphase

The MVP broadphase is an honest O(n²) pair loop with AABB overlap rejection —
no grid, no BVH, no sweep-and-prune. That's fine at the scale the demos run
at (hundreds of bodies); a spatial index is a deliberate post-MVP
optimization, not an oversight.

### Narrowphase

Dispatched per shape pair:

- **circle–circle** — overlap when `|d| < r₁ + r₂`, where `d` is the vector
  between centers. Normal is `d.normalize()`, one contact point.
- **circle–polygon** — closest point on the polygon's edges/vertices to the
  circle's center; overlap if that distance is less than the radius.
- **polygon–polygon** — SAT (Separating Axis Theorem) over both bodies' face
  normals. The axis of minimum penetration picks the *reference face*; the
  face on the other body most anti-parallel to it is the *incident face*;
  the incident face is clipped against the reference face's side planes to
  produce a 1–2 point contact manifold.

Every contact carries a `point`, a `normal`, a `penetration` depth, and a
`feature` id naming which piece of geometry produced the point (a circle's
single contact is always `0`; a polygon contact packs the reference face,
incident face and clipped endpoint). The solver uses it only to recognize the
same physical contact from one step to the next (§ Resolution, warm-starting).

### Resolution

Sequential impulses, iterated (16 velocity iterations per step, tuned in M4 on
1 m bodies — a 10-box tower needs more than the ~8 of a bare solver):

- Relative velocity at the contact point:
  `vr = (v_b + ω_b × r_b) − (v_a + ω_a × r_a)`
- Normal impulse magnitude (skipped if the bodies are already separating):

  ```
  j = −(1 + e) · (vr · n)
      ────────────────────────────────────────────
      1/m_a + 1/m_b + (r_a×n)²/I_a + (r_b×n)²/I_b
  ```

- `e` is the pair's combined restitution, `max(e_a, e_b)`, so a bouncy ball
  bounces off a default (`e = 0`) floor. Restitution is applied to the
  approach velocity *before* the current step's gravity: the contact step's
  own `g·dt` kick is subtracted out of `vr·n`. A body resting under gravity
  then approaches at 0 and never bounces, and a bouncing body decays
  geometrically to rest. (Measured after the kick, `v → e·(v + g·dt)` has a
  fixed point at `e·g·dt/(1 − e)`, so a ball settles into a tiny endless
  hop instead of stopping.)
- Because the solver iterates, `j` is applied in its iterated form. Before
  the first iteration the target normal velocity `−e·(vr·n)₀` is fixed from
  the pre-gravity approach velocity `(vr·n)₀`; each iteration then applies
  `Δj = −(vr·n − (−e·(vr·n)₀)) / K`, where `K` is the denominator above,
  and the *accumulated* `j` is clamped to `≥ 0` (contacts push, never pull).
  For a single contact on its first iteration this is exactly the `j` above.
- Coulomb friction as a tangent impulse. With the tangent `t = (−n_y, n_x)`
  (the normal rotated 90°) and the same `vr` as above, the tangential
  relative velocity is `vt = vr · t`, and the tangent effective mass uses the
  same form as the normal one:

  ```
  K_t = 1/m_a + 1/m_b + (r_a×t)²/I_a + (r_b×t)²/I_b
  Δjt = −vt / K_t
  ```

  The target is `vt = 0` (sticking; there is no "restitution" for friction).
  The *accumulated* tangent impulse `jt` is clamped to `[−μ·j, +μ·j]`, where
  `j` is the same contact's accumulated normal impulse — not the per-iteration
  increment — so friction capacity grows as load builds up through a stack.
  Each contact visit applies the tangent impulse first and the normal impulse
  second, so the non-penetration constraint has the last word.
- `μ` is the pair's combined friction, `√(μ_a · μ_b)` — symmetric in the two
  bodies, `0` if either surface is frictionless, and equal to the shared
  value when both match. (Not `max`, as for `e`: a `μ = 0` body must stay
  frictionless whatever it touches.) Bodies default to `μ = 0.5`, so the
  slide/hold threshold on a ramp is `θ = atan(μ)`.
- Warm-starting: at the end of a step, each contact's accumulated `j` and
  `jt` are remembered under `(body_a, body_b, feature)`. On the next step a
  contact with the same key starts from those impulses (re-applied to both
  bodies before iterating) instead of from zero; a contact with no match
  starts at zero, and a contact that has ended is forgotten. Restitution's
  target velocity is computed *before* this seeding, from the true approach
  velocity. Without it a 10-box tower rocks itself apart, because a
  two-point manifold solved one point at a time leaves a small residual
  torque on every step; carrying the impulses across steps lets the
  iterations converge over time instead of restarting.
- Penetration is corrected with a Baumgarte-style positional bias plus a slop
  term: after the velocity iterations, bodies are moved apart along `n` by
  `percent · max(depth − slop, 0) / (1/m_a + 1/m_b)`, split across the
  manifold's points and weighted by inverse mass (`percent = 0.4`,
  `slop = 0.002`, tuned in M4: a resting stack compresses by about `slop` per
  interface, so 0.01 sinks a 10-box tower by ~10% of a box while 0.002 keeps
  it near 2%; much smaller and resting contacts start to jitter). Correcting positions directly, rather than biasing the
  velocity, keeps the restitution solve energy-clean, and the slop stops
  resting contacts from jittering or sinking.

Known trade-off: sequential impulses converge iteratively rather than
solving the contact LCP exactly, so tall stacks stay slightly soft under
load. That's expected behavior for this class of solver, not a bug to chase.

## MVP milestones

- [x] **M0 — Scaffold & math.** `cargo new --lib`, `math.rs` with `Vec2`/
      `Rot2` and unit tests.
      *Done when:* `cargo test` passes with dot/cross/rotation coverage.
- [x] **M1 — Bodies, gravity, integrator.** `RigidBody`, `World::step`,
      fixed-timestep accumulator, `examples/bouncing.rs` rendering circles
      with no collision yet.
      *Done when:* circles fall off-screen at `9.81 m/s²` under macroquad.
- [x] **M2 — Collision detection.** AABB broadphase, all three narrowphase
      pairs, manifold generation, contact points/normals debug-drawn.
      *Done when:* contacts render correctly for a box resting on a rotated
      ramp.
- [x] **M3 — Impulse resolution.** Normal impulses with restitution and
      positional correction.
      *Done when:* a ball dropped with `e = 1.0` returns to within a few
      percent of its drop height, and with `e = 0` it stops dead.
- [ ] **M4 — Friction & stacking.** Tangent impulses, iteration count
      tuning, warm-starting if needed.
      *Done when:* `examples/stack.rs` holds a 10-box tower stable for 60
      seconds without visible jitter or sinking, and a box on a shallow ramp
      stays put while one on a steep ramp slides.

The MVP is complete at M4.

## Explicit non-goals for the MVP

- Joints and constraints (hinges, springs, motors)
- Continuous collision detection — fast-moving bodies **will** tunnel
  through thin geometry, and that's a known limitation, not a surprise
- Sleeping / deactivation of resting bodies
- Concave or compound shapes
- Spatial partitioning (grid/BVH) for broadphase
- Serialization / save-load
- Parallelism
- 3D

## Running it

```
cargo run --example stack --release
cargo test
```

`--release` matters here: debug builds of the solver are meaningfully
slower, especially with iteration counts turned up.

## References

These were read and re-derived, not copied — that distinction is the whole
premise of this repo:

- Erin Catto's GDC physics talks and [Box2D Lite](https://github.com/erincatto/box2d-lite)
- Chris Hecker's *Rigid Body Dynamics* article series
- Randy Gaul's impulse-engine writeups

## License

MIT — see [LICENSE](LICENSE).
