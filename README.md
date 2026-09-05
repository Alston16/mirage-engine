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

Pre-MVP. Nothing runs yet — this README is the spec the code will be built
against. The milestones below (§ MVP milestones) are the source of truth for
progress; this section gets updated as they land.

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
│   └── solver.rs        # sequential impulses: normal, restitution, friction
└── examples/
    ├── bouncing.rs      # circles falling onto a static floor
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

Every contact carries a `point`, a `normal`, and a `penetration` depth.

### Resolution

Sequential impulses, iterated (~8 velocity iterations per step):

- Relative velocity at the contact point:
  `vr = (v_b + ω_b × r_b) − (v_a + ω_a × r_a)`
- Normal impulse magnitude (skipped if the bodies are already separating):

  ```
  j = −(1 + e) · (vr · n)
      ────────────────────────────────────────────
      1/m_a + 1/m_b + (r_a×n)²/I_a + (r_b×n)²/I_b
  ```

- Coulomb friction as a tangent impulse, clamped to `±μ · j`.
- Penetration is corrected with Baumgarte positional bias plus a slop term,
  so resting contacts don't jitter and stacks don't slowly sink into the
  floor.

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
- [ ] **M3 — Impulse resolution.** Normal impulses with restitution and
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
