# Contract: Public Rust API surface added by M1

This is a library, not a network/CLI service, so the "contract" is the
public `mirage` API surface this milestone adds — the signatures downstream
code (including `examples/bouncing.rs`) can rely on. Exact naming may be
refined during implementation, but the shape and semantics below are
binding per the spec's functional requirements.

## `src/body.rs`

```rust
pub struct BodyId(/* opaque */);

pub struct RigidBody {
    pub position: Vec2,
    pub velocity: Vec2,
    pub orientation: Rot2,
    pub angular_velocity: f32,
    pub inv_mass: f32,
    pub is_static: bool,
}

impl RigidBody {
    /// Dynamic body with the given mass (must be > 0.0).
    pub fn new_dynamic(position: Vec2, mass: f32) -> Self;

    /// Static (immovable) body — inv_mass is 0.0.
    pub fn new_static(position: Vec2) -> Self;
}
```

- `new_dynamic`/`new_static` are the only constructors this milestone
  requires (matches FR-001/FR-002); no setters/mutators beyond what
  `World::step` needs internally.

## `src/world.rs`

```rust
pub struct World {
    // gravity, fixed_dt, accumulator, body storage — see data-model.md
}

impl World {
    /// New world with default gravity (9.81 units/s^2, downward) and an
    /// empty accumulator.
    pub fn new() -> Self;

    /// Add a body to the world, returning a handle to it.
    pub fn add_body(&mut self, body: RigidBody) -> BodyId;

    /// Look up a body's current state by id.
    pub fn body(&self, id: BodyId) -> &RigidBody;

    /// Advance the simulation by `dt_real` seconds of real (render) time.
    /// Internally drains the fixed-timestep accumulator, integrating every
    /// dynamic body with semi-implicit Euler under gravity, once per
    /// 1/60s step. No collision detection or response occurs.
    pub fn step(&mut self, dt_real: f32);
}
```

- **Pre/postconditions on `step`**:
  - Precondition: `dt_real >= 0.0`.
  - Postcondition: every dynamic body's `velocity`/`position` reflect
    exactly `floor((accumulator_before + dt_real) / fixed_dt)` applications
    of the semi-implicit Euler update; every static body is byte-for-byte
    unchanged; the internal accumulator holds the remainder
    `< fixed_dt`.
  - Determinism: for fixed `fixed_dt` and a given total elapsed time,
    repeated calls to `step` with any partition of that elapsed time into
    per-call `dt_real` values produce the same final body states (within
    `f32` tolerance), per FR-007/SC-003.

## `src/lib.rs`

```rust
pub mod body;
pub mod world;

pub use body::{BodyId, RigidBody};
pub use world::World;
```

Consistent with the existing `pub use math::{Rot2, Vec2};` re-export
pattern from M0.

## `examples/bouncing.rs` (consumer, not part of the crate's public API)

Uses only the public surface above plus `macroquad` for rendering:
constructs a `World`, adds several dynamic circle bodies (radius tracked
locally in the example, per spec Assumptions) and calls
`world.step(macroquad::time::get_frame_time())` once per frame, drawing
each body at its current `position`.
