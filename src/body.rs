//! RigidBody and BodyId — the physical objects a World simulates.

use crate::{Rot2, Shape, Vec2};

/// Opaque handle identifying a `RigidBody` within the `World` that created
/// it. Only ever constructed by `World::add_body`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct BodyId(pub(crate) u32);

/// A single physical object in the simulation.
#[derive(Clone, Debug, PartialEq)]
pub struct RigidBody {
    pub position: Vec2,
    pub velocity: Vec2,
    pub orientation: Rot2,
    pub angular_velocity: f32,
    /// `1 / mass`, or `0.0` for a static (infinite-mass) body.
    pub inv_mass: f32,
    pub is_static: bool,
    pub shape: Shape,
}

impl RigidBody {
    /// A dynamic body with the given `mass` (must be > 0.0) and `shape`,
    /// affected by gravity and integration.
    pub fn new_dynamic(position: Vec2, mass: f32, shape: Shape) -> Self {
        RigidBody {
            position,
            velocity: Vec2::ZERO,
            orientation: Rot2::IDENTITY,
            angular_velocity: 0.0,
            inv_mass: 1.0 / mass,
            is_static: false,
            shape,
        }
    }

    /// A static (immovable) body with the given `shape`, unaffected by
    /// gravity or integration.
    pub fn new_static(position: Vec2, shape: Shape) -> Self {
        RigidBody {
            position,
            velocity: Vec2::ZERO,
            orientation: Rot2::IDENTITY,
            angular_velocity: 0.0,
            inv_mass: 0.0,
            is_static: true,
            shape,
        }
    }
}
