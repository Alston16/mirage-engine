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
    /// `1 / I` about the body's origin, or `0.0` for a static body.
    pub inv_inertia: f32,
    pub is_static: bool,
    /// Bounciness `e` in `[0.0, 1.0]`: `0.0` absorbs all approach speed,
    /// `1.0` preserves it. Defaults to `0.0`.
    pub restitution: f32,
    pub shape: Shape,
}

impl RigidBody {
    /// A dynamic body with the given `mass` (must be > 0.0) and `shape`,
    /// affected by gravity and integration.
    pub fn new_dynamic(position: Vec2, mass: f32, shape: Shape) -> Self {
        let inv_inertia = 1.0 / shape.inertia(mass);
        RigidBody {
            position,
            velocity: Vec2::ZERO,
            orientation: Rot2::IDENTITY,
            angular_velocity: 0.0,
            inv_mass: 1.0 / mass,
            inv_inertia,
            is_static: false,
            restitution: 0.0,
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
            inv_inertia: 0.0,
            is_static: true,
            restitution: 0.0,
            shape,
        }
    }
}

impl RigidBody {
    /// Returns this body with its restitution `e` set, clamped to
    /// `[0.0, 1.0]`.
    pub fn with_restitution(mut self, e: f32) -> Self {
        self.restitution = e.clamp(0.0, 1.0);
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dynamic_circle_derives_inv_inertia_from_shape_and_mass() {
        // I = ½·m·r² = ½·4·2² = 8.
        let body = RigidBody::new_dynamic(Vec2::ZERO, 4.0, Shape::circle(2.0));
        assert!((body.inv_inertia - 1.0 / 8.0).abs() < 1e-6);
    }

    #[test]
    fn static_body_has_zero_inverse_mass_and_inertia() {
        let body = RigidBody::new_static(Vec2::ZERO, Shape::circle(1.0));
        assert_eq!(body.inv_mass, 0.0);
        assert_eq!(body.inv_inertia, 0.0);
    }

    #[test]
    fn restitution_defaults_to_zero() {
        assert_eq!(RigidBody::new_dynamic(Vec2::ZERO, 1.0, Shape::circle(1.0)).restitution, 0.0);
        assert_eq!(RigidBody::new_static(Vec2::ZERO, Shape::circle(1.0)).restitution, 0.0);
    }

    #[test]
    fn with_restitution_clamps_to_unit_interval() {
        let base = || RigidBody::new_dynamic(Vec2::ZERO, 1.0, Shape::circle(1.0));
        assert_eq!(base().with_restitution(0.5).restitution, 0.5);
        assert_eq!(base().with_restitution(1.5).restitution, 1.0);
        assert_eq!(base().with_restitution(-0.2).restitution, 0.0);
    }
}
