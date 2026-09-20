//! World — body storage, gravity, and the fixed-timestep integrator.

use crate::collision::{self, manifold::Manifold};
use crate::{solver, BodyId, RigidBody, Rot2, Vec2};

/// Fixed simulation timestep: 1/60 second, per the README's integration
/// model. Rendering framerate never influences this.
pub(crate) const FIXED_DT: f32 = 1.0 / 60.0;

/// Owns all bodies in the simulation and advances them forward in time.
pub struct World {
    bodies: Vec<RigidBody>,
    gravity: Vec2,
    accumulator: f32,
    /// Contacts found during the most recently completed fixed substep.
    contacts: Vec<Manifold>,
}

impl World {
    /// A new, empty world with the default gravity (9.81 units/s²,
    /// downward) and no accumulated time.
    pub fn new() -> Self {
        World {
            bodies: Vec::new(),
            gravity: Vec2::new(0.0, -9.81),
            accumulator: 0.0,
            contacts: Vec::new(),
        }
    }

    /// Adds a body to the world, returning a handle to it.
    pub fn add_body(&mut self, body: RigidBody) -> BodyId {
        let id = BodyId(self.bodies.len() as u32);
        self.bodies.push(body);
        id
    }

    /// Looks up a body's current state by id.
    pub fn body(&self, id: BodyId) -> &RigidBody {
        &self.bodies[id.0 as usize]
    }

    /// The contacts found during the most recently completed fixed
    /// substep, as they were when detected — before that substep's
    /// impulses and position correction were applied. Empty before the
    /// first substep has ever run.
    pub fn contacts(&self) -> &[Manifold] {
        &self.contacts
    }

    /// Advances the simulation by `dt_real` seconds of real (render) time.
    ///
    /// Internally accumulates `dt_real` and drains it in fixed `1/60`
    /// increments, so the number and size of simulation steps never
    /// depends on how `dt_real` was chopped up across calls. Each fixed
    /// step runs, in order:
    ///
    /// 1. `v += g * dt` for every dynamic body (`F = 0`);
    /// 2. broadphase + narrowphase to find contacts;
    /// 3. the impulse solver: velocity iterations, then positional
    ///    correction (see `solver`);
    /// 4. `x += v * dt` and `θ += ω * dt` — semi-implicit Euler, so
    ///    position uses the final post-impulse velocity.
    ///
    /// Static bodies are never touched. There is no friction yet (M4);
    /// impulses act only along contact normals.
    pub fn step(&mut self, dt_real: f32) {
        self.accumulator += dt_real;

        while self.accumulator >= FIXED_DT {
            for body in &mut self.bodies {
                if body.is_static {
                    continue;
                }
                body.velocity += self.gravity * FIXED_DT;
            }

            let manifolds = collision::detect_contacts(&self.bodies);
            solver::resolve(&mut self.bodies, &manifolds, self.gravity, FIXED_DT);

            for body in &mut self.bodies {
                if body.is_static {
                    continue;
                }
                body.position += body.velocity * FIXED_DT;
                body.orientation =
                    Rot2::new(body.orientation.angle() + body.angular_velocity * FIXED_DT);
            }

            self.accumulator -= FIXED_DT;
            self.contacts = manifolds;
        }
    }
}

impl Default for World {
    fn default() -> Self {
        World::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Shape;

    const EPS: f32 = 1e-4;

    #[test]
    fn dynamic_body_falls_at_g_times_t() {
        let mut world = World::new();
        let ball = world.add_body(RigidBody::new_dynamic(Vec2::new(0.0, 100.0), 1.0, Shape::circle(1.0)));

        let steps = 60;
        for _ in 0..steps {
            world.step(FIXED_DT);
        }

        let t = steps as f32 * FIXED_DT;
        let expected_vy = -9.81 * t;
        assert!(
            (world.body(ball).velocity.y - expected_vy).abs() < EPS,
            "expected vy ~= {expected_vy}, got {}",
            world.body(ball).velocity.y
        );
    }

    #[test]
    fn static_body_never_moves() {
        let mut world = World::new();
        let ground = world.add_body(RigidBody::new_static(Vec2::new(3.0, -5.0), Shape::circle(1.0)));

        for _ in 0..600 {
            world.step(FIXED_DT);
        }

        assert_eq!(world.body(ground).position, Vec2::new(3.0, -5.0));
        assert_eq!(world.body(ground).velocity, Vec2::ZERO);
    }

    #[test]
    fn gravity_is_mass_independent() {
        let mut world = World::new();
        let light = world.add_body(RigidBody::new_dynamic(Vec2::new(0.0, 50.0), 1.0, Shape::circle(1.0)));
        let heavy = world.add_body(RigidBody::new_dynamic(Vec2::new(0.0, 50.0), 1000.0, Shape::circle(1.0)));

        for _ in 0..30 {
            world.step(FIXED_DT);
        }

        assert!((world.body(light).velocity.y - world.body(heavy).velocity.y).abs() < EPS);
        assert!((world.body(light).position.y - world.body(heavy).position.y).abs() < EPS);
    }

    #[test]
    fn same_total_elapsed_time_is_deterministic_regardless_of_partitioning() {
        // 20.5 fixed steps' worth of real time, chosen away from an exact
        // step boundary so tiny f32 summation error can't flip which side
        // of the boundary either partition lands on.
        let total_time = 20.5 * FIXED_DT;

        let mut one_big_step = World::new();
        let a = one_big_step.add_body(RigidBody::new_dynamic(Vec2::new(0.0, 200.0), 2.0, Shape::circle(1.0)));
        one_big_step.step(total_time);

        let mut many_small_steps = World::new();
        let b = many_small_steps.add_body(RigidBody::new_dynamic(Vec2::new(0.0, 200.0), 2.0, Shape::circle(1.0)));
        let slice = total_time / 41.0;
        for _ in 0..41 {
            many_small_steps.step(slice);
        }

        assert!(
            (one_big_step.body(a).velocity.y - many_small_steps.body(b).velocity.y).abs() < EPS
        );
        assert!(
            (one_big_step.body(a).position.y - many_small_steps.body(b).position.y).abs() < EPS
        );
    }

    #[test]
    fn accumulator_never_retains_a_full_fixed_step() {
        let mut world = World::new();
        world.add_body(RigidBody::new_dynamic(Vec2::new(0.0, 0.0), 1.0, Shape::circle(1.0)));

        // An irregular, non-multiple-of-FIXED_DT real-time input.
        world.step(FIXED_DT * 3.7);

        assert!(world.accumulator < FIXED_DT);
        assert!(world.accumulator >= 0.0);
    }

    #[test]
    fn step_on_empty_world_does_not_panic() {
        let mut world = World::new();
        world.step(FIXED_DT * 5.0);
    }

    #[test]
    fn gravity_accumulates_on_top_of_initial_velocity() {
        let mut world = World::new();
        let mut initial = RigidBody::new_dynamic(Vec2::new(0.0, 0.0), 1.0, Shape::circle(1.0));
        initial.velocity = Vec2::new(3.0, 7.0);
        let id = world.add_body(initial);

        world.step(FIXED_DT);

        let expected_vy = 7.0 + (-9.81) * FIXED_DT;
        assert!((world.body(id).velocity.x - 3.0).abs() < EPS);
        assert!((world.body(id).velocity.y - expected_vy).abs() < EPS);
    }

    #[test]
    fn contacts_are_still_reported_after_resolution_is_wired_in() {
        let mut world = World::new();
        world.add_body(RigidBody::new_dynamic(Vec2::new(0.0, 0.5), 1.0, Shape::circle(1.0)));
        world.add_body(RigidBody::new_static(Vec2::new(0.0, 0.0), Shape::circle(1.0)));

        world.step(FIXED_DT);

        assert!(!world.contacts().is_empty());
    }

    #[test]
    fn angular_velocity_rotates_orientation() {
        let mut world = World::new();
        let mut body = RigidBody::new_dynamic(Vec2::new(0.0, 100.0), 1.0, Shape::circle(1.0));
        body.angular_velocity = 1.0;
        let id = world.add_body(body);

        for _ in 0..60 {
            world.step(FIXED_DT);
        }

        assert!((world.body(id).orientation.angle() - 1.0).abs() < 1e-3);
    }
}
