//! World — body storage, gravity, and the fixed-timestep integrator.

use crate::collision::{self, manifold::Manifold};
use crate::joint::{self, JointId, RevoluteJoint};
use crate::solver::{self, ImpulseCache};
use crate::{BodyId, RigidBody, Rot2, Vec2};

/// Fixed simulation timestep: 1/60 second, per the README's integration
/// model. Rendering framerate never influences this.
pub(crate) const FIXED_DT: f32 = 1.0 / 60.0;

/// Owns all bodies in the simulation and advances them forward in time.
pub struct World {
    bodies: Vec<RigidBody>,
    joints: Vec<RevoluteJoint>,
    gravity: Vec2,
    accumulator: f32,
    /// Contacts found during the most recently completed fixed substep.
    contacts: Vec<Manifold>,
    /// Accumulated impulses of the previous substep, for warm-starting.
    impulse_cache: ImpulseCache,
}

impl World {
    /// A new, empty world with the default gravity (9.81 units/s²,
    /// downward) and no accumulated time.
    pub fn new() -> Self {
        World {
            bodies: Vec::new(),
            joints: Vec::new(),
            gravity: Vec2::new(0.0, -9.81),
            accumulator: 0.0,
            contacts: Vec::new(),
            impulse_cache: ImpulseCache::default(),
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

    /// Adds a joint to the world, returning a handle to it.
    ///
    /// Panics if `joint.body_a` and `joint.body_b` are equal, or if either
    /// does not refer to a body already in this world — the same
    /// precondition class `World::body` already has for an unknown
    /// `BodyId`.
    pub fn add_joint(&mut self, joint: RevoluteJoint) -> JointId {
        assert!(joint.body_a != joint.body_b, "a joint cannot connect a body to itself");
        assert!((joint.body_a.0 as usize) < self.bodies.len(), "joint.body_a is not in this world");
        assert!((joint.body_b.0 as usize) < self.bodies.len(), "joint.body_b is not in this world");
        let id = JointId(self.joints.len() as u32);
        self.joints.push(joint);
        id
    }

    /// Looks up a joint's current configuration by id.
    pub fn joint(&self, id: JointId) -> &RevoluteJoint {
        &self.joints[id.0 as usize]
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
    /// 3. the impulse solver: velocity iterations — at every contact a
    ///    tangent (friction) impulse clamped to `±μ·j`, then the normal
    ///    impulse, then every joint's coupled point-constraint impulse
    ///    (see `solver` and `joint`) — so a body that is both jointed and
    ///    touching a contact converges under both each iteration, not as
    ///    two independently-converged passes. Contacts that persist from
    ///    the previous substep start from the impulses they ended it with
    ///    (warm-starting; joints do not warm-start — see
    ///    specs/006-revolute-hinge-joint/research.md § 5). Velocity
    ///    iterations are followed by positional correction for both
    ///    contacts and joints;
    /// 4. `x += v * dt` and `θ += ω * dt` — semi-implicit Euler, so
    ///    position uses the final post-impulse velocity.
    ///
    /// Static bodies are never touched. `μ` for a contact is
    /// `√(friction_a · friction_b)`, so a body with `friction == 0.0` is
    /// frictionless.
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
            let mut contacts =
                solver::build_contacts(&self.bodies, &manifolds, self.gravity, FIXED_DT);
            let mut joints = joint::build_joints(&self.bodies, &self.joints);
            solver::warm_start(&mut self.bodies, &mut contacts, &self.impulse_cache);

            for _ in 0..solver::VELOCITY_ITERATIONS {
                solver::iterate_once(&mut self.bodies, &mut contacts);
                joint::iterate_once(&mut self.bodies, &mut joints);
            }

            solver::correct_positions(&mut self.bodies, &contacts);
            joint::correct_positions(&mut self.bodies, &joints);
            solver::store_impulses(&mut self.impulse_cache, &contacts);

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
    fn add_joint_returns_a_usable_id_and_joint_returns_what_was_stored() {
        let mut world = World::new();
        let a = world.add_body(RigidBody::new_static(Vec2::ZERO, Shape::circle(1.0)));
        let b = world.add_body(RigidBody::new_dynamic(Vec2::new(1.0, 0.0), 1.0, Shape::circle(1.0)));
        let anchor_a = Vec2::new(0.1, 0.2);
        let anchor_b = Vec2::new(-0.1, -0.2);

        let id = world.add_joint(RevoluteJoint::new(a, b, anchor_a, anchor_b));

        let stored = world.joint(id);
        assert_eq!(stored.body_a, a);
        assert_eq!(stored.body_b, b);
        assert_eq!(stored.anchor_a, anchor_a);
        assert_eq!(stored.anchor_b, anchor_b);
    }

    #[test]
    #[should_panic(expected = "cannot connect a body to itself")]
    fn add_joint_panics_when_body_a_equals_body_b() {
        let mut world = World::new();
        let a = world.add_body(RigidBody::new_dynamic(Vec2::ZERO, 1.0, Shape::circle(1.0)));
        world.add_joint(RevoluteJoint::new(a, a, Vec2::ZERO, Vec2::ZERO));
    }

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
