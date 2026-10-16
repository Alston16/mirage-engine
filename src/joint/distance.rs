//! Distance / spring joint: a 2-body constraint on the distance between two
//! anchors, rigid or soft. See README § Joints.
//!
//! Rigid mode is a scalar equality constraint along the anchor-to-anchor
//! line `n`, solved in the shared velocity-iteration loop after the
//! revolute joints. Soft mode is an explicit Hookean force applied once per
//! step from start-of-step state, in the same "forces" phase as gravity and
//! before gravity's kick (see `apply_spring_forces`).

use super::{JOINT_CORRECTION_PERCENT, JOINT_SLOP};
use crate::{BodyId, RigidBody, Vec2};

/// A 2-body constraint on the distance between a local anchor on `body_a`
/// and a local anchor on `body_b`. Rigid by default (the separation is held
/// at `length`); with a `Spring`, a Hookean force pulls the separation
/// toward `length` instead. Never constrains direction or relative
/// rotation.
#[derive(Clone, Debug, PartialEq)]
pub struct DistanceJoint {
    pub body_a: BodyId,
    pub body_b: BodyId,
    /// Anchor on `body_a`, in `body_a`'s local frame.
    pub anchor_a: Vec2,
    /// Anchor on `body_b`, in `body_b`'s local frame.
    pub anchor_b: Vec2,
    /// Target (rest) length `L0`. `None` means "the anchors' separation
    /// when the joint is added"; `World::add_distance_joint` resolves it.
    pub length: Option<f32>,
    /// `None` = rigid; `Some` = spring.
    pub spring: Option<Spring>,
}

impl DistanceJoint {
    /// Rigid joint, length taken from the anchors' current separation when
    /// added to the world.
    pub fn new(body_a: BodyId, body_b: BodyId, anchor_a: Vec2, anchor_b: Vec2) -> Self {
        DistanceJoint { body_a, body_b, anchor_a, anchor_b, length: None, spring: None }
    }

    /// Sets the target / rest length `L0` explicitly. A rigid joint with
    /// `L0 = 0` has no defined direction at its target — pin the anchors
    /// together with a `RevoluteJoint` instead.
    pub fn with_length(mut self, length: f32) -> Self {
        self.length = Some(length);
        self
    }

    /// Switches to soft mode: `F = −(k·(L − L0) + c·L̇)` along the anchor
    /// line, with `k = stiffness` and `c = damping`.
    pub fn with_spring(mut self, stiffness: f32, damping: f32) -> Self {
        self.spring = Some(Spring { stiffness, damping });
        self
    }

    /// Damping `c = 2·√(k·m)` that makes a spring of `stiffness` critically
    /// damped (`ζ = 1`) for one body of `mass` moving along the joint line.
    pub fn critical_damping(stiffness: f32, mass: f32) -> f32 {
        2.0 * (stiffness * mass).sqrt()
    }
}

/// Hookean spring parameters for a soft `DistanceJoint`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Spring {
    /// `k`, force per unit of deviation from rest length.
    pub stiffness: f32,
    /// `c`, force per unit of separation speed.
    pub damping: f32,
}

/// Opaque handle identifying a `DistanceJoint` within the `World` that
/// created it. Only ever constructed by `World::add_distance_joint`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct DistanceJointId(pub(crate) u32);

/// Anchor separation below which the joint line `n` is undefined: the joint
/// applies nothing that step rather than normalizing a zero vector
/// (specs/007-distance-spring-joint/research.md § 6).
pub(crate) const DISTANCE_EPSILON: f32 = 1e-6;

/// Per-joint working state, built once per fixed step and discarded — the
/// distance-joint analog of `JointState`.
pub(crate) struct DistanceState {
    a: usize,
    b: usize,
    /// World-space offset from each body's center to its anchor. Fixed for
    /// the step: orientation only integrates after resolution.
    r_a: Vec2,
    r_b: Vec2,
    /// Unit vector `d / L` from `a`'s anchor to `b`'s, fixed for the step.
    n: Vec2,
    /// `C = L − L0` at step start. Feeds the spring force; rigid positional
    /// correction recomputes it from current positions instead.
    c: f32,
    /// Resolved rest length `L0`.
    length: f32,
    /// `1 / K`, `K = 1/m_a + 1/m_b + (r_a×n)²/I_a + (r_b×n)²/I_b` — the
    /// contact normal's effective mass, along `n`.
    k_inv: f32,
    spring: Option<Spring>,
}

/// Builds working state for every distance joint. Drops a joint between two
/// bodies that can't move (mirroring `joint::build_joints`) and a joint
/// whose anchors coincide, where `n` is undefined.
pub(crate) fn build_distance_joints(bodies: &[RigidBody], joints: &[DistanceJoint]) -> Vec<DistanceState> {
    let mut states = Vec::new();
    for joint in joints {
        let a = joint.body_a.0 as usize;
        let b = joint.body_b.0 as usize;
        let (body_a, body_b) = (&bodies[a], &bodies[b]);
        let inv_sum = body_a.inv_mass + body_b.inv_mass;
        if inv_sum == 0.0 {
            continue;
        }

        let r_a = body_a.orientation.rotate(joint.anchor_a);
        let r_b = body_b.orientation.rotate(joint.anchor_b);
        let d = (body_b.position + r_b) - (body_a.position + r_a);
        let l = d.length();
        if l < DISTANCE_EPSILON {
            continue;
        }
        let n = d / l;
        let length = joint.length.expect("length is resolved by World::add_distance_joint");

        let rn_a = r_a.cross(n);
        let rn_b = r_b.cross(n);
        let k = inv_sum + rn_a * rn_a * body_a.inv_inertia + rn_b * rn_b * body_b.inv_inertia;

        states.push(DistanceState {
            a,
            b,
            r_a,
            r_b,
            n,
            c: l - length,
            length,
            k_inv: 1.0 / k,
            spring: joint.spring,
        });
    }
    states
}

/// `Cdot = n · ((v_b + ω_b × r_b) − (v_a + ω_a × r_a))` — how fast the
/// anchors are separating along `n`.
fn separation_rate(bodies: &[RigidBody], s: &DistanceState) -> f32 {
    let (v_a, w_a) = (bodies[s.a].velocity, bodies[s.a].angular_velocity);
    let (v_b, w_b) = (bodies[s.b].velocity, bodies[s.b].angular_velocity);
    let vr = (v_b + Vec2::cross_sv(w_b, s.r_b)) - (v_a + Vec2::cross_sv(w_a, s.r_a));
    s.n.dot(vr)
}

/// Applies `P = λ·n` at the joint: `−P` to `body_a`, `+P` to `body_b`,
/// leaving static bodies untouched.
fn apply_impulse(bodies: &mut [RigidBody], s: &DistanceState, lambda: f32) {
    let p = s.n * lambda;
    let body_a = &mut bodies[s.a];
    if !body_a.is_static {
        body_a.velocity -= p * body_a.inv_mass;
        body_a.angular_velocity -= s.r_a.cross(p) * body_a.inv_inertia;
    }
    let body_b = &mut bodies[s.b];
    if !body_b.is_static {
        body_b.velocity += p * body_b.inv_mass;
        body_b.angular_velocity += s.r_b.cross(p) * body_b.inv_inertia;
    }
}

/// Soft mode: the Hookean force `F = −(k·C + c·Cdot)` along `n`, applied as
/// the impulse `λ = F·dt` once per step.
///
/// Runs from start-of-step state, *before* gravity's `v += g·dt` kick:
/// evaluated after it, the damper would read this step's `g·dt` as
/// separation speed and push back with a constant `c·g·dt`, settling a
/// hanging box `c·g·dt / k` away from equilibrium
/// (specs/007-distance-spring-joint/research.md § 2).
pub(crate) fn apply_spring_forces(bodies: &mut [RigidBody], joints: &[DistanceState], dt: f32) {
    for s in joints {
        let Some(Spring { stiffness: k, damping: c }) = s.spring else {
            continue;
        };
        let cdot = separation_rate(bodies, s);
        let lambda = -(k * s.c + c * cdot) * dt;
        apply_impulse(bodies, s, lambda);
    }
}

/// Rigid mode, one Gauss-Seidel pass: `λ = −Cdot / K`, driving the anchors'
/// separation rate to zero. This is the contact normal's `Δj` with no `≥ 0`
/// clamp — an equality constraint pulls as readily as it pushes.
pub(crate) fn iterate_once(bodies: &mut [RigidBody], joints: &[DistanceState]) {
    for s in joints {
        if s.spring.is_some() {
            continue;
        }
        let lambda = -separation_rate(bodies, s) * s.k_inv;
        apply_impulse(bodies, s, lambda);
    }
}

/// Rigid mode: Baumgarte-style positional correction of the length error
/// `C = L − L0`, mirroring `joint::correct_positions` — correct position,
/// not velocity, so no spurious energy enters the swing. `L`, `n` and `C`
/// are recomputed from current positions (contact and revolute correction
/// run first and may have moved these bodies), keeping the step's
/// `r_a`/`r_b` as the revolute joint does.
pub(crate) fn correct_positions(bodies: &mut [RigidBody], joints: &[DistanceState]) {
    for s in joints {
        if s.spring.is_some() {
            continue;
        }
        let (inv_a, inv_b) = (bodies[s.a].inv_mass, bodies[s.b].inv_mass);
        let inv_sum = inv_a + inv_b;
        if inv_sum == 0.0 {
            continue;
        }

        let d = (bodies[s.b].position + s.r_b) - (bodies[s.a].position + s.r_a);
        let l = d.length();
        if l < DISTANCE_EPSILON {
            continue;
        }
        let n = d / l;
        let c = l - s.length;
        if c.abs() <= JOINT_SLOP {
            continue;
        }

        let correction = JOINT_CORRECTION_PERCENT * (c.abs() - JOINT_SLOP) * c.signum() / inv_sum;
        let shift = n * correction;
        if !bodies[s.a].is_static {
            bodies[s.a].position += shift * inv_a;
        }
        if !bodies[s.b].is_static {
            bodies[s.b].position -= shift * inv_b;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Shape;

    fn dynamic(pos: Vec2, mass: f32) -> RigidBody {
        RigidBody::new_dynamic(pos, mass, Shape::circle(0.1))
    }

    fn static_body(pos: Vec2) -> RigidBody {
        RigidBody::new_static(pos, Shape::circle(0.1))
    }

    fn rigid(length: f32) -> DistanceJoint {
        DistanceJoint::new(BodyId(0), BodyId(1), Vec2::ZERO, Vec2::ZERO).with_length(length)
    }

    fn spring(length: f32, k: f32, c: f32) -> DistanceJoint {
        rigid(length).with_spring(k, c)
    }

    // --- build_distance_joints ---------------------------------------------

    #[test]
    fn joint_between_two_static_bodies_is_dropped() {
        let bodies = vec![static_body(Vec2::ZERO), static_body(Vec2::new(1.0, 0.0))];
        assert!(build_distance_joints(&bodies, &[rigid(1.0)]).is_empty());
    }

    #[test]
    fn coincident_anchors_are_dropped() {
        let bodies = vec![static_body(Vec2::ZERO), dynamic(Vec2::ZERO, 1.0)];
        assert!(build_distance_joints(&bodies, &[rigid(0.0)]).is_empty());
        assert!(build_distance_joints(&bodies, &[spring(0.0, 10.0, 1.0)]).is_empty());
    }

    #[test]
    fn n_points_from_a_to_b_and_c_has_the_sign_of_the_length_error() {
        let bodies = vec![dynamic(Vec2::ZERO, 1.0), dynamic(Vec2::new(3.0, 4.0), 1.0)];

        let stretched = build_distance_joints(&bodies, &[rigid(4.0)]);
        assert!((stretched[0].n - Vec2::new(0.6, 0.8)).length() < 1e-6);
        assert!((stretched[0].c - 1.0).abs() < 1e-6);

        let compressed = build_distance_joints(&bodies, &[rigid(6.0)]);
        assert!((compressed[0].c + 1.0).abs() < 1e-6);
    }

    #[test]
    fn k_inv_is_reduced_mass_when_anchors_are_on_the_center_line() {
        let bodies = vec![dynamic(Vec2::ZERO, 1.0), dynamic(Vec2::new(2.0, 0.0), 3.0)];
        let states = build_distance_joints(&bodies, &[rigid(2.0)]);
        let expected = 1.0 / (1.0 + 1.0 / 3.0);
        assert!((states[0].k_inv - expected).abs() < 1e-6);
    }

    // --- apply_spring_forces (soft mode) -----------------------------------

    #[test]
    fn spring_force_acts_only_along_the_anchor_line() {
        let mut bodies = vec![dynamic(Vec2::ZERO, 1.0), dynamic(Vec2::new(2.0, 2.0), 2.0)];
        let states = build_distance_joints(&bodies, &[spring(1.0, 10.0, 0.5)]);
        let n = states[0].n;

        apply_spring_forces(&mut bodies, &states, 1.0 / 60.0);

        let (dv_a, dv_b) = (bodies[0].velocity, bodies[1].velocity);
        assert!(dv_a.length() > 0.0, "stretched spring applied nothing");
        assert!(dv_a.cross(n).abs() < 1e-6 && dv_b.cross(n).abs() < 1e-6);
        assert!(dv_a.dot(n) > 0.0, "a must be pulled toward b");
        let momentum = dv_a * 1.0 + dv_b * 2.0;
        assert!(momentum.length() < 1e-6, "momentum changed: {momentum:?}");
        assert_eq!(bodies[0].angular_velocity, 0.0);
        assert_eq!(bodies[1].angular_velocity, 0.0);
    }

    #[test]
    fn apply_spring_forces_skips_rigid_joints() {
        let mut bodies = vec![dynamic(Vec2::ZERO, 1.0), dynamic(Vec2::new(3.0, 0.0), 1.0)];
        let states = build_distance_joints(&bodies, &[rigid(1.0)]);
        let before = bodies.clone();
        apply_spring_forces(&mut bodies, &states, 1.0 / 60.0);
        assert_eq!(bodies, before);
    }

    #[test]
    fn apply_spring_forces_never_moves_a_static_body() {
        let mut bodies = vec![static_body(Vec2::ZERO), dynamic(Vec2::new(3.0, 0.0), 1.0)];
        let states = build_distance_joints(&bodies, &[spring(1.0, 10.0, 1.0)]);
        let before = bodies[0].clone();
        apply_spring_forces(&mut bodies, &states, 1.0 / 60.0);
        assert_eq!(bodies[0], before);
        assert!(bodies[1].velocity.x < 0.0, "dynamic body must be pulled toward the hook");
    }

    // --- iterate_once / correct_positions (rigid mode) ---------------------

    #[test]
    fn iterate_once_drives_separation_rate_to_zero() {
        let mut bodies = vec![dynamic(Vec2::ZERO, 1.0), dynamic(Vec2::new(2.0, 0.0), 2.0)];
        bodies[0].velocity = Vec2::new(-1.0, 0.5);
        bodies[1].velocity = Vec2::new(2.0, -0.25);
        let states = build_distance_joints(&bodies, &[rigid(2.0)]);

        for _ in 0..16 {
            iterate_once(&mut bodies, &states);
        }

        assert!(separation_rate(&bodies, &states[0]).abs() < 1e-4);
        assert_eq!(bodies[0].velocity.y, 0.5, "perpendicular velocity must be untouched");
        assert_eq!(bodies[1].velocity.y, -0.25);
    }

    #[test]
    fn iterate_once_skips_spring_joints() {
        let mut bodies = vec![dynamic(Vec2::ZERO, 1.0), dynamic(Vec2::new(2.0, 0.0), 1.0)];
        bodies[1].velocity = Vec2::new(3.0, 0.0);
        let states = build_distance_joints(&bodies, &[spring(2.0, 10.0, 0.0)]);
        let before = bodies.clone();
        iterate_once(&mut bodies, &states);
        assert_eq!(bodies, before);
    }

    #[test]
    fn correct_positions_moves_along_n_by_sign_of_c() {
        // Stretched: 3.0 apart, target 2.0 → pulled together.
        let mut bodies = vec![dynamic(Vec2::ZERO, 1.0), dynamic(Vec2::new(3.0, 0.0), 1.0)];
        let states = build_distance_joints(&bodies, &[rigid(2.0)]);
        correct_positions(&mut bodies, &states);
        let closed = 3.0 - (bodies[1].position.x - bodies[0].position.x);
        let max_step = JOINT_CORRECTION_PERCENT * (1.0 - JOINT_SLOP);
        assert!(closed > 0.0 && closed <= max_step + 1e-5, "closed {closed}");

        // Compressed: 1.0 apart, target 2.0 → pushed apart.
        let mut bodies = vec![dynamic(Vec2::ZERO, 1.0), dynamic(Vec2::new(1.0, 0.0), 1.0)];
        let states = build_distance_joints(&bodies, &[rigid(2.0)]);
        correct_positions(&mut bodies, &states);
        let opened = (bodies[1].position.x - bodies[0].position.x) - 1.0;
        assert!(opened > 0.0 && opened <= max_step + 1e-5, "opened {opened}");
    }

    #[test]
    fn correct_positions_uses_current_positions_not_step_start() {
        // Built stretched (3.0 vs target 2.0), then something else already
        // moved b to exactly the target: nothing left to correct.
        let mut bodies = vec![dynamic(Vec2::ZERO, 1.0), dynamic(Vec2::new(3.0, 0.0), 1.0)];
        let states = build_distance_joints(&bodies, &[rigid(2.0)]);
        bodies[1].position = Vec2::new(2.0, 0.0);
        let before = (bodies[0].position, bodies[1].position);
        correct_positions(&mut bodies, &states);
        assert_eq!((bodies[0].position, bodies[1].position), before);
    }

    #[test]
    fn correct_positions_leaves_error_within_slop_alone() {
        let mut bodies = vec![dynamic(Vec2::ZERO, 1.0), dynamic(Vec2::new(2.0 + JOINT_SLOP / 2.0, 0.0), 1.0)];
        let states = build_distance_joints(&bodies, &[rigid(2.0)]);
        let before = (bodies[0].position, bodies[1].position);
        correct_positions(&mut bodies, &states);
        assert_eq!((bodies[0].position, bodies[1].position), before);
    }

    #[test]
    fn critical_damping_is_two_root_k_m() {
        assert!((DistanceJoint::critical_damping(10.0, 1.0) - 2.0 * 10f32.sqrt()).abs() < 1e-6);
        assert!((DistanceJoint::critical_damping(4.0, 9.0) - 12.0).abs() < 1e-6);
    }
}
