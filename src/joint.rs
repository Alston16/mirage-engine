//! Revolute (hinge) joint — a 2-body point constraint pinning a local
//! anchor on one body to a local anchor on another. Solved as a coupled
//! sequential impulse in the same per-step velocity-iteration loop that
//! resolves contacts (see `World::step`), the same way a contact is solved
//! as an inequality constraint — see README § Joints for the derivation.

use crate::math::Mat2;
use crate::{BodyId, RigidBody, Vec2};

/// A 2-body point constraint pinning `anchor_a` (local to `body_a`) to
/// `anchor_b` (local to `body_b`). Bodies remain free to rotate relative to
/// each other; only their anchor points are held coincident.
#[derive(Clone, Debug, PartialEq)]
pub struct RevoluteJoint {
    pub body_a: BodyId,
    pub body_b: BodyId,
    /// Anchor on `body_a`, in `body_a`'s local frame.
    pub anchor_a: Vec2,
    /// Anchor on `body_b`, in `body_b`'s local frame.
    pub anchor_b: Vec2,
}

impl RevoluteJoint {
    /// Builds a joint pinning `anchor_a` (local to `body_a`) to `anchor_b`
    /// (local to `body_b`). The two world-space anchor points do not need
    /// to coincide yet — the solver closes any initial gap gradually (see
    /// `correct_positions`).
    pub fn new(body_a: BodyId, body_b: BodyId, anchor_a: Vec2, anchor_b: Vec2) -> Self {
        RevoluteJoint { body_a, body_b, anchor_a, anchor_b }
    }
}

/// Opaque handle identifying a `RevoluteJoint` within the `World` that
/// created it. Only ever constructed by `World::add_joint`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct JointId(pub(crate) u32);

/// Anchor-gap tolerated before positional correction kicks in, mirroring
/// `solver::PENETRATION_SLOP`'s role for contacts — kept as its own
/// constant rather than reusing the contact one, because a joint's gap
/// isn't a compressive penetration between two solids and deserves its own
/// tuning (see specs/006-revolute-hinge-joint/research.md § 4).
pub(crate) const JOINT_SLOP: f32 = 0.002;

/// Fraction of the excess anchor gap removed per step. Starting value
/// matches `solver::CORRECTION_PERCENT`; both were tuned against the
/// pendulum demo (specs/006-revolute-hinge-joint/research.md § 4) and
/// found to need no change from the contact solver's value.
pub(crate) const JOINT_CORRECTION_PERCENT: f32 = 0.4;

/// Per-joint solver working state, built once per fixed step and
/// discarded — the joint-side analog of `solver::ContactState`.
pub(crate) struct JointState {
    a: usize,
    b: usize,
    /// World-space offset from each body's center to its anchor point.
    /// Fixed for the whole step: orientation only integrates after
    /// resolution (in `World::step`), so it's valid through every velocity
    /// iteration and the position correction that follows them.
    r_a: Vec2,
    r_b: Vec2,
    /// Coupled effective-mass matrix for the 2D point constraint, already
    /// inverted so each iteration is one matrix-vector multiply:
    ///
    /// ```text
    /// K = (invMass_a + invMass_b) * I2
    ///   + invI_a * [[r_a.y², -r_a.x*r_a.y], [-r_a.x*r_a.y,  r_a.x²]]
    ///   + invI_b * [[r_b.y², -r_b.x*r_b.y], [-r_b.x*r_b.y,  r_b.x²]]
    /// ```
    k_inv: Mat2,
}

/// Builds solver state for every joint. A joint between two bodies that
/// can't move at all (both static, so `invMass_a + invMass_b == 0`, which
/// forces `K` to the zero matrix since static bodies also carry
/// `invInertia == 0`) is dropped, mirroring `solver::build_contacts`'s
/// `k == 0.0` guard.
pub(crate) fn build_joints(bodies: &[RigidBody], joints: &[RevoluteJoint]) -> Vec<JointState> {
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

        let k11 =
            inv_sum + body_a.inv_inertia * r_a.y * r_a.y + body_b.inv_inertia * r_b.y * r_b.y;
        let k12 =
            -body_a.inv_inertia * r_a.x * r_a.y - body_b.inv_inertia * r_b.x * r_b.y;
        let k22 =
            inv_sum + body_a.inv_inertia * r_a.x * r_a.x + body_b.inv_inertia * r_b.x * r_b.x;
        let k = Mat2::new(k11, k12, k12, k22);

        states.push(JointState { a, b, r_a, r_b, k_inv: k.invert() });
    }
    states
}

/// `Cdot = (v_b + ω_b × r_b) − (v_a + ω_a × r_a)` at the joint's anchor
/// pair — the joint-side analog of `solver`'s `vr`.
fn relative_velocity(bodies: &[RigidBody], j: &JointState) -> Vec2 {
    let (v_a, w_a) = (bodies[j.a].velocity, bodies[j.a].angular_velocity);
    let (v_b, w_b) = (bodies[j.b].velocity, bodies[j.b].angular_velocity);
    (v_b + Vec2::cross_sv(w_b, j.r_b)) - (v_a + Vec2::cross_sv(w_a, j.r_a))
}

/// Applies impulse `p` at the joint: `−p` to `body_a`, `+p` to `body_b`,
/// leaving static bodies untouched — the joint-side analog of
/// `solver::apply_impulse`.
fn apply_impulse(bodies: &mut [RigidBody], j: &JointState, p: Vec2) {
    let body_a = &mut bodies[j.a];
    if !body_a.is_static {
        body_a.velocity -= p * body_a.inv_mass;
        body_a.angular_velocity -= j.r_a.cross(p) * body_a.inv_inertia;
    }
    let body_b = &mut bodies[j.b];
    if !body_b.is_static {
        body_b.velocity += p * body_b.inv_mass;
        body_b.angular_velocity += j.r_b.cross(p) * body_b.inv_inertia;
    }
}

/// One Gauss-Seidel pass over every joint: solves the coupled impulse
/// `P = K⁻¹ · (−Cdot)` and applies it, driving the anchor pair's relative
/// velocity toward zero. Run once per velocity iteration in `World::step`,
/// interleaved with `solver::iterate_once` so a body that is both jointed
/// and touching a contact converges under both constraints together.
pub(crate) fn iterate_once(bodies: &mut [RigidBody], joints: &mut [JointState]) {
    for j in joints {
        let cdot = relative_velocity(bodies, j);
        let p = j.k_inv * -cdot;
        apply_impulse(bodies, j, p);
    }
}

/// Baumgarte-style positional correction on the anchor gap
/// `C = p_b - p_a`, mirroring `solver::correct_positions`'s "correct
/// position, not velocity" approach: a joint has no restitution to keep
/// energy-clean, but biasing velocity would still inject spurious energy
/// into the joint's swing for no benefit. Moves each body toward closing
/// the gap by `JOINT_CORRECTION_PERCENT` of the excess (over `JOINT_SLOP`),
/// split by inverse mass so a heavier body moves less.
pub(crate) fn correct_positions(bodies: &mut [RigidBody], joints: &[JointState]) {
    for j in joints {
        let (inv_a, inv_b) = (bodies[j.a].inv_mass, bodies[j.b].inv_mass);
        let inv_sum = inv_a + inv_b;
        if inv_sum == 0.0 {
            continue;
        }

        let world_a = bodies[j.a].position + j.r_a;
        let world_b = bodies[j.b].position + j.r_b;
        let gap = world_b - world_a;
        let distance = gap.length();
        if distance <= JOINT_SLOP {
            continue;
        }

        let correction = JOINT_CORRECTION_PERCENT * (distance - JOINT_SLOP) / inv_sum;
        let shift = gap.normalize() * correction;
        if !bodies[j.a].is_static {
            bodies[j.a].position += shift * inv_a;
        }
        if !bodies[j.b].is_static {
            bodies[j.b].position -= shift * inv_b;
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

    #[test]
    fn degenerate_both_static_joint_is_dropped() {
        let bodies = vec![static_body(Vec2::ZERO), static_body(Vec2::new(1.0, 0.0))];
        let joint = RevoluteJoint::new(BodyId(0), BodyId(1), Vec2::ZERO, Vec2::ZERO);
        let states = build_joints(&bodies, &[joint]);
        assert!(states.is_empty(), "a joint between two static bodies must be dropped, not solved");
    }

    #[test]
    fn dynamic_static_joint_is_kept() {
        let bodies = vec![static_body(Vec2::ZERO), dynamic(Vec2::new(1.0, 0.0), 1.0)];
        let joint = RevoluteJoint::new(BodyId(0), BodyId(1), Vec2::ZERO, Vec2::ZERO);
        let states = build_joints(&bodies, &[joint]);
        assert_eq!(states.len(), 1);
    }

    #[test]
    fn iterate_once_never_moves_a_static_body() {
        let mut bodies = vec![static_body(Vec2::ZERO), dynamic(Vec2::new(1.0, 0.0), 1.0)];
        bodies[1].velocity = Vec2::new(0.0, -5.0);
        let joint = RevoluteJoint::new(BodyId(0), BodyId(1), Vec2::ZERO, Vec2::ZERO);
        let mut states = build_joints(&bodies, &[joint]);
        let before = bodies[0].clone();

        for _ in 0..16 {
            iterate_once(&mut bodies, &mut states);
        }

        assert_eq!(bodies[0], before);
    }

    #[test]
    fn iterate_once_drives_anchor_relative_velocity_to_zero() {
        // Anchors coincide (both at each body's own center): after enough
        // iterations, the two bodies' velocities at the anchor must match.
        let mut bodies = vec![dynamic(Vec2::ZERO, 1.0), dynamic(Vec2::new(0.0, 0.0), 2.0)];
        bodies[0].velocity = Vec2::new(3.0, 0.0);
        bodies[1].velocity = Vec2::new(-1.0, 0.0);
        let joint = RevoluteJoint::new(BodyId(0), BodyId(1), Vec2::ZERO, Vec2::ZERO);
        let mut states = build_joints(&bodies, &[joint]);

        for _ in 0..16 {
            iterate_once(&mut bodies, &mut states);
        }

        assert!(
            (bodies[0].velocity.x - bodies[1].velocity.x).abs() < 1e-3,
            "a = {}, b = {}",
            bodies[0].velocity.x,
            bodies[1].velocity.x
        );
    }

    #[test]
    fn correct_positions_pulls_anchors_together_without_popping() {
        // A joint between two bodies 1.0 apart, both pinned at their own
        // centers, so the gap equals the center distance.
        let mut bodies = vec![dynamic(Vec2::ZERO, 1.0), dynamic(Vec2::new(1.0, 0.0), 1.0)];
        let joint = RevoluteJoint::new(BodyId(0), BodyId(1), Vec2::ZERO, Vec2::ZERO);
        let states = build_joints(&bodies, &[joint]);

        let before_gap = (bodies[1].position - bodies[0].position).length();
        correct_positions(&mut bodies, &states);
        let after_gap = (bodies[1].position - bodies[0].position).length();

        assert!(after_gap < before_gap, "gap did not shrink: {before_gap} -> {after_gap}");
        let max_step = JOINT_CORRECTION_PERCENT * (before_gap - JOINT_SLOP);
        assert!(before_gap - after_gap <= max_step + 1e-5, "popped: closed {}", before_gap - after_gap);
    }

    #[test]
    fn correct_positions_leaves_gap_within_slop_alone() {
        let mut bodies = vec![
            dynamic(Vec2::ZERO, 1.0),
            dynamic(Vec2::new(JOINT_SLOP / 2.0, 0.0), 1.0),
        ];
        let joint = RevoluteJoint::new(BodyId(0), BodyId(1), Vec2::ZERO, Vec2::ZERO);
        let states = build_joints(&bodies, &[joint]);
        let before = (bodies[0].position, bodies[1].position);

        correct_positions(&mut bodies, &states);

        assert_eq!((bodies[0].position, bodies[1].position), before);
    }
}
