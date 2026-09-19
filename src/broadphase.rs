//! Broadphase — O(n²) candidate-pair generation with AABB overlap rejection.
//!
//! Intentionally naive per README § Broadphase: no grid, no BVH, no
//! sweep-and-prune. A spatial index is a deliberate post-MVP concern, not
//! an oversight (Constitution Principle V).

use crate::RigidBody;

/// All body-index pairs `(i, j)` with `i < j` whose current world-space
/// AABBs overlap. Pure function — does not mutate `bodies`.
pub fn candidate_pairs(bodies: &[RigidBody]) -> Vec<(usize, usize)> {
    let mut pairs = Vec::new();
    for i in 0..bodies.len() {
        let aabb_i = bodies[i].shape.aabb(bodies[i].position, bodies[i].orientation);
        for j in (i + 1)..bodies.len() {
            let aabb_j = bodies[j].shape.aabb(bodies[j].position, bodies[j].orientation);
            if aabb_i.overlaps(&aabb_j) {
                pairs.push((i, j));
            }
        }
    }
    pairs
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{RigidBody, Shape, Vec2};

    #[test]
    fn only_overlapping_aabb_pairs_are_returned() {
        let bodies = vec![
            // Two close circles: AABBs overlap.
            RigidBody::new_static(Vec2::new(0.0, 0.0), Shape::circle(1.0)),
            RigidBody::new_static(Vec2::new(1.5, 0.0), Shape::circle(1.0)),
            // A far-away circle: AABB does not overlap either of the above.
            RigidBody::new_static(Vec2::new(100.0, 100.0), Shape::circle(1.0)),
        ];

        let pairs = candidate_pairs(&bodies);

        assert_eq!(pairs, vec![(0, 1)]);
    }

    #[test]
    fn zero_or_one_body_returns_empty_without_panicking() {
        assert_eq!(candidate_pairs(&[]), Vec::<(usize, usize)>::new());

        let one = vec![RigidBody::new_static(Vec2::ZERO, Shape::circle(1.0))];
        assert_eq!(candidate_pairs(&one), Vec::<(usize, usize)>::new());
    }
}
