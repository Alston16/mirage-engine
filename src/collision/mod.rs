//! Narrowphase dispatch — routes each broadphase candidate pair to the
//! circle/polygon test matching its shape pair, per README § Narrowphase.

pub mod circle;
pub mod manifold;
pub mod polygon;

use crate::collision::manifold::Manifold;
use crate::{broadphase, BodyId, RigidBody, Shape};

/// Runs broadphase then narrowphase over the given bodies, returning a
/// `Manifold` for every pair currently touching. Pure function — no
/// mutation of `bodies`.
///
/// Normal-direction convention: for circle–circle, `Contact.normal` points
/// from the pair's first body toward its second (FR-002). For any pair
/// involving a polygon, `Contact.normal` instead always points away from
/// the polygon's touching face/vertex, toward the other shape — the
/// physically meaningful "which way to separate" direction — regardless of
/// which body happens to be first/second in `bodies`. `circle_vs_polygon`
/// already returns a normal in that "away from polygon" direction no
/// matter which argument position the polygon is passed in, so no flip is
/// needed when the pair order is polygon-then-circle.
pub fn detect_contacts(bodies: &[RigidBody]) -> Vec<Manifold> {
    let mut manifolds = Vec::new();

    for (i, j) in broadphase::candidate_pairs(bodies) {
        let a = &bodies[i];
        let b = &bodies[j];

        let points = match (&a.shape, &b.shape) {
            (Shape::Circle { .. }, Shape::Circle { .. }) => {
                circle::circle_vs_circle(a, b).map(|c| vec![c])
            }
            (Shape::Circle { .. }, Shape::Polygon { .. }) => {
                circle::circle_vs_polygon(a, b).map(|c| vec![c])
            }
            (Shape::Polygon { .. }, Shape::Circle { .. }) => {
                circle::circle_vs_polygon(b, a).map(|c| vec![c])
            }
            (Shape::Polygon { .. }, Shape::Polygon { .. }) => polygon::polygon_vs_polygon(a, b),
        };

        if let Some(points) = points {
            if !points.is_empty() {
                manifolds.push(Manifold {
                    body_a: BodyId(i as u32),
                    body_b: BodyId(j as u32),
                    points,
                });
            }
        }
    }

    manifolds
}
