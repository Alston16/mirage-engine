//! Narrowphase dispatch — routes each broadphase candidate pair to the
//! circle/polygon test matching its shape pair, per README § Narrowphase.

pub mod circle;
pub mod manifold;
pub mod polygon;

use crate::collision::manifold::{Contact, Manifold};
use crate::{broadphase, BodyId, RigidBody, Shape};

/// Runs broadphase then narrowphase over the given bodies, returning a
/// `Manifold` for every pair currently touching. Pure function — no
/// mutation of `bodies`.
///
/// Normal-direction convention: for every shape pair, in either order,
/// `Contact.normal` points from the pair's first body (`body_a`) toward its
/// second (`body_b`) — the direction the solver pushes `body_b` along.
/// `circle_vs_polygon` itself returns a normal pointing away from the
/// polygon toward the circle, so it is negated when the circle is the
/// first body.
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
                // `circle_vs_polygon`'s normal points polygon -> circle,
                // i.e. b -> a here; flip it to a -> b.
                circle::circle_vs_polygon(a, b).map(|c| {
                    vec![Contact {
                        normal: -c.normal,
                        ..c
                    }]
                })
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Rot2, Vec2};

    fn circle(position: Vec2) -> RigidBody {
        RigidBody::new_static(position, Shape::circle(1.0))
    }

    fn square(position: Vec2, orientation: Rot2) -> RigidBody {
        let mut body = RigidBody::new_static(
            position,
            Shape::polygon(vec![
                Vec2::new(-1.0, -1.0),
                Vec2::new(1.0, -1.0),
                Vec2::new(1.0, 1.0),
                Vec2::new(-1.0, 1.0),
            ]),
        );
        body.orientation = orientation;
        body
    }

    /// Asserts every contact normal of the (only) manifold points from the
    /// first body toward the second.
    fn assert_normals_point_a_to_b(bodies: &[RigidBody]) {
        let manifolds = detect_contacts(bodies);
        assert_eq!(manifolds.len(), 1, "expected exactly one touching pair");
        let m = &manifolds[0];
        assert_eq!((m.body_a, m.body_b), (BodyId(0), BodyId(1)));
        let a_to_b = bodies[1].position - bodies[0].position;
        for c in &m.points {
            assert!(
                c.normal.dot(a_to_b) > 0.0,
                "normal {:?} does not point from a to b (a->b = {a_to_b:?})",
                c.normal
            );
            assert!((c.normal.length() - 1.0).abs() < 1e-4);
        }
    }

    #[test]
    fn circle_circle_normal_points_a_to_b() {
        assert_normals_point_a_to_b(&[circle(Vec2::new(0.0, 0.0)), circle(Vec2::new(1.5, 0.0))]);
    }

    #[test]
    fn circle_then_polygon_normal_points_a_to_b() {
        // Circle above the square's top face.
        assert_normals_point_a_to_b(&[
            circle(Vec2::new(0.0, 1.8)),
            square(Vec2::new(0.0, 0.0), Rot2::IDENTITY),
        ]);
    }

    #[test]
    fn polygon_then_circle_normal_points_a_to_b() {
        assert_normals_point_a_to_b(&[
            square(Vec2::new(0.0, 0.0), Rot2::IDENTITY),
            circle(Vec2::new(0.0, 1.8)),
        ]);
    }

    #[test]
    fn polygon_polygon_normal_points_a_to_b_in_both_orders() {
        let rotated = Rot2::new(0.3);
        assert_normals_point_a_to_b(&[
            square(Vec2::new(0.0, 0.0), Rot2::IDENTITY),
            square(Vec2::new(0.2, 1.8), rotated),
        ]);
        assert_normals_point_a_to_b(&[
            square(Vec2::new(0.2, 1.8), rotated),
            square(Vec2::new(0.0, 0.0), Rot2::IDENTITY),
        ]);
    }
}
