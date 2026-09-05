//! circle–circle and circle–polygon narrowphase.

use crate::collision::manifold::Contact;
use crate::{RigidBody, Shape};

/// Minimum penetration before a pair counts as touching. Guards the
/// exact-touching boundary (`dist == r_a + r_b`) against flickering
/// contact/no-contact from floating-point noise (spec Edge Cases).
const SLOP: f32 = 1e-4;

/// Circle–circle overlap test. `None` if the circles aren't touching.
pub(crate) fn circle_vs_circle(a: &RigidBody, b: &RigidBody) -> Option<Contact> {
    let (Shape::Circle { radius: ra }, Shape::Circle { radius: rb }) = (&a.shape, &b.shape) else {
        unreachable!("circle_vs_circle called with a non-circle shape");
    };

    let delta = b.position - a.position;
    let dist = delta.length();
    let penetration = ra + rb - dist;

    if penetration <= SLOP {
        return None;
    }

    let normal = delta.normalize();
    Some(Contact {
        point: a.position + normal * *ra,
        normal,
        penetration,
    })
}

/// Circle–polygon overlap test (closest point on the polygon boundary to
/// the circle's center). `None` if they aren't touching. Follows the
/// classic Box2D-Lite `b2CollideCircle` construction (README References):
/// find the face of maximum separation, then resolve against that face's
/// two vertex regions or its interior depending on where the center falls.
pub(crate) fn circle_vs_polygon(circle: &RigidBody, polygon: &RigidBody) -> Option<Contact> {
    let Shape::Circle { radius } = &circle.shape else {
        unreachable!("circle_vs_polygon called with a non-circle first argument");
    };
    let (vertices, normals) = polygon.shape.polygon_world(polygon.position, polygon.orientation);
    let center = circle.position;

    // Face of maximum separation: the axis along which the circle center
    // sits farthest outside the polygon (or least far inside, if fully
    // contained).
    let mut best_face = 0;
    let mut best_separation = f32::MIN;
    for i in 0..vertices.len() {
        let separation = normals[i].dot(center - vertices[i]);
        if separation > *radius {
            // Definitely separated along this axis alone.
            return None;
        }
        if separation > best_separation {
            best_separation = separation;
            best_face = i;
        }
    }

    let v1 = vertices[best_face];
    let v2 = vertices[(best_face + 1) % vertices.len()];
    let normal_dir = normals[best_face];

    if best_separation < SLOP {
        // Center is inside the polygon (or right at its boundary): push
        // out along the face normal.
        let penetration = radius - best_separation;
        if penetration <= SLOP {
            return None;
        }
        return Some(Contact {
            point: center - normal_dir * best_separation,
            normal: normal_dir,
            penetration,
        });
    }

    // Center is outside the polygon along this face — determine whether
    // it's nearest a vertex or the face's interior.
    let u1 = (center - v1).dot(v2 - v1);
    let u2 = (center - v2).dot(v1 - v2);

    if u1 <= 0.0 {
        vertex_contact(center, v1, *radius)
    } else if u2 <= 0.0 {
        vertex_contact(center, v2, *radius)
    } else {
        let penetration = radius - best_separation;
        if penetration <= SLOP {
            return None;
        }
        Some(Contact {
            point: center - normal_dir * best_separation,
            normal: normal_dir,
            penetration,
        })
    }
}

fn vertex_contact(center: crate::Vec2, vertex: crate::Vec2, radius: f32) -> Option<Contact> {
    let delta = center - vertex;
    let dist = delta.length();
    let penetration = radius - dist;
    if penetration <= SLOP {
        return None;
    }
    Some(Contact {
        point: vertex,
        normal: delta.normalize(),
        penetration,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Vec2;

    fn circle(position: Vec2, radius: f32) -> RigidBody {
        RigidBody::new_static(position, Shape::circle(radius))
    }

    /// A CCW-wound unit square (half-extent 1) centered on `position`.
    fn box_body(position: Vec2) -> RigidBody {
        RigidBody::new_static(
            position,
            Shape::polygon(vec![
                Vec2::new(-1.0, -1.0),
                Vec2::new(1.0, -1.0),
                Vec2::new(1.0, 1.0),
                Vec2::new(-1.0, 1.0),
            ]),
        )
    }

    #[test]
    fn overlapping_circles_produce_correct_normal_and_penetration() {
        let a = circle(Vec2::new(0.0, 0.0), 1.0);
        let b = circle(Vec2::new(1.5, 0.0), 1.0);

        let contact = circle_vs_circle(&a, &b).expect("expected a contact");

        assert!((contact.normal - Vec2::new(1.0, 0.0)).length() < 1e-4);
        assert!((contact.penetration - 0.5).abs() < 1e-4);
    }

    #[test]
    fn separated_circles_produce_no_contact() {
        let a = circle(Vec2::new(0.0, 0.0), 1.0);
        let b = circle(Vec2::new(5.0, 0.0), 1.0);

        assert!(circle_vs_circle(&a, &b).is_none());
    }

    #[test]
    fn exactly_touching_circles_produce_no_flickering_contact() {
        let a = circle(Vec2::new(0.0, 0.0), 1.0);
        let b = circle(Vec2::new(2.0, 0.0), 1.0);

        // Distance == r_a + r_b exactly; recomputing repeatedly must
        // consistently report no contact, never toggling.
        for _ in 0..5 {
            assert!(circle_vs_circle(&a, &b).is_none());
        }
    }

    #[test]
    fn circle_overlapping_polygon_face_has_normal_away_from_face() {
        let poly = box_body(Vec2::ZERO);
        let c = circle(Vec2::new(0.0, 1.5), 1.0);

        let contact = circle_vs_polygon(&c, &poly).expect("expected a contact");

        assert!((contact.normal - Vec2::new(0.0, 1.0)).length() < 1e-4);
        assert!((contact.penetration - 0.5).abs() < 1e-4);
        assert!((contact.point - Vec2::new(0.0, 1.0)).length() < 1e-4);
    }

    #[test]
    fn circle_nearest_polygon_corner_has_normal_away_from_corner() {
        let poly = box_body(Vec2::ZERO);
        let c = circle(Vec2::new(2.0, 2.0), 1.5);

        let contact = circle_vs_polygon(&c, &poly).expect("expected a contact");

        let away_from_corner = (Vec2::new(2.0, 2.0) - Vec2::new(1.0, 1.0)).normalize();
        assert!((contact.normal - away_from_corner).length() < 1e-4);
        assert!((contact.point - Vec2::new(1.0, 1.0)).length() < 1e-4);
    }

    #[test]
    fn circle_separated_from_polygon_produces_no_contact() {
        let poly = box_body(Vec2::ZERO);
        let c = circle(Vec2::new(0.0, 10.0), 1.0);

        assert!(circle_vs_polygon(&c, &poly).is_none());
    }
}
