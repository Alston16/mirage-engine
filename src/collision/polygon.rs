//! polygon–polygon narrowphase: SAT axis test, reference/incident face,
//! manifold clipping.

use crate::collision::manifold::Contact;
use crate::{RigidBody, Vec2};

/// Same slop used in `circle.rs` — guards the exact-touching boundary
/// against float-noise flicker (spec Edge Cases).
const SLOP: f32 = 1e-4;

/// The axis (a face normal, and how far the other polygon penetrates past
/// it) of least separation found while testing one polygon's faces against
/// the other.
struct AxisResult {
    separation: f32,
    face: usize,
}

/// Finds, among `from`'s faces, the axis of maximum separation from `to`
/// (the point where `to` penetrates `from` least, i.e. the best candidate
/// reference axis for `from`). Returns `None` if any axis is fully
/// separating (no overlap).
fn max_separation(
    from_vertices: &[Vec2],
    from_normals: &[Vec2],
    to_vertices: &[Vec2],
) -> Option<AxisResult> {
    let mut best_face = 0;
    let mut best_separation = f32::MIN;

    for i in 0..from_vertices.len() {
        let normal = from_normals[i];
        let v = from_vertices[i];
        // The point of `to` least favorable to this axis (i.e. deepest
        // behind the face) is the one with minimum support along `normal`.
        let min_support = to_vertices
            .iter()
            .map(|&tv| normal.dot(tv - v))
            .fold(f32::MAX, f32::min);

        if min_support > best_separation {
            best_separation = min_support;
            best_face = i;
        }
    }

    if best_separation > 0.0 {
        None
    } else {
        Some(AxisResult { separation: best_separation, face: best_face })
    }
}

/// Polygon–polygon SAT test with reference/incident face clipping, per
/// README § Narrowphase (Box2D-Lite-style construction, README References).
/// `None` if any face axis shows separation (the polygons aren't touching).
pub(crate) fn polygon_vs_polygon(a: &RigidBody, b: &RigidBody) -> Option<Vec<Contact>> {
    let (a_vertices, a_normals) = a.shape.polygon_world(a.position, a.orientation);
    let (b_vertices, b_normals) = b.shape.polygon_world(b.position, b.orientation);

    let axis_a = max_separation(&a_vertices, &a_normals, &b_vertices)?;
    let axis_b = max_separation(&b_vertices, &b_normals, &a_vertices)?;

    // Reference face is whichever body's best axis has the larger (least
    // negative) separation — the axis of minimum penetration overall.
    // A tiny bias toward `a` avoids flip-flopping the reference face when
    // both axes are (near-)equal, which would otherwise jitter the normal.
    let (ref_vertices, ref_normals, inc_vertices, flip, ref_face) =
        if axis_b.separation > axis_a.separation + SLOP {
            (&b_vertices, &b_normals, &a_vertices, true, axis_b.face)
        } else {
            (&a_vertices, &a_normals, &b_vertices, false, axis_a.face)
        };

    let ref_normal = ref_normals[ref_face];
    let ref_v1 = ref_vertices[ref_face];
    let ref_v2 = ref_vertices[(ref_face + 1) % ref_vertices.len()];

    // Incident face: the other body's face most anti-parallel to the
    // reference normal.
    let inc_normals = if flip { &a_normals } else { &b_normals };
    let incident_face = (0..inc_vertices.len())
        .min_by(|&i, &j| {
            inc_normals[i]
                .dot(ref_normal)
                .partial_cmp(&inc_normals[j].dot(ref_normal))
                .unwrap()
        })
        .unwrap();
    let mut inc_p1 = inc_vertices[incident_face];
    let mut inc_p2 = inc_vertices[(incident_face + 1) % inc_vertices.len()];

    // Clip the incident edge against the reference face's two side planes
    // (Sutherland–Hodgman-style, one edge against two half-planes).
    let tangent = (ref_v2 - ref_v1).normalize();
    if !clip_segment(&mut inc_p1, &mut inc_p2, -tangent, -tangent.dot(ref_v1)) {
        return None;
    }
    if !clip_segment(&mut inc_p1, &mut inc_p2, tangent, tangent.dot(ref_v2)) {
        return None;
    }

    // Keep only clipped points still behind the reference face.
    let mut points = Vec::with_capacity(2);
    for p in [inc_p1, inc_p2] {
        let separation = ref_normal.dot(p - ref_v1);
        if separation <= SLOP {
            let penetration = -separation;
            if penetration > SLOP {
                points.push(Contact {
                    point: p,
                    normal: if flip { -ref_normal } else { ref_normal },
                    penetration,
                });
            }
        }
    }

    if points.is_empty() {
        None
    } else {
        Some(points)
    }
}

/// Clips the segment `p1`-`p2` against the half-plane `dot(v, normal) <=
/// offset`, moving whichever endpoint is outside it to the boundary.
/// Returns `false` if the whole segment is outside (degenerate clip).
fn clip_segment(p1: &mut Vec2, p2: &mut Vec2, normal: Vec2, offset: f32) -> bool {
    let d1 = normal.dot(*p1) - offset;
    let d2 = normal.dot(*p2) - offset;

    if d1 <= 0.0 && d2 <= 0.0 {
        return true;
    }
    if d1 > 0.0 && d2 > 0.0 {
        return false;
    }

    let t = d1 / (d1 - d2);
    let clipped = *p1 + (*p2 - *p1) * t;
    if d1 > 0.0 {
        *p1 = clipped;
    } else {
        *p2 = clipped;
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Rot2, Shape};

    fn box_body(position: Vec2, orientation: Rot2) -> RigidBody {
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

    #[test]
    fn overlapping_axis_aligned_boxes_produce_two_point_manifold() {
        let a = box_body(Vec2::new(0.0, 0.0), Rot2::IDENTITY);
        let b = box_body(Vec2::new(1.5, 0.0), Rot2::IDENTITY);

        let points = polygon_vs_polygon(&a, &b).expect("expected a manifold");

        assert_eq!(points.len(), 2);
        for c in &points {
            assert!((c.normal - Vec2::new(1.0, 0.0)).length() < 1e-3);
            assert!((c.penetration - 0.5).abs() < 1e-3);
        }
    }

    #[test]
    fn separated_boxes_produce_no_manifold() {
        let a = box_body(Vec2::new(0.0, 0.0), Rot2::IDENTITY);
        let b = box_body(Vec2::new(10.0, 0.0), Rot2::IDENTITY);

        assert!(polygon_vs_polygon(&a, &b).is_none());
    }

    #[test]
    fn rotated_box_on_axis_aligned_box_produces_nondegenerate_manifold() {
        let ramp = box_body(Vec2::new(0.0, 0.0), Rot2::new(0.2));
        let falling = box_body(Vec2::new(0.0, 1.3), Rot2::IDENTITY);

        let points = polygon_vs_polygon(&ramp, &falling).expect("expected a manifold");

        assert!(!points.is_empty());
        assert!(points.len() <= 2);
        for c in &points {
            assert!(c.penetration > 0.0);
            assert!((c.normal.length() - 1.0).abs() < 1e-3);
        }
    }

    #[test]
    fn deeply_overlapping_boxes_produce_nondegenerate_manifold() {
        let a = box_body(Vec2::new(0.0, 0.0), Rot2::IDENTITY);
        let b = box_body(Vec2::new(0.2, 0.0), Rot2::IDENTITY);

        let points = polygon_vs_polygon(&a, &b).expect("expected a manifold");

        assert!(!points.is_empty());
    }
}
