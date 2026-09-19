//! Shape — the geometric form attached to a body: Circle or Polygon, plus
//! the AABB broadphase queries.

use crate::{Rot2, Vec2};

/// An axis-aligned bounding box, used only to cheaply reject non-overlapping
/// body pairs before narrowphase runs.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Aabb {
    pub min: Vec2,
    pub max: Vec2,
}

impl Aabb {
    /// True if this box and `other` overlap on both axes.
    pub fn overlaps(&self, other: &Aabb) -> bool {
        self.min.x <= other.max.x
            && self.max.x >= other.min.x
            && self.min.y <= other.max.y
            && self.max.y >= other.min.y
    }
}

/// The geometry attached to a `RigidBody`, in the body's local space
/// (relative to its `position`, before `orientation` is applied).
#[derive(Clone, Debug, PartialEq)]
pub enum Shape {
    Circle {
        radius: f32,
    },
    Polygon {
        /// Wound counter-clockwise, local space.
        vertices: Vec<Vec2>,
        /// Outward-facing unit normal per edge; `normals[i]` is the normal
        /// of the edge from `vertices[i]` to `vertices[(i + 1) % n]`.
        normals: Vec<Vec2>,
    },
}

impl Shape {
    pub fn circle(radius: f32) -> Self {
        Shape::Circle { radius }
    }

    /// Builds a convex polygon from counter-clockwise-wound local-space
    /// vertices, deriving each edge's outward normal.
    pub fn polygon(vertices: Vec<Vec2>) -> Self {
        let normals = (0..vertices.len())
            .map(|i| {
                let a = vertices[i];
                let b = vertices[(i + 1) % vertices.len()];
                let edge = b - a;
                // Outward normal for a CCW-wound edge is the edge vector
                // rotated -90° (clockwise), i.e. `(edge.y, -edge.x)` — the
                // negation of `Vec2::perp`'s +90° (CCW) rotation.
                Vec2::new(edge.y, -edge.x).normalize()
            })
            .collect();
        Shape::Polygon { vertices, normals }
    }

    /// World-space vertices and outward-facing normals for a `Polygon`
    /// shape at the given transform. Narrowphase-internal helper — panics
    /// if called on a `Circle`.
    pub(crate) fn polygon_world(&self, position: Vec2, orientation: Rot2) -> (Vec<Vec2>, Vec<Vec2>) {
        match self {
            Shape::Polygon { vertices, normals } => {
                let world_vertices = vertices.iter().map(|&v| position + orientation.rotate(v)).collect();
                let world_normals = normals.iter().map(|&n| orientation.rotate(n)).collect();
                (world_vertices, world_normals)
            }
            Shape::Circle { .. } => unreachable!("polygon_world called on a Circle shape"),
        }
    }

    /// World-space AABB for this shape at the given transform.
    pub fn aabb(&self, position: Vec2, orientation: Rot2) -> Aabb {
        match self {
            Shape::Circle { radius } => Aabb {
                min: position - Vec2::new(*radius, *radius),
                max: position + Vec2::new(*radius, *radius),
            },
            Shape::Polygon { vertices, .. } => {
                let mut min = position + orientation.rotate(vertices[0]);
                let mut max = min;
                for &v in &vertices[1..] {
                    let world_v = position + orientation.rotate(v);
                    min = Vec2::new(min.x.min(world_v.x), min.y.min(world_v.y));
                    max = Vec2::new(max.x.max(world_v.x), max.y.max(world_v.y));
                }
                Aabb { min, max }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f32 = 1e-5;

    fn approx_eq_vec(a: Vec2, b: Vec2) {
        assert!((a.x - b.x).abs() < EPS && (a.y - b.y).abs() < EPS, "expected {b:?}, got {a:?}");
    }

    fn unit_square() -> Vec<Vec2> {
        vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(1.0, 0.0),
            Vec2::new(1.0, 1.0),
            Vec2::new(0.0, 1.0),
        ]
    }

    #[test]
    fn aabb_overlaps_detects_overlap_and_separation() {
        let a = Aabb { min: Vec2::new(0.0, 0.0), max: Vec2::new(1.0, 1.0) };
        let overlapping = Aabb { min: Vec2::new(0.5, 0.5), max: Vec2::new(1.5, 1.5) };
        let separate = Aabb { min: Vec2::new(2.0, 2.0), max: Vec2::new(3.0, 3.0) };

        assert!(a.overlaps(&overlapping));
        assert!(!a.overlaps(&separate));
    }

    #[test]
    fn circle_aabb_is_centered_box_of_radius() {
        let shape = Shape::circle(2.0);
        let aabb = shape.aabb(Vec2::new(5.0, 5.0), Rot2::IDENTITY);
        assert_eq!(aabb.min, Vec2::new(3.0, 3.0));
        assert_eq!(aabb.max, Vec2::new(7.0, 7.0));
    }

    #[test]
    fn polygon_normals_point_outward_for_ccw_winding() {
        let shape = Shape::polygon(unit_square());
        if let Shape::Polygon { normals, .. } = shape {
            // Bottom edge (0,0)->(1,0): outward normal points down.
            approx_eq_vec(normals[0], Vec2::new(0.0, -1.0));
            // Right edge (1,0)->(1,1): outward normal points right.
            approx_eq_vec(normals[1], Vec2::new(1.0, 0.0));
            // Top edge (1,1)->(0,1): outward normal points up.
            approx_eq_vec(normals[2], Vec2::new(0.0, 1.0));
            // Left edge (0,1)->(0,0): outward normal points left.
            approx_eq_vec(normals[3], Vec2::new(-1.0, 0.0));
        } else {
            panic!("expected Polygon");
        }
    }

    #[test]
    fn polygon_aabb_axis_aligned_matches_extent() {
        let shape = Shape::polygon(unit_square());
        let aabb = shape.aabb(Vec2::new(10.0, 10.0), Rot2::IDENTITY);
        assert_eq!(aabb.min, Vec2::new(10.0, 10.0));
        assert_eq!(aabb.max, Vec2::new(11.0, 11.0));
    }

    #[test]
    fn polygon_aabb_grows_when_rotated() {
        let shape = Shape::polygon(unit_square());
        let axis_aligned = shape.aabb(Vec2::ZERO, Rot2::IDENTITY);
        let rotated = shape.aabb(Vec2::ZERO, Rot2::new(std::f32::consts::FRAC_PI_4));

        let axis_aligned_extent = axis_aligned.max.x - axis_aligned.min.x;
        let rotated_extent = rotated.max.x - rotated.min.x;
        assert!(rotated_extent > axis_aligned_extent);
    }
}
