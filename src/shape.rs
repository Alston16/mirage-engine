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
    ///
    /// The vertices must be centered so that local `(0, 0)` — the point the
    /// body rotates about — is the polygon's actual center of mass. This is
    /// the caller's responsibility: no recentering happens here or in
    /// `inertia`. See `inertia`'s doc comment for what breaks if it isn't.
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

    /// Moment of inertia of this shape about its local origin for a body of
    /// total `mass`.
    ///
    /// Circle: `I = ½·m·r²`. Polygon (triangle fan about the origin):
    /// `I = m / (6·Σcᵢ) · Σ cᵢ·(pᵢ·pᵢ + pᵢ·pᵢ₊₁ + pᵢ₊₁·pᵢ₊₁)` with
    /// `cᵢ = pᵢ × pᵢ₊₁`.
    ///
    /// This assumes the polygon's vertices are centered on its center of
    /// mass (COM), i.e. that the local origin *is* the COM — the formula
    /// integrates about `(0, 0)` with no centroid computation or shift.
    /// Nothing here checks that assumption. If it doesn't hold (e.g. an
    /// off-center polygon built with a corner at the local origin instead
    /// of its centroid), `I` is silently computed about the wrong point:
    /// it won't panic, but the body's angular response to torque/impulses
    /// will be physically wrong (over- or under-rotating, drifting under
    /// spin that should be stable). If a future milestone needs polygons
    /// built from arbitrary (non-centered) vertices, this is the spot that
    /// would need a centroid computation feeding the parallel-axis theorem
    /// before this integral, plus a recentering of the stored vertices (or
    /// of `RigidBody::position`) so `position` still tracks the true COM.
    pub fn inertia(&self, mass: f32) -> f32 {
        match self {
            Shape::Circle { radius } => 0.5 * mass * radius * radius,
            Shape::Polygon { vertices, .. } => {
                let mut cross_sum = 0.0;
                let mut weighted_sum = 0.0;
                for i in 0..vertices.len() {
                    let p = vertices[i];
                    let q = vertices[(i + 1) % vertices.len()];
                    let c = p.cross(q);
                    cross_sum += c;
                    weighted_sum += c * (p.dot(p) + p.dot(q) + q.dot(q));
                }
                mass * weighted_sum / (6.0 * cross_sum)
            }
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

    #[test]
    fn circle_inertia_is_half_m_r_squared() {
        assert!((Shape::circle(1.0).inertia(1.0) - 0.5).abs() < EPS);
        assert!((Shape::circle(2.0).inertia(4.0) - 8.0).abs() < EPS);
    }

    #[test]
    fn square_inertia_is_two_thirds_m_for_half_extent_one() {
        let square = Shape::polygon(vec![
            Vec2::new(-1.0, -1.0),
            Vec2::new(1.0, -1.0),
            Vec2::new(1.0, 1.0),
            Vec2::new(-1.0, 1.0),
        ]);
        assert!((square.inertia(3.0) - 2.0).abs() < 1e-4);
    }

    #[test]
    fn rectangle_inertia_matches_closed_form() {
        // 2 wide x 4 tall: I = m·(w² + h²)/12.
        let rect = Shape::polygon(vec![
            Vec2::new(-1.0, -2.0),
            Vec2::new(1.0, -2.0),
            Vec2::new(1.0, 2.0),
            Vec2::new(-1.0, 2.0),
        ]);
        let mass = 5.0;
        let expected = mass * (2.0 * 2.0 + 4.0 * 4.0) / 12.0;
        assert!((rect.inertia(mass) - expected).abs() < 1e-4);
    }
}
