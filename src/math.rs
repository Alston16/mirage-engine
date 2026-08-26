//! Vec2, Rot2, and the cross-product helpers the rest of the engine builds on.

use std::ops::{Add, AddAssign, Div, Mul, MulAssign, Neg, Sub, SubAssign};

/// A 2D vector, used for both positions/velocities and forces/impulses.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Vec2 {
    pub x: f32,
    pub y: f32,
}

impl Vec2 {
    pub const ZERO: Vec2 = Vec2 { x: 0.0, y: 0.0 };

    pub fn new(x: f32, y: f32) -> Self {
        Vec2 { x, y }
    }

    /// `a · b = a.x*b.x + a.y*b.y`
    pub fn dot(self, other: Vec2) -> f32 {
        self.x * other.x + self.y * other.y
    }

    /// 2D cross product of two vectors is a scalar (the z-component of the
    /// 3D cross product with both inputs' z = 0): `a.x*b.y - a.y*b.x`.
    pub fn cross(self, other: Vec2) -> f32 {
        self.x * other.y - self.y * other.x
    }

    /// Cross product of a scalar (e.g. angular velocity `ω`) with a vector,
    /// `s × v`, used for `ω × r` terms. Returns a vector.
    pub fn cross_sv(s: f32, v: Vec2) -> Vec2 {
        Vec2::new(-s * v.y, s * v.x)
    }

    /// Cross product of a vector with a scalar, `v × s`. Returns a vector.
    pub fn cross_vs(v: Vec2, s: f32) -> Vec2 {
        Vec2::new(s * v.y, -s * v.x)
    }

    pub fn length_squared(self) -> f32 {
        self.dot(self)
    }

    pub fn length(self) -> f32 {
        self.length_squared().sqrt()
    }

    /// Unit vector in the same direction, or `Vec2::ZERO` if this vector is
    /// (near) zero length.
    pub fn normalize(self) -> Vec2 {
        let len = self.length();
        if len < f32::EPSILON {
            Vec2::ZERO
        } else {
            self / len
        }
    }

    /// Perpendicular vector, rotated 90° counter-clockwise: `(-y, x)`.
    pub fn perp(self) -> Vec2 {
        Vec2::new(-self.y, self.x)
    }
}

impl Add for Vec2 {
    type Output = Vec2;
    fn add(self, rhs: Vec2) -> Vec2 {
        Vec2::new(self.x + rhs.x, self.y + rhs.y)
    }
}

impl AddAssign for Vec2 {
    fn add_assign(&mut self, rhs: Vec2) {
        self.x += rhs.x;
        self.y += rhs.y;
    }
}

impl Sub for Vec2 {
    type Output = Vec2;
    fn sub(self, rhs: Vec2) -> Vec2 {
        Vec2::new(self.x - rhs.x, self.y - rhs.y)
    }
}

impl SubAssign for Vec2 {
    fn sub_assign(&mut self, rhs: Vec2) {
        self.x -= rhs.x;
        self.y -= rhs.y;
    }
}

impl Neg for Vec2 {
    type Output = Vec2;
    fn neg(self) -> Vec2 {
        Vec2::new(-self.x, -self.y)
    }
}

impl Mul<f32> for Vec2 {
    type Output = Vec2;
    fn mul(self, rhs: f32) -> Vec2 {
        Vec2::new(self.x * rhs, self.y * rhs)
    }
}

impl Mul<Vec2> for f32 {
    type Output = Vec2;
    fn mul(self, rhs: Vec2) -> Vec2 {
        rhs * self
    }
}

impl MulAssign<f32> for Vec2 {
    fn mul_assign(&mut self, rhs: f32) {
        self.x *= rhs;
        self.y *= rhs;
    }
}

impl Div<f32> for Vec2 {
    type Output = Vec2;
    fn div(self, rhs: f32) -> Vec2 {
        Vec2::new(self.x / rhs, self.y / rhs)
    }
}

/// A 2D rotation, stored as `(sin, cos)` rather than an angle so repeated
/// composition doesn't need trig on every step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Rot2 {
    pub sin: f32,
    pub cos: f32,
}

impl Rot2 {
    pub const IDENTITY: Rot2 = Rot2 { sin: 0.0, cos: 1.0 };

    /// Builds a rotation from an angle in radians.
    pub fn new(angle: f32) -> Self {
        Rot2 {
            sin: angle.sin(),
            cos: angle.cos(),
        }
    }

    /// Recovers the angle in radians via `atan2(sin, cos)`.
    pub fn angle(self) -> f32 {
        self.sin.atan2(self.cos)
    }

    /// Rotates a vector by this rotation.
    pub fn rotate(self, v: Vec2) -> Vec2 {
        Vec2::new(
            self.cos * v.x - self.sin * v.y,
            self.sin * v.x + self.cos * v.y,
        )
    }

    /// Rotates a vector by the inverse of this rotation.
    pub fn unrotate(self, v: Vec2) -> Vec2 {
        Vec2::new(
            self.cos * v.x + self.sin * v.y,
            -self.sin * v.x + self.cos * v.y,
        )
    }

}

impl Mul for Rot2 {
    type Output = Rot2;

    /// Composes two rotations: `self` followed by `rhs`, i.e. rotating by
    /// the sum of their angles.
    fn mul(self, rhs: Rot2) -> Rot2 {
        Rot2 {
            sin: self.sin * rhs.cos + self.cos * rhs.sin,
            cos: self.cos * rhs.cos - self.sin * rhs.sin,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f32 = 1e-5;

    fn approx_eq(a: f32, b: f32) {
        assert!((a - b).abs() < EPS, "expected {b}, got {a}");
    }

    fn approx_eq_vec(a: Vec2, b: Vec2) {
        approx_eq(a.x, b.x);
        approx_eq(a.y, b.y);
    }

    #[test]
    fn vec2_add_sub_neg() {
        let a = Vec2::new(1.0, 2.0);
        let b = Vec2::new(3.0, -1.0);
        approx_eq_vec(a + b, Vec2::new(4.0, 1.0));
        approx_eq_vec(a - b, Vec2::new(-2.0, 3.0));
        approx_eq_vec(-a, Vec2::new(-1.0, -2.0));
    }

    #[test]
    fn vec2_scalar_mul_div() {
        let a = Vec2::new(2.0, -3.0);
        approx_eq_vec(a * 2.0, Vec2::new(4.0, -6.0));
        approx_eq_vec(2.0 * a, Vec2::new(4.0, -6.0));
        approx_eq_vec(a / 2.0, Vec2::new(1.0, -1.5));
    }

    #[test]
    fn vec2_dot_orthogonal_is_zero() {
        let a = Vec2::new(1.0, 0.0);
        let b = Vec2::new(0.0, 1.0);
        approx_eq(a.dot(b), 0.0);
    }

    #[test]
    fn vec2_dot_parallel_is_length_product() {
        let a = Vec2::new(3.0, 4.0);
        approx_eq(a.dot(a), 25.0);
    }

    #[test]
    fn vec2_cross_matches_scalar_z_component() {
        let a = Vec2::new(1.0, 0.0);
        let b = Vec2::new(0.0, 1.0);
        // x-axis × y-axis = +1 (counter-clockwise, right-handed 2D convention).
        approx_eq(a.cross(b), 1.0);
        approx_eq(b.cross(a), -1.0);
    }

    #[test]
    fn vec2_cross_parallel_is_zero() {
        let a = Vec2::new(2.0, 3.0);
        let b = a * 4.0;
        approx_eq(a.cross(b), 0.0);
    }

    #[test]
    fn vec2_cross_sv_and_vs_are_negatives() {
        let s = 2.0;
        let v = Vec2::new(1.0, 0.0);
        approx_eq_vec(Vec2::cross_sv(s, v), -Vec2::cross_vs(v, s));
    }

    #[test]
    fn vec2_length_and_normalize() {
        let a = Vec2::new(3.0, 4.0);
        approx_eq(a.length(), 5.0);
        approx_eq_vec(a.normalize(), Vec2::new(0.6, 0.8));
        approx_eq_vec(Vec2::ZERO.normalize(), Vec2::ZERO);
    }

    #[test]
    fn vec2_perp_is_90_degrees_ccw() {
        let a = Vec2::new(1.0, 0.0);
        approx_eq_vec(a.perp(), Vec2::new(0.0, 1.0));
        // Perpendicular vectors are orthogonal to the original.
        approx_eq(a.dot(a.perp()), 0.0);
    }

    #[test]
    fn rot2_identity_is_noop() {
        let v = Vec2::new(1.0, 2.0);
        approx_eq_vec(Rot2::IDENTITY.rotate(v), v);
    }

    #[test]
    fn rot2_quarter_turn_matches_perp() {
        let r = Rot2::new(std::f32::consts::FRAC_PI_2);
        let v = Vec2::new(1.0, 0.0);
        approx_eq_vec(r.rotate(v), Vec2::new(0.0, 1.0));
    }

    #[test]
    fn rot2_half_turn_negates() {
        let r = Rot2::new(std::f32::consts::PI);
        let v = Vec2::new(1.0, 2.0);
        approx_eq_vec(r.rotate(v), -v);
    }

    #[test]
    fn rot2_rotate_then_unrotate_is_identity() {
        let r = Rot2::new(0.73);
        let v = Vec2::new(-2.0, 5.5);
        approx_eq_vec(r.unrotate(r.rotate(v)), v);
    }

    #[test]
    fn rot2_angle_roundtrip() {
        let angle = 1.2345_f32;
        let r = Rot2::new(angle);
        approx_eq(r.angle(), angle);
    }

    #[test]
    fn rot2_mul_composes_angles() {
        let a = Rot2::new(0.4);
        let b = Rot2::new(0.9);
        let combined = a * b;
        approx_eq(combined.angle(), 1.3);
    }

    #[test]
    fn rot2_rotation_preserves_length() {
        let r = Rot2::new(2.1);
        let v = Vec2::new(3.0, -4.0);
        approx_eq(r.rotate(v).length(), v.length());
    }
}
