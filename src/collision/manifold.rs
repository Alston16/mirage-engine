//! Contact and Manifold — the output of narrowphase.

use crate::{BodyId, Vec2};

/// A single point of overlap between two bodies.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Contact {
    pub point: Vec2,
    /// Unit vector pointing from the first body toward the second.
    pub normal: Vec2,
    /// Depth of overlap along `normal`. Always `>= 0.0`.
    pub penetration: f32,
}

/// The full set of contacts between one pair of bodies for the current
/// step — one point for circle pairs, one or two for polygon pairs.
#[derive(Clone, Debug, PartialEq)]
pub struct Manifold {
    pub body_a: BodyId,
    pub body_b: BodyId,
    pub points: Vec<Contact>,
}
