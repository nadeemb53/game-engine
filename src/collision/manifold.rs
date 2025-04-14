use crate::math::vec2::Vec2;

/// Stores information about a collision contact.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ContactPoint {
    /// Contact point on body A in world coordinates.
    pub point_a: Vec2,
    /// Contact point on body B in world coordinates.
    pub point_b: Vec2,
}

/// Stores information about a collision between two bodies.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CollisionManifold {
    /// Index of the first body involved in the collision.
    pub body_a_idx: usize,
    /// Index of the second body involved in the collision.
    pub body_b_idx: usize,
    /// The collision normal, pointing from body A towards body B.
    pub normal: Vec2,
    /// The amount of penetration between the shapes.
    pub depth: f64,
    /// The contact point (for circle-circle, same as average point).
    pub contact: ContactPoint, // Simplified for now, might need Vec<ContactPoint> later
} 