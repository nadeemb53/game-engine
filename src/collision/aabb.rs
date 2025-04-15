// Defines an Axis-Aligned Bounding Box

use crate::math::vec2::Vec2;

/// An Axis-Aligned Bounding Box defined by its minimum and maximum corner points.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AABB {
    pub min: Vec2,
    pub max: Vec2,
}

impl AABB {
    /// Creates a new AABB.
    pub fn new(min: Vec2, max: Vec2) -> Self {
        // Ensure min coordinates are <= max coordinates
        AABB {
            min: Vec2::new(min.x.min(max.x), min.y.min(max.y)),
            max: Vec2::new(min.x.max(max.x), min.y.max(max.y)),
        }
    }

    /// Checks if this AABB overlaps with another AABB.
    pub fn overlaps(&self, other: &AABB) -> bool {
        // Check for overlap on each axis
        let x_overlap = self.max.x > other.min.x && self.min.x < other.max.x;
        let y_overlap = self.max.y > other.min.y && self.min.y < other.max.y;
        x_overlap && y_overlap
    }

    /// Merges another AABB into this one, expanding this AABB to contain both.
    pub fn merge(&mut self, other: &AABB) {
        self.min.x = self.min.x.min(other.min.x);
        self.min.y = self.min.y.min(other.min.y);
        self.max.x = self.max.x.max(other.max.x);
        self.max.y = self.max.y.max(other.max.y);
    }

    /// Creates an AABB that encompasses a set of points.
    pub fn from_points(points: &[Vec2]) -> Option<Self> {
        if points.is_empty() {
            return None;
        }
        let mut min_pt = points[0];
        let mut max_pt = points[0];
        for point in points.iter().skip(1) {
            min_pt.x = min_pt.x.min(point.x);
            min_pt.y = min_pt.y.min(point.y);
            max_pt.x = max_pt.x.max(point.x);
            max_pt.y = max_pt.y.max(point.y);
        }
        Some(AABB::new(min_pt, max_pt))
    }
} 