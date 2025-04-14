use crate::math::vec2::Vec2;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LineSegment {
    pub a: Vec2,
    pub b: Vec2,
}

impl LineSegment {
    pub fn new(a: Vec2, b: Vec2) -> Self {
        Self { a, b }
    }

    /// Calculates the length of the line segment.
    pub fn length(&self) -> f64 {
        self.a.distance(self.b)
    }

    /// Calculates the squared length of the line segment.
    pub fn length_squared(&self) -> f64 {
        self.a.distance_squared(self.b)
    }

    /// Returns the direction vector of the line segment (from a to b).
    pub fn direction(&self) -> Vec2 {
        self.b - self.a
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::vec2::Vec2;
    const EPSILON: f64 = 1e-10;

    #[test]
    fn test_line_segment_new() {
        let a = Vec2::new(1.0, 2.0);
        let b = Vec2::new(4.0, 6.0);
        let line = LineSegment::new(a, b);
        assert_eq!(line.a, a);
        assert_eq!(line.b, b);
    }

    #[test]
    fn test_line_segment_length() {
        let a = Vec2::new(1.0, 2.0);
        let b = Vec2::new(4.0, 6.0); // Difference (3, 4), length 5
        let line = LineSegment::new(a, b);
        assert!((line.length() - 5.0).abs() < EPSILON);
    }

    #[test]
    fn test_line_segment_length_squared() {
        let a = Vec2::new(1.0, 2.0);
        let b = Vec2::new(4.0, 6.0); // Difference (3, 4), length^2 25
        let line = LineSegment::new(a, b);
        assert!((line.length_squared() - 25.0).abs() < EPSILON);
    }

    #[test]
    fn test_line_segment_direction() {
        let a = Vec2::new(1.0, 2.0);
        let b = Vec2::new(4.0, 6.0);
        let line = LineSegment::new(a, b);
        let dir = line.direction();
        let expected_dir = Vec2::new(3.0, 4.0);
        assert!((dir.x - expected_dir.x).abs() < EPSILON);
        assert!((dir.y - expected_dir.y).abs() < EPSILON);
    }
} 