use super::vec2::Vec2;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Transform {
    pub position: Vec2,
    pub rotation: f64, // Angle in radians
}

impl Transform {
    /// Creates a new transform.
    pub fn new(position: Vec2, rotation: f64) -> Self {
        Self { position, rotation }
    }

    /// Creates an identity transform (no translation, no rotation).
    pub fn identity() -> Self {
        Self {
            position: Vec2::new(0.0, 0.0),
            rotation: 0.0,
        }
    }

    /// Applies the transform (rotation then translation) to a point.
    pub fn apply(self, point: Vec2) -> Vec2 {
        let cos_a = self.rotation.cos();
        let sin_a = self.rotation.sin();
        let rotated_x = point.x * cos_a - point.y * sin_a;
        let rotated_y = point.x * sin_a + point.y * cos_a;
        Vec2::new(rotated_x, rotated_y) + self.position
    }

    /// Applies the inverse transform (inverse translation then inverse rotation) to a point.
    pub fn apply_inverse(self, point: Vec2) -> Vec2 {
        let translated_point = point - self.position;
        // Use negative rotation angle: cos(-a) = cos(a), sin(-a) = -sin(a)
        let cos_a = self.rotation.cos();
        let sin_a = self.rotation.sin(); // But we need sin(-a) which is -sin(a)
        let rotated_x = translated_point.x * cos_a + translated_point.y * sin_a; // Equivalent to x*cos(-a) - y*sin(-a)
        let rotated_y = -translated_point.x * sin_a + translated_point.y * cos_a; // Equivalent to x*sin(-a) + y*cos(-a)
        Vec2::new(rotated_x, rotated_y)
    }
}

// Basic tests will go here later
#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::vec2::Vec2;
    use std::f64::consts::PI;
    const EPSILON: f64 = 1e-10;

    #[test]
    fn test_transform_new() {
        let pos = Vec2::new(1.0, 2.0);
        let rot = std::f64::consts::PI / 4.0;
        let t = Transform::new(pos, rot);
        assert_eq!(t.position, pos);
        assert!((t.rotation - rot).abs() < EPSILON);
    }

    #[test]
    fn test_transform_identity() {
        let t = Transform::identity();
        assert_eq!(t.position, Vec2::new(0.0, 0.0));
        assert!((t.rotation - 0.0).abs() < EPSILON);
    }

    #[test]
    fn test_transform_apply_identity() {
        let t = Transform::identity();
        let p = Vec2::new(5.0, -3.0);
        let tp = t.apply(p);
        assert!((tp.x - p.x).abs() < EPSILON);
        assert!((tp.y - p.y).abs() < EPSILON);
    }

    #[test]
    fn test_transform_apply_translation() {
        let t = Transform::new(Vec2::new(10.0, 5.0), 0.0);
        let p = Vec2::new(1.0, 2.0);
        let tp = t.apply(p);
        assert!((tp.x - 11.0).abs() < EPSILON);
        assert!((tp.y - 7.0).abs() < EPSILON);
    }

    #[test]
    fn test_transform_apply_rotation_90_deg() {
        // 90 degrees rotation
        let t = Transform::new(Vec2::new(0.0, 0.0), PI / 2.0);
        let p = Vec2::new(1.0, 0.0);
        let tp = t.apply(p);
        // Should rotate (1,0) to (0,1)
        assert!((tp.x - 0.0).abs() < EPSILON);
        assert!((tp.y - 1.0).abs() < EPSILON);

        let p2 = Vec2::new(0.0, 1.0);
        let tp2 = t.apply(p2);
        // Should rotate (0,1) to (-1, 0)
        assert!((tp2.x - -1.0).abs() < EPSILON);
        assert!((tp2.y - 0.0).abs() < EPSILON);
    }

    #[test]
    fn test_transform_apply_combined() {
        // Translate by (10, 5), then rotate by 90 degrees
        let t = Transform::new(Vec2::new(10.0, 5.0), PI / 2.0);
        let p = Vec2::new(1.0, 0.0);
        // Rotation of (1,0) -> (0,1)
        // Translation of (0,1) -> (10, 6)
        let tp = t.apply(p);
        assert!((tp.x - 10.0).abs() < EPSILON);
        assert!((tp.y - 6.0).abs() < EPSILON);
    }

    #[test]
    fn test_transform_apply_inverse_combined() {
        let pos = Vec2::new(10.0, 5.0);
        let rot = PI / 4.0;
        let t = Transform::new(pos, rot);
        let p_local = Vec2::new(1.0, 1.0);

        // Transform point from local to world
        let p_world = t.apply(p_local);

        // Transform point back from world to local
        let p_local_again = t.apply_inverse(p_world);

        // Check if it matches the original local point
        assert!((p_local_again.x - p_local.x).abs() < EPSILON);
        assert!((p_local_again.y - p_local.y).abs() < EPSILON);
    }

    #[test]
    fn test_transform_apply_inverse_identity() {
        let t = Transform::identity();
        let p = Vec2::new(5.0, -3.0);
        let tp = t.apply_inverse(p);
        assert!((tp.x - p.x).abs() < EPSILON);
        assert!((tp.y - p.y).abs() < EPSILON);
    }

    #[test]
    fn test_transform_apply_inverse_translation() {
        let t = Transform::new(Vec2::new(10.0, 5.0), 0.0);
        let p_world = Vec2::new(11.0, 7.0);
        let p_local = t.apply_inverse(p_world);
        assert!((p_local.x - 1.0).abs() < EPSILON);
        assert!((p_local.y - 2.0).abs() < EPSILON);
    }

    #[test]
    fn test_transform_apply_inverse_rotation_90_deg() {
        let t = Transform::new(Vec2::new(0.0, 0.0), PI / 2.0);
        let p_world = Vec2::new(0.0, 1.0); // Result of rotating (1,0)
        let p_local = t.apply_inverse(p_world);
        assert!((p_local.x - 1.0).abs() < EPSILON);
        assert!((p_local.y - 0.0).abs() < EPSILON);

        let p_world2 = Vec2::new(-1.0, 0.0); // Result of rotating (0,1)
        let p_local2 = t.apply_inverse(p_world2);
        assert!((p_local2.x - 0.0).abs() < EPSILON);
        assert!((p_local2.y - 1.0).abs() < EPSILON);
    }
} 