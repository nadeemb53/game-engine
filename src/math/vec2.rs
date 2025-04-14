use std::ops::{Add, Sub, Mul, AddAssign, SubAssign, Neg, Div};

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Vec2 {
    pub x: f64,
    pub y: f64,
}

impl Vec2 {
    pub const ZERO: Vec2 = Vec2 { x: 0.0, y: 0.0 };
    pub const UP: Vec2 = Vec2 { x: 0.0, y: 1.0 };

    /// Creates a new Vec2.
    pub fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    /// Calculates the dot product of two vectors.
    pub fn dot(self, other: Self) -> f64 {
        self.x * other.x + self.y * other.y
    }

    /// Calculates the squared magnitude (length) of the vector.
    /// Useful for comparisons as it avoids a square root.
    pub fn magnitude_squared(self) -> f64 {
        self.dot(self) // x*x + y*y
    }

    /// Calculates the magnitude (length) of the vector.
    pub fn magnitude(self) -> f64 {
        self.magnitude_squared().sqrt()
    }

    /// Returns a normalized version of the vector (unit vector).
    /// If the magnitude is zero, it returns a zero vector.
    pub fn normalize(self) -> Self {
        let mag = self.magnitude();
        if mag == 0.0 {
            Self::new(0.0, 0.0)
        } else {
            self * (1.0 / mag)
        }
    }

    /// Calculates the squared distance between two vector points.
    pub fn distance_squared(self, other: Self) -> f64 {
        (self - other).magnitude_squared()
    }

    /// Calculates the distance between two vector points.
    pub fn distance(self, other: Self) -> f64 {
        (self - other).magnitude()
    }

    /// Returns a vector perpendicular to this vector (90-degree counter-clockwise rotation).
    pub fn perpendicular(self) -> Self {
        Self::new(-self.y, self.x)
    }

    /// Rotates the vector by a given angle (in radians).
    pub fn rotate(self, angle: f64) -> Self {
        let cos_a = angle.cos();
        let sin_a = angle.sin();
        Self::new(
            self.x * cos_a - self.y * sin_a,
            self.x * sin_a + self.y * cos_a,
        )
    }

    /// Computes the 2D cross product (scalar). Equivalent to z-component of 3D cross product.
    pub fn cross(&self, other: Vec2) -> f64 {
        self.x * other.y - self.y * other.x
    }
}

// Implement Add trait
impl Add for Vec2 {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }
}

// Implement Sub trait
impl Sub for Vec2 {
    type Output = Self;

    fn sub(self, other: Self) -> Self {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
        }
    }
}

// Implement Mul trait for scalar multiplication (Vec2 * f64)
impl Mul<f64> for Vec2 {
    type Output = Self;

    fn mul(self, scalar: f64) -> Self {
        Self {
            x: self.x * scalar,
            y: self.y * scalar,
        }
    }
}

// Implement Mul trait for scalar multiplication (f64 * Vec2)
impl Mul<Vec2> for f64 {
    type Output = Vec2;

    fn mul(self, vec: Vec2) -> Vec2 {
        vec * self // Reuse the Vec2 * f64 implementation
    }
}

// Implement AddAssign for Vec2 += Vec2
impl AddAssign for Vec2 {
    fn add_assign(&mut self, rhs: Self) {
        self.x += rhs.x;
        self.y += rhs.y;
    }
}

// Implement SubAssign for Vec2 -= Vec2
impl SubAssign for Vec2 {
    fn sub_assign(&mut self, rhs: Self) {
        self.x -= rhs.x;
        self.y -= rhs.y;
    }
}

// Implement Div for Vec2 / f64
impl Div<f64> for Vec2 {
    type Output = Self;

    fn div(self, rhs: f64) -> Self::Output {
        // Handle division by zero or very small numbers if necessary
        if rhs.abs() < 1e-10 {
            // Maybe return ZERO, or a very large vector, or panic?
            // For now, let standard f64 division handle it (results in INFINITY or NaN)
            Vec2::new(self.x / rhs, self.y / rhs)
        } else {
             Vec2::new(self.x / rhs, self.y / rhs)
        }
    }
}

// Negation
impl Neg for Vec2 {
    type Output = Self;

    fn neg(self) -> Self {
        Self {
            x: -self.x,
            y: -self.y,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;
    const EPSILON: f64 = 1e-10; // For floating point comparisons

    #[test]
    fn test_vec2_new() {
        let v = Vec2::new(1.0, 2.0);
        assert_eq!(v.x, 1.0);
        assert_eq!(v.y, 2.0);
    }

    #[test]
    fn test_vec2_add() {
        let v1 = Vec2::new(1.0, 2.0);
        let v2 = Vec2::new(3.0, 4.0);
        let result = v1 + v2;
        assert_eq!(result, Vec2::new(4.0, 6.0));
    }

    #[test]
    fn test_vec2_sub() {
        let v1 = Vec2::new(3.0, 4.0);
        let v2 = Vec2::new(1.0, 2.0);
        let result = v1 - v2;
        assert_eq!(result, Vec2::new(2.0, 2.0));
    }

    #[test]
    fn test_vec2_scalar_mul() {
        let v = Vec2::new(1.0, 2.0);
        let scalar = 3.0;
        let result1 = v * scalar;
        let result2 = scalar * v;
        assert_eq!(result1, Vec2::new(3.0, 6.0));
        assert_eq!(result2, Vec2::new(3.0, 6.0));
    }

    #[test]
    fn test_vec2_dot() {
        let v1 = Vec2::new(1.0, 2.0);
        let v2 = Vec2::new(3.0, 4.0);
        let result = v1.dot(v2);
        assert!((result - 11.0).abs() < EPSILON);
    }

    #[test]
    fn test_vec2_magnitude_squared() {
        let v = Vec2::new(3.0, 4.0);
        assert!((v.magnitude_squared() - 25.0).abs() < EPSILON);
        let zero = Vec2::new(0.0, 0.0);
        assert!((zero.magnitude_squared() - 0.0).abs() < EPSILON);
    }

    #[test]
    fn test_vec2_magnitude() {
        let v = Vec2::new(3.0, 4.0);
        assert!((v.magnitude() - 5.0).abs() < EPSILON);
        let zero = Vec2::new(0.0, 0.0);
        assert!((zero.magnitude() - 0.0).abs() < EPSILON);
    }

    #[test]
    fn test_vec2_normalize() {
        let v = Vec2::new(3.0, 4.0);
        let norm_v = v.normalize();
        assert!((norm_v.magnitude() - 1.0).abs() < EPSILON);
        let expected = Vec2::new(3.0 / 5.0, 4.0 / 5.0);
        assert!((norm_v.x - expected.x).abs() < EPSILON);
        assert!((norm_v.y - expected.y).abs() < EPSILON);

        let zero = Vec2::new(0.0, 0.0);
        let norm_zero = zero.normalize();
        assert_eq!(norm_zero, Vec2::new(0.0, 0.0));
        assert!((norm_zero.magnitude() - 0.0).abs() < EPSILON);
    }

    #[test]
    fn test_vec2_distance_squared() {
        let v1 = Vec2::new(1.0, 2.0);
        let v2 = Vec2::new(4.0, 6.0); // Difference is (3.0, 4.0)
        assert!((v1.distance_squared(v2) - 25.0).abs() < EPSILON);
        assert!((v2.distance_squared(v1) - 25.0).abs() < EPSILON);
    }

    #[test]
    fn test_vec2_distance() {
        let v1 = Vec2::new(1.0, 2.0);
        let v2 = Vec2::new(4.0, 6.0); // Difference is (3.0, 4.0)
        assert!((v1.distance(v2) - 5.0).abs() < EPSILON);
        assert!((v2.distance(v1) - 5.0).abs() < EPSILON);
    }

    #[test]
    fn test_vec2_perpendicular() {
        let v = Vec2::new(3.0, 4.0);
        let perp = v.perpendicular();
        assert_eq!(perp, Vec2::new(-4.0, 3.0));
        // Dot product of perpendicular vectors should be zero
        assert!((v.dot(perp) - 0.0).abs() < EPSILON);
    }

    #[test]
    fn test_vec2_rotate() {
        let v = Vec2::new(1.0, 0.0);

        // Rotate by 90 degrees (PI / 2)
        let v90 = v.rotate(PI / 2.0);
        assert!((v90.x - 0.0).abs() < EPSILON);
        assert!((v90.y - 1.0).abs() < EPSILON);

        // Rotate by 180 degrees (PI)
        let v180 = v.rotate(PI);
        assert!((v180.x - -1.0).abs() < EPSILON);
        assert!((v180.y - 0.0).abs() < EPSILON);

        // Rotate by -90 degrees (-PI / 2)
        let v_neg90 = v.rotate(-PI / 2.0);
        assert!((v_neg90.x - 0.0).abs() < EPSILON);
        assert!((v_neg90.y - -1.0).abs() < EPSILON);

        // Rotate non-axis aligned vector
        let v2 = Vec2::new(1.0, 1.0);
        let v2_rot90 = v2.rotate(PI / 2.0); // Should be (-1, 1)
        assert!((v2_rot90.x - -1.0).abs() < EPSILON);
        assert!((v2_rot90.y - 1.0).abs() < EPSILON);
    }
}
