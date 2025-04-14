// use crate::math::vec2::Vec2; // Unused for now

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Circle {
    pub radius: f64,
}

impl Circle {
    pub fn new(radius: f64) -> Self {
        assert!(radius >= 0.0, "Circle radius cannot be negative");
        Self { radius }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_circle_new() {
        let c = Circle::new(5.0);
        assert_eq!(c.radius, 5.0);
    }

    #[test]
    #[should_panic]
    fn test_circle_new_negative_radius() {
        Circle::new(-1.0);
    }
} 