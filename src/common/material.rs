//! Defines physical material properties.

/// Represents the physical properties of a rigid body affecting collisions.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Material {
    /// Coefficient of restitution (bounciness). Range [0, 1].
    /// 0 = perfectly inelastic (no bounce), 1 = perfectly elastic.
    pub restitution: f64,
    /// Coefficient of static friction. Range [0, infinity).
    /// Higher values mean more resistance to sliding.
    pub friction: f64,
    // Could add density here later if needed for mass calculation from volume
}

impl Material {
    /// Creates a new material with the given restitution and friction.
    pub fn new(restitution: f64, friction: f64) -> Self {
        Material {
            // Clamp values to reasonable ranges
            restitution: restitution.clamp(0.0, 1.0),
            friction: friction.max(0.0),
        }
    }
}

impl Default for Material {
    /// Default material properties (moderate restitution, moderate friction).
    fn default() -> Self {
        Material {
            restitution: 0.2,
            friction: 0.5,
        }
    }
} 