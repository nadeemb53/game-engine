pub fn add(left: u64, right: u64) -> u64 {
    left + right
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        let result = add(2, 2);
        assert_eq!(result, 4);
    }
}

pub mod math;
pub mod objects;
pub mod constraints;
pub mod integration;
pub mod collision;
pub mod shapes;
pub mod world;
pub mod common;

// Re-export key types for easier use
pub use math::vec2::Vec2;
pub use objects::rigid_body::RigidBody;
pub use shapes::{Shape, Circle, LineSegment, Polygon};
pub use constraints::{DistanceConstraint, PinJoint, Constraint};
pub use world::PhysicsWorld;
pub use common::Material;
