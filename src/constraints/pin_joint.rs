use crate::math::vec2::Vec2;
use crate::objects::rigid_body::RigidBody;
use super::Constraint; // Import the trait
// use crate::constraints::distance_constraint::DistanceConstraint; // No longer needed
 // Added Shape, Circle

/// A constraint that forces two anchor points (one on each body) to coincide.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PinJoint {
    /// Index of the first rigid body.
    pub body_a_idx: usize,
    /// Index of the second rigid body.
    pub body_b_idx: usize,
    /// Anchor point on body A, in local coordinates.
    pub anchor_a_local: Vec2,
    /// Anchor point on body B, in local coordinates.
    pub anchor_b_local: Vec2,
    // We can add softness/compliance later if needed
}

impl PinJoint {
    pub fn new(
        body_a_idx: usize,
        body_b_idx: usize,
        anchor_a_local: Vec2,
        anchor_b_local: Vec2,
    ) -> Self {
        Self {
            body_a_idx,
            body_b_idx,
            anchor_a_local,
            anchor_b_local,
        }
    }
}

// Implement the Constraint trait
impl Constraint for PinJoint {
    /// Solves the constraint by directly adjusting body positions (PBD style).
    fn solve_position(&self, bodies: &mut [RigidBody]) {
        // Ensure indices are valid
        if self.body_a_idx >= bodies.len() || self.body_b_idx >= bodies.len() {
            eprintln!("Warning: Invalid body index in PinJoint.");
            return;
        }

        let (body_a, body_b) = super::get_mutable_body_pair(bodies, self.body_a_idx, self.body_b_idx);

        // 1. Calculate world anchor points
        let anchor_a_world = body_a.position + self.anchor_a_local.rotate(body_a.rotation);
        let anchor_b_world = body_b.position + self.anchor_b_local.rotate(body_b.rotation);

        // 2. Calculate delta vector (the error)
        let delta = anchor_b_world - anchor_a_world;

        // If delta is negligible, the constraint is already satisfied
        if delta.magnitude_squared() < 1e-12 { // Use squared magnitude to avoid sqrt
            return;
        }

        // 3. Calculate total inverse mass
        let total_inv_mass = body_a.inv_mass + body_b.inv_mass;

        // 4. If both bodies are static, we can't do anything
        if total_inv_mass == 0.0 {
            return;
        }

        // 5. Calculate correction magnitude (scalar)
        let correction_scalar = 1.0 / total_inv_mass; // Simplified: scale the delta vector

        // 6. Apply position corrections
        if body_a.inv_mass > 0.0 {
             body_a.position = body_a.position + delta * (body_a.inv_mass * correction_scalar);
        }
        if body_b.inv_mass > 0.0 {
            body_b.position = body_b.position - delta * (body_b.inv_mass * correction_scalar);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::vec2::Vec2;
    use crate::objects::rigid_body::RigidBody;
    use crate::shapes::{Shape, Circle}; // Added Shape, Circle
    const EPSILON: f64 = 1e-9;

    // Helper
    fn default_test_shape() -> Shape {
        Shape::Circle(Circle::new(1.0))
    }

    #[test]
    fn test_pin_joint_new() {
        let pj = PinJoint::new(0, 1, Vec2::new(1.0, 0.0), Vec2::new(-1.0, 0.0));
        assert_eq!(pj.body_a_idx, 0);
        assert_eq!(pj.body_b_idx, 1);
        assert_eq!(pj.anchor_a_local, Vec2::new(1.0, 0.0));
        assert_eq!(pj.anchor_b_local, Vec2::new(-1.0, 0.0));
    }

    #[test]
    fn test_solve_position_pin_separate() {
        let mut bodies = vec![
            RigidBody::new(1.0, default_test_shape()), // Use helper
            RigidBody::new(1.0, default_test_shape()),
        ];
        bodies[0].position = Vec2::new(0.0, 0.0);
        bodies[1].position = Vec2::new(4.0, 0.0);

        let joint = PinJoint::new(0, 1, Vec2::new(0.0, 0.0), Vec2::new(0.0, 0.0));
        joint.solve_position(&mut bodies);

        assert!((bodies[0].position.x - 2.0).abs() < EPSILON);
        assert!((bodies[0].position.y - 0.0).abs() < EPSILON);
        assert!((bodies[1].position.x - 2.0).abs() < EPSILON);
        assert!((bodies[1].position.y - 0.0).abs() < EPSILON);
    }

    #[test]
    fn test_solve_position_pin_one_static() {
         let mut bodies = vec![
            RigidBody::new(0.0, default_test_shape()), // Static
            RigidBody::new(1.0, default_test_shape()),
        ];
        bodies[0].position = Vec2::new(0.0, 0.0);
        bodies[1].position = Vec2::new(4.0, 3.0);

        let joint = PinJoint::new(0, 1, Vec2::new(0.0, 0.0), Vec2::new(0.0, 0.0));
        joint.solve_position(&mut bodies);

        assert!((bodies[0].position.x - 0.0).abs() < EPSILON);
        assert!((bodies[0].position.y - 0.0).abs() < EPSILON);
        assert!((bodies[1].position.x - 0.0).abs() < EPSILON);
        assert!((bodies[1].position.y - 0.0).abs() < EPSILON);
    }

     #[test]
    fn test_solve_position_pin_with_anchors_and_rotation() {
        let mut bodies = vec![
            RigidBody::new(1.0, default_test_shape()),
            RigidBody::new(1.0, default_test_shape()),
        ];
        bodies[0].position = Vec2::new(0.0, 0.0);
        bodies[0].rotation = std::f64::consts::PI / 2.0;
        bodies[1].position = Vec2::new(5.0, 0.0);
        bodies[1].rotation = 0.0;

        let joint = PinJoint::new(0, 1, Vec2::new(0.0, 1.0), Vec2::new(-1.0, 0.0));
        joint.solve_position(&mut bodies);

        assert!((bodies[0].position.x - 2.5).abs() < EPSILON);
        assert!((bodies[0].position.y - 0.0).abs() < EPSILON);
        assert!((bodies[1].position.x - 2.5).abs() < EPSILON);
        assert!((bodies[1].position.y - 0.0).abs() < EPSILON);

        let anchor_a_world = bodies[0].position + joint.anchor_a_local.rotate(bodies[0].rotation);
        let anchor_b_world = bodies[1].position + joint.anchor_b_local.rotate(bodies[1].rotation);
        assert!((anchor_a_world - anchor_b_world).magnitude_squared() < EPSILON * EPSILON);
    }

    #[test]
    fn test_solve_position_pin_already_satisfied() {
        let mut bodies = vec![
            RigidBody::new(1.0, default_test_shape()),
            RigidBody::new(1.0, default_test_shape()),
        ];
        bodies[0].position = Vec2::new(1.0, 1.0);
        bodies[1].position = Vec2::new(1.0, 1.0);
        let initial_bodies = bodies.clone();

        let joint = PinJoint::new(0, 1, Vec2::new(0.0, 0.0), Vec2::new(0.0, 0.0));
        joint.solve_position(&mut bodies);

        assert_eq!(bodies[0].position, initial_bodies[0].position);
        assert_eq!(bodies[1].position, initial_bodies[1].position);
    }
} 