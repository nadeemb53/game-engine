use crate::math::vec2::Vec2;
use crate::objects::rigid_body::RigidBody;
use super::Constraint; // Import the new trait
 // Added Shape, Circle

/// A constraint that keeps two points on two bodies at a fixed distance.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DistanceConstraint {
    /// Index of the first rigid body.
    pub body_a_idx: usize,
    /// Index of the second rigid body.
    pub body_b_idx: usize,
    /// Anchor point on body A, in local coordinates.
    pub anchor_a_local: Vec2,
    /// Anchor point on body B, in local coordinates.
    pub anchor_b_local: Vec2,
    /// The target distance to maintain between the anchor points.
    pub distance: f64,
    // We can add stiffness/compliance later if needed
}

impl DistanceConstraint {
    pub fn new(
        body_a_idx: usize,
        body_b_idx: usize,
        anchor_a_local: Vec2,
        anchor_b_local: Vec2,
        distance: f64,
    ) -> Self {
        assert!(distance >= 0.0, "Distance cannot be negative");
        Self {
            body_a_idx,
            body_b_idx,
            anchor_a_local,
            anchor_b_local,
            distance,
        }
    }
}

// Implement the Constraint trait
impl Constraint for DistanceConstraint {
    /// Solves the constraint by directly adjusting body positions (PBD style).
    fn solve_position(&self, bodies: &mut [RigidBody]) {
        // Ensure indices are valid (simple bounds check)
        if self.body_a_idx >= bodies.len() || self.body_b_idx >= bodies.len() {
            eprintln!("Warning: Invalid body index in DistanceConstraint.");
            return;
        }

        // Get mutable references to the bodies involved.
        let (body_a, body_b) = super::get_mutable_body_pair(bodies, self.body_a_idx, self.body_b_idx);

        // 1. Calculate world anchor points
        let anchor_a_world = body_a.position + self.anchor_a_local.rotate(body_a.rotation);
        let anchor_b_world = body_b.position + self.anchor_b_local.rotate(body_b.rotation);

        // 2. Calculate delta vector and current distance
        let delta = anchor_b_world - anchor_a_world;
        let current_dist = delta.magnitude();

        // 3. Check for zero distance (or very small distance)
        if current_dist < 1e-10 { // Avoid division by zero / instability
            return;
        }

        // 4. Calculate error
        let error = current_dist - self.distance;

        // No correction needed if error is negligible
        if error.abs() < 1e-10 {
            return;
        }

        // 5. Calculate correction direction
        let direction = delta * (1.0 / current_dist); // Normalized direction using multiplication

        // 6. Calculate total inverse mass
        let total_inv_mass = body_a.inv_mass + body_b.inv_mass;

        // 7. If both bodies are static, we can't do anything
        if total_inv_mass == 0.0 {
            return;
        }

        // 8. Calculate correction magnitude (scalar)
        let correction_scalar = error / total_inv_mass;

        // 9. Apply position corrections
        if body_a.inv_mass > 0.0 {
             body_a.position = body_a.position + direction * correction_scalar * body_a.inv_mass;
        }
        if body_b.inv_mass > 0.0 {
            body_b.position = body_b.position - direction * correction_scalar * body_b.inv_mass;
        }

        // Note: This doesn't handle rotation induced by the position change.
        // More advanced PBD solvers might include angular corrections.
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
    fn test_distance_constraint_new() {
        let dc = DistanceConstraint::new(0, 1, Vec2::new(1.0, 0.0), Vec2::new(-1.0, 0.0), 5.0);
        assert_eq!(dc.body_a_idx, 0);
        assert_eq!(dc.body_b_idx, 1);
        assert_eq!(dc.anchor_a_local, Vec2::new(1.0, 0.0));
        assert_eq!(dc.anchor_b_local, Vec2::new(-1.0, 0.0));
        assert_eq!(dc.distance, 5.0);
    }

    #[test]
    #[should_panic]
    fn test_distance_constraint_new_negative_distance() {
        DistanceConstraint::new(0, 1, Vec2::new(0.0, 0.0), Vec2::new(0.0, 0.0), -1.0);
    }

    #[test]
    fn test_solve_position_stretch() {
        let mut bodies = vec![
            RigidBody::new(1.0, default_test_shape()), // Use helper
            RigidBody::new(1.0, default_test_shape()),
        ];
        // Place bodies too close together
        bodies[0].position = Vec2::new(0.0, 0.0);
        bodies[1].position = Vec2::new(3.0, 0.0);

        let constraint = DistanceConstraint::new(0, 1, Vec2::new(0.0, 0.0), Vec2::new(0.0, 0.0), 5.0);
        // Anchors at centers, target distance 5.0, current distance 3.0
        // Error = 3.0 - 5.0 = -2.0
        // Total inv mass = 1/1 + 1/1 = 2.0
        // Correction scalar = -2.0 / 2.0 = -1.0
        // Body A correction = dir * scalar * inv_mass_a = (1,0) * -1.0 * 1.0 = (-1, 0)
        // Body B correction = -dir * scalar * inv_mass_b = -(1,0) * -1.0 * 1.0 = (1, 0)

        constraint.solve_position(&mut bodies);

        // Expected: body 0 moves to (-1, 0), body 1 moves to (4, 0)
        // Distance becomes 4 - (-1) = 5
        assert!((bodies[0].position.x - -1.0).abs() < EPSILON);
        assert!((bodies[0].position.y - 0.0).abs() < EPSILON);
        assert!((bodies[1].position.x - 4.0).abs() < EPSILON);
        assert!((bodies[1].position.y - 0.0).abs() < EPSILON);
        assert!((bodies[0].position.distance(bodies[1].position) - 5.0).abs() < EPSILON);
    }

    #[test]
    fn test_solve_position_compress() {
         let mut bodies = vec![
            RigidBody::new(1.0, default_test_shape()),
            RigidBody::new(1.0, default_test_shape()),
        ];
        // Place bodies too far apart
        bodies[0].position = Vec2::new(0.0, 0.0);
        bodies[1].position = Vec2::new(6.0, 0.0);

        let constraint = DistanceConstraint::new(0, 1, Vec2::new(0.0, 0.0), Vec2::new(0.0, 0.0), 4.0);
        // Anchors at centers, target distance 4.0, current distance 6.0
        // Error = 6.0 - 4.0 = 2.0
        // Total inv mass = 1/1 + 1/1 = 2.0
        // Correction scalar = 2.0 / 2.0 = 1.0
        // Body A correction = dir * scalar * inv_mass_a = (1,0) * 1.0 * 1.0 = (1, 0)
        // Body B correction = -dir * scalar * inv_mass_b = -(1,0) * 1.0 * 1.0 = (-1, 0)

        constraint.solve_position(&mut bodies);

        // Expected: body 0 moves to (1, 0), body 1 moves to (5, 0)
        // Distance becomes 5 - 1 = 4
        assert!((bodies[0].position.x - 1.0).abs() < EPSILON);
        assert!((bodies[0].position.y - 0.0).abs() < EPSILON);
        assert!((bodies[1].position.x - 5.0).abs() < EPSILON);
        assert!((bodies[1].position.y - 0.0).abs() < EPSILON);
        assert!((bodies[0].position.distance(bodies[1].position) - 4.0).abs() < EPSILON);
    }

    #[test]
    fn test_solve_position_one_static() {
        let mut bodies = vec![
            RigidBody::new(0.0, default_test_shape()), // Static
            RigidBody::new(1.0, default_test_shape()),
        ];
        bodies[0].position = Vec2::new(0.0, 0.0);
        bodies[1].position = Vec2::new(3.0, 0.0);

        let constraint = DistanceConstraint::new(0, 1, Vec2::new(0.0, 0.0), Vec2::new(0.0, 0.0), 5.0);
        // Target distance 5.0, current distance 3.0
        // Error = 3.0 - 5.0 = -2.0
        // Total inv mass = 0 + 1/1 = 1.0
        // Correction scalar = -2.0 / 1.0 = -2.0
        // Body A correction = dir * scalar * inv_mass_a = (1,0) * -2.0 * 0 = (0, 0)
        // Body B correction = -dir * scalar * inv_mass_b = -(1,0) * -2.0 * 1.0 = (2, 0)

        constraint.solve_position(&mut bodies);

        // Expected: body 0 stays at (0, 0), body 1 moves to (3+2, 0) = (5, 0)
        assert!((bodies[0].position.x - 0.0).abs() < EPSILON);
        assert!((bodies[0].position.y - 0.0).abs() < EPSILON);
        assert!((bodies[1].position.x - 5.0).abs() < EPSILON);
        assert!((bodies[1].position.y - 0.0).abs() < EPSILON);
        assert!((bodies[0].position.distance(bodies[1].position) - 5.0).abs() < EPSILON);
    }

     #[test]
    fn test_solve_position_with_anchors_and_rotation() {
        let mut bodies = vec![
            RigidBody::new(1.0, default_test_shape()),
            RigidBody::new(1.0, default_test_shape()),
        ];
        bodies[0].position = Vec2::new(0.0, 0.0);
        bodies[0].rotation = std::f64::consts::PI / 2.0; // Rotated 90 deg
        bodies[1].position = Vec2::new(5.0, 1.0);
        bodies[1].rotation = 0.0;

        // Anchor A is (1, 0) local on body 0. World pos: (0,0) + (1,0).rotate(pi/2) = (0,0) + (0,1) = (0, 1)
        // Anchor B is (-1, 0) local on body 1. World pos: (5,1) + (-1,0).rotate(0) = (5,1) + (-1,0) = (4, 1)
        // Current distance between (0, 1) and (4, 1) is 4.0
        let constraint = DistanceConstraint::new(0, 1, Vec2::new(1.0, 0.0), Vec2::new(-1.0, 0.0), 6.0);
        // Target distance 6.0. Error = 4.0 - 6.0 = -2.0
        // Total inv mass = 1 + 1 = 2.0
        // Correction scalar = -2.0 / 2.0 = -1.0
        // Direction = ((4,1) - (0,1)).normalize() = (4,0).normalize() = (1, 0)
        // Body A correction = (1,0) * -1.0 * 1.0 = (-1, 0)
        // Body B correction = -(1,0) * -1.0 * 1.0 = (1, 0)

        constraint.solve_position(&mut bodies);

        // Expected: body 0 moves to (-1, 0), body 1 moves to (6, 1)
        // New anchor A world = (-1,0) + (0,1) = (-1, 1)
        // New anchor B world = (6,1) + (-1,0) = (5, 1)
        // New distance = 5 - (-1) = 6
        assert!((bodies[0].position.x - -1.0).abs() < EPSILON);
        assert!((bodies[0].position.y - 0.0).abs() < EPSILON);
        assert!((bodies[1].position.x - 6.0).abs() < EPSILON);
        assert!((bodies[1].position.y - 1.0).abs() < EPSILON);

        // Verify distance after correction
        let anchor_a_world = bodies[0].position + constraint.anchor_a_local.rotate(bodies[0].rotation);
        let anchor_b_world = bodies[1].position + constraint.anchor_b_local.rotate(bodies[1].rotation);
        assert!((anchor_a_world.distance(anchor_b_world) - 6.0).abs() < EPSILON);
    }

     #[test]
    fn test_solve_position_no_correction_needed() {
        let mut bodies = vec![
            RigidBody::new(1.0, default_test_shape()),
            RigidBody::new(1.0, default_test_shape()),
        ];
        bodies[0].position = Vec2::new(0.0, 0.0);
        bodies[1].position = Vec2::new(5.0, 0.0);
        let initial_bodies = bodies.clone();

        let constraint = DistanceConstraint::new(0, 1, Vec2::new(0.0, 0.0), Vec2::new(0.0, 0.0), 5.0);
        // Current distance = target distance = 5.0

        constraint.solve_position(&mut bodies);

        // Bodies should not have moved
        assert_eq!(bodies[0].position, initial_bodies[0].position);
        assert_eq!(bodies[1].position, initial_bodies[1].position);
    }
} 