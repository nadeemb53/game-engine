use crate::math::vec2::Vec2;
use crate::objects::rigid_body::RigidBody;
// Import the trait and specific types needed for adding
use crate::constraints::{
    Constraint,
    DistanceConstraint,
    PinJoint,
};
use crate::integration::integrator;
use crate::collision::detection; // For collision check functions
use crate::collision::manifold::CollisionManifold;
use crate::shapes::Shape;
use std::f64::EPSILON; // Import standard EPSILON
// Added Shape, Circle

pub struct PhysicsWorld {
    pub bodies: Vec<RigidBody>,
    // Store constraints as boxed trait objects
    pub constraints: Vec<Box<dyn Constraint>>,
    pub gravity: Vec2,
    pub solver_iterations: usize,
    // Store detected collisions from the last step
    pub contacts: Vec<CollisionManifold>,
}

impl PhysicsWorld {
    /// Constants for resolution
    const RESTITUTION: f64 = 0.1; // Low bounciness
    const POSITIONAL_CORRECTION_PERCENT: f64 = 0.2; // Penetration correction factor (e.g., 20%-80%)
    const POSITIONAL_CORRECTION_SLOP: f64 = 0.01; // Allowable penetration depth

    /// Creates a new, empty physics world with default settings.
    pub fn new() -> Self {
        Self {
            bodies: Vec::new(),
            constraints: Vec::new(),
            gravity: Vec2::new(0.0, -9.81), // Default gravity downwards
            solver_iterations: 8, // A reasonable default for iterations
            contacts: Vec::new(), // Initialize contacts
        }
    }

    /// Adds a rigid body to the world and returns its index.
    pub fn add_body(&mut self, body: RigidBody) -> usize {
        let index = self.bodies.len();
        self.bodies.push(body);
        index
    }

    /// Adds any constraint that implements the `Constraint` trait.
    /// The constraint is boxed and added to the world.
    pub fn add_constraint(&mut self, constraint: Box<dyn Constraint>) {
        // Optional: Add checks here to ensure body indices are valid? Requires access
        // to indices, which might need methods on the Constraint trait itself.
        self.constraints.push(constraint);
    }

    // Convenience methods for specific types (optional but nice)
    pub fn add_distance_constraint_specific(&mut self, constraint: DistanceConstraint) {
        self.add_constraint(Box::new(constraint));
    }

    pub fn add_pin_joint_specific(&mut self, constraint: PinJoint) {
        self.add_constraint(Box::new(constraint));
    }

    /// Convenience method to add a static body (infinite mass/inertia).
    /// Returns the index of the added static body.
    pub fn add_static_body(&mut self, shape: Shape, position: Vec2, rotation: f64) -> usize {
        // Use new_with_inertia, providing 0.0 for mass and inertia explicitly
        let mut static_body = RigidBody::new_with_inertia(0.0, 0.0, shape);
        static_body.position = position;
        static_body.rotation = rotation;
        self.add_body(static_body)
    }

    /// Checks for collisions between all pairs of bodies.
    fn check_collisions(&mut self) {
        self.contacts.clear(); // Clear contacts from previous step

        // Naive N^2 check (can be optimized later with broadphase)
        for i in 0..self.bodies.len() {
            for j in (i + 1)..self.bodies.len() {
                let body_a = &self.bodies[i];
                let body_b = &self.bodies[j];

                // Simplified check for identical bodies
                if i == j { continue; }

                // Skip checks between two static bodies
                if body_a.inv_mass == 0.0 && body_b.inv_mass == 0.0 {
                    continue;
                }

                // Dispatch to the correct collision check based on shapes
                let manifold = match (&body_a.shape, &body_b.shape) {
                    (&Shape::Circle(ref _c_a), &Shape::Circle(ref _c_b)) => {
                        detection::check_circle_circle(body_a, i, body_b, j)
                    }
                    (&Shape::Circle(ref _c), &Shape::Line(ref _l)) => {
                        // Note the swapped indices in the call!
                        detection::check_circle_line(body_a, i, body_b, j)
                    }
                    (&Shape::Line(ref _l), &Shape::Circle(ref _c)) => {
                        // Note the swapped indices in the call!
                        detection::check_circle_line(body_b, j, body_a, i)
                    }
                    // No need to bind inner shape data if not used by the check function
                    (&Shape::Line(_), &Shape::Line(_)) => { 
                        detection::check_line_line(body_a, i, body_b, j)
                    }
                    // Polygon Collision (Not Implemented Yet)
                    (&Shape::Circle(ref _c), &Shape::Polygon(ref _p)) => {
                        detection::check_circle_polygon(body_a, i, body_b, j)
                    }
                    (&Shape::Polygon(ref _p), &Shape::Circle(ref _c)) => {
                        let manifold = detection::check_circle_polygon(body_b, j, body_a, i);
                        manifold.map(|mut m| { m.normal = -m.normal; m })
                    }
                    (&Shape::Polygon(_), &Shape::Line(_)) => None, // TODO
                    (&Shape::Line(_), &Shape::Polygon(_)) => None, // TODO
                    (&Shape::Polygon(_), &Shape::Polygon(_)) => None, // TODO
                };

                if let Some(manifold) = manifold {
                    self.contacts.push(manifold);
                }
            }
        }
    }

    /// Applies impulse-based resolution for a single contact.
    fn apply_collision_impulse(&mut self, manifold: &CollisionManifold) {
        // Get mutable references using split_at_mut to satisfy the borrow checker on stable
        let (body_a, body_b) = {
            let idx_a = manifold.body_a_idx;
            let idx_b = manifold.body_b_idx;
            if idx_a == idx_b { return; }
            if idx_a < idx_b {
                let (slice_a, slice_b) = self.bodies.split_at_mut(idx_b);
                (&mut slice_a[idx_a], &mut slice_b[0])
            } else { // idx_b < idx_a
                let (slice_b, slice_a) = self.bodies.split_at_mut(idx_a);
                (&mut slice_a[0], &mut slice_b[idx_b])
            }
        };

        let r_a = manifold.contact.point_a - body_a.position;
        let r_b = manifold.contact.point_b - body_b.position;

        let v_a_contact = body_a.linear_velocity + Vec2::new(-body_a.angular_velocity * r_a.y, body_a.angular_velocity * r_a.x);
        let v_b_contact = body_b.linear_velocity + Vec2::new(-body_b.angular_velocity * r_b.y, body_b.angular_velocity * r_b.x);
        let v_rel = v_b_contact - v_a_contact;

        let v_normal = v_rel.dot(manifold.normal);

        if v_normal > 0.0 {
            return;
        }

        let ra_cross_n = r_a.cross(manifold.normal);
        let rb_cross_n = r_b.cross(manifold.normal);

        let effective_mass_denominator = body_a.inv_mass + body_b.inv_mass +
            (ra_cross_n * ra_cross_n) * body_a.inv_inertia +
            (rb_cross_n * rb_cross_n) * body_b.inv_inertia;

        if effective_mass_denominator < EPSILON {
            return;
        }

        let j = -(1.0 + Self::RESTITUTION) * v_normal / effective_mass_denominator;

        let impulse = j * manifold.normal;

        body_a.linear_velocity -= impulse * body_a.inv_mass;
        body_a.angular_velocity -= r_a.cross(impulse) * body_a.inv_inertia;

        body_b.linear_velocity += impulse * body_b.inv_mass;
        body_b.angular_velocity += r_b.cross(impulse) * body_b.inv_inertia;
    }

    /// Applies simple positional correction to resolve penetration.
    fn apply_positional_correction(&mut self, manifold: &CollisionManifold) {
        let (body_a, body_b) = {
            let idx_a = manifold.body_a_idx;
            let idx_b = manifold.body_b_idx;
            if idx_a == idx_b { return; }
            if idx_a < idx_b {
                let (slice_a, slice_b) = self.bodies.split_at_mut(idx_b);
                (&mut slice_a[idx_a], &mut slice_b[0])
            } else { // idx_b < idx_a
                let (slice_b, slice_a) = self.bodies.split_at_mut(idx_a);
                (&mut slice_a[0], &mut slice_b[idx_b])
            }
        };

        let correction_magnitude = (manifold.depth - PhysicsWorld::POSITIONAL_CORRECTION_SLOP).max(0.0)
            / (body_a.inv_mass + body_b.inv_mass)
            * PhysicsWorld::POSITIONAL_CORRECTION_PERCENT;

        let correction = manifold.normal * correction_magnitude;

        body_a.position -= correction * body_a.inv_mass;
        body_b.position += correction * body_b.inv_mass;
    }

    /// Resolves detected collisions using simple positional correction.
    fn resolve_collisions(&mut self) {
        let contacts = self.contacts.clone();

        // 1. Apply impulses
        for manifold in &contacts {
             self.apply_collision_impulse(manifold);
        }

        // 2. Apply positional correction (stabilization)
        for manifold in &contacts {
             self.apply_positional_correction(manifold);
        }
    }

    /// Advances the simulation by one time step `dt`.
    pub fn step(&mut self, dt: f64) {
        // 1. Apply gravity
        for body in self.bodies.iter_mut() {
            if body.inv_mass > 0.0 { // Don't apply gravity to static bodies
                let gravity_force = self.gravity * body.mass;
                body.apply_force(gravity_force);
            }
        }

        // 2. Integrate motion
        for body in self.bodies.iter_mut() {
            integrator::integrate(body, dt);
        }

        // 3. Collision Detection
        self.check_collisions();

        // 4. Collision Resolution
        self.resolve_collisions(); // Resolve penetration

        // 5. Solve constraints (iteratively)
        for _ in 0..self.solver_iterations {
            for constraint in &self.constraints {
                constraint.solve_position(&mut self.bodies);
            }
        }
    }
}

impl Default for PhysicsWorld {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use crate::shapes::LineSegment;
    use super::*;
    use crate::math::vec2::Vec2;
    use crate::objects::rigid_body::RigidBody;
    use crate::constraints::{DistanceConstraint, PinJoint}; // Added Constraint
    use crate::shapes::{Shape, Circle}; // Added Shape, Circle
    const EPSILON: f64 = 1e-9;

    // Helper
    fn default_test_shape() -> Shape {
        Shape::Circle(Circle::new(1.0))
    }

    #[test]
    fn test_world_new() {
        let world = PhysicsWorld::new();
        assert!(world.bodies.is_empty());
        assert!(world.constraints.is_empty());
        assert_eq!(world.gravity, Vec2::new(0.0, -9.81));
        assert_eq!(world.solver_iterations, 8);
        assert!(world.contacts.is_empty()); // Check contacts init
    }

    #[test]
    fn test_add_body() {
        let mut world = PhysicsWorld::new();
        let body1 = RigidBody::new(1.0, default_test_shape()); // Use helper
        let body2 = RigidBody::new(2.0, default_test_shape());
        let idx1 = world.add_body(body1);
        let idx2 = world.add_body(body2);
        assert_eq!(idx1, 0);
        assert_eq!(idx2, 1);
        assert_eq!(world.bodies.len(), 2);
        assert_eq!(world.bodies[0].mass, 1.0);
        assert_eq!(world.bodies[1].mass, 2.0);
    }

    #[test]
    fn test_add_constraints() {
        let mut world = PhysicsWorld::new();
        let _idx1 = world.add_body(RigidBody::new(1.0, default_test_shape()));
        let _idx2 = world.add_body(RigidBody::new(1.0, default_test_shape()));

        let dist_constraint = DistanceConstraint::new(0, 1, Vec2::new(0.0, 0.0), Vec2::new(0.0, 0.0), 1.0);
        let pin_joint = PinJoint::new(0, 1, Vec2::new(0.5, 0.0), Vec2::new(-0.5, 0.0));

        world.add_distance_constraint_specific(dist_constraint);
        world.add_pin_joint_specific(pin_joint);

        assert_eq!(world.constraints.len(), 2);
        // We can't easily compare the Box<dyn Constraint> directly
        // We might need to add methods to the trait to get type info or specific data if needed for tests
    }

    #[test]
    fn test_step_gravity() {
        let mut world = PhysicsWorld::new();
        world.gravity = Vec2::new(0.0, -10.0);
        let idx = world.add_body(RigidBody::new(1.0, default_test_shape())); // Use helper
        let dt = 0.1;

        world.step(dt);
        assert!((world.bodies[idx].linear_velocity.x - 0.0).abs() < EPSILON);
        assert!((world.bodies[idx].linear_velocity.y - -1.0).abs() < EPSILON);
        assert!((world.bodies[idx].position.x - 0.0).abs() < EPSILON);
        assert!((world.bodies[idx].position.y - -0.1).abs() < EPSILON);
    }

    #[test]
    fn test_step_no_gravity_on_static() {
        let mut world = PhysicsWorld::new();
        world.gravity = Vec2::new(0.0, -10.0);
        let idx = world.add_body(RigidBody::new(0.0, default_test_shape())); // Static
        let initial_state = world.bodies[idx].clone();
        let dt = 0.1;

        world.step(dt);
        assert_eq!(world.bodies[idx], initial_state);
    }

    #[test]
    fn test_step_distance_constraint_solving() {
        let mut world = PhysicsWorld::new();
        world.gravity = Vec2::new(0.0, 0.0);
        world.solver_iterations = 1;
        let idx_a = world.add_body(RigidBody::new(1.0, default_test_shape()));
        let idx_b = world.add_body(RigidBody::new(1.0, default_test_shape()));
        world.bodies[idx_a].position = Vec2::new(0.0, 0.0);
        world.bodies[idx_b].position = Vec2::new(1.0, 0.0);

        let constraint = DistanceConstraint::new(idx_a, idx_b, Vec2::new(0.0,0.0), Vec2::new(0.0,0.0), 3.0);
        world.add_distance_constraint_specific(constraint);

        world.step(0.1);

        assert!((world.bodies[idx_a].position.x - -1.0).abs() < EPSILON);
        assert!((world.bodies[idx_a].position.y - 0.0).abs() < EPSILON);
        assert!((world.bodies[idx_b].position.x - 2.0).abs() < EPSILON);
        assert!((world.bodies[idx_b].position.y - 0.0).abs() < EPSILON);
    }

    #[test]
    fn test_step_pin_joint_solving() {
        let mut world = PhysicsWorld::new();
        world.gravity = Vec2::new(0.0, 0.0);
        world.solver_iterations = 1;
        let idx_a = world.add_body(RigidBody::new(1.0, default_test_shape()));
        let idx_b = world.add_body(RigidBody::new(1.0, default_test_shape()));
        world.bodies[idx_a].position = Vec2::new(0.0, 0.0);
        world.bodies[idx_b].position = Vec2::new(4.0, 0.0);

        let constraint = PinJoint::new(idx_a, idx_b, Vec2::new(0.0, 0.0), Vec2::new(0.0, 0.0));
        world.add_pin_joint_specific(constraint);

        world.step(0.1);

        // Expected: body 0 moves to (2,0), body 1 moves to (2,0)
        assert!((world.bodies[idx_a].position.x - 2.0).abs() < EPSILON);
        assert!((world.bodies[idx_a].position.y - 0.0).abs() < EPSILON);
        assert!((world.bodies[idx_b].position.x - 2.0).abs() < EPSILON);
        assert!((world.bodies[idx_b].position.y - 0.0).abs() < EPSILON);
    }

    #[test]
    fn test_step_distance_constraint_with_gravity() {
        let mut world = PhysicsWorld::new();
        world.gravity = Vec2::new(0.0, -10.0);
        world.solver_iterations = 10;
        let idx_static = world.add_body(RigidBody::new(0.0, default_test_shape())); // Static
        let idx_bob = world.add_body(RigidBody::new(1.0, default_test_shape()));
        world.bodies[idx_static].position = Vec2::new(0.0, 0.0);
        world.bodies[idx_bob].position = Vec2::new(2.0, 0.0);

        let target_dist = 2.0;
        let constraint = DistanceConstraint::new(idx_static, idx_bob, Vec2::new(0.0,0.0), Vec2::new(0.0,0.0), target_dist);
        world.add_distance_constraint_specific(constraint);

        let dt = 0.016;
        for _ in 0..5 {
             world.step(dt);
        }

        let bob_pos = world.bodies[idx_bob].position;
        let static_pos = world.bodies[idx_static].position;
        assert!(bob_pos.y < 0.0);
        let current_dist = bob_pos.distance(static_pos);
        assert!((current_dist - target_dist).abs() < 0.01, "Distance deviation: {}", (current_dist - target_dist).abs());
        assert_eq!(static_pos, Vec2::new(0.0, 0.0));
    }

    #[test]
    fn test_check_collisions() {
        let mut world = PhysicsWorld::new();
        let shape_a = Shape::Circle(Circle::new(1.0));
        let shape_b = Shape::Circle(Circle::new(1.0));
        let shape_c = Shape::Circle(Circle::new(0.5));

        let idx_a = world.add_body(RigidBody::new(1.0, shape_a)); // Circle at (0,0)
        let idx_b = world.add_body(RigidBody::new(1.0, shape_b)); // Circle at (1.5,0) - Collides with A
        let idx_c = world.add_body(RigidBody::new(1.0, shape_c)); // Circle at (5,0) - No collision

        world.bodies[idx_b].position = Vec2::new(1.5, 0.0);
        world.bodies[idx_c].position = Vec2::new(5.0, 0.0);

        // Call the internal check directly for this test
        world.check_collisions();

        assert_eq!(world.contacts.len(), 1);
        let manifold = world.contacts[0];
        // Check if the correct bodies are involved (order might vary)
        assert!(
            (manifold.body_a_idx == idx_a && manifold.body_b_idx == idx_b) ||
            (manifold.body_a_idx == idx_b && manifold.body_b_idx == idx_a)
        );
        // Basic check on depth (should be positive)
        assert!(manifold.depth > 0.0);
    }

    #[test]
    fn test_resolve_collisions_positional_correction_basic() {
        let mut world = PhysicsWorld::new();
        let shape = Shape::Circle(Circle::new(1.0));
        let idx_a = world.add_body(RigidBody::new(1.0, shape.clone())); // mass = 1 -> inv_mass = 1
        let idx_b = world.add_body(RigidBody::new(1.0, shape.clone())); // mass = 1 -> inv_mass = 1

        // Place them overlapping
        world.bodies[idx_a].position = Vec2::new(0.0, 0.0);
        world.bodies[idx_b].position = Vec2::new(1.5, 0.0);
        let initial_pos_a = world.bodies[idx_a].position;
        let initial_pos_b = world.bodies[idx_b].position;

        // Manually create a collision manifold for testing resolution
        let radii_sum = 1.0 + 1.0;
        let distance = 1.5;
        let depth = radii_sum - distance; // 0.5
        let normal = (initial_pos_b - initial_pos_a).normalize(); // (1, 0)
        let manifold = CollisionManifold {
            body_a_idx: idx_a,
            body_b_idx: idx_b,
            normal,
            depth,
            contact: Default::default(), // Contact point not needed for pure positional correction
        };

        // Manually call the positional correction function
        world.apply_positional_correction(&manifold);

        // Calculate expected correction based on the new logic
        let total_inv_mass = world.bodies[idx_a].inv_mass + world.bodies[idx_b].inv_mass;
        let correction_magnitude = (manifold.depth - PhysicsWorld::POSITIONAL_CORRECTION_SLOP).max(0.0)
            / total_inv_mass
            * PhysicsWorld::POSITIONAL_CORRECTION_PERCENT;

        let correction_vector = manifold.normal * correction_magnitude;

        let expected_pos_a = initial_pos_a - correction_vector * world.bodies[idx_a].inv_mass;
        let expected_pos_b = initial_pos_b + correction_vector * world.bodies[idx_b].inv_mass;

        assert!((world.bodies[idx_a].position - expected_pos_a).magnitude_squared() < EPSILON * EPSILON,
                "Body A final pos: {:?}, expected: {:?}", world.bodies[idx_a].position, expected_pos_a);
        assert!((world.bodies[idx_b].position - expected_pos_b).magnitude_squared() < EPSILON * EPSILON,
                "Body B final pos: {:?}, expected: {:?}", world.bodies[idx_b].position, expected_pos_b);

        // Check they are still overlapping, but less so (since correction is partial)
        let final_dist = world.bodies[idx_a].position.distance(world.bodies[idx_b].position);
        // Expected final distance = 1.549 - (-0.049) = 1.598
        assert!((final_dist - 1.598).abs() < EPSILON, "Final distance: {}", final_dist);
        assert!(final_dist < radii_sum); // Should still overlap unless percent=1 and slop=0
    }

    #[test]
    fn test_add_static_body() {
        let mut world = PhysicsWorld::new();
        let shape = Shape::Circle(Circle::new(10.0));
        let position = Vec2::new(5.0, -10.0);
        let rotation = 0.5;

        let idx = world.add_static_body(shape.clone(), position, rotation);

        assert_eq!(idx, 0);
        assert_eq!(world.bodies.len(), 1);
        let body = &world.bodies[0];
        assert_eq!(body.mass, 0.0);
        assert_eq!(body.inv_mass, 0.0);
        assert!(body.inertia.is_infinite(), "Static body inertia should be infinite");
        assert_eq!(body.inv_inertia, 0.0);
        assert_eq!(body.shape, shape);
        assert_eq!(body.position, position);
        assert_eq!(body.rotation, rotation);
    }

    #[test]
    fn test_world_line_line_collision() {
        let mut world = PhysicsWorld::new();
        world.gravity = Vec2::new(0.0, 0.0); // Disable gravity for precise intersection test

        let line_shape_a = Shape::Line(LineSegment::new(Vec2::new(-1.0, -1.0), Vec2::new(1.0, 1.0)));
        let line_shape_b = Shape::Line(LineSegment::new(Vec2::new(-1.0, 1.0), Vec2::new(1.0, -1.0)));

        let body_a = RigidBody{ position: Vec2::new(0.0, 0.0), ..RigidBody::new(1.0, line_shape_a) };
        let body_b = RigidBody{ position: Vec2::new(0.0, 0.0), ..RigidBody::new(1.0, line_shape_b) };

        let idx_a = world.add_body(body_a);
        let idx_b = world.add_body(body_b);

        world.step(0.01); // Step to trigger collision detection

        assert_eq!(world.contacts.len(), 1);
        let manifold = &world.contacts[0];

        // Check indices (order might vary)
        assert!((manifold.body_a_idx == idx_a && manifold.body_b_idx == idx_b) ||
                (manifold.body_a_idx == idx_b && manifold.body_b_idx == idx_a));

        // Check intersection point (should be 0,0)
        assert!((manifold.contact.point_a.x - 0.0).abs() < EPSILON);
        assert!((manifold.contact.point_a.y - 0.0).abs() < EPSILON);
        assert!((manifold.contact.point_b.x - 0.0).abs() < EPSILON);
        assert!((manifold.contact.point_b.y - 0.0).abs() < EPSILON);

        // Check depth
        assert!((manifold.depth - 0.0).abs() < EPSILON);
    }

    #[test]
    fn test_world_line_line_no_collision() {
        let mut world = PhysicsWorld::new();

        let line_shape_a = Shape::Line(LineSegment::new(Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0))); // Horizontal line at y=0
        let line_shape_b = Shape::Line(LineSegment::new(Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0))); // Horizontal line at y=1

        let body_a = RigidBody{ position: Vec2::new(0.0, 0.0), ..RigidBody::new(1.0, line_shape_a) };
        let body_b = RigidBody{ position: Vec2::new(0.0, 1.0), ..RigidBody::new(1.0, line_shape_b) };

        world.add_body(body_a);
        world.add_body(body_b);

        world.step(0.01); // Step to trigger collision detection

        assert!(world.contacts.is_empty());
    }

    #[test]
    fn test_resolve_collisions_impulse_head_on() {
        let mut world = PhysicsWorld::new();
        let shape = Shape::Circle(Circle::new(1.0)); // Radius 1
        let idx_a = world.add_body(RigidBody::new(1.0, shape.clone())); // Mass 1, inv_mass 1
        let idx_b = world.add_body(RigidBody::new(1.0, shape.clone())); // Mass 1, inv_mass 1

        // Setup: Body A moving right, Body B moving left, about to collide at origin
        world.bodies[idx_a].position = Vec2::new(-1.0 - EPSILON, 0.0);
        world.bodies[idx_b].position = Vec2::new( 1.0 + EPSILON, 0.0);
        world.bodies[idx_a].linear_velocity = Vec2::new(10.0, 0.0);
        world.bodies[idx_b].linear_velocity = Vec2::new(-10.0, 0.0);
        world.bodies[idx_a].angular_velocity = 0.0;
        world.bodies[idx_b].angular_velocity = 0.0;

        // Manually create contact at the point of impact (origin)
        let manifold = CollisionManifold {
            body_a_idx: idx_a,
            body_b_idx: idx_b,
            normal: Vec2::new(1.0, 0.0), // Normal pointing from A to B
            depth: 0.0, // Assume just touching for impulse test
            contact: crate::collision::manifold::ContactPoint { point_a: Vec2::new(0.0, 0.0), point_b: Vec2::new(0.0, 0.0) }
        };

        // Apply the impulse directly
        world.apply_collision_impulse(&manifold);

        // Calculate expected velocities after impulse
        // v_a = 10, v_b = -10
        // v_rel = v_b - v_a = -10 - 10 = -20
        // normal = (1, 0)
        // v_normal = v_rel . normal = -20
        // r_a = contact - pos_a = (0,0) - (-1,0) = (1,0)  <- Mistake: position is -1-eps, contact is 0. r_a = 0 - (-1) = 1 approx.
        // r_b = contact - pos_b = (0,0) - (1,0) = (-1,0) <- Mistake: position is 1+eps, contact is 0. r_b = 0 - 1 = -1 approx.
        // Let's re-evaluate r_a and r_b based on manifold contact point and body positions.
        // In a real collision, contact point might not be exactly 0,0 if bodies interpenetrate slightly before step resolves.
        // However, for this test, we defined contact point as (0,0). So r_a = (0,0) - (-1,0) = (1,0) and r_b = (0,0) - (1,0) = (-1,0) is correct for the test setup.

        // ra_cross_n = (1,0) x (1,0) = 0
        // rb_cross_n = (-1,0) x (1,0) = 0
        // effective_mass_denom = inv_mass_a + inv_mass_b + 0 + 0 = 1 + 1 = 2
        // j = -(1 + RESTITUTION) * v_normal / effective_mass_denom
        // j = -(1 + 0.1) * (-20) / 2 = -1.1 * (-20) / 2 = 22 / 2 = 11
        // impulse_vec = j * normal = 11 * (1, 0) = (11, 0)
        // final_v_a = initial_v_a - impulse_vec * inv_mass_a = (10, 0) - (11, 0) * 1 = (-1, 0)
        // final_v_b = initial_v_b + impulse_vec * inv_mass_b = (-10, 0) + (11, 0) * 1 = (1, 0)

        let expected_v_a = Vec2::new(-1.0, 0.0); // -1 = 10 - 11
        let expected_v_b = Vec2::new(1.0, 0.0);  //  1 = -10 + 11

        assert!((world.bodies[idx_a].linear_velocity - expected_v_a).magnitude_squared() < EPSILON * EPSILON,
                "Body A final vel: {:?}, expected: {:?}", world.bodies[idx_a].linear_velocity, expected_v_a);
        assert!((world.bodies[idx_b].linear_velocity - expected_v_b).magnitude_squared() < EPSILON * EPSILON,
                "Body B final vel: {:?}, expected: {:?}", world.bodies[idx_b].linear_velocity, expected_v_b);

        // Angular velocities should remain 0 for central collision
        assert!(world.bodies[idx_a].angular_velocity.abs() < EPSILON);
        assert!(world.bodies[idx_b].angular_velocity.abs() < EPSILON);
    }
}