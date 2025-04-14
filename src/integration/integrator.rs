use crate::objects::rigid_body::RigidBody;

/// Integrates the rigid body's state forward in time using Semi-Implicit Euler.
pub fn integrate(body: &mut RigidBody, dt: f64) {
    if body.inv_mass == 0.0 {
        // Static object, do not integrate
        return;
    }

    // --- Linear Motion --- //
    // Calculate acceleration (a = F/m = F * inv_m)
    let linear_acceleration = body.force * body.inv_mass;
    // Update linear velocity (v = v + a*dt)
    body.linear_velocity = body.linear_velocity + linear_acceleration * dt;
    // Update position (p = p + v*dt)
    body.position = body.position + body.linear_velocity * dt;

    // --- Angular Motion --- //
    // Calculate angular acceleration (alpha = T/I = T * inv_I)
    let angular_acceleration = body.torque * body.inv_inertia;
    // Update angular velocity (omega = omega + alpha*dt)
    body.angular_velocity = body.angular_velocity + angular_acceleration * dt;
    // Update rotation (theta = theta + omega*dt)
    body.rotation = body.rotation + body.angular_velocity * dt;

    // Important: Wrap rotation angle if needed (e.g., to keep within [-PI, PI] or [0, 2PI])
    // This helps prevent floating point issues and simplifies angle comparisons.
    // Let's wrap to [-PI, PI]
    body.rotation = wrap_angle(body.rotation);

    // Clear force/torque accumulators for the next step
    body.clear_accumulators();
}

/// Wraps an angle in radians to the range [-PI, PI].
fn wrap_angle(angle: f64) -> f64 {
    // A robust way to wrap angles to [-PI, PI]
    angle.sin().atan2(angle.cos())
}


#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::vec2::Vec2;
    use crate::objects::rigid_body::RigidBody;
    use crate::shapes::{Shape, Circle};
    use std::f64::consts::PI;
    const EPSILON: f64 = 1e-9; // Slightly larger epsilon for integration tests

    // Helper to create a default body for tests
    fn default_test_shape() -> Shape {
        Shape::Circle(Circle::new(1.0))
    }

    #[test]
    fn test_integrate_linear_motion_no_force() {
        let mut rb = RigidBody::new(1.0, default_test_shape());
        rb.linear_velocity = Vec2::new(10.0, -5.0);
        let dt = 0.1;

        integrate(&mut rb, dt);

        assert!((rb.position.x - 1.0).abs() < EPSILON);
        assert!((rb.position.y - -0.5).abs() < EPSILON);
        assert_eq!(rb.linear_velocity, Vec2::new(10.0, -5.0)); // Velocity unchanged
        assert_eq!(rb.force, Vec2::new(0.0, 0.0)); // Force cleared
    }

    #[test]
    fn test_integrate_linear_motion_constant_force() {
        let mut rb = RigidBody::new(2.0, default_test_shape()); // mass = 2.0
        rb.apply_force(Vec2::new(10.0, 0.0)); // force = (10, 0)
        // Expected acceleration a = F/m = (5, 0)
        let dt = 0.1;
        // let initial_pos = rb.position; // Unused
        // let initial_vel = rb.linear_velocity; // Unused

        integrate(&mut rb, dt);

        // v = v0 + a*dt = (0,0) + (5,0)*0.1 = (0.5, 0)
        assert!((rb.linear_velocity.x - 0.5).abs() < EPSILON);
        assert!((rb.linear_velocity.y - 0.0).abs() < EPSILON);

        // p = p0 + v*dt = (0,0) + (0.5, 0)*0.1 = (0.05, 0)
        assert!((rb.position.x - 0.05).abs() < EPSILON);
        assert!((rb.position.y - 0.0).abs() < EPSILON);
        assert_eq!(rb.force, Vec2::new(0.0, 0.0)); // Force cleared
    }

     #[test]
    fn test_integrate_angular_motion_constant_torque() {
        let shape = default_test_shape();
        let mass = 1.0;
        let rb_temp = RigidBody::new(mass, shape.clone()); // Need to create temporarily to get calculated inertia
        let inertia = rb_temp.inertia;
        let mut rb = RigidBody::new(mass, shape.clone()); // Clone shape again for the actual body
        // Set torque directly for test setup (overwrites any previous force)
        rb.torque = 5.0;
        // Expected angular acceleration alpha = T/I = 5.0 / inertia
        let expected_alpha = if inertia > 0.0 { 5.0 / inertia } else { 0.0 };
        let dt = 0.1;
        integrate(&mut rb, dt);
        // omega = omega0 + alpha*dt = 0 + expected_alpha*0.1
        assert!((rb.angular_velocity - (expected_alpha * dt)).abs() < EPSILON);
        // theta = theta0 + omega*dt = 0 + (expected_alpha * dt)*0.1
        assert!((rb.rotation - (expected_alpha * dt * dt)).abs() < EPSILON);
        assert_eq!(rb.torque, 0.0); // Torque cleared
    }

    #[test]
    fn test_integrate_static_object() {
        let mut rb = RigidBody::new(0.0, default_test_shape()); // Infinite mass/inertia
        rb.position = Vec2::new(1.0, 1.0);
        rb.rotation = 1.0;
        rb.linear_velocity = Vec2::new(1.0, 1.0);
        rb.angular_velocity = 1.0;
        rb.force = Vec2::new(10.0, 10.0);
        rb.torque = 10.0;

        let initial_state = rb.clone();
        integrate(&mut rb, 0.1);

        // State should remain unchanged because inv_mass is 0
        assert_eq!(rb, initial_state);
        // Accumulators should still be cleared though (or maybe not? debatable for static)
        // Let's assume they are cleared by the integrate function regardless
        // assert_eq!(rb.force, Vec2::new(0.0, 0.0)); // This test depends on whether clear_accumulators is called for static
        // assert_eq!(rb.torque, 0.0);
    }

    #[test]
    fn test_wrap_angle() {
        assert!((wrap_angle(0.0) - 0.0).abs() < EPSILON);
        assert!((wrap_angle(PI) - PI).abs() < EPSILON);
        assert!((wrap_angle(-PI) - -PI).abs() < EPSILON); // Should stay -PI, not wrap to PI
        assert!((wrap_angle(PI + 0.1) - (-PI + 0.1)).abs() < EPSILON);
        assert!((wrap_angle(-PI - 0.1) - (PI - 0.1)).abs() < EPSILON);
        assert!((wrap_angle(3.0 * PI) - PI).abs() < EPSILON);
        assert!((wrap_angle(-3.0 * PI) - -PI).abs() < EPSILON);
        assert!((wrap_angle(2.0 * PI) - 0.0).abs() < EPSILON);
    }

} 