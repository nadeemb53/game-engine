use crate::objects::rigid_body::RigidBody;

pub mod distance_constraint;
pub mod pin_joint;

// Re-export the constraint types for easier access
pub use distance_constraint::DistanceConstraint;
pub use pin_joint::PinJoint;

/// A trait representing a physics constraint.
pub trait Constraint {
    /// Solves the constraint by adjusting body positions.
    /// Takes a mutable slice of all bodies in the world.
    fn solve_position(&self, bodies: &mut [RigidBody]);

    // We could add methods for solve_velocity, getters for body indices etc. later
}

/// Helper to safely get mutable references to two potentially different bodies in a slice.
/// Panics if indices are the same or out of bounds.
pub(crate) fn get_mutable_body_pair(bodies: &mut [RigidBody], idx_a: usize, idx_b: usize) -> (&mut RigidBody, &mut RigidBody) {
    if idx_a == idx_b {
        panic!("Constraint cannot connect a body to itself with different anchors using indices.");
    }
    if idx_a >= bodies.len() || idx_b >= bodies.len() {
         panic!("Body index out of bounds");
    }

    // Ensure a < b for split_at_mut
    if idx_a < idx_b {
        let (slice_a, slice_b) = bodies.split_at_mut(idx_b);
        (&mut slice_a[idx_a], &mut slice_b[0])
    } else { // idx_b < idx_a
        let (slice_b, slice_a) = bodies.split_at_mut(idx_a);
        (&mut slice_a[0], &mut slice_b[idx_b])
    }
} 