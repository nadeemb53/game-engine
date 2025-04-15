use crate::math::vec2::Vec2;
use crate::shapes::Shape;
use crate::common::Material;
use crate::collision::AABB; // Import AABB

#[derive(Debug, Clone, PartialEq)] // Removed Copy
pub struct RigidBody {
    // Geometry
    pub shape: Shape,
    pub local_center_of_mass: Vec2, // Offset from shape's origin to center of mass
    pub material: Material, // Physical material properties

    // Primary state
    pub position: Vec2, // World position of the center of mass
    pub rotation: f64, // Radians
    pub linear_velocity: Vec2,
    pub angular_velocity: f64, // Radians per second

    // Accumulators for forces/torques applied during a time step
    pub force: Vec2,
    pub torque: f64,

    // Physical properties
    pub mass: f64,        // Kilograms?
    pub inv_mass: f64,    // 1.0 / mass (0.0 for static)
    pub inertia: f64,     // Moment of inertia about the center of mass (kg*m^2?)
    pub inv_inertia: f64, // 1.0 / inertia (0.0 for static / infinite inertia)
}

impl RigidBody {
    /// Creates a new RigidBody with the given mass (for Circle/Line) or density (for Polygon)
    /// and shape. Calculates inverse mass, inertia, and inverse inertia based on shape properties.
    /// If mass/density is <= 0.0, it creates a static body.
    /// The initial world position of the center of mass is set to (0,0).
    pub fn new(mass_or_density: f64, shape: Shape) -> Self {
        let (inv_mass, inertia, inv_inertia, local_com) = if mass_or_density <= 0.0 {
            // Treat mass <= 0 as infinite mass (static object)
            (0.0, f64::INFINITY, 0.0, Vec2::ZERO)
        } else {
            match &shape {
                Shape::Circle(circle) => {
                    let mass = mass_or_density;
                    let inv_mass = 1.0 / mass;
                    let inertia = 0.5 * mass * circle.radius * circle.radius;
                    let inv_inertia = if inertia > 0.0 { 1.0 / inertia } else { 0.0 };
                    (inv_mass, inertia, inv_inertia, Vec2::ZERO)
                }
                Shape::Line(segment) => {
                    let mass = mass_or_density;
                    let inv_mass = 1.0 / mass;
                    let length = segment.length();
                    let inertia = (1.0 / 12.0) * mass * length * length;
                    let inv_inertia = if inertia > 0.0 { 1.0 / inertia } else { 0.0 };
                    let local_com = (segment.a + segment.b) / 2.0;
                    (inv_mass, inertia, inv_inertia, local_com)
                }
                Shape::Polygon(polygon) => {
                    let density = mass_or_density;
                    if density <= 0.0 {
                         (0.0, f64::INFINITY, 0.0, Vec2::ZERO)
                    } else {
                        let area = polygon.calculate_area();
                        let mass = area * density;
                        if mass < 1e-10 {
                             (0.0, f64::INFINITY, 0.0, Vec2::ZERO)
                        } else {
                            let inv_mass = 1.0 / mass;
                            let inertia = polygon.calculate_inertia(Some(density));
                            let inv_inertia = if inertia.is_finite() && inertia > 0.0 { 1.0 / inertia } else { 0.0 };
                            let local_com = polygon.calculate_centroid();
                            (inv_mass, inertia, inv_inertia, local_com)
                        }
                    }
                }
            }
        };

        let initial_position = -local_com;

        let mass_value = if inv_mass > 0.0 { 1.0 / inv_mass } else { 0.0 };

        Self {
            shape,
            local_center_of_mass: local_com,
            material: Material::default(),
            position: initial_position,
            rotation: 0.0,
            linear_velocity: Vec2::ZERO,
            angular_velocity: 0.0,
            force: Vec2::ZERO,
            torque: 0.0,
            mass: mass_value,
            inv_mass,
            inertia,
            inv_inertia,
        }
    }

    /// Creates a new static RigidBody with the given shape.
    /// The position argument defines the world location of the shape's local origin,
    /// and the body's `position` field will store the calculated world position of the center of mass.
    pub fn new_static(shape: Shape, position: Vec2, rotation: f64) -> Self {
         let local_com = match &shape {
             Shape::Circle(_) => Vec2::ZERO,
             Shape::Line(segment) => (segment.a + segment.b) / 2.0,
             Shape::Polygon(polygon) => polygon.calculate_centroid(),
         };
         let world_com = position + local_com.rotate(rotation);

        Self {
            shape,
            local_center_of_mass: local_com,
            material: Material::default(),
            position: world_com,
            rotation,
            linear_velocity: Vec2::ZERO,
            angular_velocity: 0.0,
            force: Vec2::ZERO,
            torque: 0.0,
            mass: 0.0,
            inv_mass: 0.0,
            inertia: f64::INFINITY,
            inv_inertia: 0.0,
        }
    }

    /// Creates a new dynamic RigidBody with explicitly provided mass and inertia.
    /// The shape's geometry center is assumed to be the center of mass.
    /// NOTE: This may be less physically accurate for complex shapes if inertia doesn't match geometry.
    /// Panics if used with a Polygon shape, as properties should be derived.
    pub fn new_with_inertia(mass: f64, inertia: f64, shape: Shape) -> Self {
         let (inv_mass, final_inertia, inv_inertia) = if mass > 0.0 {
            let inv_inertia_val = if inertia > 0.0 { 1.0 / inertia } else { 0.0 };
            (1.0 / mass, inertia, inv_inertia_val)
        } else {
            // Treat 0 mass as static
            (0.0, f64::INFINITY, 0.0)
        };

        let local_com = match shape {
            Shape::Circle(_) | Shape::Line(_) => Vec2::ZERO,
            Shape::Polygon(_) => {
                 panic!("RigidBody::new_with_inertia is not intended for direct use with Polygon shapes.");
            }
        };

         Self {
            shape,
            local_center_of_mass: local_com,
            material: Material::default(),
            position: Vec2::ZERO,
            rotation: 0.0,
            linear_velocity: Vec2::ZERO,
            angular_velocity: 0.0,
            force: Vec2::ZERO,
            torque: 0.0,
            mass,
            inv_mass,
            inertia: final_inertia,
            inv_inertia,
        }
    }

    /// Calculates the world-space Axis-Aligned Bounding Box (AABB) for this body.
    pub fn calculate_aabb(&self) -> AABB {
        match &self.shape {
            Shape::Circle(circle) => {
                // AABB for a circle is simply the center +/- radius
                let radius_vec = Vec2::new(circle.radius, circle.radius);
                AABB::new(self.position - radius_vec, self.position + radius_vec)
            }
            Shape::Line(segment) => {
                // Transform endpoints to world space
                let world_a = self.position + (segment.a - self.local_center_of_mass).rotate(self.rotation);
                let world_b = self.position + (segment.b - self.local_center_of_mass).rotate(self.rotation);
                // AABB::from_points handles min/max correctly
                AABB::from_points(&[world_a, world_b]).unwrap_or_else(|| {
                    // Fallback if line is degenerate (shouldn't happen ideally)
                    AABB::new(self.position, self.position)
                })
            }
            Shape::Polygon(polygon) => {
                // Transform all vertices to world space and find min/max
                let world_shape_origin = self.position - self.local_center_of_mass.rotate(self.rotation);
                let world_vertices: Vec<Vec2> = polygon.vertices.iter()
                    .map(|&v| world_shape_origin + v.rotate(self.rotation))
                    .collect();

                AABB::from_points(&world_vertices).unwrap_or_else(|| {
                     // Fallback if polygon is degenerate
                     AABB::new(self.position, self.position)
                })
            }
        }
    }

    /// Applies a force at the center of mass.
    pub fn apply_force(&mut self, force: Vec2) {
        self.force += force;
    }

    /// Applies a force at a specific point (in world coordinates).
    /// This generates both linear force and torque.
    pub fn apply_force_at_point(&mut self, force: Vec2, point_world: Vec2) {
        self.force += force;
        let radius_vector = point_world - self.position;
        self.torque += radius_vector.cross(force);
    }

    /// Should typically be called after integration in each simulation step.
    pub fn clear_accumulators(&mut self) {
        self.force = Vec2::ZERO;
        self.torque = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shapes::{Circle, LineSegment, Polygon}; // Added Polygon
    const EPSILON: f64 = 1e-10;

    #[test]
    fn test_rigidbody_new_circle() {
        let mass = 10.0;
        let radius = 2.0;
        let shape = Shape::Circle(Circle::new(radius));
        let expected_inertia = 0.5 * mass * radius * radius; // 20
        let rb = RigidBody::new(mass, shape.clone());

        assert_eq!(rb.mass, mass);
        assert!((rb.inv_mass - (1.0 / mass)).abs() < EPSILON);
        assert_eq!(rb.shape, shape);
        assert!((rb.inertia - expected_inertia).abs() < EPSILON);
        assert!((rb.inv_inertia - (1.0 / expected_inertia)).abs() < EPSILON);
        assert_eq!(rb.local_center_of_mass, Vec2::ZERO);
        assert_eq!(rb.position, Vec2::ZERO); // Should start at origin (CoM)
        assert_eq!(rb.rotation, 0.0);
        assert_eq!(rb.linear_velocity, Vec2::ZERO);
        assert_eq!(rb.angular_velocity, 0.0);
        assert_eq!(rb.force, Vec2::ZERO);
        assert_eq!(rb.torque, 0.0);
    }

    #[test]
    fn test_rigidbody_new_line() {
        let mass = 12.0;
        // Define line centered at origin (-2,0) to (2,0)
        let line = LineSegment::new(Vec2::new(-2.0, 0.0), Vec2::new(2.0, 0.0));
        let length_sq = line.length_squared(); // 16
        let shape = Shape::Line(line.clone());
        let expected_inertia = (1.0 / 12.0) * mass * length_sq; // 16
        let rb = RigidBody::new(mass, shape.clone());

        assert_eq!(rb.mass, mass);
        assert!((rb.inv_mass - (1.0 / mass)).abs() < EPSILON);
        assert_eq!(rb.shape, shape);
        assert!((rb.inertia - expected_inertia).abs() < EPSILON);
        assert!((rb.inv_inertia - (1.0 / expected_inertia)).abs() < EPSILON);
        // Line (-2,0) to (2,0), midpoint is (0,0)
        assert_eq!(rb.local_center_of_mass, Vec2::ZERO);
        assert_eq!(rb.position, Vec2::ZERO); // CoM starts at origin
    }

     #[test]
    fn test_rigidbody_new_line_offset() {
        let mass = 1.0;
        // Line from (1,1) to (3,1)
        let line = LineSegment::new(Vec2::new(1.0, 1.0), Vec2::new(3.0, 1.0));
        let length_sq = line.length_squared(); // (2^2) = 4
        let shape = Shape::Line(line.clone());
        let expected_inertia = (1.0 / 12.0) * mass * length_sq; // 1 * 4 / 12 = 1/3
        let expected_local_com = Vec2::new(2.0, 1.0); // Midpoint of (1,1) and (3,1)

        let rb = RigidBody::new(mass, shape.clone());

        assert_eq!(rb.mass, mass);
        assert!((rb.inv_mass - (1.0 / mass)).abs() < EPSILON);
        assert_eq!(rb.shape, shape);
        assert!((rb.inertia - expected_inertia).abs() < EPSILON);
        assert!((rb.inv_inertia - (1.0 / expected_inertia)).abs() < EPSILON);
        assert_eq!(rb.local_center_of_mass, expected_local_com);
        // Position should be negative of local_com to place CoM at origin
        assert_eq!(rb.position, -expected_local_com);
    }

    #[test]
    fn test_rigidbody_new_zero_mass() {
        let shape = Shape::Circle(Circle::new(1.0));
        let rb = RigidBody::new(0.0, shape); // Static body via mass=0
        assert_eq!(rb.mass, 0.0);
        assert_eq!(rb.inv_mass, 0.0);
        assert!(rb.inertia.is_infinite());
        assert_eq!(rb.inv_inertia, 0.0);
         assert_eq!(rb.local_center_of_mass, Vec2::ZERO);
        assert_eq!(rb.position, Vec2::ZERO);
    }

     #[test]
    fn test_rigidbody_new_static() {
        let shape = Shape::Circle(Circle::new(1.0));
        let pos = Vec2::new(1.0, 2.0);
        let rot = 0.5;
        let rb = RigidBody::new_static(shape.clone(), pos, rot);
        assert_eq!(rb.mass, 0.0);
        assert_eq!(rb.inv_mass, 0.0);
        assert!(rb.inertia.is_infinite());
        assert_eq!(rb.inv_inertia, 0.0);
        assert_eq!(rb.shape, shape);
        assert_eq!(rb.local_center_of_mass, Vec2::ZERO); // For circle
        assert_eq!(rb.position, pos); // For circle, CoM = shape origin
        assert_eq!(rb.rotation, rot);
    }

     #[test]
    fn test_rigidbody_new_static_polygon() {
        // Square offset from origin
        let offset = Vec2::new(10.0, -5.0);
        let vertices = vec![
             offset + Vec2::new(0.0, 0.0),
             offset + Vec2::new(1.0, 0.0),
             offset + Vec2::new(1.0, 1.0),
             offset + Vec2::new(0.0, 1.0),
         ];
        let shape = Shape::Polygon(Polygon::new(vertices.clone()));
        let shape_origin_pos = Vec2::new(100.0, 200.0); // Where we place the shape's reference point (vertex 0 implicitly)
        let rot = 0.0; // No rotation for simplicity

        let rb = RigidBody::new_static(shape.clone(), shape_origin_pos, rot);

        let expected_local_com = offset + Vec2::new(0.5, 0.5); // Centroid relative to shape's internal coords
        // World CoM = shape_origin_pos + rotated local_com
        let expected_world_com = shape_origin_pos + expected_local_com.rotate(rot);

        assert_eq!(rb.mass, 0.0);
        assert_eq!(rb.inv_mass, 0.0);
        assert!(rb.inertia.is_infinite());
        assert_eq!(rb.inv_inertia, 0.0);
        assert_eq!(rb.shape, shape);
        assert_eq!(rb.local_center_of_mass, expected_local_com);
        assert_eq!(rb.position, expected_world_com); // Position field stores world CoM
        assert_eq!(rb.rotation, rot);
    }


     #[test]
    fn test_rigidbody_new_polygon() {
        // Unit square centered at origin
        let vertices = vec![
             Vec2::new(-0.5, -0.5), Vec2::new( 0.5, -0.5),
             Vec2::new( 0.5,  0.5), Vec2::new(-0.5,  0.5),
         ];
        let shape = Shape::Polygon(Polygon::new(vertices.clone()));
        let density = 2.0;
        let rb = RigidBody::new(density, shape.clone());

        let expected_area = 1.0;
        let expected_mass = expected_area * density; // 2.0
        let expected_inertia = expected_mass * (1.0f64.powi(2) + 1.0f64.powi(2)) / 12.0; // 1/3

        assert!((rb.mass - expected_mass).abs() < EPSILON);
        assert!((rb.inv_mass - (1.0 / expected_mass)).abs() < EPSILON);
        assert_eq!(rb.shape, shape);
        assert!((rb.inertia - expected_inertia).abs() < EPSILON);
        assert!((rb.inv_inertia - (1.0 / expected_inertia)).abs() < EPSILON);
        assert_eq!(rb.local_center_of_mass, Vec2::ZERO); // Centroid at (0,0)
        assert_eq!(rb.position, Vec2::ZERO); // CoM starts at origin
    }

     #[test]
    fn test_rigidbody_new_polygon_offset() {
        // Unit square offset from origin
        let offset = Vec2::new(10.0, -5.0);
        let vertices = vec![
             offset + Vec2::new(0.0, 0.0), offset + Vec2::new(1.0, 0.0),
             offset + Vec2::new(1.0, 1.0), offset + Vec2::new(0.0, 1.0),
         ];
        let shape = Shape::Polygon(Polygon::new(vertices.clone()));
        let density = 1.0;
        let rb = RigidBody::new(density, shape.clone());

        let expected_area = 1.0;
        let expected_mass = expected_area * density; // 1.0
        let expected_inertia = expected_mass * (1.0f64.powi(2) + 1.0f64.powi(2)) / 12.0; // 1/6
        let expected_local_com = offset + Vec2::new(0.5, 0.5);

        assert!((rb.mass - expected_mass).abs() < EPSILON);
        assert!((rb.inv_mass - (1.0 / expected_mass)).abs() < EPSILON);
        assert_eq!(rb.shape, shape);
        assert!((rb.inertia - expected_inertia).abs() < EPSILON);
        assert!((rb.inv_inertia - (1.0 / expected_inertia)).abs() < EPSILON);
        assert_eq!(rb.local_center_of_mass, expected_local_com);
         // Position should be negative of local_com to place CoM at origin
        assert_eq!(rb.position, -expected_local_com);
    }

    #[test]
    fn test_apply_force() {
        let shape = Shape::Circle(Circle::new(1.0));
        let mut rb = RigidBody::new(1.0, shape);
        rb.apply_force(Vec2::new(10.0, 0.0));
        rb.apply_force(Vec2::new(0.0, 5.0));
        assert!((rb.force.x - 10.0).abs() < EPSILON);
        assert!((rb.force.y - 5.0).abs() < EPSILON);
        assert!((rb.torque - 0.0).abs() < EPSILON); // No torque from force at CoM
    }

    #[test]
    fn test_apply_force_at_point() {
        let shape = Shape::Circle(Circle::new(1.0));
        let mut rb = RigidBody::new(1.0, shape); // CoM starts at (0,0)
        let force = Vec2::new(10.0, 0.0);
        let point = Vec2::new(0.0, 1.0); // Apply 1 unit above CoM
        rb.apply_force_at_point(force, point);

        assert!((rb.force.x - 10.0).abs() < EPSILON);
        assert!((rb.force.y - 0.0).abs() < EPSILON);
        // r = point - CoM = (0,1) - (0,0) = (0,1)
        // torque = r x F = 0*0 - 1*10 = -10
        assert!((rb.torque - (-10.0)).abs() < EPSILON);
    }

     #[test]
    fn test_apply_force_at_point_offset_body() {
        // Body CoM is offset
        let shape = Shape::Circle(Circle::new(1.0));
        let mut rb = RigidBody::new(1.0, shape);
        rb.position = Vec2::new(5.0, 5.0); // Manually set CoM position

        let force = Vec2::new(0.0, 10.0); // Upward force
        let point_world = Vec2::new(6.0, 5.0); // Apply 1 unit right of CoM
        rb.apply_force_at_point(force, point_world);

        assert!((rb.force.x - 0.0).abs() < EPSILON);
        assert!((rb.force.y - 10.0).abs() < EPSILON);
         // r = point_world - CoM = (6,5) - (5,5) = (1,0)
         // torque = r x F = 1*10 - 0*0 = 10
        assert!((rb.torque - 10.0).abs() < EPSILON);
    }

     #[test]
    fn test_clear_accumulators() {
         let shape = Shape::Circle(Circle::new(1.0));
         let mut rb = RigidBody::new(1.0, shape);
         rb.apply_force(Vec2::new(1.0, 1.0));
         rb.torque = 5.0;
         rb.clear_accumulators();
         assert_eq!(rb.force, Vec2::ZERO);
         assert_eq!(rb.torque, 0.0);
    }


    #[test]
    fn test_rigidbody_new_with_inertia() {
        let mass = 10.0;
        let inertia = 50.0; // Explicit inertia
        let shape = Shape::Circle(Circle::new(1.0));
        let rb = RigidBody::new_with_inertia(mass, inertia, shape.clone());

        assert_eq!(rb.mass, mass);
        assert!((rb.inv_mass - (1.0 / mass)).abs() < EPSILON);
        assert_eq!(rb.shape, shape);
        assert_eq!(rb.inertia, inertia); // Should use provided inertia
        assert!((rb.inv_inertia - (1.0 / inertia)).abs() < EPSILON);
        assert_eq!(rb.local_center_of_mass, Vec2::ZERO); // Assumed zero for this constructor
        assert_eq!(rb.position, Vec2::ZERO); // CoM starts at origin
    }

    #[test]
    #[should_panic]
    fn test_rigidbody_new_with_inertia_polygon_panics() {
        let mass = 10.0;
        let inertia = 50.0;
         let vertices = vec![
             Vec2::new(-0.5, -0.5), Vec2::new( 0.5, -0.5),
             Vec2::new( 0.5,  0.5), Vec2::new(-0.5,  0.5),
         ];
        let shape = Shape::Polygon(Polygon::new(vertices.clone()));
        // This should panic
        RigidBody::new_with_inertia(mass, inertia, shape);
    }
} 