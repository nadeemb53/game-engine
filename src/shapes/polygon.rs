use crate::math::vec2::Vec2;

/// Represents a convex polygon shape defined by its vertices in local space.
/// Vertices should be ordered counter-clockwise (or clockwise, consistently).
#[derive(Debug, Clone, PartialEq)]
pub struct Polygon {
    pub vertices: Vec<Vec2>,
    // Potential future fields: precomputed normals, center
}

impl Polygon {
    /// Creates a new polygon from a vector of vertices.
    ///
    /// Panics if fewer than 3 vertices are provided.
    /// TODO: Add validation for convexity.
    pub fn new(vertices: Vec<Vec2>) -> Self {
        if vertices.len() < 3 {
            panic!("Polygon must have at least 3 vertices.");
        }
        Polygon { vertices }
    }

    /// Calculates the area of the polygon using the Shoelace formula.
    /// Assumes vertices are ordered consistently (e.g., counter-clockwise for positive area).
    pub fn calculate_area(&self) -> f64 {
        let n = self.vertices.len();
        if n < 3 { return 0.0; }
        let mut area = 0.0;
        for i in 0..n {
            let v1 = self.vertices[i];
            let v2 = self.vertices[(i + 1) % n];
            area += v1.cross(v2);
        }
        (area / 2.0).abs()
    }

    /// Calculates the centroid (center of mass for uniform density) of the polygon.
    /// Assumes vertices are ordered consistently (e.g., counter-clockwise).
    pub fn calculate_centroid(&self) -> Vec2 {
        let n = self.vertices.len();
        if n < 3 { return Vec2::ZERO; } // Or handle error/panic

        let mut centroid = Vec2::ZERO;
        let mut signed_area_sum = 0.0;
        let origin = self.vertices[0];

        for i in 1..(n - 1) {
            let v1 = origin;
            let v2 = self.vertices[i];
            let v3 = self.vertices[i + 1];

            let triangle_signed_area = (v2 - v1).cross(v3 - v1) / 2.0;
            signed_area_sum += triangle_signed_area;

            let triangle_centroid = (v1 + v2 + v3) / 3.0;

            centroid += triangle_centroid * triangle_signed_area;
        }

        if signed_area_sum.abs() < 1e-10 { // Handle degenerate case (e.g., collinear vertices)
            let mut avg = Vec2::ZERO;
            for v in &self.vertices {
                avg += *v;
            }
            if n > 0 { avg / (n as f64) } else { Vec2::ZERO }
        } else {
            centroid / signed_area_sum
        }
    }

    /// Calculates the moment of inertia for the polygon (with density=1) about the origin (0,0).
    /// Assumes vertices are ordered consistently (e.g., counter-clockwise).
    fn calculate_inertia_about_origin(&self) -> f64 {
        let n = self.vertices.len();
        if n < 3 { return 0.0; }

        let mut inertia_sum = 0.0;
        for i in 0..n {
            let v1 = self.vertices[i];
            let v2 = self.vertices[(i + 1) % n];
            let cross_prod = v1.cross(v2);
            let dot_prod = v1.dot(v2);
            let mag_sq_v1 = v1.magnitude_squared();
            let mag_sq_v2 = v2.magnitude_squared();

            inertia_sum += cross_prod * (mag_sq_v1 + dot_prod + mag_sq_v2);
        }
        inertia_sum / 12.0
    }

    /// Calculates the moment of inertia about the polygon's centroid (center of mass).
    /// Requires density to calculate mass for the parallel axis theorem.
    /// Returns f64::INFINITY if density is non-positive or area is zero.
    pub fn calculate_inertia(&self, density: Option<f64>) -> f64 {
        let density_val = density.unwrap_or(1.0);
        if density_val <= 0.0 { return f64::INFINITY; }

        let area = self.calculate_area();
        if area < 1e-10 { return f64::INFINITY; }

        let mass = area * density_val;
        let centroid = self.calculate_centroid();

        let inertia_origin = self.calculate_inertia_about_origin() * density_val;

        let d_squared = centroid.magnitude_squared();
        let inertia_centroid = inertia_origin - mass * d_squared;

        inertia_centroid.max(0.0)
    }

    /// Returns the outward-facing normal vectors for each edge of the polygon.
    /// Assumes vertices are ordered counter-clockwise.
    pub fn get_edge_normals(&self) -> Vec<Vec2> {
        let mut normals = Vec::with_capacity(self.vertices.len());
        let n = self.vertices.len();

        for i in 0..n {
            let v1 = self.vertices[i];
            let v2 = self.vertices[(i + 1) % n];
            let edge = v2 - v1;
            let normal = edge.perpendicular().normalize();
            normals.push(normal);
        }
        normals
    }

    // TODO: Add method to check convexity
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::vec2::Vec2;

    const EPSILON: f64 = 1e-9;

    #[test]
    fn test_polygon_new() {
        let vertices = vec![Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0), Vec2::new(0.0, 1.0)];
        let polygon = Polygon::new(vertices);
        assert_eq!(polygon.vertices.len(), 3);
    }

    #[test]
    #[should_panic]
    fn test_polygon_new_too_few_vertices() {
        let vertices = vec![Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0)];
        Polygon::new(vertices); // Should panic
    }

    #[test]
    fn test_polygon_area_square() {
        // Unit square centered at origin
        let vertices = vec![
            Vec2::new(-0.5, -0.5),
            Vec2::new( 0.5, -0.5),
            Vec2::new( 0.5,  0.5),
            Vec2::new(-0.5,  0.5),
        ];
        let polygon = Polygon::new(vertices);
        assert!((polygon.calculate_area() - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_polygon_area_triangle() {
        // Right triangle
        let vertices = vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(1.0, 0.0),
            Vec2::new(0.0, 1.0),
        ];
        let polygon = Polygon::new(vertices);
        assert!((polygon.calculate_area() - 0.5).abs() < EPSILON);
    }

    #[test]
    fn test_polygon_centroid_square_centered() {
        // Unit square centered at origin
        let vertices = vec![
            Vec2::new(-0.5, -0.5),
            Vec2::new( 0.5, -0.5),
            Vec2::new( 0.5,  0.5),
            Vec2::new(-0.5,  0.5),
        ];
        let polygon = Polygon::new(vertices);
        let centroid = polygon.calculate_centroid();
        assert!(centroid.x.abs() < EPSILON);
        assert!(centroid.y.abs() < EPSILON);
    }

    #[test]
    fn test_polygon_centroid_square_offset() {
        // Unit square offset
        let offset = Vec2::new(10.0, -5.0);
        let vertices = vec![
            offset + Vec2::new(0.0, 0.0),
            offset + Vec2::new(1.0, 0.0),
            offset + Vec2::new(1.0, 1.0),
            offset + Vec2::new(0.0, 1.0),
        ];
        let polygon = Polygon::new(vertices);
        let centroid = polygon.calculate_centroid();
        let expected_centroid = offset + Vec2::new(0.5, 0.5);
        assert!((centroid.x - expected_centroid.x).abs() < EPSILON);
        assert!((centroid.y - expected_centroid.y).abs() < EPSILON);
    }

    #[test]
    fn test_polygon_centroid_triangle() {
        // Right triangle
        let vertices = vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(3.0, 0.0), // Base = 3
            Vec2::new(0.0, 3.0), // Height = 3
        ];
        let polygon = Polygon::new(vertices);
        let centroid = polygon.calculate_centroid();
        // Centroid of triangle is (v1+v2+v3)/3
        let expected_centroid = (Vec2::new(0.0,0.0) + Vec2::new(3.0, 0.0) + Vec2::new(0.0, 3.0)) / 3.0;
        assert!((centroid.x - expected_centroid.x).abs() < EPSILON);
        assert!((centroid.y - expected_centroid.y).abs() < EPSILON);
        assert!((centroid.x - 1.0).abs() < EPSILON);
        assert!((centroid.y - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_polygon_inertia_square_centered() {
        // Unit square centered at origin, density = 1.0
        let vertices = vec![
            Vec2::new(-0.5, -0.5),
            Vec2::new( 0.5, -0.5),
            Vec2::new( 0.5,  0.5),
            Vec2::new(-0.5,  0.5),
        ];
        let polygon = Polygon::new(vertices);
        let density = 1.0;
        let mass = polygon.calculate_area() * density; // mass = 1.0
        let inertia = polygon.calculate_inertia(Some(density));

        // Inertia of rectangle (w=1, h=1) about centroid = mass * (w^2 + h^2) / 12
        let expected_inertia = mass * (1.0f64.powi(2) + 1.0f64.powi(2)) / 12.0;
        assert!((inertia - expected_inertia).abs() < EPSILON);
        assert!((inertia - (1.0/6.0)).abs() < EPSILON); // 1.0 * (1+1)/12 = 2/12 = 1/6
    }

    #[test]
    fn test_polygon_inertia_square_offset() {
        // Unit square offset from origin, density = 2.0
        let offset = Vec2::new(10.0, 0.0);
        let vertices = vec![
            offset + Vec2::new(-0.5, -0.5),
            offset + Vec2::new( 0.5, -0.5),
            offset + Vec2::new( 0.5,  0.5),
            offset + Vec2::new(-0.5,  0.5),
        ];
        let polygon = Polygon::new(vertices);
        let density = 2.0;
        let mass = polygon.calculate_area() * density; // mass = 1.0 * 2.0 = 2.0
        let inertia = polygon.calculate_inertia(Some(density));

        // Inertia about centroid should be the same regardless of offset
        let expected_inertia = mass * (1.0f64.powi(2) + 1.0f64.powi(2)) / 12.0;
        assert!((inertia - expected_inertia).abs() < EPSILON);
        assert!((inertia - (2.0/6.0)).abs() < EPSILON); // 2.0 * (1+1)/12 = 4/12 = 1/3
    }
} 