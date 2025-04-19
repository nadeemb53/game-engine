use crate::math::vec2::Vec2;
use crate::objects::rigid_body::RigidBody;
use crate::shapes::{Polygon, Shape};
use super::manifold::{CollisionManifold, ContactPoint};
use std::f64::EPSILON; // Import EPSILON for float comparisons
// Specific shape imports are unused now as shapes are accessed via RigidBody.shape
// use crate::shapes::Circle;
// use crate::shapes::LineSegment;
// use crate::shapes::Polygon;

/// Checks for collision between two circles.
/// Returns a CollisionManifold if they collide, None otherwise.
pub fn check_circle_circle(
    body_a: &RigidBody,
    body_a_idx: usize,
    body_b: &RigidBody,
    body_b_idx: usize,
) -> Option<CollisionManifold> {
    // Ensure both bodies are circles
    let circle_a = match body_a.shape {
        Shape::Circle(c) => c,
        _ => return None, // Or panic, depending on how shape matching is handled
    };
    let circle_b = match body_b.shape {
        Shape::Circle(c) => c,
        _ => return None,
    };

    let dist_vec = body_b.position - body_a.position;
    let dist_sq = dist_vec.magnitude_squared();
    let radii_sum = circle_a.radius + circle_b.radius;

    // Check if distance squared is less than sum of radii squared
    if dist_sq < radii_sum * radii_sum {
        let distance = dist_sq.sqrt();
        let penetration_depth = radii_sum - distance;

        let collision_normal = if distance > 1e-10 {
            dist_vec * (1.0 / distance)
        } else {
            // Circles are exactly on top of each other, choose an arbitrary normal
            Vec2::new(0.0, 1.0)
        };

        // Calculate contact points (on the surface of each circle)
        let contact_a = body_a.position + collision_normal * circle_a.radius;
        let contact_b = body_b.position - collision_normal * circle_b.radius;
        // For circles, the most representative single contact point is halfway between centers
        // or we can use the calculated points above.
        let contact_point = ContactPoint { point_a: contact_a, point_b: contact_b };

        Some(CollisionManifold {
            body_a_idx,
            body_b_idx,
            normal: collision_normal,
            depth: penetration_depth,
            contact: contact_point,
        })
    } else {
        None // No collision
    }
}

/// Finds the point on a line segment closest to a given point.
/// Returns the closest point and the parameter `t` (0 <= t <= 1) along the segment.
fn closest_point_on_segment(segment_a: Vec2, segment_b: Vec2, point: Vec2) -> (Vec2, f64) {
    let segment_vec = segment_b - segment_a;
    let length_sq = segment_vec.magnitude_squared();
    if length_sq < 1e-12 { // Treat as a point if segment is too short
        return (segment_a, 0.0);
    }

    // Project point onto the line defined by the segment
    // t = dot(point - segment_a, segment_vec) / length_sq
    let t = (point - segment_a).dot(segment_vec) * (1.0 / length_sq);

    // Clamp t to the segment [0, 1]
    let t_clamped = t.clamp(0.0, 1.0);

    // Calculate the closest point on the segment
    let closest_point = segment_a + segment_vec * t_clamped;
    (closest_point, t_clamped)
}

/// Checks for collision between a circle and a line segment.
/// Returns a CollisionManifold if they collide, None otherwise.
pub fn check_circle_line(
    circle_body: &RigidBody,
    circle_body_idx: usize,
    line_body: &RigidBody,
    line_body_idx: usize,
) -> Option<CollisionManifold> {
    let circle = match circle_body.shape {
        Shape::Circle(c) => c,
        _ => return None, // Or panic
    };
    let line = match line_body.shape {
        Shape::Line(l) => l,
        _ => return None,
    };

    // Transform line segment endpoints to world space
    let line_a_world = line_body.position + line.a.rotate(line_body.rotation);
    let line_b_world = line_body.position + line.b.rotate(line_body.rotation);

    // Find the closest point on the world-space line segment to the circle's center
    let (closest_point, _) = closest_point_on_segment(line_a_world, line_b_world, circle_body.position);

    // Check distance between circle center and closest point
    let dist_vec = circle_body.position - closest_point;
    let dist_sq = dist_vec.magnitude_squared();

    if dist_sq < circle.radius * circle.radius {
        // Collision detected
        let distance = dist_sq.sqrt();
        let penetration_depth = circle.radius - distance;

        let collision_normal = if distance > 1e-10 {
            dist_vec * (1.0 / distance) // Normal from closest point on line towards circle center
        } else {
            // Circle center is exactly on the line segment, need a fallback normal.
            // Using the line segment's perpendicular might work, but requires care
            // if the center is near an endpoint. A simple vertical normal is often okay.
            Vec2::new(0.0, 1.0)
        };

        // Contact points
        let contact_on_circle = circle_body.position - collision_normal * circle.radius;
        // Contact point on line is the closest point we found
        let contact_point = ContactPoint { point_a: contact_on_circle, point_b: closest_point };

        Some(CollisionManifold {
            body_a_idx: circle_body_idx, // Circle is body A
            body_b_idx: line_body_idx,   // Line is body B
            normal: collision_normal, // Normal points from Line towards Circle
            depth: penetration_depth,
            contact: contact_point,
        })
    } else {
        None // No collision
    }
}

/// Checks for intersection between two line segments in world space.
/// Returns the intersection point and parameters (t, u) if they intersect strictly within segments (0 < t < 1, 0 < u < 1),
/// None otherwise. This version focuses on point intersection, not overlap.
fn intersect_line_segments(
    a1: Vec2, a2: Vec2, // Endpoints of segment A
    b1: Vec2, b2: Vec2, // Endpoints of segment B
) -> Option<(Vec2, f64, f64)> {
    let d1 = a2 - a1; // Direction vector of segment A
    let d2 = b2 - b1; // Direction vector of segment B
    let delta_start = b1 - a1;

    let denominator = d1.cross(d2);

    if denominator.abs() < 1e-10 {
        // TODO: Handle collinear overlapping case if needed
        return None;
    }

    let t = delta_start.cross(d2) / denominator;
    let u = delta_start.cross(d1) / denominator;

    if t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0 {
        let intersection_point = a1 + d1 * t;
        Some((intersection_point, t, u))
    } else {
        None
    }
}

/// Checks for collision between two line segments.
/// Returns a CollisionManifold if they collide, None otherwise.
pub fn check_line_line(
    body_a: &RigidBody,
    body_a_idx: usize,
    body_b: &RigidBody,
    body_b_idx: usize,
) -> Option<CollisionManifold> {
    let line_a = match body_a.shape {
        Shape::Line(l) => l,
        _ => return None,
    };
    let line_b = match body_b.shape {
        Shape::Line(l) => l,
        _ => return None,
    };

    // Transform endpoints to world space
    let a1_world = body_a.position + line_a.a.rotate(body_a.rotation);
    let a2_world = body_a.position + line_a.b.rotate(body_a.rotation);
    let b1_world = body_b.position + line_b.a.rotate(body_b.rotation);
    let b2_world = body_b.position + line_b.b.rotate(body_b.rotation);

    // Check for intersection
    if let Some((intersection_point, _t, _u)) = intersect_line_segments(a1_world, a2_world, b1_world, b2_world) {
        // Segments intersect
        let d2_world = b2_world - b1_world; // Direction vector of line B in world space

        // Calculate normal (perpendicular to line B, arbitrary direction for now)
        let normal = if d2_world.magnitude_squared() > 1e-12 {
             d2_world.perpendicular().normalize()
        } else {
            // Fallback normal if line B is a point (shouldn't happen ideally)
            // Or maybe use perpendicular of line A?
             (a2_world - a1_world).perpendicular().normalize()
        };

        // Set depth to 0 for point intersection
        let depth = 0.0;

        // Contact point is the intersection point
        let contact_point = ContactPoint {
            point_a: intersection_point,
            point_b: intersection_point,
        };

        Some(CollisionManifold {
            body_a_idx,
            body_b_idx,
            normal,
            depth,
            contact: contact_point,
        })

    } else {
        None // No intersection
    }
}

// --- Projection Helper for SAT ---

/// Projects a polygon onto a given axis and returns the min/max interval.
fn project_polygon_onto_axis(body: &RigidBody, polygon: &Polygon, axis: Vec2) -> (f64, f64) {
    // Calculate world position of the shape's origin
    let world_shape_origin = body.position - body.local_center_of_mass.rotate(body.rotation);

    let mut min_proj = f64::INFINITY;
    let mut max_proj = f64::NEG_INFINITY;

    for vertex in &polygon.vertices {
        // Transform vertex relative to the world shape origin
        let world_vertex = world_shape_origin + vertex.rotate(body.rotation);
        let projection = world_vertex.dot(axis);
        min_proj = min_proj.min(projection);
        max_proj = max_proj.max(projection);
    }
    (min_proj, max_proj)
}

/// Projects a shape onto a given axis and returns the min/max interval.
fn project_shape_onto_axis(body: &RigidBody, axis: Vec2) -> (f64, f64) {
    match &body.shape {
        Shape::Circle(circle) => {
            // Project the center
            let center_proj = body.position.dot(axis);
            (center_proj - circle.radius, center_proj + circle.radius)
        }
        Shape::Line(segment) => {
            let a_world = body.position + segment.a.rotate(body.rotation);
            let b_world = body.position + segment.b.rotate(body.rotation);
            let proj_a = a_world.dot(axis);
            let proj_b = b_world.dot(axis);
            if proj_a < proj_b {
                (proj_a, proj_b)
            } else {
                (proj_b, proj_a)
            }
        }
        Shape::Polygon(polygon) => {
            project_polygon_onto_axis(body, polygon, axis)
        }
    }
}

/// Checks for collision between a circle and a polygon using SAT.
pub fn check_circle_polygon(
    circle_body: &RigidBody,
    circle_body_idx: usize,
    polygon_body: &RigidBody,
    polygon_body_idx: usize,
) -> Option<CollisionManifold> {
    println!("--- Checking Circle-Polygon Collision ---");
    println!("Circle Body: Pos={:?}, Rot={:.2}", circle_body.position, circle_body.rotation);
    println!("Polygon Body: Pos={:?}, Rot={:.2}", polygon_body.position, polygon_body.rotation);

    let circle = match circle_body.shape {
        Shape::Circle(ref c) => c,
        _ => return None, // Should not happen
    };
    let polygon = match polygon_body.shape {
        Shape::Polygon(ref p) => p,
        _ => return None,
    };

    let mut axes = Vec::new();

    // Calculate world position of the polygon's shape origin
    let polygon_world_origin = polygon_body.position - polygon_body.local_center_of_mass.rotate(polygon_body.rotation);
    let polygon_rotation = polygon_body.rotation;
    println!("Polygon World Origin: {:?}", polygon_world_origin);

    // 1. Polygon edge normals
    for i in 0..polygon.vertices.len() {
        let p1_world = polygon_world_origin + polygon.vertices[i].rotate(polygon_rotation);
        let p2_world = polygon_world_origin + polygon.vertices[(i + 1) % polygon.vertices.len()].rotate(polygon_rotation);
        let edge = p2_world - p1_world;
        if edge.magnitude_squared() > 1e-12 {
            let axis = edge.perpendicular().normalize(); // World normal directly
            axes.push(axis);
            println!("  Adding Polygon Edge Normal Axis: {:?}", axis);
        }
    }

    // 2. Axis from closest polygon vertex to circle center
    let mut closest_vertex_dist_sq = f64::INFINITY;
    let mut closest_vertex_world = Vec2::ZERO;
    for local_vertex in &polygon.vertices {
        // Transform vertex correctly
        let world_vertex = polygon_world_origin + local_vertex.rotate(polygon_rotation);
        let dist_sq = world_vertex.distance_squared(circle_body.position);
        if dist_sq < closest_vertex_dist_sq {
            closest_vertex_dist_sq = dist_sq;
            closest_vertex_world = world_vertex;
        }
    }
    println!("  Closest Polygon Vertex (World): {:?}", closest_vertex_world);
    let circle_to_vertex_vec = closest_vertex_world - circle_body.position;
    if circle_to_vertex_vec.magnitude_squared() > 1e-12 {
        let axis = circle_to_vertex_vec.normalize();
        axes.push(axis);
        println!("  Adding Circle-to-Vertex Axis: {:?}", axis);
    }

    let mut min_overlap = f64::INFINITY;
    let mut mtv_axis = Vec2::ZERO; // Minimum Translation Vector axis

    // Project onto each axis
    for axis in axes {
        // Ensure axis is valid (ignore zero vectors if normalization failed)
        if axis.magnitude_squared() < 1e-10 { continue; }
        println!("Testing Axis: {:?}", axis);

        let (min_c, max_c) = project_shape_onto_axis(circle_body, axis);
        // Use the dedicated polygon projection function
        let (min_p, max_p) = project_polygon_onto_axis(polygon_body, polygon, axis);
        println!("  Circle Proj: [{:.3}, {:.3}], Polygon Proj: [{:.3}, {:.3}]", min_c, max_c, min_p, max_p);

        // Check for non-overlap
        let overlap1 = max_c - min_p;
        let overlap2 = max_p - min_c;

        // Use epsilon check for separation
        if overlap1 < -EPSILON || overlap2 < -EPSILON {
            // Separating axis found, no collision
            println!("  Separating axis found. No collision.");
            println!("--- End Circle-Polygon Check (Separated) ---");
            return None;
        }

        // Determine the overlap amount on this axis (ensure non-negative)
        let current_overlap = overlap1.min(overlap2).max(0.0);
        println!("  Overlap on this axis: {:.3}", current_overlap);

        // Keep track of the minimum overlap and the corresponding axis
        if current_overlap < min_overlap {
            min_overlap = current_overlap;
            mtv_axis = axis;
        }
    }

    println!("Min Overlap: {:.3}, MTV Axis (pre-flip): {:?}", min_overlap, mtv_axis);

    // If we reach here, the shapes are colliding.
    // The MTV axis currently points in an arbitrary direction along the axis of minimum overlap.
    // We need to ensure the normal points from body A (circle) to body B (polygon).
    let normal = mtv_axis; // Use MTV axis directly as normal

    println!("Final Normal: {:?}, Depth: {:.3}", normal, min_overlap);

    // TODO: Calculate accurate contact points.
    // For now, approximate contact point B on the polygon as the closest point on polygon
    // to the circle center, projected onto the normal direction?
    // Or use polygon vertex closest to circle center?
    // A simpler approximation: point on circle circumference along the normal.
    let contact_point_a = circle_body.position + normal * circle.radius;
    // Approximate point B by pushing point A back into B by the depth
    let contact_point_b = contact_point_a - normal * min_overlap;

    println!("Collision Detected: Normal={:?}, Depth={:.3}", normal, min_overlap);
    println!("--- End Circle-Polygon Check (Collision) ---");

    Some(CollisionManifold {
        body_a_idx: circle_body_idx,
        body_b_idx: polygon_body_idx,
        normal,
        depth: min_overlap,
        contact: ContactPoint { point_a: contact_point_a, point_b: contact_point_b },
    })
}

/// Finds the vertex on a polygon furthest in a given world direction.
fn find_support_point(body: &RigidBody, polygon: &Polygon, direction: Vec2) -> Vec2 {
    let world_shape_origin = body.position - body.local_center_of_mass.rotate(body.rotation);
    let rotation = body.rotation;
    let mut best_projection = f64::NEG_INFINITY;
    // Initialize with the first vertex transformed, in case loop doesn't run or all points project equally
    let mut support_point = if !polygon.vertices.is_empty() {
        world_shape_origin + polygon.vertices[0].rotate(rotation)
    } else {
        body.position // Fallback to CoM if polygon is somehow empty
    };

    for vertex in &polygon.vertices {
        let world_vertex = world_shape_origin + vertex.rotate(rotation);
        let projection = world_vertex.dot(direction);
        if projection > best_projection {
            best_projection = projection;
            support_point = world_vertex;
        }
    }
    support_point
}

/// Checks for collision between two polygons using the Separating Axis Theorem (SAT).
/// Returns a CollisionManifold if they collide, None otherwise.
pub fn check_polygon_polygon(
    body_a: &RigidBody,
    body_a_idx: usize,
    body_b: &RigidBody,
    body_b_idx: usize,
) -> Option<CollisionManifold> {
    let poly_a = match &body_a.shape {
        Shape::Polygon(p) => p,
        _ => return None, // Or panic
    };
    let poly_b = match &body_b.shape {
        Shape::Polygon(p) => p,
        _ => return None,
    };

    // Pre-calculate world origins
    let world_origin_a = body_a.position - body_a.local_center_of_mass.rotate(body_a.rotation);
    let world_origin_b = body_b.position - body_b.local_center_of_mass.rotate(body_b.rotation);

    let mut min_overlap = f64::INFINITY;
    let mut smallest_axis = Vec2::ZERO;

    // --- Helper Function to get axes ---
    let get_axes = |poly: &Polygon, world_origin: Vec2, rotation: f64| -> Vec<Vec2> {
        let mut axes = Vec::with_capacity(poly.vertices.len());
        for i in 0..poly.vertices.len() {
            let p1_world = world_origin + poly.vertices[i].rotate(rotation);
            let p2_world = world_origin + poly.vertices[(i + 1) % poly.vertices.len()].rotate(rotation);
            let edge = p2_world - p1_world;
            if edge.magnitude_squared() > 1e-12 {
                axes.push(edge.perpendicular().normalize());
            }
        }
        axes
    };

    // --- Check axes of Polygon A ---
    let axes_a = get_axes(poly_a, world_origin_a, body_a.rotation);
    for axis in axes_a {
        let (min_a, max_a) = project_polygon_onto_axis(body_a, poly_a, axis);
        let (min_b, max_b) = project_polygon_onto_axis(body_b, poly_b, axis);

        let overlap = (max_a.min(max_b)) - (min_a.max(min_b));
        if overlap < -EPSILON { // Allow for tiny negative overlap due to float errors
            return None; // Found a separating axis
        } else {
            let non_negative_overlap = overlap.max(0.0);
            if non_negative_overlap < min_overlap {
                min_overlap = non_negative_overlap;
                smallest_axis = axis;
            }
        }
    }

    // --- Check axes of Polygon B ---
    let axes_b = get_axes(poly_b, world_origin_b, body_b.rotation);
     for axis in axes_b {
        let (min_a, max_a) = project_polygon_onto_axis(body_a, poly_a, axis);
        let (min_b, max_b) = project_polygon_onto_axis(body_b, poly_b, axis);

        let overlap = (max_a.min(max_b)) - (min_a.max(min_b));
         if overlap < -EPSILON { // Allow for tiny negative overlap due to float errors
            return None; // Found a separating axis
        } else {
            let non_negative_overlap = overlap.max(0.0);
             if non_negative_overlap < min_overlap {
                min_overlap = non_negative_overlap;
                smallest_axis = axis;
            }
        }
    }

    // --- If we got here, they are colliding ---
    // Ensure the normal points from B to A
    let center_a = body_a.position; 
    let center_b = body_b.position;
    let direction = center_a - center_b;
    let normal = smallest_axis; // Normal points B -> A

    // --- Calculate Contact Points (Approximate) ---
    // Find support point on B in the direction of the normal (most penetrated point on B)
    let contact_point_b = find_support_point(body_b, poly_b, normal);
    // Estimate contact point on A by moving back along the normal from B's contact by the depth
    let contact_point_a = contact_point_b - normal * min_overlap;

    Some(CollisionManifold {
        body_a_idx,
        body_b_idx,
        normal,
        depth: min_overlap,
        contact: ContactPoint { point_a: contact_point_a, point_b: contact_point_b },
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::objects::rigid_body::RigidBody;
    use crate::shapes::{Circle, LineSegment, Polygon}; // Import needed shapes for tests
    // Specific shape imports are unused now as shapes are accessed via RigidBody.shape
    // use crate::shapes::{Circle, LineSegment};
    // use crate::shapes::Polygon;
    use crate::math::vec2::Vec2;
    const EPSILON: f64 = 1e-9;

    #[test]
    fn test_check_circle_circle_no_collision() {
        let shape_a = Shape::Circle(Circle::new(1.0));
        let shape_b = Shape::Circle(Circle::new(1.0));
        let body_a = RigidBody {
            position: Vec2::new(0.0, 0.0),
            shape: shape_a.clone(),
            ..RigidBody::new(1.0, shape_a) // Use dummy values for other fields
        };
        let body_b = RigidBody {
            position: Vec2::new(3.0, 0.0),
            shape: shape_b.clone(),
            ..RigidBody::new(1.0, shape_b)
        };

        let manifold = check_circle_circle(&body_a, 0, &body_b, 1);
        assert!(manifold.is_none());
    }

     #[test]
    fn test_check_circle_circle_touching() {
        let shape_a = Shape::Circle(Circle::new(1.0));
        let shape_b = Shape::Circle(Circle::new(1.0));
        let body_a = RigidBody { position: Vec2::new(0.0, 0.0), shape: shape_a.clone(), ..RigidBody::new(1.0, shape_a) };
        let body_b = RigidBody { position: Vec2::new(2.0, 0.0), shape: shape_b.clone(), ..RigidBody::new(1.0, shape_b) };

        let manifold = check_circle_circle(&body_a, 0, &body_b, 1);
        // Touching exactly means depth is 0, might return None or Some depending on < vs <=
        // Current logic uses <, so touching should return None or a manifold with near-zero depth.
        // Let's expect None for strict '<'
         assert!(manifold.is_none()); // Or check for depth ~ 0 if using <=
    }

     #[test]
    fn test_check_circle_circle_colliding() {
        let shape_a = Shape::Circle(Circle::new(1.0));
        let shape_b = Shape::Circle(Circle::new(1.0));
        let body_a = RigidBody { position: Vec2::new(0.0, 0.0), shape: shape_a.clone(), ..RigidBody::new(1.0, shape_a) };
        let body_b = RigidBody { position: Vec2::new(1.5, 0.0), shape: shape_b.clone(), ..RigidBody::new(1.0, shape_b) };

        let manifold = check_circle_circle(&body_a, 0, &body_b, 1);
        assert!(manifold.is_some());
        let m = manifold.unwrap();

        assert_eq!(m.body_a_idx, 0);
        assert_eq!(m.body_b_idx, 1);
        assert!((m.normal.x - 1.0).abs() < EPSILON); // Normal points from A to B (along +x)
        assert!((m.normal.y - 0.0).abs() < EPSILON);
        let expected_depth = (1.0 + 1.0) - 1.5; // radii_sum - distance
        assert!((m.depth - expected_depth).abs() < EPSILON);
        // Contact point A: (0,0) + (1,0)*1.0 = (1,0)
        // Contact point B: (1.5,0) - (1,0)*1.0 = (0.5, 0)
        assert!((m.contact.point_a.x - 1.0).abs() < EPSILON);
        assert!((m.contact.point_a.y - 0.0).abs() < EPSILON);
         assert!((m.contact.point_b.x - 0.5).abs() < EPSILON);
        assert!((m.contact.point_b.y - 0.0).abs() < EPSILON);
    }

    #[test]
    fn test_check_circle_circle_concentric() {
        let shape_a = Shape::Circle(Circle::new(2.0));
        let shape_b = Shape::Circle(Circle::new(1.0));
        let body_a = RigidBody { position: Vec2::new(0.0, 0.0), shape: shape_a.clone(), ..RigidBody::new(1.0, shape_a) };
        let body_b = RigidBody { position: Vec2::new(0.0, 0.0), shape: shape_b.clone(), ..RigidBody::new(1.0, shape_b) };

        let manifold = check_circle_circle(&body_a, 0, &body_b, 1);
        assert!(manifold.is_some());
        let m = manifold.unwrap();
        let expected_depth = (2.0 + 1.0) - 0.0;
        assert!((m.depth - expected_depth).abs() < EPSILON);
        // Normal is arbitrary, test checks (0,1) was chosen
        assert!((m.normal.x - 0.0).abs() < EPSILON);
        assert!((m.normal.y - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_check_circle_line_no_collision() {
        let circle_shape = Shape::Circle(Circle::new(1.0));
        let line_shape = Shape::Line(LineSegment::new(Vec2::new(2.0, 1.0), Vec2::new(4.0, 1.0)));
        let circle_body = RigidBody{ position: Vec2::new(0.0, 0.0), ..RigidBody::new(1.0, circle_shape) };
        let line_body = RigidBody{ position: Vec2::new(0.0, 0.0), ..RigidBody::new(1.0, line_shape) }; // Line at y=1, x=[2,4]

        let manifold = check_circle_line(&circle_body, 0, &line_body, 1);
        assert!(manifold.is_none());
    }

    #[test]
    fn test_check_circle_line_collision_mid() {
        let circle_shape = Shape::Circle(Circle::new(1.0));
        let line_shape = Shape::Line(LineSegment::new(Vec2::new(-2.0, 0.5), Vec2::new(2.0, 0.5)));
        let circle_body = RigidBody{ position: Vec2::new(0.0, 0.0), ..RigidBody::new(1.0, circle_shape) };
        let line_body = RigidBody{ position: Vec2::new(0.0, 0.0), ..RigidBody::new(1.0, line_shape) }; // Line at y=0.5

        let manifold = check_circle_line(&circle_body, 0, &line_body, 1);
        assert!(manifold.is_some());
        let m = manifold.unwrap();

        assert_eq!(m.body_a_idx, 0);
        assert_eq!(m.body_b_idx, 1);
        let expected_depth = 1.0 - 0.5; // radius - distance_to_line
        assert!((m.depth - expected_depth).abs() < EPSILON);
        assert!((m.normal.x - 0.0).abs() < EPSILON); // Normal points from line to circle (-y)
        assert!((m.normal.y - -1.0).abs() < EPSILON);
        // point_a on circle surface: (0,0) - (0,-1)*1 = (0, 1)
        // point_b on line: closest point = (0, 0.5)
         assert!((m.contact.point_a.x - 0.0).abs() < EPSILON);
         assert!((m.contact.point_a.y - 1.0).abs() < EPSILON); // Corrected assertion: y should be 1.0
         assert!((m.contact.point_b.x - 0.0).abs() < EPSILON);
         assert!((m.contact.point_b.y - 0.5).abs() < EPSILON);
    }

     #[test]
    fn test_check_circle_line_collision_endpoint() {
        let circle_shape_enum = Shape::Circle(Circle::new(1.0));
        let circle = match circle_shape_enum { Shape::Circle(c) => c, _ => panic!("Not a circle") }; // Extract Circle
        let line_shape = Shape::Line(LineSegment::new(Vec2::new(1.0, 0.0), Vec2::new(3.0, 0.0)));
        let circle_body = RigidBody{ position: Vec2::new(0.5, 0.5), ..RigidBody::new(1.0, circle_shape_enum) };
        let line_body = RigidBody{ position: Vec2::new(0.0, 0.0), ..RigidBody::new(1.0, line_shape) };

        let manifold = check_circle_line(&circle_body, 0, &line_body, 1);
        assert!(manifold.is_some());
        let m = manifold.unwrap();

        let dist_vec = Vec2::new(-0.5, 0.5);
        let distance = dist_vec.magnitude();
        let expected_depth = 1.0 - distance;
        let expected_normal = dist_vec * (1.0 / distance); // From line point to circle center

        assert_eq!(m.body_a_idx, 0);
        assert_eq!(m.body_b_idx, 1);
        assert!((m.depth - expected_depth).abs() < EPSILON);
        assert!((m.normal.x - expected_normal.x).abs() < EPSILON);
        assert!((m.normal.y - expected_normal.y).abs() < EPSILON);
        // point_a on circle surface: (0.5, 0.5) - normal * radius
        // Use extracted circle.radius here
        assert!((m.contact.point_a - (circle_body.position - m.normal * circle.radius)).magnitude_squared() < EPSILON * EPSILON);
        assert!((m.contact.point_b.x - 1.0).abs() < EPSILON);
        assert!((m.contact.point_b.y - 0.0).abs() < EPSILON);
    }

     #[test]
    fn test_check_circle_line_touching_endpoint() {
        let radius = 1.0;
        let circle_shape = Shape::Circle(Circle::new(radius));
        let line_shape = Shape::Line(LineSegment::new(Vec2::new(1.0, 0.0), Vec2::new(3.0, 0.0)));
        // Place circle center exactly radius away from endpoint (1,0)
        let circle_pos = Vec2::new(1.0, radius); // at (1, 1)
        let circle_body = RigidBody{ position: circle_pos, ..RigidBody::new(1.0, circle_shape) };
        let line_body = RigidBody{ position: Vec2::new(0.0, 0.0), ..RigidBody::new(1.0, line_shape) };

        let manifold = check_circle_line(&circle_body, 0, &line_body, 1);
        // Touching -> no penetration depth -> None (due to strict < check)
        assert!(manifold.is_none());
    }

    // --- Line-Line Tests ---

    #[test]
    fn test_intersect_line_segments_intersecting() {
        let a1 = Vec2::new(0.0, 0.0);
        let a2 = Vec2::new(2.0, 2.0);
        let b1 = Vec2::new(0.0, 2.0);
        let b2 = Vec2::new(2.0, 0.0);

        let result = intersect_line_segments(a1, a2, b1, b2);
        assert!(result.is_some());
        let (p, t, u) = result.unwrap();
        assert!((p.x - 1.0).abs() < EPSILON);
        assert!((p.y - 1.0).abs() < EPSILON);
        assert!((t - 0.5).abs() < EPSILON);
        assert!((u - 0.5).abs() < EPSILON);
    }

    #[test]
    fn test_intersect_line_segments_non_intersecting() {
        let a1 = Vec2::new(0.0, 0.0);
        let a2 = Vec2::new(1.0, 1.0);
        let b1 = Vec2::new(1.0, 2.0);
        let b2 = Vec2::new(3.0, 0.0);

        let result = intersect_line_segments(a1, a2, b1, b2);
        assert!(result.is_none(), "Intersection should be outside segment A (t=1.5)");
    }

    #[test]
    fn test_intersect_line_segments_parallel() {
        let a1 = Vec2::new(0.0, 0.0);
        let a2 = Vec2::new(1.0, 0.0);
        let b1 = Vec2::new(0.0, 1.0);
        let b2 = Vec2::new(1.0, 1.0);

        let result = intersect_line_segments(a1, a2, b1, b2);
        assert!(result.is_none());
    }

    #[test]
    fn test_intersect_line_segments_collinear_non_overlapping() {
        let a1 = Vec2::new(0.0, 0.0);
        let a2 = Vec2::new(1.0, 0.0);
        let b1 = Vec2::new(2.0, 0.0);
        let b2 = Vec2::new(3.0, 0.0);

        let result = intersect_line_segments(a1, a2, b1, b2);
        assert!(result.is_none()); // Current logic doesn't handle overlap
    }

    #[test]
    fn test_intersect_line_segments_touching_endpoints() {
        let a1 = Vec2::new(0.0, 0.0);
        let a2 = Vec2::new(1.0, 1.0);
        let b1 = Vec2::new(1.0, 1.0);
        let b2 = Vec2::new(2.0, 0.0);

        let result = intersect_line_segments(a1, a2, b1, b2);
        assert!(result.is_some()); // t=1, u=0
        let (p, t, u) = result.unwrap();
        assert!((p.x - 1.0).abs() < EPSILON);
        assert!((p.y - 1.0).abs() < EPSILON);
        assert!((t - 1.0).abs() < EPSILON);
        assert!((u - 0.0).abs() < EPSILON);
    }


    #[test]
    fn test_check_line_line_intersecting() {
        let line_shape_a = Shape::Line(LineSegment::new(Vec2::new(-1.0, -1.0), Vec2::new(1.0, 1.0)));
        let line_shape_b = Shape::Line(LineSegment::new(Vec2::new(-1.0, 1.0), Vec2::new(1.0, -1.0)));
        let body_a = RigidBody{ position: Vec2::new(0.0, 0.0), ..RigidBody::new(1.0, line_shape_a) };
        let body_b = RigidBody{ position: Vec2::new(0.0, 0.0), ..RigidBody::new(1.0, line_shape_b) };

        let manifold = check_line_line(&body_a, 0, &body_b, 1);
        assert!(manifold.is_some());
        let m = manifold.unwrap();

        assert_eq!(m.body_a_idx, 0);
        assert_eq!(m.body_b_idx, 1);
        assert!((m.depth - 0.0).abs() < EPSILON); // Depth is 0 for point intersection

        // Intersection point is (0,0)
        assert!((m.contact.point_a.x - 0.0).abs() < EPSILON);
        assert!((m.contact.point_a.y - 0.0).abs() < EPSILON);
        assert!((m.contact.point_b.x - 0.0).abs() < EPSILON);
        assert!((m.contact.point_b.y - 0.0).abs() < EPSILON);

        // Normal is perpendicular to line B's direction (1, -1) -> (2, -2)
        // Perpendicular is (2, 2) or (-2, -2). Normalized: (1/sqrt(2), 1/sqrt(2)) or (-1/sqrt(2), -1/sqrt(2))
        // Our perp function (-y, x) gives (1, 1) -> normalize.
        let expected_norm_x = 1.0 / 2.0f64.sqrt();
        let expected_norm_y = 1.0 / 2.0f64.sqrt();
        assert!((m.normal.x.abs() - expected_norm_x).abs() < EPSILON);
        assert!((m.normal.y.abs() - expected_norm_y).abs() < EPSILON);
        // Don't assert exact sign, it's arbitrary for now
    }

    #[test]
    fn test_check_line_line_non_intersecting() {
        let line_shape_a = Shape::Line(LineSegment::new(Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0)));
        let line_shape_b = Shape::Line(LineSegment::new(Vec2::new(0.0, 1.0), Vec2::new(1.0, 1.0)));
        let body_a = RigidBody{ position: Vec2::new(0.0, 0.0), ..RigidBody::new(1.0, line_shape_a) };
        let body_b = RigidBody{ position: Vec2::new(0.0, 0.0), ..RigidBody::new(1.0, line_shape_b) };

        let manifold = check_line_line(&body_a, 0, &body_b, 1);
        assert!(manifold.is_none());
    }

    // --- Circle-Polygon Tests ---

    // Helper to create a standard square polygon centered at origin
    fn create_centered_square_polygon(width: f64) -> Polygon {
        let hw = width / 2.0;
         Polygon::new(vec![
             Vec2::new(-hw, -hw),
             Vec2::new( hw, -hw),
             Vec2::new( hw,  hw),
             Vec2::new(-hw,  hw),
         ])
    }

     #[test]
    fn test_check_circle_polygon_collision() {
         let circle_shape = Shape::Circle(Circle::new(0.5));
         let polygon_shape = Shape::Polygon(create_centered_square_polygon(1.0));

         // Circle slightly overlapping the square
         let circle_body = RigidBody { position: Vec2::new(0.8, 0.0), ..RigidBody::new(1.0, circle_shape)};
         let polygon_body = RigidBody { position: Vec2::new(0.0, 0.0), ..RigidBody::new(1.0, polygon_shape)};

         let manifold = check_circle_polygon(&circle_body, 0, &polygon_body, 1);

         assert!(manifold.is_some());
         let m = manifold.unwrap();

         // Expected normal should point from circle towards polygon (approx -1, 0)
         // Depth should be (radius + half_width) - distance = (0.5 + 0.5) - 0.8 = 0.2
         assert!((m.normal.x - (-1.0)).abs() < EPSILON);
         assert!(m.normal.y.abs() < EPSILON);
         assert!((m.depth - 0.2).abs() < EPSILON);
         assert_eq!(m.body_a_idx, 0);
         assert_eq!(m.body_b_idx, 1);
     }

     #[test]
    fn test_check_circle_polygon_no_collision() {
         let circle_shape = Shape::Circle(Circle::new(0.5));
         let polygon_shape = Shape::Polygon(create_centered_square_polygon(1.0));

         // Circle far away
         let circle_body = RigidBody { position: Vec2::new(2.0, 0.0), ..RigidBody::new(1.0, circle_shape)};
         let polygon_body = RigidBody { position: Vec2::new(0.0, 0.0), ..RigidBody::new(1.0, polygon_shape)};

         let manifold = check_circle_polygon(&circle_body, 0, &polygon_body, 1);
         assert!(manifold.is_none());
     }

     #[test]
    fn test_check_circle_polygon_touching_edge() {
         let circle_shape = Shape::Circle(Circle::new(0.5));
         let polygon_shape = Shape::Polygon(create_centered_square_polygon(1.0)); // Square from -0.5 to 0.5

         // Circle touching the right edge (x=0.5) of the square
         let circle_body = RigidBody { position: Vec2::new(1.0, 0.0), ..RigidBody::new(1.0, circle_shape)};
         let polygon_body = RigidBody { position: Vec2::new(0.0, 0.0), ..RigidBody::new(1.0, polygon_shape)};

         let manifold = check_circle_polygon(&circle_body, 0, &polygon_body, 1);

         assert!(manifold.is_some());
         let m = manifold.unwrap();
         // Normal should point along x-axis (from circle to polygon)
         assert!((m.normal.x - (-1.0)).abs() < EPSILON, "Normal: {:?}", m.normal);
         assert!(m.normal.y.abs() < EPSILON);
         // Depth should be zero (or very close)
         assert!(m.depth.abs() < EPSILON);
     }

      #[test]
     fn test_check_circle_polygon_touching_vertex() {
         let circle_shape = Shape::Circle(Circle::new(0.5));
         let polygon_shape = Shape::Polygon(create_centered_square_polygon(1.0)); // Square vertices at (+/-0.5, +/-0.5)

         // Position circle center such that its circumference touches the top-right vertex (0.5, 0.5)
         // Distance from origin to vertex is sqrt(0.5^2 + 0.5^2) = sqrt(0.5) approx 0.707
         // Position circle center along the diagonal direction, radius distance away from vertex
         let vertex_pos = Vec2::new(0.5, 0.5);
         let direction = vertex_pos.normalize(); // Direction from origin to vertex
         let radius = match circle_shape {
             Shape::Circle(ref c) => c.radius,
             _ => panic!("Test setup error: circle_shape is not a Circle")
         };
         let circle_pos = vertex_pos + direction * radius;

         let circle_body = RigidBody { position: circle_pos, ..RigidBody::new(1.0, circle_shape)};
         let polygon_body = RigidBody { position: Vec2::new(0.0, 0.0), ..RigidBody::new(1.0, polygon_shape)};

         let manifold = check_circle_polygon(&circle_body, 0, &polygon_body, 1);

         assert!(manifold.is_some());
         let m = manifold.unwrap();
         // Normal should point from circle towards polygon (approx negative of direction)
         assert!((m.normal.x - (-direction.x)).abs() < EPSILON, "Normal: {:?}, Expected Dir: {:?}", m.normal, -direction);
         assert!((m.normal.y - (-direction.y)).abs() < EPSILON);
         // Depth should be zero (or very close)
         assert!(m.depth.abs() < EPSILON);
     }

    // --- Polygon-Polygon Tests ---

    #[test]
    fn test_check_polygon_polygon_no_collision() {
        let poly_shape = Shape::Polygon(create_centered_square_polygon(1.0));
        let body_a = RigidBody { position: Vec2::new(0.0, 0.0), ..RigidBody::new(1.0, poly_shape.clone()) };
        let body_b = RigidBody { position: Vec2::new(2.0, 0.0), ..RigidBody::new(1.0, poly_shape.clone()) }; // Separated

        let manifold = check_polygon_polygon(&body_a, 0, &body_b, 1);
        assert!(manifold.is_none());
    }

    #[test]
    fn test_check_polygon_polygon_touching_edge() {
        let poly_shape = Shape::Polygon(create_centered_square_polygon(1.0)); // Edges at +/- 0.5
        let body_a = RigidBody { position: Vec2::new(0.0, 0.0), ..RigidBody::new(1.0, poly_shape.clone()) };
        let body_b = RigidBody { position: Vec2::new(1.0, 0.0), ..RigidBody::new(1.0, poly_shape.clone()) }; // Touching right edge of A

        let manifold = check_polygon_polygon(&body_a, 0, &body_b, 1);
        assert!(manifold.is_some());
        let m = manifold.unwrap();
        // Normal should be along x-axis, pointing from B to A
        assert!((m.normal.x - (-1.0)).abs() < EPSILON);
        assert!(m.normal.y.abs() < EPSILON);
        // Depth should be zero (or very close)
        assert!(m.depth.abs() < EPSILON);
    }

    #[test]
    fn test_check_polygon_polygon_overlapping() {
        let poly_shape = Shape::Polygon(create_centered_square_polygon(1.0)); // Edges at +/- 0.5
        let body_a = RigidBody { position: Vec2::new(0.0, 0.0), ..RigidBody::new(1.0, poly_shape.clone()) };
        let body_b = RigidBody { position: Vec2::new(0.8, 0.0), ..RigidBody::new(1.0, poly_shape.clone()) }; // Overlapping by 0.2

        let manifold = check_polygon_polygon(&body_a, 0, &body_b, 1);
        assert!(manifold.is_some());
        let m = manifold.unwrap();
        // Normal should be along x-axis, pointing from B to A
        assert!((m.normal.x - (-1.0)).abs() < EPSILON);
        assert!(m.normal.y.abs() < EPSILON);
        // Depth should be (half_width_a + half_width_b) - distance = (0.5 + 0.5) - 0.8 = 0.2
        assert!((m.depth - 0.2).abs() < EPSILON);
    }

    #[test]
    fn test_check_polygon_polygon_rotated_overlapping() {
        let poly_shape = Shape::Polygon(create_centered_square_polygon(1.0)); // Square A
        let body_a = RigidBody { position: Vec2::new(0.0, 0.0), ..RigidBody::new(1.0, poly_shape.clone()) };

        // Square B rotated 45 degrees, slightly overlapping A
        let body_b = RigidBody {
            position: Vec2::new(0.6, 0.0),
            rotation: std::f64::consts::PI / 4.0,
            ..RigidBody::new(1.0, poly_shape.clone())
        };

        let manifold = check_polygon_polygon(&body_a, 0, &body_b, 1);
        assert!(manifold.is_some());
        let m = manifold.unwrap();

        // The MTV axis for a box vs rotated box can be complex.
        // In this case (slight overlap on x-axis with 45 deg rotation),
        // the separation axis is likely one of the non-rotated box's axes (x or y).
        // The overlap is smallest along the x-axis.
        assert!((m.normal.x - (-1.0)).abs() < EPSILON, "Normal: {:?}", m.normal); // Expecting normal approx (-1, 0)
        assert!(m.normal.y.abs() < EPSILON, "Normal: {:?}", m.normal);
        // Calculate expected depth: project rotated box onto x-axis.
        // Vertices of rotated box: project [(0.6, 0.0) +/- (0.707, 0.0)], [(0.6, 0.0) +/- (0.0, 0.707)] (approx)
        // Rotated vertices approx: (0.6, 0.707), (1.307, 0), (0.6, -0.707), (-0.107, 0)
        // World vertices approx (relative to B's center 0.6,0):
        // rotated vertex (local 0.5, 0.5) -> world (0.6, 0.707)
        // rotated vertex (local -0.5, 0.5) -> world (-0.107, 0)
        // rotated vertex (local 0.5, -0.5) -> world (1.307, 0)
        // rotated vertex (local -0.5, -0.5) -> world (0.6, -0.707)
        // Projection of B onto x-axis: [-0.107, 1.307]
        // Projection of A onto x-axis: [-0.5, 0.5]
        // Overlap = min(0.5, 1.307) - max(-0.5, -0.107) = 0.5 - (-0.107) = 0.607
        // Let's re-check. Project A onto x-axis: [-0.5, 0.5].
        // Project B onto x-axis requires transforming B's vertices and projecting.
        // B's local vertices: (-0.5,-0.5), (0.5,-0.5), (0.5,0.5), (-0.5,0.5)
        // Rotate by pi/4 (cos=sin=0.7071): (-0.7071, 0), (0, -0.7071), (0.7071, 0), (0, 0.7071) relative to B's center
        // World vertices of B: (0.6 - 0.7071, 0), (0.6, -0.7071), (0.6 + 0.7071, 0), (0.6, 0.7071)
        // => (-0.1071, 0), (0.6, -0.7071), (1.3071, 0), (0.6, 0.7071)
        // Projection of B onto x-axis: [-0.1071, 1.3071]
        // Projection of A onto x-axis: [-0.5, 0.5]
        // Overlap on x-axis = min(0.5, 1.3071) - max(-0.5, -0.1071) = 0.5 - (-0.1071) = 0.6071
        // Now check A's y-axis: Proj A = [-0.5, 0.5]. Proj B = [-0.7071, 0.7071].
        // Overlap = min(0.5, 0.7071) - max(-0.5, -0.7071) = 0.5 - (-0.5) = 1.0
        // Now check B's axes (rotated by pi/4, e.g., (1,1)/sqrt(2) )
        // Axis = (0.7071, 0.7071).
        // Proj A: vertices (-0.5,-0.5) -> -0.7071; (0.5,-0.5) -> 0; (0.5,0.5) -> 0.7071; (-0.5,0.5) -> 0.  Interval [ -0.7071, 0.7071 ]
        // Proj B: vertices (-0.1071, 0) -> -0.0757; (0.6, -0.7071) -> -0.0757; (1.3071, 0) -> 0.9243; (0.6, 0.7071) -> 0.9243. Interval [-0.0757, 0.9243]
        // Overlap = min(0.7071, 0.9243) - max(-0.7071, -0.0757) = 0.7071 - (-0.0757) = 0.7828
        // Minimum overlap was 0.6071 along the x-axis.

        assert!((m.depth - 0.6071).abs() < 1e-4, "Depth: {}", m.depth); // Allow slightly larger tolerance due to rotation math
    }

} 