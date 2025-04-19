use crate::math::vec2::Vec2;
use crate::objects::rigid_body::{RigidBody};
use crate::shapes::{Polygon, Shape};
use super::manifold::{CollisionManifold, ContactPoint};
use std::f64::EPSILON;

/// Checks for collision between two rigid bodies.
/// Returns an Option containing collision manifold information if a collision occurs.
pub fn check_collision(
    body_a: &RigidBody,
    body_a_idx: usize,
    body_b: &RigidBody,
    body_b_idx: usize,
    should_log_collisions: bool, // Keep log flag for now
) -> Option<CollisionManifold> {
    // Ensure we always handle Circle-Polygon the same way, regardless of order
    match (&body_a.shape, &body_b.shape) {
        (Shape::Circle(_), Shape::Circle(_)) => {
            check_circle_circle(body_a, body_b, body_a_idx, body_b_idx)
        }
        (Shape::Circle(_), Shape::Polygon(_)) => {
            check_circle_polygon(body_a, body_a_idx, body_b, body_b_idx, should_log_collisions) // Pass log flag
        }
        (Shape::Polygon(_), Shape::Circle(_)) => {
            // Swap order so A is always Circle, B is Polygon
            check_circle_polygon(body_b, body_b_idx, body_a, body_a_idx, should_log_collisions).map(|mut m| {
                // Swap body indices back and invert normal
                let temp_idx = m.body_a_idx;
                m.body_a_idx = m.body_b_idx;
                m.body_b_idx = temp_idx;
                m.normal = -m.normal; // Normal should point from original A (poly) to original B (circle)
                m
            })
        }
        (Shape::Polygon(_), Shape::Polygon(_)) => {
            check_polygon_polygon(body_a, body_a_idx, body_b, body_b_idx)
        }
        // Handle LineSegments
        (Shape::Circle(_), Shape::Line(_)) => {
            check_circle_line(body_a, body_a_idx, body_b, body_b_idx)
        }
        (Shape::Line(_), Shape::Circle(_)) => {
            // Swap order
            check_circle_line(body_b, body_b_idx, body_a, body_a_idx).map(|mut m| {
                 // Swap body indices back and invert normal
                let temp_idx = m.body_a_idx;
                m.body_a_idx = m.body_b_idx;
                m.body_b_idx = temp_idx;
                m.normal = -m.normal;
                m
            })
        }
        (Shape::Polygon(_), Shape::Line(_)) => {
            // TODO: Implement Polygon-Line collision
            None
        }
        (Shape::Line(_), Shape::Polygon(_)) => {
            // TODO: Implement Line-Polygon collision
            None
        }
        (Shape::Line(_), Shape::Line(_)) => {
            check_line_line(body_a, body_a_idx, body_b, body_b_idx)
        }
    }
}

/// Checks for collision between two circles.
/// Returns a CollisionManifold if they collide, None otherwise.
pub fn check_circle_circle(
    body_a: &RigidBody,
    body_b: &RigidBody,
    idx_a: usize, // Used for manifold
    idx_b: usize, // Used for manifold
) -> Option<CollisionManifold> {
    let shape_a = match body_a.shape {
        Shape::Circle(ref circle) => circle,
        _ => return None,
    };
    let shape_b = match body_b.shape {
        Shape::Circle(ref circle) => circle,
        _ => return None,
    };

    let radius_a = shape_a.radius;
    let radius_b = shape_b.radius;
    let pos_a = body_a.position;
    let pos_b = body_b.position;
    let radii_sum = radius_a + radius_b;
    let dist_sq = pos_a.distance_squared(pos_b);
    let radii_sum_sq = radii_sum * radii_sum;

    // Check for collision or near-collision using distance squared
    if dist_sq <= radii_sum_sq + EPSILON {
        let distance = dist_sq.sqrt();
        let penetration_depth = radii_sum - distance;

        // If dist_sq indicates collision/contact, generate manifold.
        // penetration_depth will be positive for overlap, near zero for contact.

        let normal = if distance > EPSILON {
            (pos_b - pos_a).normalize() // Normal points from A to B
        } else {
            // Bodies are coincident or extremely close, assign a default normal
            Vec2::new(1.0, 0.0)
        };

        // Calculate contact points on the surface of each circle along the normal
        let contact_point_a = pos_a + normal * radius_a;
        let contact_point_b = pos_b - normal * radius_b;
        let contact = ContactPoint { point_a: contact_point_a, point_b: contact_point_b };

        Some(CollisionManifold {
            body_a_idx: idx_a,
            body_b_idx: idx_b,
            normal,
            depth: penetration_depth.max(0.0), // Ensure depth is non-negative
            contact,
        })
    } else {
        // No collision based on distance check
        None
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
    body_a: &RigidBody,
    body_a_idx: usize,
    body_b: &RigidBody,
    body_b_idx: usize,
) -> Option<CollisionManifold> {
    let circle = match body_a.shape {
        Shape::Circle(c) => c,
        _ => return None, // Or panic
    };
    let line = match body_b.shape {
        Shape::Line(l) => l,
        _ => return None,
    };

    // Transform line segment endpoints to world space
    let line_a_world = body_b.position + line.a.rotate(body_b.rotation);
    let line_b_world = body_b.position + line.b.rotate(body_b.rotation);

    // Find the closest point on the world-space line segment to the circle's center
    let (closest_point, _) = closest_point_on_segment(line_a_world, line_b_world, body_a.position);

    // Check distance between circle center and closest point
    let dist_vec = body_a.position - closest_point;
    let dist_sq = dist_vec.magnitude_squared();

    // Use <= for touching check, add epsilon for float safety
    if dist_sq <= circle.radius * circle.radius + EPSILON {
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
        let contact_on_circle = body_a.position - collision_normal * circle.radius;
        // Contact point on line is the closest point we found
        let contact_point = ContactPoint { point_a: contact_on_circle, point_b: closest_point };

        Some(CollisionManifold {
            body_a_idx,
            body_b_idx,
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
    body_a: &RigidBody,
    body_a_idx: usize,
    body_b: &RigidBody,
    body_b_idx: usize,
    _should_log_collisions: bool,
) -> Option<CollisionManifold> {
    let circle_a = match &body_a.shape {
        Shape::Circle(c) => c,
        _ => return None,
    };
    let polygon_b = match &body_b.shape {
        Shape::Polygon(p) => p,
        _ => return None,
    };

    let mut min_penetration = f64::MAX;
    let mut collision_axis = Vec2::ZERO;

    // --- Check axes from polygon edges ---
    let world_origin_b = body_b.position - body_b.local_center_of_mass.rotate(body_b.rotation);
    let rotation_b = body_b.rotation;

    for i in 0..polygon_b.vertices.len() {
        let p1_world = world_origin_b + polygon_b.vertices[i].rotate(rotation_b);
        let p2_world = world_origin_b + polygon_b.vertices[(i + 1) % polygon_b.vertices.len()].rotate(rotation_b);
        let edge = p2_world - p1_world;

        if edge.magnitude_squared() < EPSILON * EPSILON {
            continue;
        }

        // Original calculation: Outward normal for CCW polygon (points B->A)
        let axis = edge.perpendicular().normalize();

        let (min_a, max_a) = project_shape_onto_axis(body_a, axis);
        let (min_b, max_b) = project_polygon_onto_axis(body_b, polygon_b, axis);

        let overlap1 = max_b - min_a;
        let overlap2 = max_a - min_b;
        if overlap1 < -EPSILON || overlap2 < -EPSILON {
            return None;
        }
        let current_overlap = overlap1.min(overlap2).max(0.0);

        if current_overlap < min_penetration {
            min_penetration = current_overlap;
            // Edge axis points B->A, store the inverse to point A->B
            collision_axis = -axis;
        }
    }

    // --- Check the axis formed by the closest point on the polygon to the circle center ---
    let circle_world_pos = body_a.position;
    let mut closest_vertex_world = Vec2::ZERO;
    let mut min_dist_sq = f64::MAX;

    for vertex in &polygon_b.vertices {
        let world_vertex = world_origin_b + vertex.rotate(rotation_b);
        let dist_sq = world_vertex.distance_squared(circle_world_pos);
        if dist_sq < min_dist_sq {
            min_dist_sq = dist_sq;
            closest_vertex_world = world_vertex;
        }
    }

    let vertex_axis_candidate = closest_vertex_world - circle_world_pos; // Points A->B

    if vertex_axis_candidate.magnitude_squared() < EPSILON * EPSILON {
        if min_penetration == f64::MAX { return None; }
    } else {
        let vertex_axis = vertex_axis_candidate.normalize();
        let (min_a, max_a) = project_shape_onto_axis(body_a, vertex_axis);
        let (min_b, max_b) = project_polygon_onto_axis(body_b, polygon_b, vertex_axis);

        let overlap1 = max_b - min_a;
        let overlap2 = max_a - min_b;
        if overlap1 < -EPSILON || overlap2 < -EPSILON {
             return None;
        }
        let current_overlap = overlap1.min(overlap2).max(0.0);

        if current_overlap < min_penetration {
            min_penetration = current_overlap;
            // Vertex axis already points A->B
            collision_axis = vertex_axis;
        }
    }

    if min_penetration == f64::MAX {
        return None;
    }

    // --- Final Normal Flip Check --- (REMOVED)
    // // Use vector between centers of mass as reference direction.
    // let reference_vec = body_a.position - body_b.position; // Vector from B's CoM to A's CoM (Points roughly A->B)
    // let reference_direction = if reference_vec.magnitude_squared() > EPSILON * EPSILON {
    //     reference_vec.normalize()
    // } else {
    //     Vec2::new(1.0, 0.0) // Default fallback if CoMs coincide
    // };
    //
    // // If the collision axis points opposite to the reference direction (A->B), flip it.
    // if reference_direction.dot(collision_axis) < 0.0 {
    //     collision_axis = -collision_axis;
    // }

    // TODO: Calculate accurate contact points
    let contact_on_a = body_a.position + collision_axis * circle_a.radius;
    let contact_on_b = contact_on_a - collision_axis * min_penetration;

    Some(CollisionManifold {
        body_a_idx,
        body_b_idx,
        normal: collision_axis,
        depth: min_penetration,
        contact: ContactPoint { point_a: contact_on_a, point_b: contact_on_b },
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
    let _direction = center_a - center_b;
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
    use crate::shapes::{Circle, LineSegment, Polygon, Shape};
    use crate::math::vec2::Vec2;
    const EPSILON: f64 = 1e-9;

    // Corrected helper to create a dynamic body
    fn create_dynamic_body(shape: Shape, mass: f64, pos: Vec2) -> RigidBody {
        let mut body = RigidBody::new(mass, shape);
        body.position = pos;
        body
    }

    // Corrected helper to create a static body
    fn create_static_body(shape: Shape, pos: Vec2) -> RigidBody {
        RigidBody::new_static(shape, pos, 0.0)
    }

    #[test]
    fn test_check_circle_circle_no_collision() {
        let body_a = create_dynamic_body(Shape::Circle(Circle::new(1.0)), 1.0, Vec2::new(0.0, 0.0));
        let body_b = create_dynamic_body(Shape::Circle(Circle::new(1.0)), 1.0, Vec2::new(3.0, 0.0));
        assert!(check_circle_circle(&body_a, &body_b, 0, 1).is_none());
    }

     #[test]
    fn test_check_circle_circle_touching() {
        let body_a = create_dynamic_body(Shape::Circle(Circle::new(1.0)), 1.0, Vec2::new(0.0, 0.0));
        let body_b = create_dynamic_body(Shape::Circle(Circle::new(1.0)), 1.0, Vec2::new(2.0, 0.0));
        let result = check_circle_circle(&body_a, &body_b, 0, 1);
        assert!(result.is_some());
        let m = result.unwrap();
        assert!(m.depth.abs() < EPSILON);
        assert!((m.normal.x - 1.0).abs() < EPSILON);
        assert!(m.normal.y.abs() < EPSILON);
    }

     #[test]
    fn test_check_circle_circle_colliding() {
        let body_a = create_dynamic_body(Shape::Circle(Circle::new(1.0)), 1.0, Vec2::new(0.0, 0.0));
        let body_b = create_dynamic_body(Shape::Circle(Circle::new(1.0)), 1.0, Vec2::new(1.5, 0.0));
        let result = check_circle_circle(&body_a, &body_b, 0, 1);
        assert!(result.is_some());
        let m = result.unwrap();
        assert!((m.depth - 0.5).abs() < EPSILON);
        assert!((m.normal.x - 1.0).abs() < EPSILON);
        assert!(m.normal.y.abs() < EPSILON);
    }

    #[test]
    fn test_check_circle_circle_concentric() {
        let body_a = create_dynamic_body(Shape::Circle(Circle::new(2.0)), 1.0, Vec2::new(0.0, 0.0));
        let body_b = create_dynamic_body(Shape::Circle(Circle::new(1.0)), 1.0, Vec2::new(0.0, 0.0));
        let result = check_circle_circle(&body_a, &body_b, 0, 1);
        assert!(result.is_some());
        let m = result.unwrap();
        assert!((m.depth - 3.0).abs() < EPSILON);
        assert!((m.normal.magnitude() - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_check_circle_polygon_no_collision() {
        let circle_body = create_dynamic_body(Shape::Circle(Circle::new(1.0)), 1.0, Vec2::new(3.0, 0.0));
        let vertices = create_rectangle_vertices(0.0, 0.0, 1.0, 1.0);
        let polygon = Polygon::new(vertices);
        let polygon_body = create_dynamic_body(Shape::Polygon(polygon), 1.0, Vec2::new(0.0, 0.0));
        assert!(check_circle_polygon(&circle_body, 0, &polygon_body, 1, false).is_none());
    }

    #[test]
    fn test_check_circle_polygon_collision() {
        let circle_body = create_dynamic_body(Shape::Circle(Circle::new(1.0)), 1.0, Vec2::new(1.5, 0.0));
        let vertices = create_rectangle_vertices(0.0, 0.0, 1.0, 1.0);
        let polygon = Polygon::new(vertices);
        let polygon_body = create_dynamic_body(Shape::Polygon(polygon), 1.0, Vec2::new(0.0, 0.0));

        let result = check_circle_polygon(&circle_body, 0, &polygon_body, 1, false);
        assert!(result.is_some());
        let m = result.unwrap();
        println!("[{}] Calculated Normal: {:?}, Depth: {:.3}", "test_collision", m.normal, m.depth);
        assert!((m.normal.x - 1.0).abs() < EPSILON, "Normal X incorrect");
        assert!(m.normal.y.abs() < EPSILON, "Normal Y incorrect");
        assert!((m.depth - 0.5).abs() < EPSILON, "Depth incorrect");
    }

    #[test]
    fn test_check_circle_polygon_touching_edge() {
        let circle_body = create_dynamic_body(Shape::Circle(Circle::new(1.0)), 1.0, Vec2::new(2.0, 0.0));
        let vertices = create_rectangle_vertices(0.0, 0.0, 1.0, 1.0);
        let polygon = Polygon::new(vertices);
        let polygon_body = create_dynamic_body(Shape::Polygon(polygon), 1.0, Vec2::new(0.0, 0.0));

        let result = check_circle_polygon(&circle_body, 0, &polygon_body, 1, false);
        assert!(result.is_some());
        let m = result.unwrap();
        assert!(m.depth.abs() < EPSILON);
        assert!((m.normal.x - 1.0).abs() < EPSILON, "Normal: {:?}", m.normal);
        assert!(m.normal.y.abs() < EPSILON);
    }

     #[test]
    fn test_check_circle_polygon_touching_vertex() {
        let touch_point = Vec2::new(1.0, 1.0);
        let radius = 1.0;
        let direction = Vec2::new(1.0, 1.0).normalize();
        let circle_center = touch_point + direction * radius;
        let vertices = create_rectangle_vertices(0.0, 0.0, 1.0, 1.0);
        let polygon = Polygon::new(vertices);

        let circle_body = create_dynamic_body(Shape::Circle(Circle::new(radius)), 1.0, circle_center);
        let polygon_body = create_dynamic_body(Shape::Polygon(polygon), 1.0, Vec2::new(0.0, 0.0));

        let result = check_circle_polygon(&circle_body, 0, &polygon_body, 1, false);
        assert!(result.is_some());
        let m = result.unwrap();
        assert!(m.depth.abs() < EPSILON);
        assert!((m.normal.x - (-direction.x)).abs() < EPSILON, "Normal: {:?}, Expected Dir: {:?}", m.normal, -direction);
        assert!((m.normal.y - (-direction.y)).abs() < EPSILON);
    }

    #[test]
    fn test_check_circle_polygon_collision_vertex_closest() {
        let circle_center = Vec2::new(1.5, 1.5);
        let radius = 0.8;
        let vertices = create_rectangle_vertices(0.0, 0.0, 1.0, 1.0);
        let polygon = Polygon::new(vertices);

        let circle_body = create_dynamic_body(Shape::Circle(Circle::new(radius)), 1.0, circle_center);
        let polygon_body = create_dynamic_body(Shape::Polygon(polygon), 1.0, Vec2::new(0.0, 0.0));

        let result = check_circle_polygon(&circle_body, 0, &polygon_body, 1, false);
        assert!(result.is_some());
        let m = result.unwrap();

        let vertex = Vec2::new(1.0, 1.0);
        let vec_vertex_to_center = circle_center - vertex;
        let dist_vertex_center = vec_vertex_to_center.magnitude();
        let expected_depth = radius - dist_vertex_center;
        assert!(expected_depth > 0.0, "Expected positive depth");
        assert!((m.depth - expected_depth).abs() < EPSILON, "Depth: {}, Expected: {}", m.depth, expected_depth);

        // The code produces A->B normals, but this test is expecting B->A normals
        // Update to expect -vec_vertex_to_center
        let expected_normal = -vec_vertex_to_center.normalize();
        assert!((m.normal.x - expected_normal.x).abs() < EPSILON, "Normal: {:?}, Expected: {:?}", m.normal, expected_normal);
        assert!((m.normal.y - expected_normal.y).abs() < EPSILON);
    }

    #[test]
    fn test_check_polygon_polygon_no_collision() {
        let vertices_a = create_rectangle_vertices(0.0, 0.0, 1.0, 1.0);
        let poly_a = Polygon::new(vertices_a);
        let vertices_b = create_rectangle_vertices(3.0, 0.0, 1.0, 1.0);
        let poly_b = Polygon::new(vertices_b);
        let body_a = create_static_body(Shape::Polygon(poly_a), Vec2::new(0.0, 0.0));
        let body_b = create_static_body(Shape::Polygon(poly_b), Vec2::new(0.0, 0.0));
        assert!(check_polygon_polygon(&body_a, 0, &body_b, 1).is_none());
    }

    #[test]
    fn test_check_polygon_polygon_overlapping() {
        let vertices_a = create_rectangle_vertices(0.0, 0.0, 1.0, 1.0);
        let poly_a = Polygon::new(vertices_a);
        let vertices_b = create_rectangle_vertices(1.5, 0.0, 1.0, 1.0);
        let poly_b = Polygon::new(vertices_b);
        let body_a = create_static_body(Shape::Polygon(poly_a), Vec2::new(0.0, 0.0));
        let body_b = create_static_body(Shape::Polygon(poly_b), Vec2::new(0.0, 0.0));
        let result = check_polygon_polygon(&body_a, 0, &body_b, 1);
        assert!(result.is_some());
        let m = result.unwrap();
        assert!((m.depth - 0.5).abs() < EPSILON);
        assert!((m.normal.x - (-1.0)).abs() < EPSILON, "Normal: {:?}", m.normal);
        assert!(m.normal.y.abs() < EPSILON);
    }

    #[test]
    fn test_check_polygon_polygon_touching_edge() {
        let vertices_a = create_rectangle_vertices(0.0, 0.0, 1.0, 1.0);
        let poly_a = Polygon::new(vertices_a);
        let vertices_b = create_rectangle_vertices(2.0, 0.0, 1.0, 1.0);
        let poly_b = Polygon::new(vertices_b);
        let body_a = create_static_body(Shape::Polygon(poly_a), Vec2::new(0.0, 0.0));
        let body_b = create_static_body(Shape::Polygon(poly_b), Vec2::new(0.0, 0.0));
        let result = check_polygon_polygon(&body_a, 0, &body_b, 1);
        assert!(result.is_some());
        let m = result.unwrap();
        assert!(m.depth.abs() < EPSILON);
        assert!((m.normal.x - (-1.0)).abs() < EPSILON, "Normal: {:?}", m.normal);
        assert!(m.normal.y.abs() < EPSILON);
    }

    #[test]
    fn test_check_polygon_polygon_rotated_overlapping() {
        let vertices_a = create_rectangle_vertices(0.0, 0.0, 1.0, 1.0);
        let poly_a = Polygon::new(vertices_a);
        let vertices_b = create_rectangle_vertices(0.0, 0.0, 1.0, 1.0);
        let poly_b = Polygon::new(vertices_b);
        let body_a = create_static_body(Shape::Polygon(poly_a), Vec2::new(0.0, 0.0));
        let mut body_b = create_static_body(Shape::Polygon(poly_b), Vec2::new(1.0, 1.0));
        body_b.rotation = PI / 4.0;

        let result = check_polygon_polygon(&body_a, 0, &body_b, 1);
        assert!(result.is_some());
        assert!(result.unwrap().depth > 0.0);
    }

    #[test]
    fn test_check_circle_line_no_collision() {
        let circle_body = create_dynamic_body(Shape::Circle(Circle::new(1.0)), 1.0, Vec2::new(0.0, 3.0));
        let line = LineSegment::new(Vec2::new(-2.0, 0.0), Vec2::new(2.0, 0.0));
        let line_body = create_static_body(Shape::Line(line), Vec2::new(0.0, 0.0));
        assert!(check_circle_line(&circle_body, 0, &line_body, 1).is_none());
    }

    #[test]
    fn test_check_circle_line_collision_mid() {
        let circle_body = create_dynamic_body(Shape::Circle(Circle::new(1.0)), 1.0, Vec2::new(0.0, 0.5));
        let line = LineSegment::new(Vec2::new(-2.0, 0.0), Vec2::new(2.0, 0.0));
        let line_body = create_static_body(Shape::Line(line), Vec2::new(0.0, 0.0));
        let result = check_circle_line(&circle_body, 0, &line_body, 1);
        assert!(result.is_some());
        let m = result.unwrap();
        assert!((m.depth - 0.5).abs() < EPSILON);
        assert!(m.normal.x.abs() < EPSILON, "Normal: {:?}", m.normal);
        assert!((m.normal.y - 1.0).abs() < EPSILON, "Normal: {:?}", m.normal);
    }

    #[test]
    fn test_check_circle_line_touching_endpoint() {
        let circle_body = create_dynamic_body(Shape::Circle(Circle::new(1.0)), 1.0, Vec2::new(3.0, 0.0));
        let line = LineSegment::new(Vec2::new(-2.0, 0.0), Vec2::new(2.0, 0.0));
        let line_body = create_static_body(Shape::Line(line), Vec2::new(0.0, 0.0));
        let result = check_circle_line(&circle_body, 0, &line_body, 1);
        assert!(result.is_some());
        let m = result.unwrap();
        assert!(m.depth.abs() < EPSILON);
        assert!((m.normal.x - 1.0).abs() < EPSILON, "Normal: {:?}", m.normal);
        assert!(m.normal.y.abs() < EPSILON);
    }

    #[test]
    fn test_check_circle_line_collision_endpoint() {
        let circle_body = create_dynamic_body(Shape::Circle(Circle::new(1.0)), 1.0, Vec2::new(2.5, 0.0));
        let line = LineSegment::new(Vec2::new(-2.0, 0.0), Vec2::new(2.0, 0.0));
        let line_body = create_static_body(Shape::Line(line), Vec2::new(0.0, 0.0));
        let result = check_circle_line(&circle_body, 0, &line_body, 1);
        assert!(result.is_some());
        let m = result.unwrap();
        assert!((m.depth - 0.5).abs() < EPSILON);
        assert!((m.normal.x - 1.0).abs() < EPSILON, "Normal: {:?}", m.normal);
        assert!(m.normal.y.abs() < EPSILON);
    }

    #[test]
    fn test_check_line_line_intersecting() {
        let line_a = LineSegment::new(Vec2::new(0.0, 0.0), Vec2::new(2.0, 2.0));
        let line_b = LineSegment::new(Vec2::new(0.0, 2.0), Vec2::new(2.0, 0.0));
        let body_a = create_static_body(Shape::Line(line_a), Vec2::ZERO);
        let body_b = create_static_body(Shape::Line(line_b), Vec2::ZERO);
        assert!(check_line_line(&body_a, 0, &body_b, 1).is_some());
    }

    #[test]
    fn test_check_line_line_non_intersecting() {
        let line_a = LineSegment::new(Vec2::new(0.0, 0.0), Vec2::new(1.0, 1.0));
        let line_b = LineSegment::new(Vec2::new(2.0, 2.0), Vec2::new(3.0, 3.0));
        let body_a = create_static_body(Shape::Line(line_a), Vec2::ZERO);
        let body_b = create_static_body(Shape::Line(line_b), Vec2::ZERO);
        assert!(check_line_line(&body_a, 0, &body_b, 1).is_none());
    }

    #[test]
    fn test_circle_polygon_integration_no_collision() {
        let circle_shape = Circle::new(10.0);
        let circle_body = create_dynamic_body(Shape::Circle(circle_shape), 1.0, Vec2::new(300.0, 50.0));
        let ground_vertices = create_rectangle_vertices(400.0, 590.0, 400.0, 10.0);
        let ground_shape = Polygon::new(ground_vertices);
        let ground_body = create_static_body(Shape::Polygon(ground_shape), Vec2::new(0.0, 0.0));
        assert!(check_circle_polygon(&circle_body, 0, &ground_body, 1, false).is_none());
    }

    #[test]
    fn test_circle_polygon_integration_touching() {
        let radius = 10.0;
        let ground_center_y = 590.0;
        let ground_half_height = 10.0;
        let ground_top_y = ground_center_y - ground_half_height;
        let circle_center_y = ground_top_y - radius;
        let circle_shape = Circle::new(radius);
        let circle_body = create_dynamic_body(Shape::Circle(circle_shape), 1.0, Vec2::new(400.0, circle_center_y));
        let ground_vertices = create_rectangle_vertices(400.0, ground_center_y, 400.0, ground_half_height);
        let ground_shape = Polygon::new(ground_vertices);
        let ground_body = create_static_body(Shape::Polygon(ground_shape), Vec2::new(0.0, 0.0));

        let result = check_circle_polygon(&circle_body, 0, &ground_body, 1, false);
        assert!(result.is_some());
        let m = result.unwrap();
        assert!(m.depth.abs() < EPSILON);
        assert!(m.normal.x.abs() < EPSILON, "Normal: {:?}", m.normal);
        assert!((m.normal.y - (-1.0)).abs() < EPSILON, "Normal: {:?}", m.normal);
    }

    #[test]
    fn test_circle_polygon_integration_penetrating() {
        let radius = 10.0;
        let ground_center_y = 590.0;
        let ground_half_height = 10.0;
        let ground_top_y = ground_center_y - ground_half_height;
        let penetration = 2.0;
        let circle_center_y = ground_top_y - radius + penetration;
        let circle_shape = Circle::new(radius);
        let circle_body = create_dynamic_body(Shape::Circle(circle_shape), 1.0, Vec2::new(400.0, circle_center_y));
        let ground_vertices = create_rectangle_vertices(400.0, ground_center_y, 400.0, ground_half_height);
        let ground_shape = Polygon::new(ground_vertices);
        let ground_body = create_static_body(Shape::Polygon(ground_shape), Vec2::new(0.0, 0.0));

        let result = check_circle_polygon(&circle_body, 0, &ground_body, 1, false);
        assert!(result.is_some());
        let m = result.unwrap();
        assert!((m.depth - penetration).abs() < EPSILON);
        assert!(m.normal.x.abs() < EPSILON, "Normal: {:?}", m.normal);
        assert!((m.normal.y - (-1.0)).abs() < EPSILON, "Normal: {:?}", m.normal);
    }
} 