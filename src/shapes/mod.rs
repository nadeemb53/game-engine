pub mod circle;
pub mod line_segment;
pub mod polygon;
// pub mod shape; // Define enum here instead

// Re-export the specific shape types
pub use circle::Circle;
pub use line_segment::LineSegment;
pub use polygon::Polygon;

/// Enum representing the geometric shape of a rigid body.
#[derive(Debug, Clone, PartialEq)]
pub enum Shape {
    Circle(Circle),
    Line(LineSegment),
    Polygon(Polygon),
} 