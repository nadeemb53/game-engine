# Physics Engine

**A fast, reliable 2D physics engine written in Rust.**

 ## Overview
 
 This engine aims to provide a fast and memory-safe foundation for 2D physics in games and simulations. It includes the essentials:
 
 *   Vector math (`Vec2`)
 *   Rigid bodies with mass, inertia, position, linear/angular velocity, forces, and torque.
 *   Shapes: `Circle`, `LineSegment`, `Polygon` (convex).
 *   Constraint system (e.g., `DistanceConstraint`, `PinJoint`).
 *   Collision detection for `Circle`/`Circle`, `Circle`/`Line`, `Line`/`Line`, and `Circle`/`Polygon` pairs (using SAT).
 *   Collision resolution: Impulse-based (with basic restitution) + positional correction stabilization.
 *   A `PhysicsWorld` to manage all the bodies, constraints, and simulation steps.
 
 ## Features
 
 *   **Rigid Body Dynamics:** Simulates 2D rigid body motion (position and rotation) using a stable Semi-Implicit Euler integrator.
 *   **Shapes:** Supports `Circle`, `LineSegment`, and convex `Polygon` shapes. Mass properties for polygons are calculated automatically.
 *   **Constraints:** Includes basics like `DistanceConstraint` (fixed links) and `PinJoint` (like a pin on a board).
 *   **Collision Detection:** Detects collisions between all supported shape pairs.
 *   **Collision Resolution:** Uses physics-based impulses for realistic bounces (restitution) and positional correction to keep things stable.
 *   **Extensible Design:** Built with modules and traits (like `Constraint`) making it easier to add new features later.
 *   **Why Rust?** Built in Rust for performance (fast execution, no GC pauses) and reliability (memory safety helps prevent many common crashes).
 
 ## Modules
 
 *   `math`: Core 2D vector (`Vec2`) type and operations.
 *   `shapes`: Defines the geometric shapes (`Circle`, `LineSegment`, `Polygon`, `Shape` enum).
 *   `objects`: Contains the `RigidBody` struct.
 *   `integration`: Numerical integration scheme (currently `Semi-Implicit Euler`).
 *   `constraints`: Code for constraints like `DistanceConstraint`, `PinJoint`, and the `Constraint` trait.
 *   `collision`: Handles collision detection logic and the `CollisionManifold` (contact info).
 *   `world`: Contains the `PhysicsWorld` which brings everything together.
 
 ## Getting Started (Basic Usage)
 
 ```rust
 use physics_engine::{
     world::PhysicsWorld,
     objects::RigidBody,
     shapes::{Shape, Circle},
     constraints::DistanceConstraint,
     math::vec2::Vec2,
 };

 fn main() {
     // Create a world
     let mut world = PhysicsWorld::new();
     world.gravity = Vec2::new(0.0, -10.0);

     // Add bodies (static anchor and dynamic circle)
     let static_shape = Shape::Circle(Circle::new(0.1)); // Shape doesn't matter much for static point
     let bob_shape = Shape::Circle(Circle::new(0.5));

     let static_idx = world.add_static_body(static_shape, Vec2::new(5.0, 5.0), 0.0);
     let bob_idx = world.add_body(RigidBody::new(1.0, bob_shape));
     world.bodies[bob_idx].position = Vec2::new(7.0, 5.0);

     // Add a constraint (pendulum)
     let constraint = DistanceConstraint::new(
         static_idx,
         bob_idx,
         Vec2::new(0.0, 0.0), // Anchor at center of static body
         Vec2::new(0.0, 0.0), // Anchor at center of bob
         2.0                  // Target distance
     );
     world.add_distance_constraint_specific(constraint);

     // Simulation loop
     let dt = 1.0 / 60.0; // Time step for ~60 FPS
     for _ in 0..100 { // Simulate 100 steps
         world.step(dt);
         // In a real application, you would get body positions and render them here
         // println!("Bob position: {:?}", world.bodies[bob_idx].position);
     }
     println!("Simulation finished. Final bob position: {:?}", world.bodies[bob_idx].position);
 }
 ```

 ## Performance
 
 Speed is important! While the engine is still evolving and major optimizations like broadphase detection are planned, the current benchmarks give a rough idea.
 
 _(Run on [Your CPU/OS])_ with `cargo bench`:
 
 *   **`circle_stack_10`**: [Update with latest if available]
     *   Simulates 10 dynamic circles + 1 static ground circle falling for 30 steps (4 solver iterations).
 *   **`constraint_chain_10`**: [Update with latest if available]
     *   Simulates 10 dynamic circles linked by PinJoints to a static anchor, falling for 30 steps (8 solver iterations).
 
 **Planned Benchmark Scenarios (Post-Optimization):**
 *   `polygon_stack_N`: Measuring performance with stacked polygons.
 *   `large_mixed_scene_N`: Seeing how it handles lots of different shapes interacting.
 *   `high_constraint_scene_N`: Testing scenes with many joints and constraints.
 
 *(Current benchmarks are preliminary. More comprehensive and representative benchmarks will be added as core optimizations are implemented.)*
 
 ## Roadmap / Future Enhancements
 
 Here's what's planned next to make the engine more capable for complex games or simulations:
 
 1.  **More Shapes & Geometry:**
     *   (Completed) Convex Polygon support with automated mass property calculation.
     *   Concave polygon support (likely via decomposition).
 2.  **Advanced Collision Detection:**
     *   (Completed) Circle-Polygon collision detection (SAT).
     *   **Polygon-Polygon:** Implement collision detection between pairs of convex polygons (e.g., using the Separating Axis Theorem - SAT).
 3.  **Material System & Friction:**
     *   Introduce a `Material` struct to define properties like friction (static/kinetic) and restitution.
     *   Associate materials with rigid bodies/shapes.
     *   Enhance collision resolution to calculate and apply friction impulses (tangential to the collision normal).
     *   Use material properties to determine friction/restitution coefficients during collision resolution.
 4.  **Performance Boost:**
     *   Implement broadphase collision detection (e.g., Grid, Quadtree) to accelerate pair finding.
 5.  **More Physics!**
     *   Air resistance / Drag models.
     *   More constraint types (e.g., motors, angular limits).
 6.  **Usability:**
     *   Refinement of the public API for improved ergonomics.
     *   Comprehensive documentation generation.
