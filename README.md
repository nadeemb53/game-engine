# Physics Engine

**A 2D rigid-body physics engine written in Rust.**

 ## Overview
 
 This is a 2D physics engine designed for simulations and games. Built with Rust, it aims to be fast and reliable. Here's what it offers:
 
 *   Vector math (`Vec2`)
 *   Rigid bodies with mass, inertia, position, linear/angular velocity, forces, and torque.
 *   Shapes: `Circle`, `LineSegment`, `Polygon` (convex).
 *   Constraint system (e.g., `DistanceConstraint`, `PinJoint`).
 *   Collision detection for various shape pairs (`Circle`/`Circle`, `Circle`/`Line`, `Line`/`Line`, `Circle`/`Polygon`, `Polygon`/`Polygon`) using techniques like the Separating Axis Theorem (SAT).
 *   Collision resolution: Impulse-based (with restitution/bounciness) + positional correction for stability.
 *   A `PhysicsWorld` to manage bodies, constraints, and simulation steps.
 
 ## Features
 
 *   **Rigid Body Dynamics:** Simulates how objects move and rotate using Semi-Implicit Euler integration.
 *   **Shapes:** Supports `Circle`, `LineSegment`, and convex `Polygon` shapes. Calculates mass and inertia automatically for polygons.
 *   **Constraints:** Includes common joints like `DistanceConstraint` (fixed links) and `PinJoint`.
 *   **Collision Detection:** Detects collisions between all supported shape pairs, using a spatial grid for efficiency with many objects.
 *   **Collision Resolution:** Handles collisions realistically using impulses (for bounces) and positional correction (to prevent sinking).
 *   **Materials:** Bodies have `Material` properties (friction, restitution) affecting collisions.
 *   **Extensible:** Designed in modules, making it easier to add new shapes, constraints, or features.
 *   **Built with Rust:** Aims for good performance and fewer bugs thanks to Rust's focus on safety and speed.
 
 ## Modules
 
 *   `math`: Basic 2D vector (`Vec2`) math.
 *   `shapes`: Shape definitions (`Circle`, `LineSegment`, `Polygon`, etc.).
 *   `objects`: The `RigidBody` definition.
 *   `common`: Shared components like `Material`.
 *   `integration`: Code for updating motion (integrator).
 *   `constraints`: Joints and links between bodies.
 *   `collision`: Collision detection (AABB, SpatialGrid, SAT) and contact info (`CollisionManifold`).
 *   `world`: The main `PhysicsWorld` that manages the simulation.
 
 ## Getting Started (Basic Usage)
 
 ```rust
 use physics_engine::{
     PhysicsWorld,
     RigidBody,
     Shape,
     Circle,
     Vec2,
     Material,
 };
 
 fn main() {
     // 1. Create a world
     let mut world = PhysicsWorld::new();
     world.gravity = Vec2::new(0.0, -10.0);
 
     // 2. Add bodies
     // A static body (e.g., the ground)
     let ground_shape = Shape::Circle(Circle::new(0.1)); // Small circle as a fixed point
     let static_body_idx = world.add_static_body(ground_shape, Vec2::new(5.0, 0.0), 0.0);

     // A dynamic body (e.g., a falling ball)
     let ball_shape = Shape::Circle(Circle::new(0.5));
     let mut ball_body = RigidBody::new(1.0, ball_shape); // mass = 1.0
     ball_body.position = Vec2::new(5.0, 5.0); // Start above the static body
     // Optionally set a custom material
     ball_body.material = Material::new(0.7, 0.4); // Bouncy, moderate friction
     let ball_body_idx = world.add_body(ball_body);
 
     // 3. Simulation loop
     let dt = 1.0 / 60.0; // Time step for ~60 FPS
     for frame in 0..120 { // Simulate for 120 frames (2 seconds)
         world.step(dt);
         // In a real application, you would get body positions and render them here
         let ball_pos = world.bodies[ball_body_idx].position;
         println!("Frame {}: Ball position: ({:.2}, {:.2})", frame, ball_pos.x, ball_pos.y);
     }
     println!("Simulation finished.");
 }
 ```

 ## Performance
 
 Performance is a key goal. The engine uses a Spatial Grid for broadphase collision detection to efficiently handle scenes with many objects.
 
 _(Benchmarks run with `cargo bench --release`)_
 
 | Benchmark           | Bodies | Time (approx) |
 |---------------------|--------|---------------|
 | `circle_stack`      | 10     | ~12.4 µs      |
 | `circle_stack`      | 100    | ~247 µs       |
 | `circle_stack`      | 500    | ~5.3 ms       |
 | `constraint_chain`  | 10     | ~19.9 µs      |
 | `constraint_chain`  | 100    | ~401 µs       |
 | `constraint_chain`  | 500    | ~6.0 ms       |
 
 *   **`circle_stack`**: Simulates N dynamic circles stacked vertically, falling onto a static ground (30 steps, 4 solver iterations).
 *   **`constraint_chain`**: Simulates N dynamic circles linked by `PinJoint`s hanging from a static anchor (30 steps, 8 solver iterations).
 
 **Planned Benchmark Scenarios (Post-Optimization):**
 *   `polygon_stack_N`: Measuring performance with stacked polygons.
 *   `large_mixed_scene_N`: Assessing scalability with a large number of bodies with mixed shapes.
 *   `high_constraint_scene_N`: Evaluating performance under heavy constraint loads.
 
 _(Note: Current benchmarks use few bodies and may not fully reflect performance gains from broadphase in larger scenes. Regressions compared to earlier versions are expected due to added features like polygon collisions, friction, and grid management overhead.)_
 
 ## Roadmap / Future Enhancements
 
 Here's what's planned next to make the engine more capable for complex games or simulations:
 
 1.  **More Shapes & Geometry:**
     *   (Completed) Convex Polygon support with automated mass property calculation.
     *   Concave polygon support (likely via decomposition).
 2.  **Advanced Collision Detection:**
     *   (Completed) Circle-Polygon collision detection (SAT).
     *   (Completed) Polygon-Polygon collision detection (SAT, basic contact points).
 3.  **Material System & Friction:**
     *   (Completed) `Material` struct added (restitution, friction).
     *   (Completed) Collision resolution uses restitution & friction (Coulomb model, iterative solver).
 4.  **(Next)** **Broadphase Collision Detection:**
     *   (Completed) Uniform Spatial Grid implemented.
 5.  **Further Physics Enhancements:**
     *   Air resistance / Drag models.
     *   More constraint types (e.g., motors, angular limits).
     *   More accurate contact point generation (e.g., clipping).
 6.  **API & Usability:**
     *   Refinement of the public API for improved ergonomics.
     *   Comprehensive documentation generation.
