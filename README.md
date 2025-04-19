# Physics Engine

**A 2D rigid-body physics engine written in Rust.**

[![Rust CI](https://github.com/nadeemb53/game-engine/actions/workflows/rust.yml/badge.svg)](https://github.com/nadeemb53/game-engine/actions/workflows/rust.yml)

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
 *   **Thoroughly Tested:** Comprehensive test suite covering 100+ test cases across all components, ensuring reliability and correctness of the physics simulation.
 
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

 ## Fun Examples

 ### Bouncy Ball Pit
 ```rust
 use physics_engine::*;
 
 fn create_ball_pit(width: f64, height: f64) -> PhysicsWorld {
     let mut world = PhysicsWorld::new();
     world.gravity = Vec2::new(0.0, -9.81);
     
     // Create walls (static bodies)
     let floor_shape = Shape::Line(LineSegment::new(Vec2::new(0.0, 0.0), Vec2::new(width, 0.0)));
     let left_wall_shape = Shape::Line(LineSegment::new(Vec2::new(0.0, 0.0), Vec2::new(0.0, height)));
     let right_wall_shape = Shape::Line(LineSegment::new(Vec2::new(width, 0.0), Vec2::new(width, height)));
     
     // Add static bodies for the walls
     let floor_material = Material::new(0.2, 0.1); // Low bounce, low friction
     let mut floor = RigidBody::new_static(floor_shape, Vec2::new(0.0, 0.0), 0.0);
     floor.material = floor_material;
     world.add_body(floor);
     
     let mut left_wall = RigidBody::new_static(left_wall_shape, Vec2::new(0.0, 0.0), 0.0);
     left_wall.material = floor_material;
     world.add_body(left_wall);
     
     let mut right_wall = RigidBody::new_static(right_wall_shape, Vec2::new(0.0, 0.0), 0.0);
     right_wall.material = floor_material;
     world.add_body(right_wall);
     
     // Add 20 bouncy balls with different sizes
     for i in 0..20 {
         let radius = 0.2 + (i % 4) as f64 * 0.1; // Varying sizes from 0.2 to 0.5
         let x = 1.0 + i as f64 * 0.5; // Spread them out horizontally
         let y = height - i as f64 * 0.2; // Start from different heights
         
         let circle_shape = Shape::Circle(Circle::new(radius));
         let mut ball = RigidBody::new(1.0, circle_shape);
         ball.position = Vec2::new(x, y);
         ball.material = Material::new(0.8, 0.1); // Very bouncy!
         
         world.add_body(ball);
     }
     
     world
 }
 
 // In your game loop:
 // world.step(1.0/60.0, false);
 // Render all the balls at their updated positions!
 ```

 ### Newton's Cradle
 ```rust
 use physics_engine::*;
 
 fn create_newtons_cradle(num_balls: usize, ball_radius: f64) -> PhysicsWorld {
     let mut world = PhysicsWorld::new();
     world.gravity = Vec2::new(0.0, -9.81);
     
     // Create anchor point (static body)
     let anchor_shape = Shape::Circle(Circle::new(0.1));
     let anchor_pos = Vec2::new(num_balls as f64 * ball_radius * 2.0, 5.0);
     let anchor_idx = world.add_body(RigidBody::new_static(anchor_shape, anchor_pos, 0.0));
     
     // Create the pendulum balls
     let mut ball_indices = Vec::new();
     for i in 0..num_balls {
         let x_pos = i as f64 * ball_radius * 2.0 + ball_radius;
         let ball_shape = Shape::Circle(Circle::new(ball_radius));
         
         let mut ball = RigidBody::new(1.0, ball_shape);
         ball.position = Vec2::new(x_pos, 2.0);
         ball.material = Material::new(1.0, 0.0); // Perfect elasticity, no friction
         
         ball_indices.push(world.add_body(ball));
     }
     
     // Connect balls to anchor with distance constraints
     for &ball_idx in &ball_indices {
         let constraint = DistanceConstraint::new(
             anchor_idx,
             ball_idx,
             Vec2::new(0.0, 0.0), // Anchor point on static body
             Vec2::new(0.0, 0.0), // Anchor at center of ball
             3.0 // Length of pendulum
         );
         
         world.add_constraint(Box::new(constraint));
     }
     
     // Pull the first ball to the side to start the motion
     if let Some(first_ball) = world.bodies.get_mut(ball_indices[0]) {
         first_ball.position.x -= 1.5;
     }
     
     world
 }

 ### Simple Ragdoll
 ```rust
 use physics_engine::*;
 
 fn create_simple_ragdoll() -> PhysicsWorld {
     let mut world = PhysicsWorld::new();
     world.gravity = Vec2::new(0.0, -9.81);
     
     // Create body parts (head, torso, arms, legs)
     let head_shape = Shape::Circle(Circle::new(0.25));
     let torso_shape = Shape::Circle(Circle::new(0.4));
     let limb_shape = Shape::Circle(Circle::new(0.2));
     
     // Add body parts
     let mut head = RigidBody::new(1.0, head_shape);
     head.position = Vec2::new(0.0, 3.0);
     let head_idx = world.add_body(head);
     
     let mut torso = RigidBody::new(5.0, torso_shape);
     torso.position = Vec2::new(0.0, 2.0);
     let torso_idx = world.add_body(torso);
     
     let mut left_arm = RigidBody::new(1.0, limb_shape);
     left_arm.position = Vec2::new(-0.7, 2.0);
     let left_arm_idx = world.add_body(left_arm);
     
     let mut right_arm = RigidBody::new(1.0, limb_shape);
     right_arm.position = Vec2::new(0.7, 2.0);
     let right_arm_idx = world.add_body(right_arm);
     
     let mut left_leg = RigidBody::new(1.0, limb_shape);
     left_leg.position = Vec2::new(-0.3, 1.0);
     let left_leg_idx = world.add_body(left_leg);
     
     let mut right_leg = RigidBody::new(1.0, limb_shape);
     right_leg.position = Vec2::new(0.3, 1.0);
     let right_leg_idx = world.add_body(right_leg);
     
     // Connect parts with pin joints
     // Head to torso
     let head_joint = PinJoint::new(
         head_idx, torso_idx,
         Vec2::new(0.0, -0.25), // Bottom of head
         Vec2::new(0.0, 0.4)    // Top of torso
     );
     world.add_constraint(Box::new(head_joint));
     
     // Arms to torso
     let left_arm_joint = PinJoint::new(
         left_arm_idx, torso_idx,
         Vec2::new(0.2, 0.0),  // Inner side of arm
         Vec2::new(-0.4, 0.1)  // Left side of torso
     );
     world.add_constraint(Box::new(left_arm_joint));
     
     let right_arm_joint = PinJoint::new(
         right_arm_idx, torso_idx,
         Vec2::new(-0.2, 0.0), // Inner side of arm
         Vec2::new(0.4, 0.1)   // Right side of torso
     );
     world.add_constraint(Box::new(right_arm_joint));
     
     // Legs to torso
     let left_leg_joint = PinJoint::new(
         left_leg_idx, torso_idx,
         Vec2::new(0.0, 0.2),   // Top of leg
         Vec2::new(-0.2, -0.4)  // Bottom left of torso
     );
     world.add_constraint(Box::new(left_leg_joint));
     
     let right_leg_joint = PinJoint::new(
         right_leg_idx, torso_idx,
         Vec2::new(0.0, 0.2),   // Top of leg
         Vec2::new(0.2, -0.4)   // Bottom right of torso
     );
     world.add_constraint(Box::new(right_leg_joint));
     
     // Create ground
     let ground_shape = Shape::Line(LineSegment::new(Vec2::new(-5.0, 0.0), Vec2::new(5.0, 0.0)));
     let ground = RigidBody::new_static(ground_shape, Vec2::new(0.0, 0.0), 0.0);
     world.add_body(ground);
     
     world
 }

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
 
 ## Testing
 
 The physics engine is backed by a comprehensive test suite with over 100 test cases.
 
 Run the test suite with:
 ```
 cd physics_engine
 cargo test
 ```
 
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
