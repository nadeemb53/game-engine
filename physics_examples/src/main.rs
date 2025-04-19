use physics_engine::*;
use std::env;

mod visual;

fn main() {
    let args: Vec<String> = env::args().collect();
    
    // Check if first arg is "visual"
    if args.len() > 1 && args[1] == "visual" {
        let example = if args.len() > 2 { 
            &args[2] 
        } else { 
            "ball_pit" 
        };
        
        println!("Running example: {}", example);
        
        if let Err(e) = visual::run_visual(example) {
            eprintln!("Error running visual example: {}", e);
        }
        return;
    }
    
    // Original code for non-visual examples
    let example = if args.len() > 1 { &args[1] } else { "basic" };
    
    println!("Running example: {}", example);
    
    match example {
        "basic" => run_basic_example(),
        "ball_pit" => run_ball_pit_example(),
        "newtons_cradle" => run_newtons_cradle_example(),
        "ragdoll" => run_ragdoll_example(),
        _ => println!("Unknown example: {}. Available examples: basic, ball_pit, newtons_cradle, ragdoll", example),
    }
}

fn run_basic_example() {
    // 1. Create a world
    let mut world = PhysicsWorld::new();
    world.gravity = Vec2::new(0.0, -10.0);

    // 2. Add bodies
    // A static body (e.g., the ground)
    let ground_shape = Shape::Circle(Circle::new(0.1)); // Small circle as a fixed point
    let static_body_idx = world.add_static_body(ground_shape, Vec2::new(5.0, 0.0), 0.0);
    println!("Added static body at index: {}", static_body_idx);

    // A dynamic body (e.g., a falling ball)
    let ball_shape = Shape::Circle(Circle::new(0.5));
    let mut ball_body = RigidBody::new(1.0, ball_shape); // mass = 1.0
    ball_body.position = Vec2::new(5.0, 5.0); // Start above the static body
    // Optionally set a custom material
    ball_body.material = Material::new(0.7, 0.4); // Bouncy, moderate friction
    let ball_body_idx = world.add_body(ball_body);
    println!("Added ball body at index: {}", ball_body_idx);

    // 3. Simulation loop
    let dt = 1.0 / 60.0; // Time step for ~60 FPS
    for frame in 0..120 { // Simulate for 120 frames (2 seconds)
        world.step(dt, frame % 30 == 0); // Log every 30 frames
        // Get the ball position and print it
        let ball_pos = world.bodies[ball_body_idx].position;
        if frame % 10 == 0 {
            println!("Frame {}: Ball position: ({:.2}, {:.2})", frame, ball_pos.x, ball_pos.y);
        }
    }
    println!("Simulation finished.");
}

fn run_ball_pit_example() {
    let width = 10.0;
    let height = 8.0;
    let mut world = create_ball_pit(width, height);
    
    // Run simulation for a while
    println!("Simulating ball pit with {} bodies", world.bodies.len());
    let dt = 1.0 / 60.0;
    for frame in 0..180 {
        world.step(dt, frame == 0);
        if frame % 30 == 0 {
            println!("Frame {}: {} bodies simulated", frame, world.bodies.len());
        }
    }
    println!("Ball pit simulation finished.");
}

fn run_newtons_cradle_example() {
    let mut world = create_newtons_cradle(5, 0.5);
    
    // Run simulation for a while
    println!("Simulating Newton's cradle with {} bodies", world.bodies.len());
    let dt = 1.0 / 60.0;
    for frame in 0..300 {
        world.step(dt, frame == 0);
        if frame % 30 == 0 {
            println!("Frame {}: {} bodies simulated", frame, world.bodies.len());
        }
    }
    println!("Newton's cradle simulation finished.");
}

fn run_ragdoll_example() {
    let mut world = create_simple_ragdoll();
    
    // Run simulation for a while
    println!("Simulating ragdoll with {} bodies", world.bodies.len());
    let dt = 1.0 / 60.0;
    for frame in 0..300 {
        world.step(dt, frame == 0);
        if frame % 30 == 0 {
            println!("Frame {}: {} bodies simulated", frame, world.bodies.len());
        }
    }
    println!("Ragdoll simulation finished.");
}

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

fn create_simple_ragdoll() -> PhysicsWorld {
    let mut world = PhysicsWorld::new();
    world.gravity = Vec2::new(0.0, -9.81);
    
    // Create body parts (head, torso, arms, legs)
    let head_shape = Shape::Circle(Circle::new(0.25));
    let torso_shape = Shape::Circle(Circle::new(0.4));
    
    // Add body parts
    let mut head = RigidBody::new(1.0, head_shape);
    head.position = Vec2::new(0.0, 3.0);
    let head_idx = world.add_body(head);
    
    let mut torso = RigidBody::new(5.0, torso_shape);
    torso.position = Vec2::new(0.0, 2.0);
    let torso_idx = world.add_body(torso);
    
    let mut left_arm = RigidBody::new(1.0, Shape::Circle(Circle::new(0.2)));
    left_arm.position = Vec2::new(-0.7, 2.0);
    let left_arm_idx = world.add_body(left_arm);
    
    let mut right_arm = RigidBody::new(1.0, Shape::Circle(Circle::new(0.2)));
    right_arm.position = Vec2::new(0.7, 2.0);
    let right_arm_idx = world.add_body(right_arm);
    
    let mut left_leg = RigidBody::new(1.0, Shape::Circle(Circle::new(0.2)));
    left_leg.position = Vec2::new(-0.3, 1.0);
    let left_leg_idx = world.add_body(left_leg);
    
    let mut right_leg = RigidBody::new(1.0, Shape::Circle(Circle::new(0.2)));
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
