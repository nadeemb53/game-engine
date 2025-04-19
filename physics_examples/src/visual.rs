use ggez::{Context, GameResult};
use ggez::graphics::{self, Color, DrawMode, DrawParam, Mesh};
use ggez::event::{self, EventHandler};
use ggez::input::keyboard::{KeyCode, KeyInput};
use ggez::glam::Vec2 as GVec2;
use physics_engine::*;

struct MainState {
    world: PhysicsWorld,
    example_type: ExampleType,
}

enum ExampleType {
    Basic,
    BallPit,
    NewtonsCradle,
    Ragdoll,
}

impl MainState {
    fn new(_ctx: &mut Context, example_type: ExampleType) -> GameResult<MainState> {
        let world = match example_type {
            ExampleType::Basic => {
                let mut world = PhysicsWorld::new();
                world.gravity = Vec2::new(0.0, 9.81); // Note: ggez y is down
                
                // Ground
                let ground_shape = Shape::Circle(Circle::new(0.1));
                world.add_static_body(ground_shape, Vec2::new(400.0, 500.0), 0.0);
                
                // Ball
                let ball_shape = Shape::Circle(Circle::new(25.0));
                let mut ball = RigidBody::new(1.0, ball_shape);
                ball.position = Vec2::new(400.0, 100.0);
                ball.material = Material::new(0.7, 0.4);
                world.add_body(ball);
                
                world
            },
            ExampleType::BallPit => create_visual_ball_pit(800.0, 600.0),
            ExampleType::NewtonsCradle => create_visual_newtons_cradle(5, 25.0),
            ExampleType::Ragdoll => create_visual_ragdoll(),
        };
        
        Ok(MainState { world, example_type })
    }
}

impl EventHandler for MainState {
    fn update(&mut self, _ctx: &mut Context) -> GameResult {
        // Step physics at 60 FPS
        self.world.step(1.0/60.0, false);
        Ok(())
    }

    fn draw(&mut self, ctx: &mut Context) -> GameResult {
        let mut canvas = graphics::Canvas::from_frame(ctx, Color::BLACK);
        
        // Draw each body
        for body in &self.world.bodies {
            match &body.shape {
                Shape::Circle(circle) => {
                    let circle_mesh = Mesh::new_circle(
                        ctx,
                        DrawMode::fill(),
                        GVec2::new(0.0, 0.0),
                        circle.radius as f32,
                        0.1,
                        Color::WHITE,
                    )?;
                    
                    canvas.draw(
                        &circle_mesh,
                        DrawParam::new()
                            .dest(GVec2::new(body.position.x as f32, body.position.y as f32))
                            .rotation(body.rotation as f32)
                    );
                },
                Shape::Line(line) => {
                    let start = body.position + (line.a - body.local_center_of_mass).rotate(body.rotation);
                    let end = body.position + (line.b - body.local_center_of_mass).rotate(body.rotation);
                    
                    let line_mesh = Mesh::new_line(
                        ctx,
                        &[
                            GVec2::new(start.x as f32, start.y as f32),
                            GVec2::new(end.x as f32, end.y as f32),
                        ],
                        2.0,
                        Color::WHITE,
                    )?;
                    
                    canvas.draw(&line_mesh, DrawParam::new());
                },
                Shape::Polygon(poly) => {
                    let world_verts = poly.vertices.iter().map(|v| {
                        let world_v = body.position + (*v - body.local_center_of_mass).rotate(body.rotation);
                        GVec2::new(world_v.x as f32, world_v.y as f32)
                    }).collect::<Vec<_>>();
                    
                    let poly_mesh = Mesh::new_polygon(
                        ctx,
                        DrawMode::fill(),
                        &world_verts,
                        Color::WHITE,
                    )?;
                    
                    canvas.draw(&poly_mesh, DrawParam::new());
                },
            }
        }
        
        canvas.finish(ctx)?;
        Ok(())
    }
    
    fn key_down_event(&mut self, ctx: &mut Context, input: KeyInput, _repeated: bool) -> GameResult {
        if input.keycode == Some(KeyCode::Escape) {
            ctx.request_quit();
        }
        Ok(())
    }
}

fn create_visual_ball_pit(width: f64, height: f64) -> PhysicsWorld {
    let mut world = PhysicsWorld::new();
    world.gravity = Vec2::new(0.0, 9.81); // ggez y-down
    
    // Create walls
    let floor_shape = Shape::Line(LineSegment::new(Vec2::new(0.0, height), Vec2::new(width, height)));
    let left_wall_shape = Shape::Line(LineSegment::new(Vec2::new(0.0, 0.0), Vec2::new(0.0, height)));
    let right_wall_shape = Shape::Line(LineSegment::new(Vec2::new(width, 0.0), Vec2::new(width, height)));
    
    let wall_material = Material::new(0.2, 0.1);
    
    let mut floor = RigidBody::new_static(floor_shape, Vec2::new(0.0, 0.0), 0.0);
    floor.material = wall_material;
    world.add_body(floor);
    
    let mut left_wall = RigidBody::new_static(left_wall_shape, Vec2::new(0.0, 0.0), 0.0);
    left_wall.material = wall_material;
    world.add_body(left_wall);
    
    let mut right_wall = RigidBody::new_static(right_wall_shape, Vec2::new(0.0, 0.0), 0.0);
    right_wall.material = wall_material;
    world.add_body(right_wall);
    
    // Add 20 bouncy balls
    for i in 0..20 {
        let radius = 10.0 + (i % 4) as f64 * 5.0;
        let x = 50.0 + i as f64 * 30.0;
        let y = 50.0 + (i % 5) as f64 * 40.0;
        
        let ball_shape = Shape::Circle(Circle::new(radius));
        let mut ball = RigidBody::new(1.0, ball_shape);
        ball.position = Vec2::new(x, y);
        ball.material = Material::new(0.8, 0.1);
        
        world.add_body(ball);
    }
    
    world
}

fn create_visual_newtons_cradle(num_balls: usize, ball_radius: f64) -> PhysicsWorld {
    let mut world = PhysicsWorld::new();
    world.gravity = Vec2::new(0.0, 9.81);
    
    // Create anchor point
    let anchor_shape = Shape::Circle(Circle::new(5.0));
    let anchor_pos = Vec2::new(400.0, 100.0);
    let anchor_idx = world.add_body(RigidBody::new_static(anchor_shape, anchor_pos, 0.0));
    
    // Create balls
    let mut ball_indices = Vec::new();
    for i in 0..num_balls {
        let x_pos = 350.0 + i as f64 * ball_radius * 2.0;
        let ball_shape = Shape::Circle(Circle::new(ball_radius));
        
        let mut ball = RigidBody::new(1.0, ball_shape);
        ball.position = Vec2::new(x_pos, 300.0);
        ball.material = Material::new(1.0, 0.0);
        
        ball_indices.push(world.add_body(ball));
    }
    
    // Connect balls to anchor
    for &ball_idx in &ball_indices {
        let constraint = DistanceConstraint::new(
            anchor_idx,
            ball_idx,
            Vec2::new(0.0, 0.0),
            Vec2::new(0.0, 0.0),
            200.0
        );
        
        world.add_constraint(Box::new(constraint));
    }
    
    // Pull first ball back
    if let Some(first_ball) = world.bodies.get_mut(ball_indices[0]) {
        first_ball.position.x -= 100.0;
    }
    
    world
}

fn create_visual_ragdoll() -> PhysicsWorld {
    let mut world = PhysicsWorld::new();
    world.gravity = Vec2::new(0.0, 9.81);
    
    // Create body parts (head, torso, arms, legs)
    let head_shape = Shape::Circle(Circle::new(25.0));
    let torso_shape = Shape::Circle(Circle::new(40.0));
    
    // Add body parts
    let mut head = RigidBody::new(1.0, head_shape);
    head.position = Vec2::new(400.0, 150.0);
    let head_idx = world.add_body(head);
    
    let mut torso = RigidBody::new(5.0, torso_shape);
    torso.position = Vec2::new(400.0, 250.0);
    let torso_idx = world.add_body(torso);
    
    let mut left_arm = RigidBody::new(1.0, Shape::Circle(Circle::new(20.0)));
    left_arm.position = Vec2::new(330.0, 250.0);
    let left_arm_idx = world.add_body(left_arm);
    
    let mut right_arm = RigidBody::new(1.0, Shape::Circle(Circle::new(20.0)));
    right_arm.position = Vec2::new(470.0, 250.0);
    let right_arm_idx = world.add_body(right_arm);
    
    let mut left_leg = RigidBody::new(1.0, Shape::Circle(Circle::new(20.0)));
    left_leg.position = Vec2::new(370.0, 350.0);
    let left_leg_idx = world.add_body(left_leg);
    
    let mut right_leg = RigidBody::new(1.0, Shape::Circle(Circle::new(20.0)));
    right_leg.position = Vec2::new(430.0, 350.0);
    let right_leg_idx = world.add_body(right_leg);
    
    // Connect parts with pin joints
    // Head to torso
    let head_joint = PinJoint::new(
        head_idx, torso_idx,
        Vec2::new(0.0, 25.0), // Bottom of head
        Vec2::new(0.0, -40.0)  // Top of torso
    );
    world.add_constraint(Box::new(head_joint));
    
    // Arms to torso
    let left_arm_joint = PinJoint::new(
        left_arm_idx, torso_idx,
        Vec2::new(20.0, 0.0),  // Inner side of arm
        Vec2::new(-40.0, 0.0)  // Left side of torso
    );
    world.add_constraint(Box::new(left_arm_joint));
    
    let right_arm_joint = PinJoint::new(
        right_arm_idx, torso_idx,
        Vec2::new(-20.0, 0.0), // Inner side of arm
        Vec2::new(40.0, 0.0)   // Right side of torso
    );
    world.add_constraint(Box::new(right_arm_joint));
    
    // Legs to torso
    let left_leg_joint = PinJoint::new(
        left_leg_idx, torso_idx,
        Vec2::new(0.0, -20.0),   // Top of leg
        Vec2::new(-20.0, 40.0)  // Bottom left of torso
    );
    world.add_constraint(Box::new(left_leg_joint));
    
    let right_leg_joint = PinJoint::new(
        right_leg_idx, torso_idx,
        Vec2::new(0.0, -20.0),   // Top of leg
        Vec2::new(20.0, 40.0)   // Bottom right of torso
    );
    world.add_constraint(Box::new(right_leg_joint));
    
    // Create ground
    let ground_shape = Shape::Line(LineSegment::new(Vec2::new(0.0, 500.0), Vec2::new(800.0, 500.0)));
    let ground = RigidBody::new_static(ground_shape, Vec2::new(0.0, 0.0), 0.0);
    world.add_body(ground);
    
    world
}

pub fn run_visual(example: &str) -> GameResult {
    let example_type = match example {
        "basic" => ExampleType::Basic,
        "ball_pit" => ExampleType::BallPit,
        "newtons_cradle" => ExampleType::NewtonsCradle,
        "ragdoll" => ExampleType::Ragdoll,
        _ => ExampleType::BallPit,
    };
    
    let cb = ggez::ContextBuilder::new("physics_examples", "author")
        .window_setup(ggez::conf::WindowSetup::default().title("Physics Examples"))
        .window_mode(ggez::conf::WindowMode::default().dimensions(800.0, 600.0));
    
    let (mut ctx, event_loop) = cb.build()?;
    let state = MainState::new(&mut ctx, example_type)?;
    event::run(ctx, event_loop, state)
} 