use criterion::{black_box, criterion_group, criterion_main, Criterion};
use physics_engine::{
    math::vec2::Vec2,
    objects::RigidBody,
    shapes::{Circle, Shape},
    world::PhysicsWorld,
};

// Benchmark for a stack of circles falling under gravity
fn bench_circle_stack(c: &mut Criterion) {
    c.bench_function("circle_stack_10", |b| {
        b.iter(|| {
            let mut world = PhysicsWorld::new();
            world.gravity = Vec2::new(0.0, -10.0);
            world.solver_iterations = 4; // Fewer iterations for benchmark speed

            let ground_shape = Shape::Circle(Circle::new(1.0)); // Represent ground as large static circle (or line later)
            world.add_static_body(ground_shape, Vec2::new(0.0, -1.0), 0.0);

            let radius = 0.5;
            let shape = Shape::Circle(Circle::new(radius));
            let num_circles = 10;

            for i in 0..num_circles {
                let y_pos = radius + (i as f64 * (radius * 2.1)); // Stack with slight gap
                let mut body = RigidBody::new(1.0, shape.clone());
                body.position = Vec2::new(0.0, y_pos);
                world.add_body(body);
            }

            // Simulate for a fixed number of steps
            let dt = 1.0 / 60.0;
            let steps = 30;
            for _ in 0..steps {
                world.step(black_box(dt)); // Use black_box for inputs
            }
        })
    });
}

// Benchmark for a chain of bodies linked by constraints
fn bench_constraint_chain(c: &mut Criterion) {
    c.bench_function("constraint_chain_10", |b| {
        b.iter(|| {
            let mut world = PhysicsWorld::new();
            world.gravity = Vec2::new(0.0, -10.0);
            world.solver_iterations = 8; // Use a reasonable number of iterations

            let shape = Shape::Circle(Circle::new(0.2)); // Small circles for links
            let link_length = 0.5;
            let num_links = 10;

            // Create the anchor point (static)
            let anchor_pos = Vec2::new(0.0, 5.0);
            let anchor_idx = world.add_static_body(shape.clone(), anchor_pos, 0.0);

            let mut last_idx = anchor_idx;
            let mut current_pos = anchor_pos;

            for i in 0..num_links {
                current_pos.x += link_length;
                let mut body = RigidBody::new(1.0, shape.clone());
                body.position = current_pos;
                let current_idx = world.add_body(body);

                // Add a pin joint connecting the new body to the previous one
                let joint = physics_engine::constraints::PinJoint::new(
                    last_idx,
                    current_idx,
                    if i == 0 { Vec2::default() } else { Vec2::new(link_length / 2.0, 0.0) }, // Anchor on previous
                    Vec2::new(-link_length / 2.0, 0.0) // Anchor on current
                    // For simplicity, using PinJoint. A DistanceConstraint might be more stable here.
                );
                world.add_pin_joint_specific(joint);

                last_idx = current_idx;
            }

            // Simulate
            let dt = 1.0 / 60.0;
            let steps = 30;
            for _ in 0..steps {
                world.step(black_box(dt));
            }
        })
    });
}

criterion_group!(benches, bench_circle_stack, bench_constraint_chain);
criterion_main!(benches); 