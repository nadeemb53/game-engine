use criterion::{black_box, criterion_group, criterion_main, Criterion};
use physics_engine::{
    math::vec2::Vec2,
    objects::RigidBody,
    shapes::{Circle, Shape},
    world::PhysicsWorld,
};

// --- Helper for creating stack benchmarks ---
fn run_circle_stack_bench(world: &mut PhysicsWorld, num_circles: usize) {
    let radius = 0.5;
    let shape = Shape::Circle(Circle::new(radius));

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
}

// --- Helper for creating chain benchmarks ---
fn run_constraint_chain_bench(world: &mut PhysicsWorld, num_links: usize) {
    let shape = Shape::Circle(Circle::new(0.2)); // Small circles for links
    let link_length = 0.5;

    // Create the anchor point (static)
    let anchor_pos = Vec2::new(0.0, 5.0);
    // Use a distinct shape instance for the static anchor if needed, or clone
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
            if i == 0 { Vec2::default() } else { Vec2::new(link_length / 2.0, 0.0) },
            Vec2::new(-link_length / 2.0, 0.0)
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
}

// Benchmark for a stack of circles falling under gravity
fn bench_circle_stack(c: &mut Criterion) {
    let mut group = c.benchmark_group("circle_stack");

    for num_circles in [10, 100, 500].iter() {
        group.bench_with_input(criterion::BenchmarkId::from_parameter(num_circles), num_circles, |b, &n| {
            b.iter(|| {
                let mut world = PhysicsWorld::new();
                world.gravity = Vec2::new(0.0, -10.0);
                world.solver_iterations = 4; // Fewer iterations for benchmark speed
                let ground_shape = Shape::Circle(Circle::new(1.0));
                world.add_static_body(ground_shape, Vec2::new(0.0, -1.0), 0.0);
                run_circle_stack_bench(&mut world, black_box(n));
            });
        });
    }
    group.finish();
}

// Benchmark for a chain of bodies linked by constraints
fn bench_constraint_chain(c: &mut Criterion) {
     let mut group = c.benchmark_group("constraint_chain");

     for num_links in [10, 100, 500].iter() {
         group.bench_with_input(criterion::BenchmarkId::from_parameter(num_links), num_links, |b, &n| {
             b.iter(|| {
                 let mut world = PhysicsWorld::new();
                 world.gravity = Vec2::new(0.0, -10.0);
                 world.solver_iterations = 8;
                 run_constraint_chain_bench(&mut world, black_box(n));
             });
         });
     }
     group.finish();
}

criterion_group!(benches, bench_circle_stack, bench_constraint_chain);
criterion_main!(benches); 