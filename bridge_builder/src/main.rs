use ggez::{
    conf, event, graphics, mint,
    input::keyboard::{KeyCode, KeyInput},
    input::mouse::MouseButton,
    Context, ContextBuilder, GameResult,
};
use physics_engine::{PhysicsWorld, RigidBody, Shape, Vec2, Circle, Polygon};
use physics_engine::constraints::distance_constraint::DistanceConstraint;

const SCREEN_WIDTH: f32 = 1280.0;
const SCREEN_HEIGHT: f32 = 720.0;

// --- Game Object Structs ---
#[derive(Debug, Clone, Copy, PartialEq)]
enum NodeType {
    Fixed,  // Anchored to the background
    Dynamic, // Part of the physics simulation
}

// Add GameMode Enum
#[derive(Debug, Clone, Copy, PartialEq)]
enum GameMode {
    Build,
    Test,
}

#[derive(Debug, Clone)]
struct Node {
    position: Vec2, // World position
    node_type: NodeType,
    // Index of the corresponding RigidBody in world.bodies, if dynamic or fixed anchor
    body_index: Option<usize>,
    is_active: bool, // Added flag
}

#[derive(Debug, Clone)]
struct Beam {
    node_a_idx: usize, // Index in GameState.nodes
    node_b_idx: usize, // Index in GameState.nodes
    is_active: bool, // Added flag
    // We might add physics body/constraint indices later
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum BuildTool {
    Select, // (Maybe later)
    PlaceNode(NodeType),
    PlaceBeam,
    Delete, // Added delete tool
}

// --- Game State ---
struct GameState {
    world: PhysicsWorld,
    nodes: Vec<Node>,
    beams: Vec<Beam>,
    mouse_pos: Vec2,
    current_tool: BuildTool,
    start_node_idx: Option<usize>,
    game_mode: GameMode,
    goal_ball_body_index: Option<usize>,
    ball_start_pos: Vec2, // Added to store initial ball position
    // UI Button State
    sim_button_rect: graphics::Rect,
    tool_buttons: Vec<(BuildTool, graphics::Rect)>, // Store tool and its Rect
}

impl GameState {
    fn new(_ctx: &mut Context) -> GameResult<GameState> {
        let mut world = PhysicsWorld::new();
        world.gravity = Vec2::new(0.0, 500.0);

        let mut nodes = Vec::new();
        let beams = Vec::new();
        let mut goal_ball_body_index = None;
        let mut ball_start_pos = Vec2::ZERO; // Initialize

        // --- Add Anchor Nodes ---
        let anchor_y = SCREEN_HEIGHT as f64 * 0.3;
        let anchor_positions = [
            Vec2::new(SCREEN_WIDTH as f64 * 0.1, anchor_y),
            Vec2::new(SCREEN_WIDTH as f64 * 0.9, anchor_y),
        ];
        for pos in anchor_positions {
            let node_radius = 5.0;
            let node_shape = Shape::Circle(Circle::new(node_radius));
            let anchor_body = RigidBody::new_static(node_shape, pos, 0.0);
            let body_idx = world.add_body(anchor_body);
            println!(" Added static anchor body with index: {}", body_idx);
            nodes.push(Node {
                position: pos,
                node_type: NodeType::Fixed,
                body_index: Some(body_idx),
                is_active: true,
            });
        }

        // --- Add Ground Platforms ---
        let ground_y = SCREEN_HEIGHT as f64 * 0.9;
        let ground_height = 20.0;
        let ground_width = SCREEN_WIDTH as f64 * 0.3; // Make platforms wider
        let gap_width = SCREEN_WIDTH as f64 * 0.1;
        let platform_1_x = SCREEN_WIDTH as f64 * 0.05;
        let platform_2_x = platform_1_x + ground_width + gap_width;

        let ground_vertices = vec![
             Vec2::new(0.0, 0.0), Vec2::new(ground_width, 0.0),
             Vec2::new(ground_width, -ground_height), Vec2::new(0.0, -ground_height),
        ];
        let ground_shape = Shape::Polygon(Polygon::new(ground_vertices));
        let ground_pos_1 = Vec2::new(platform_1_x, ground_y);
        let ground_body_1 = RigidBody::new_static(ground_shape.clone(), ground_pos_1, 0.0);
        world.add_body(ground_body_1);
        println!(" Added ground body 1");
        let ground_pos_2 = Vec2::new(platform_2_x, ground_y);
        let ground_body_2 = RigidBody::new_static(ground_shape, ground_pos_2, 0.0);
        world.add_body(ground_body_2);
        println!(" Added ground body 2");

        // --- Add Goal Ball ---
        let ball_radius = 15.0;
        let ball_mass = 1.0;
        let ball_shape = Shape::Circle(Circle::new(ball_radius));
        let mut ball_body = RigidBody::new(ball_mass, ball_shape);
        // Calculate and store start position
        ball_start_pos = Vec2::new(
            ground_pos_1.x + ground_width * 0.75, // Start towards the right edge of platform 1
            ground_pos_1.y - ground_height - ball_radius - 1.0, // 1 unit buffer
        );
        ball_body.position = ball_start_pos;
        let ball_idx = world.add_body(ball_body);
        println!(" Added goal ball body with index: {}", ball_idx);
        goal_ball_body_index = Some(ball_idx);

        // --- Define UI Buttons ---
        let sim_button_rect = graphics::Rect::new(SCREEN_WIDTH * 0.5 - 60.0, 10.0, 120.0, 30.0);

        let tool_button_width = 80.0;
        let tool_button_height = 30.0;
        let tool_button_padding = 10.0;
        let ui_margin = 10.0; // Define ui_margin locally for button positioning
        let mut tool_buttons = Vec::new();
        let mut current_button_x = SCREEN_WIDTH - ui_margin - tool_button_width; // Start from right

        // Add buttons in reverse order for right-alignment
        let tools_to_add = [
            (BuildTool::Delete, "Delete"),
            (BuildTool::PlaceBeam, "Beam"),
            (BuildTool::PlaceNode(NodeType::Dynamic), "Joint"),
            // (BuildTool::PlaceNode(NodeType::Fixed), "Anchor"), // Maybe disable placing fixed nodes
        ];

        for (tool, _label) in tools_to_add.iter() {
            let rect = graphics::Rect::new(current_button_x, ui_margin, tool_button_width, tool_button_height);
            tool_buttons.push((*tool, rect));
            current_button_x -= tool_button_width + tool_button_padding;
        }
        tool_buttons.reverse(); // Put back in logical order

        Ok(GameState {
            world,
            nodes,
            beams,
            mouse_pos: Vec2::ZERO,
            current_tool: BuildTool::PlaceNode(NodeType::Dynamic), // Default tool
            start_node_idx: None,
            game_mode: GameMode::Build,
            goal_ball_body_index,
            ball_start_pos,
            sim_button_rect,
            tool_buttons,
        })
    }

    // Helper function to find an *active* node near a point
    fn find_node_near_point(&self, point: Vec2, tolerance: f64) -> Option<usize> {
        let tolerance_sq = tolerance * tolerance;
        for (index, node) in self.nodes.iter().enumerate() {
            if node.is_active {
                let distance_sq = (node.position - point).magnitude_squared();
                if distance_sq < tolerance_sq {
                    return Some(index);
                }
            }
        }
        None
    }
}

// --- Event Handler ---
impl event::EventHandler<ggez::GameError> for GameState {
    fn update(&mut self, ctx: &mut Context) -> GameResult {
        let dt = ctx.time.delta().as_secs_f64();
        if self.game_mode == GameMode::Test {
            for node in self.nodes.iter_mut() {
                if node.is_active && node.node_type == NodeType::Dynamic {
                    if let Some(body_idx) = node.body_index {
                         if body_idx < self.world.bodies.len() {
                             if self.world.bodies[body_idx].inv_mass > 0.0 {
                                node.position = self.world.bodies[body_idx].position;
                             }
                         }
                    }
                }
            }
            self.world.step(dt);
        }
        Ok(())
    }

    fn draw(&mut self, ctx: &mut Context) -> GameResult {
        let mut canvas = graphics::Canvas::from_frame(ctx, graphics::Color::from([0.1, 0.1, 0.15, 1.0]));

        // --- Draw Physics Bodies (Ground, Goal Ball) ---
        let ground_color = graphics::Color::from([0.3, 0.5, 0.3, 1.0]);
        let goal_ball_color = graphics::Color::from([0.9, 0.9, 0.2, 1.0]); // Yellow

        for (index, body) in self.world.bodies.iter().enumerate() {
            // Check if this body is the goal ball
            let is_goal_ball = self.goal_ball_body_index.map_or(false, |idx| idx == index);

            match &body.shape {
                Shape::Polygon(polygon) => {
                    // Calculate world vertices based on body's position/rotation
                    // The body.position is the CoM, so we need the offset from CoM to the shape's origin
                    let world_shape_origin = body.position - body.local_center_of_mass.rotate(body.rotation);

                    let world_vertices: Vec<mint::Point2<f32>> = polygon.vertices.iter()
                        .map(|&v_local| {
                            let v_world = world_shape_origin + v_local.rotate(body.rotation);
                            mint::Point2 { x: v_world.x as f32, y: v_world.y as f32 }
                        })
                        .collect();

                    if world_vertices.len() >= 3 {
                         let polygon_mesh = graphics::Mesh::new_polygon(
                             ctx,
                             graphics::DrawMode::fill(),
                             &world_vertices,
                             ground_color,
                         )?;
                         canvas.draw(&polygon_mesh, graphics::DrawParam::default());
                    }
                }
                Shape::Circle(circle) => {
                    // If it's the goal ball, draw it with its specific color
                    if is_goal_ball {
                         let ball_mesh = graphics::Mesh::new_circle(
                             ctx,
                             graphics::DrawMode::fill(),
                             mint::Point2{ x: body.position.x as f32, y: body.position.y as f32 },
                             circle.radius as f32,
                             0.5, // tolerance
                             goal_ball_color,
                         )?;
                         canvas.draw(&ball_mesh, graphics::DrawParam::default());
                    }
                    // Note: Regular nodes (which are also Circles) are drawn later
                }
                // Lines are not currently used for ground or goal objects
                Shape::Line(_) => {}
            }
        }

        // --- Draw Active Beams ---
        let beam_color = graphics::Color::from([0.5, 0.5, 0.9, 1.0]);
        let line_width = 2.0;
        for beam in &self.beams {
            // Only draw active beams with valid, active nodes
            if beam.is_active
               && beam.node_a_idx < self.nodes.len() && self.nodes[beam.node_a_idx].is_active
               && beam.node_b_idx < self.nodes.len() && self.nodes[beam.node_b_idx].is_active
            {
                let pos_a = self.nodes[beam.node_a_idx].position;
                let pos_b = self.nodes[beam.node_b_idx].position;
                let points = [
                    // Correct mint::Point2 syntax (single braces)
                    mint::Point2{ x: pos_a.x as f32, y: pos_a.y as f32 },
                    mint::Point2{ x: pos_b.x as f32, y: pos_b.y as f32 },
                ];
                let beam_mesh = graphics::Mesh::new_line(ctx, &points, line_width, beam_color)?;
                canvas.draw(&beam_mesh, graphics::DrawParam::default());
            }
        }

        // --- Draw In-Progress Beam (only if start node is active) ---
        if let Some(start_idx) = self.start_node_idx {
             if start_idx < self.nodes.len() && self.nodes[start_idx].is_active {
                let start_pos = self.nodes[start_idx].position;
                let end_pos = self.mouse_pos;
                let points = [
                    // Correct mint::Point2 syntax
                    mint::Point2{ x: start_pos.x as f32, y: start_pos.y as f32 },
                    mint::Point2{ x: end_pos.x as f32, y: end_pos.y as f32 },
                ];
                let temp_beam_color = graphics::Color::from([0.7, 0.7, 1.0, 0.7]);
                let temp_beam_mesh = graphics::Mesh::new_line(ctx, &points, line_width, temp_beam_color)?;
                canvas.draw(&temp_beam_mesh, graphics::DrawParam::default());
             } else {
                 // If start node somehow became inactive, cancel beam placement
                 self.start_node_idx = None;
             }
        }

        // --- Draw Active Nodes ---
        for node in &self.nodes {
            // Only draw active nodes
            if node.is_active {
                let node_radius = 5.0;
                let color = match node.node_type {
                    NodeType::Fixed => graphics::Color::from([0.8, 0.2, 0.2, 1.0]),
                    NodeType::Dynamic => graphics::Color::from([0.8, 0.8, 0.8, 1.0]),
                };
                let node_mesh = graphics::Mesh::new_circle(
                    ctx,
                    graphics::DrawMode::fill(),
                    // Correct mint::Point2 syntax
                    mint::Point2{ x: node.position.x as f32, y: node.position.y as f32 },
                    node_radius,
                    0.5,
                    color,
                )?;
                canvas.draw(&node_mesh, graphics::DrawParam::default());
            }
        }

        // --- Draw UI / Tool Indicator ---
        let ui_margin = 10.0;
        let line_height = 20.0;
        let mut current_y = ui_margin;

        // Instructions & Goal
        let goal_text_str = "Goal: Build a bridge to get the Yellow Ball across the gap!";
        let mut goal_text = graphics::Text::new(goal_text_str);
        goal_text.set_scale(16.0);
        canvas.draw(
            &goal_text,
            graphics::DrawParam::new().dest(mint::Point2{ x: ui_margin, y: current_y }).color(graphics::Color::WHITE),
        );
        current_y += line_height;

        let node_instr_str = "Nodes: Red=Fixed Anchor, Gray=Dynamic Bridge Part";
        let mut node_instr = graphics::Text::new(node_instr_str);
        node_instr.set_scale(16.0);
        canvas.draw(
            &node_instr,
            graphics::DrawParam::new().dest(mint::Point2{ x: ui_margin, y: current_y }).color(graphics::Color::WHITE),
        );
        current_y += line_height;

        // Tool & Mode Info (moved down)
        let mode_text_str = format!("Mode: {:?}", self.game_mode);
        let mut mode_text = graphics::Text::new(mode_text_str);
        mode_text.set_scale(16.0);
        canvas.draw(
            &mode_text,
            graphics::DrawParam::new().dest(mint::Point2{ x: ui_margin, y: current_y }).color(graphics::Color::WHITE),
        );
        current_y += line_height;

        let tool_instr_str = "Tools: Click buttons (top right) or use Keys (1-4/D)"; // Updated instructions
        let mut tool_instr = graphics::Text::new(tool_instr_str);
        tool_instr.set_scale(16.0);
        canvas.draw(
            &tool_instr,
            graphics::DrawParam::new().dest(mint::Point2{ x: ui_margin, y: current_y }).color(graphics::Color::WHITE),
        );
        current_y += line_height;

        // Draw FPS (optional, can be moved)
        let fps_text_str = format!("FPS: {:.0}", ctx.time.fps());
        let mut fps_text = graphics::Text::new(fps_text_str);
        fps_text.set_scale(16.0);
        canvas.draw(
            &fps_text,
             graphics::DrawParam::new().dest(mint::Point2{ x: ui_margin, y: current_y }).color(graphics::Color::WHITE),
        );

        // Draw Simulation Button
        let button_color = if self.game_mode == GameMode::Build {
            graphics::Color::from([0.2, 0.7, 0.2, 1.0]) // Greenish for "Start"
        } else {
            graphics::Color::from([0.7, 0.2, 0.2, 1.0]) // Reddish for "Reset"
        };
        let button_mesh = graphics::Mesh::new_rectangle(
            ctx, graphics::DrawMode::fill(), self.sim_button_rect, button_color
        )?;
        canvas.draw(&button_mesh, graphics::DrawParam::default());

        let button_label_str = if self.game_mode == GameMode::Build {
            "Start Test"
        } else {
            "Reset Build"
        };
        let mut button_label = graphics::Text::new(button_label_str);
        button_label.set_scale(18.0);
        let label_width = button_label.measure(ctx)?.x;
        let label_height = button_label.measure(ctx)?.y;
        let label_x = self.sim_button_rect.x + (self.sim_button_rect.w - label_width) / 2.0;
        let label_y = self.sim_button_rect.y + (self.sim_button_rect.h - label_height) / 2.0;
        canvas.draw(
            &button_label,
            graphics::DrawParam::new().dest(mint::Point2{ x: label_x, y: label_y }).color(graphics::Color::BLACK),
        );

        // Draw Tool Buttons
        let tool_button_color = graphics::Color::from([0.4, 0.4, 0.5, 1.0]);
        let tool_button_active_color = graphics::Color::from([0.6, 0.6, 0.7, 1.0]);
        let tool_labels = ["Joint", "Beam", "Delete"]; // Match order of tools_to_add reversed & reversed again

        for (i, (tool, rect)) in self.tool_buttons.iter().enumerate() {
            let color = if *tool == self.current_tool { tool_button_active_color } else { tool_button_color };
            let mesh = graphics::Mesh::new_rectangle(ctx, graphics::DrawMode::fill(), *rect, color)?;
            canvas.draw(&mesh, graphics::DrawParam::default());

            if i < tool_labels.len() {
                let mut label = graphics::Text::new(tool_labels[i]);
                label.set_scale(16.0);
                let lw = label.measure(ctx)?.x;
                let lh = label.measure(ctx)?.y;
                let lx = rect.x + (rect.w - lw) / 2.0;
                let ly = rect.y + (rect.h - lh) / 2.0;
                canvas.draw(&label, graphics::DrawParam::new().dest(mint::Point2{x: lx, y: ly}).color(graphics::Color::WHITE));
            }
        }

        canvas.finish(ctx)
    }

     fn mouse_motion_event(&mut self, _ctx: &mut Context, x: f32, y: f32, _dx: f32, _dy: f32) -> GameResult {
        self.mouse_pos = Vec2::new(x as f64, y as f64);
        Ok(())
    }

    fn mouse_button_down_event(
        &mut self,
        _ctx: &mut Context,
        button: MouseButton, // Use imported MouseButton
        x: f32,
        y: f32,
    ) -> GameResult {
        if button == MouseButton::Left {
            let click_pos_screen = mint::Point2{ x, y };
            let click_pos_world = Vec2::new(x as f64, y as f64);

            // Check for Simulation Button Click FIRST
            if self.sim_button_rect.contains(click_pos_screen) {
                match self.game_mode {
                    GameMode::Build => {
                        println!("Starting Test Mode");
                        self.game_mode = GameMode::Test;
                        // Apply initial velocity to the ball
                        if let Some(ball_idx) = self.goal_ball_body_index {
                            if ball_idx < self.world.bodies.len() {
                                // Give it a push to the right
                                self.world.bodies[ball_idx].linear_velocity = Vec2::new(150.0, -50.0);
                                println!(" Applied initial velocity to ball {}", ball_idx);
                            } else {
                                println!(" Warning: Goal ball index {} out of bounds.", ball_idx);
                            }
                        } else {
                            println!(" Warning: Goal ball index not set.");
                        }
                    }
                    GameMode::Test => {
                        println!("Resetting to Build Mode");
                        self.game_mode = GameMode::Build;
                        // Reset ball position and velocity
                        if let Some(ball_idx) = self.goal_ball_body_index {
                            if ball_idx < self.world.bodies.len() {
                                self.world.bodies[ball_idx].position = self.ball_start_pos;
                                self.world.bodies[ball_idx].linear_velocity = Vec2::ZERO;
                                self.world.bodies[ball_idx].angular_velocity = 0.0;
                                self.world.bodies[ball_idx].force = Vec2::ZERO; // Clear any accumulated force
                                self.world.bodies[ball_idx].torque = 0.0; // Clear any accumulated torque
                                println!(" Reset ball {}", ball_idx);
                            } else {
                                println!(" Warning: Goal ball index {} out of bounds on reset.", ball_idx);
                            }
                        } else {
                            println!(" Warning: Goal ball index not set for reset.");
                        }
                         // TODO: Optionally reset dynamic nodes/beams too?
                         // Resetting dynamic nodes requires storing their initial build state.
                         // For now, the collapsed bridge remains when resetting.
                    }
                }
                 return Ok(()); // Consumed the click, don't process other tools
            }

            // Check for Tool Button Click
            for (tool, rect) in &self.tool_buttons {
                 if rect.contains(click_pos_screen) {
                     if self.game_mode == GameMode::Build { // Only allow tool switching in Build mode
                        println!("Tool selected: {:?}", tool);
                        self.current_tool = *tool;
                        self.start_node_idx = None; // Cancel beam placement
                        return Ok(()); // Consumed click
                     } else {
                         println!("Cannot switch tools in Test Mode.");
                         return Ok(()); // Consumed click, do nothing
                     }
                 }
            }

            // If no UI button was clicked, proceed with build tools (only in Build mode)
            if self.game_mode == GameMode::Build { // Only allow building/deleting in Build mode
                let click_tolerance = 10.0;
                match self.current_tool {
                    BuildTool::PlaceNode(node_type) => {
                        if self.find_node_near_point(click_pos_world, click_tolerance).is_none() {
                            println!("Placing {:?} node at {:.2},{:.2}", node_type, click_pos_world.x, click_pos_world.y);
                            let node_radius = 5.0;
                            let node_shape = Shape::Circle(Circle::new(node_radius));
                            let mut body_idx = None;
                            let position_to_add = click_pos_world;
                            match node_type {
                                NodeType::Dynamic => {
                                    let mut node_body = RigidBody::new(0.1, node_shape);
                                    node_body.position = position_to_add;
                                    let idx = self.world.add_body(node_body);
                                    body_idx = Some(idx);
                                    println!(" Added dynamic body with index: {}", idx);
                                },
                                NodeType::Fixed => {
                                    let mut anchor_body = RigidBody::new(0.0, node_shape);
                                    anchor_body.position = position_to_add;
                                    let idx = self.world.add_body(anchor_body);
                                    body_idx = Some(idx);
                                    println!(" Added static body with index: {}", idx);
                                },
                            }
                            let new_node = Node {
                                position: position_to_add,
                                node_type,
                                body_index: body_idx,
                                is_active: true,
                            };
                            self.nodes.push(new_node);
                        } else {
                            println!("Clicked near existing node, not placing new node.");
                        }
                    },
                    BuildTool::PlaceBeam => {
                        if let Some(clicked_node_idx) = self.find_node_near_point(click_pos_world, click_tolerance) {
                            // Ensure clicked node is active (find_node_near_point already checks this)
                            if let Some(start_idx) = self.start_node_idx {
                                // Ensure start node is still active and valid
                                 if start_idx < self.nodes.len() && self.nodes[start_idx].is_active {
                                    if start_idx != clicked_node_idx {
                                        if let (Some(body_a_idx), Some(body_b_idx)) = (self.nodes[start_idx].body_index, self.nodes[clicked_node_idx].body_index) {
                                            let node_a_pos = self.nodes[start_idx].position;
                                            let node_b_pos = self.nodes[clicked_node_idx].position;
                                            let length = (node_a_pos - node_b_pos).magnitude();
                                            if length > 1e-6 {
                                                println!("Placing beam between node {} and node {}", start_idx, clicked_node_idx);
                                                let beam = Beam {
                                                    node_a_idx: start_idx,
                                                    node_b_idx: clicked_node_idx,
                                                    is_active: true, // New beams are active
                                                };
                                                self.beams.push(beam);
                                                let constraint = DistanceConstraint::new(body_a_idx, body_b_idx, Vec2::ZERO, Vec2::ZERO, length);
                                                self.world.add_constraint(Box::new(constraint));
                                                self.start_node_idx = None;
                                            } else {
                                                println!("Nodes too close, not creating beam.");
                                                self.start_node_idx = None;
                                            }
                                        } else {
                                             println!("One or both selected nodes do not have a physics body. Cannot create constraint.");
                                             self.start_node_idx = None;
                                        }
                                    } else {
                                        println!("Clicked the same node twice, cancelling beam placement.");
                                        self.start_node_idx = None;
                                    }
                                 } else {
                                    // Start node became inactive or index invalid, cancel
                                    self.start_node_idx = None;
                                 }
                            } else {
                                // This is the first node click (and it's active)
                                println!("Selected start node for beam: {}", clicked_node_idx);
                                self.start_node_idx = Some(clicked_node_idx);
                            }
                        } else {
                            println!("Clicked empty space, cancelling beam placement.");
                            self.start_node_idx = None;
                        }
                    },
                    BuildTool::Delete => {
                         println!("Delete tool active - Click node to delete.");
                         if let Some(clicked_node_idx) = self.find_node_near_point(click_pos_world, click_tolerance) {
                             // Check if node type is Fixed (cannot delete anchors/fixed nodes)
                             if self.nodes[clicked_node_idx].node_type == NodeType::Fixed {
                                 println!("Cannot delete fixed anchor nodes.");
                                 return Ok(());
                             }

                             println!("Deleting node index: {}", clicked_node_idx);
                             // 1. Mark node as inactive
                             self.nodes[clicked_node_idx].is_active = false;

                             // Store body index before potentially clearing node's body_index field
                             let deleted_body_idx_opt = self.nodes[clicked_node_idx].body_index;

                             // 2. Deactivate physics body (if it exists)
                             if let Some(body_idx) = deleted_body_idx_opt {
                                 if body_idx < self.world.bodies.len() {
                                     println!(" Deactivating body index: {}", body_idx);
                                     self.world.bodies[body_idx].inv_mass = 0.0;
                                     self.world.bodies[body_idx].inv_inertia = 0.0;
                                     self.world.bodies[body_idx].linear_velocity = Vec2::ZERO;
                                     self.world.bodies[body_idx].angular_velocity = 0.0;
                                     // Maybe move it away too? self.world.bodies[body_idx].position = Vec2::new(-1000.0, -1000.0);
                                 } else {
                                     println!(" Warning: Body index {} out of bounds for node {}", body_idx, clicked_node_idx);
                                 }
                             }

                             // 3. Deactivate connected beams and remove constraints
                             let mut constraints_to_remove = Vec::new();
                             for beam_idx in 0..self.beams.len() {
                                // Check beam status and connectivity *before* getting immutable borrow
                                let should_deactivate = {
                                    let beam = &self.beams[beam_idx]; // Short-lived immutable borrow
                                    beam.is_active && (beam.node_a_idx == clicked_node_idx || beam.node_b_idx == clicked_node_idx)
                                };

                                 if should_deactivate {
                                     println!(" Deactivating beam connected to node {}", clicked_node_idx);

                                     // Get node indices *before* mutable borrow
                                     let node_a_idx = self.beams[beam_idx].node_a_idx;
                                     let node_b_idx = self.beams[beam_idx].node_b_idx;

                                     // Now perform the mutable borrow
                                     self.beams[beam_idx].is_active = false;

                                     // Find the corresponding constraint to remove using stored indices
                                     if let (Some(body_a_idx), Some(body_b_idx)) = (self.nodes[node_a_idx].body_index, self.nodes[node_b_idx].body_index) {
                                         for constraint_idx in (0..self.world.constraints.len()).rev() {
                                             if let Some(dc) = (*self.world.constraints[constraint_idx]).as_any().downcast_ref::<DistanceConstraint>() {
                                                if (dc.body_a_idx == body_a_idx && dc.body_b_idx == body_b_idx) ||
                                                   (dc.body_a_idx == body_b_idx && dc.body_b_idx == body_a_idx)
                                                {
                                                    println!(" Marking constraint index {} for removal.", constraint_idx);
                                                    constraints_to_remove.push(constraint_idx);
                                                    break;
                                                }
                                             }
                                         }
                                     } else {
                                        println!(" Warning: Could not get body indices for beam connected to deleted node {}.", clicked_node_idx);
                                    }
                                 }
                             }
                             // Remove constraints outside the loop
                             constraints_to_remove.sort_unstable_by(|a, b| b.cmp(a));
                             constraints_to_remove.dedup();
                             for constraint_idx in constraints_to_remove {
                                 println!(" Removing constraint index: {}", constraint_idx);
                                 if constraint_idx < self.world.constraints.len() {
                                     self.world.constraints.remove(constraint_idx);
                                 } else {
                                     println!(" Warning: Constraint index {} out of bounds during removal.", constraint_idx);
                                 }
                             }

                             // Also cancel beam placement if the deleted node was the start node
                             if self.start_node_idx == Some(clicked_node_idx) {
                                 self.start_node_idx = None;
                             }
                         } else {
                             println!("Delete tool: Clicked empty space.");
                         }
                    },
                    BuildTool::Select => {} // Ignore for now
                }
            } else if button == MouseButton::Left {
                 println!("Building/deleting only allowed in Build mode.");
            }
        }
        Ok(())
    }

    fn key_down_event(
            &mut self,
            ctx: &mut Context,
            input: KeyInput,
            _repeated: bool,
        ) -> GameResult {
            match input.keycode {
                Some(KeyCode::Escape) => ctx.request_quit(),
                // Tool Switching (only works in Build mode?)
                Some(k) if self.game_mode == GameMode::Build => {
                    match k {
                        KeyCode::Key1 => {
                            self.current_tool = BuildTool::PlaceNode(NodeType::Dynamic);
                            self.start_node_idx = None;
                            println!("Tool: {:?}", self.current_tool);
                        },
                        KeyCode::Key2 => {
                            self.current_tool = BuildTool::PlaceNode(NodeType::Fixed);
                            self.start_node_idx = None;
                            println!("Tool: {:?}", self.current_tool);
                        },
                        KeyCode::Key3 => {
                            self.current_tool = BuildTool::PlaceBeam;
                            self.start_node_idx = None;
                            println!("Tool: {:?}", self.current_tool);
                        },
                        KeyCode::Key4 | KeyCode::D => { // Use Key4 or D
                            self.current_tool = BuildTool::Delete;
                            self.start_node_idx = None;
                            println!("Tool: {:?}", self.current_tool);
                        },
                        _ => (), // Ignore other keys in build mode
                    }
                }
                // Remove Tab key mode switching
                // Some(KeyCode::Tab) => { ... }
                _ => (), // Ignore all keys in Test mode or unhandled keys in Build mode
            }
            Ok(())
        }

}

// --- Main Function ---
pub fn main() -> GameResult {
    let (mut ctx, event_loop) = ContextBuilder::new("bridge_builder", "Your Name")
        .window_setup(conf::WindowSetup::default().title("Bridge Builder!"))
        .window_mode(conf::WindowMode::default().dimensions(SCREEN_WIDTH, SCREEN_HEIGHT))
        // .add_resource_path("resources") // If you have resources like fonts
        .build()?;

    let state = GameState::new(&mut ctx)?;
    event::run(ctx, event_loop, state)
} 