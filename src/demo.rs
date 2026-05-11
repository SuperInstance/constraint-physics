use crate::body::PhysicsBody;
use crate::constraint::{distance_constraint, Constraint, ConstraintType};
use crate::shape::Shape;
use crate::vec2::Vec2;
use crate::world::PhysicsWorld;

/// Pendulum demo: a single ball hanging from a fixed point by a distance constraint
pub fn pendulum_demo(steps: usize) {
    println!("=== Pendulum Demo ===");
    println!("A ball on a pendulum, driven by constraint satisfaction (no integration)\n");

    let mut world = PhysicsWorld::new();

    // Fixed pivot
    let pivot = world.add_body(PhysicsBody::static_rect(Vec2::new(0.0, 0.0), 0.1, 0.1));

    // Pendulum ball
    let ball = world.add_body(PhysicsBody::ball_with_mass(
        Vec2::new(2.0, -2.0), // Start at angle (offset from straight down)
        0.2,
        1.0,
    ));

    // Distance constraint (pendulum arm)
    world.add_constraint(distance_constraint(
        world.bodies[pivot].id,
        world.bodies[ball].id,
        3.0,
    ));

    println!("Step {:>5} | Position           | Velocity          | Holonomy", "");
    for step in 0..steps {
        // Apply gravity (modifies velocity)
        world.apply_gravity(9.81);

        // Integrate velocity into position
        world.apply_velocity(0.016);

        // Constraint-based resolution (replaces force-based correction)
        world.satisfy_constraints(15);
        world.resolve_violations();

        if step % 5 == 0 || step == steps - 1 {
            let pos = world.bodies[ball].position;
            let vel = world.bodies[ball].velocity;
            println!(
                "Step {:>5} | ({:>7.3}, {:>7.3}) | ({:>7.3}, {:>7.3}) | {:.6}",
                step, pos.x, pos.y, vel.x, vel.y, world.holonomy
            );
        }
    }

    // Verify pendulum behavior
    let ball_pos = world.bodies[ball].position;
    let pivot_pos = world.bodies[pivot].position;
    let dist = ball_pos.distance(pivot_pos);
    println!("\nFinal distance from pivot: {:.4} (target: 3.0)", dist);
    println!("Holonomy: {:.6}", world.holonomy);
    println!("Constraint violation: {:.6}\n", (dist - 3.0).abs());
}

/// Cloth demo: 10x10 grid of bodies connected by distance constraints
pub fn cloth_demo(steps: usize) {
    println!("=== Cloth Demo ===");
    println!("10x10 cloth grid with ~200 constraints\n");

    let mut world = PhysicsWorld::new();
    let grid_size = 10;
    let spacing = 0.5;

    // Create grid of bodies
    let mut grid: Vec<Vec<usize>> = Vec::new();
    let start_x = -(grid_size as f64 * spacing) / 2.0;
    let start_y = 5.0;

    for row in 0..grid_size {
        let mut row_indices = Vec::new();
        for col in 0..grid_size {
            let x = start_x + col as f64 * spacing;
            let y = start_y - row as f64 * spacing;

            // Pin top row
            let body = if row == 0 {
                PhysicsBody::static_rect(Vec2::new(x, y), 0.05, 0.05)
            } else {
                PhysicsBody::ball_with_mass(Vec2::new(x, y), 0.05, 0.1)
            };

            let idx = world.add_body(body);
            row_indices.push(idx);
        }
        grid.push(row_indices);
    }

    // Add distance constraints between adjacent bodies
    for row in 0..grid_size {
        for col in 0..grid_size {
            let id = world.bodies[grid[row][col]].id;

            // Right neighbor
            if col + 1 < grid_size {
                let right_id = world.bodies[grid[row][col + 1]].id;
                world.add_constraint(distance_constraint(id, right_id, spacing));
            }

            // Bottom neighbor
            if row + 1 < grid_size {
                let bottom_id = world.bodies[grid[row + 1][col]].id;
                world.add_constraint(distance_constraint(id, bottom_id, spacing));
            }

            // Diagonal constraints for shear stiffness
            if row + 1 < grid_size && col + 1 < grid_size {
                let diag_id = world.bodies[grid[row + 1][col + 1]].id;
                world.add_constraint(distance_constraint(
                    id,
                    diag_id,
                    spacing * 2.0_f64.sqrt(),
                ));
            }
        }
    }

    let total_bodies = world.bodies.len();
    let total_constraints = world.constraints.len();
    println!("Bodies: {}, Constraints: {}", total_bodies, total_constraints);

    // Run simulation
    for step in 0..steps {
        world.apply_gravity(2.0);
        world.apply_velocity(0.016);
        world.satisfy_constraints(20);
        world.resolve_violations();

        if step % 10 == 0 || step == steps - 1 {
            // Report center point position (approximate cloth behavior)
            let center_idx = grid[grid_size / 2][grid_size / 2];
            let center_pos = world.bodies[center_idx].position;

            // Report max drift of non-pinned nodes
            let max_drift = world.bodies.iter()
                .filter(|b| !b.is_static)
                .map(|b| b.velocity.length())
                .fold(0.0_f64, f64::max);

            println!(
                "Step {:>5} | Center: ({:>7.3}, {:>7.3}) | MaxVel: {:.4} | Holonomy: {:.6}",
                step, center_pos.x, center_pos.y, max_drift, world.holonomy
            );
        }
    }

    let laman = world.laman_check();
    let h1 = world.h1_detection();
    println!("\nLaman: {:?}", laman);
    println!("H1 cycles: {} (b1 = {})", h1.num_cycles, h1.b1);
    println!("Holonomy: {:.6}", world.holonomy);
    println!("Converged: {}\n", world.is_converged(1.0));
}

/// Stack demo: stacking blocks with collision constraints
pub fn stack_demo(num_blocks: usize) {
    println!("=== Stack Demo ===");
    println!("Stacking {} blocks with collision constraints\n", num_blocks);

    let mut world = PhysicsWorld::new();

    // Ground
    let _ground = world.add_body(PhysicsBody::static_rect(
        Vec2::new(0.0, -1.0),
        8.0,
        0.5,
    ));

    // Stack blocks
    let block_size = 0.8;
    let block_mass = 1.0;
    let mut block_indices = Vec::new();

    for i in 0..num_blocks {
        let x = 0.0;
        let y = -1.0 + block_size + i as f64 * block_size * 1.1;

        let block = PhysicsBody::with_id(
            (i + 100) as u64,
            Vec2::new(x, y),
            Shape::Rectangle {
                half_width: block_size / 2.0,
                half_height: block_size / 2.0,
            },
            block_mass,
        );
        let idx = world.add_body(block);
        block_indices.push(idx);
    }

    println!("Bodies: {} (ground + {} blocks)", world.bodies.len(), num_blocks);

    // Run simulation
    for step in 0..120 {
        world.apply_gravity(15.0);
        world.apply_velocity(0.016);
        world.detect_collisions_h1();
        world.satisfy_constraints(15);
        world.resolve_violations();

        if step % 10 == 0 || step == 119 {
            let top = &world.bodies[block_indices[num_blocks - 1]];
            let bot = &world.bodies[block_indices[0]];
            println!(
                "Step {:>5} | Top: ({:>7.3}, {:>7.3}) | Bot: ({:>7.3}, {:>7.3}) | Holonomy: {:.6} | Constraints: {}",
                step,
                top.position.x, top.position.y,
                bot.position.x, bot.position.y,
                world.holonomy,
                world.constraints.len()
            );
        }
    }

    let laman = world.laman_check();
    println!("\nFinal Laman: {:?}", laman);
    println!("Total constraints: {}", world.constraints.len());

    // Check if top block is stable (minimal movement)
    let top_vel = world.bodies[block_indices[num_blocks - 1]].velocity.length();
    println!("Top block velocity: {:.4} (stable if < 0.5)", top_vel);
    println!();
}

/// Analyze a physics scene from a JSON file
pub fn analyze_scene(path: &str) -> Result<(), String> {
    let data = std::fs::read_to_string(path)
        .map_err(|e| format!("Failed to read {}: {}", path, e))?;
    let scene: SceneFile = serde_json::from_str(&data)
        .map_err(|e| format!("Failed to parse {}: {}", path, e))?;

    println!("=== Scene Analysis ===");
    println!("File: {}", path);
    println!("Bodies: {}", scene.bodies.len());
    println!("Constraints: {}", scene.constraints.len());

    // Build world
    let mut world = PhysicsWorld::new();
    for body in &scene.bodies {
        let shape = if body.shape == "circle" {
            Shape::Circle { radius: body.radius.unwrap_or(0.5) }
        } else {
            Shape::Rectangle {
                half_width: body.width.unwrap_or(1.0) / 2.0,
                half_height: body.height.unwrap_or(1.0) / 2.0,
            }
        };
        world.add_body(PhysicsBody::with_id(body.id, Vec2::new(body.x, body.y), shape, body.mass));
    }
    for c in &scene.constraints {
        world.add_constraint(Constraint::new(
            c.id, c.body_a, c.body_b,
            match c.constraint_type.as_str() {
                "distance" => ConstraintType::Distance(c.value.unwrap_or(1.0)),
                "hinge" => ConstraintType::Hinge,
                "fixed" => ConstraintType::Fixed,
                "spring" => ConstraintType::Spring(c.value.unwrap_or(1.0), c.damping.unwrap_or(0.1)),
                "collision" => ConstraintType::Collision,
                _ => ConstraintType::Distance(c.value.unwrap_or(1.0)),
            },
            c.stiffness.unwrap_or(1.0),
        ));
    }

    world.build_graph();

    let laman = world.laman_check();
    let h1 = world.h1_detection();

    println!("\n--- Laman Check ---");
    match laman {
        LamanResult::UnderConstrained { edges_needed, degree } => {
            println!("Status: Under-constrained");
            println!("Edges needed for rigidity: {}", edges_needed);
            println!("Rigidity degree: {:.2}%", degree * 100.0);
        }
        LamanResult::MinimallyRigid => {
            println!("Status: Minimally rigid");
        }
        LamanResult::OverConstrained { excess, degree } => {
            println!("Status: Over-constrained");
            println!("Excess constraints: {}", excess);
            println!("Stress degree: {:.2}%", (degree - 1.0) * 100.0);
        }
    }

    println!("\n--- H1 Detection ---");
    println!("B1 (cycles): {}", h1.b1);
    println!("Cycles found: {}", h1.num_cycles);
    println!("Over-constrained: {}", h1.is_over_constrained);
    println!("Emergence locations: {}", h1.emergence_locations.len());

    for loc in &h1.emergence_locations {
        println!("  - {} region: {} bodies, excess: {}",
                 loc.location_type, loc.components.len(), loc.excess);
    }

    Ok(())
}

/// Run a benchmark
pub fn benchmark(num_bodies: usize, num_steps: usize) {
    println!("=== Benchmark ===");
    println!("Bodies: {}, Steps: {}", num_bodies, num_steps);

    use std::time::Instant;

    let mut world = PhysicsWorld::new();

    // Create a grid of bodies
    let side = (num_bodies as f64).sqrt().ceil() as usize;
    let spacing = 1.0;

    let mut indices = Vec::new();
    for i in 0..num_bodies {
        let row = i / side;
        let col = i % side;
        let x = col as f64 * spacing;
        let y = -(row as f64 * spacing);
        let body = PhysicsBody::ball_with_mass(
            Vec2::new(x, y), 0.2, 1.0,
        );
        indices.push(world.add_body(body));
    }

    // Create constraints to form a connected mesh
    for i in 0..num_bodies {
        let row = i / side;
        let col = i % side;
        let id = world.bodies[indices[i]].id;

        if col + 1 < side && i + 1 < num_bodies {
            let right_id = world.bodies[indices[i + 1]].id;
            world.add_constraint(distance_constraint(id, right_id, spacing));
        }
        if row + 1 < side && i + side < num_bodies {
            let bottom_id = world.bodies[indices[i + side]].id;
            world.add_constraint(distance_constraint(id, bottom_id, spacing));
        }
    }

    println!("Setup: {} bodies, {} constraints", world.bodies.len(), world.constraints.len());

    let start = Instant::now();
    for _step in 0..num_steps {
        world.apply_gravity(9.81);
        world.apply_velocity(0.016);
        world.satisfy_constraints(10);
        world.resolve_violations();
    }
    let elapsed = start.elapsed();

    println!("\nTime: {:?}", elapsed);
    println!("Per step: {:?}", elapsed / num_steps as u32);
    println!("Holonomy: {:.6}", world.holonomy);
    println!();
}

// ============================================================
// JSON scene file format
// ============================================================

#[derive(serde::Deserialize)]
struct SceneFile {
    bodies: Vec<SceneBody>,
    constraints: Vec<SceneConstraint>,
}

#[derive(serde::Deserialize)]
struct SceneBody {
    id: u64,
    x: f64,
    y: f64,
    mass: f64,
    shape: String,
    radius: Option<f64>,
    width: Option<f64>,
    height: Option<f64>,
}

#[derive(serde::Deserialize)]
struct SceneConstraint {
    id: u64,
    body_a: u64,
    body_b: u64,
    constraint_type: String,
    value: Option<f64>,
    damping: Option<f64>,
    stiffness: Option<f64>,
}

// Re-export for main.rs
pub use crate::graph::LamanResult;
