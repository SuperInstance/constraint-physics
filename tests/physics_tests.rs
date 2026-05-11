use constraint_physics::*;

/// Helper to reset body ID counter between tests
fn reset_ids() {
    PhysicsBody::reset_id_counter();
}

#[test]
fn test_distance_constraint_satisfaction() {
    reset_ids();
    let mut world = PhysicsWorld::new();

    let pivot = world.add_body(PhysicsBody::static_rect(Vec2::new(0.0, 0.0), 0.1, 0.1));
    let ball = world.add_body(PhysicsBody::ball_with_mass(Vec2::new(2.0, 0.0), 0.2, 1.0));

    world.add_constraint(distance_constraint(
        world.bodies[pivot].id,
        world.bodies[ball].id,
        3.0,
    ));

    // Run constraint satisfaction
    for _ in 0..20 {
        world.satisfy_constraints(10);
    }

    let pivot_pos = world.bodies[pivot].position;
    let ball_pos = world.bodies[ball].position;
    let dist = ball_pos.distance(pivot_pos);

    // Distance should be close to 3.0
    assert!((dist - 3.0).abs() < 0.1,
        "Distance constraint violation too large: {} (expected ~3.0)", dist);
}

#[test]
fn test_collision_constraint_prevents_penetration() {
    reset_ids();
    let mut world = PhysicsWorld::new();

    // Two balls placed overlapping
    let ball1 = world.add_body(PhysicsBody::ball_with_mass(Vec2::new(0.0, 0.0), 0.5, 1.0));
    let ball2 = world.add_body(PhysicsBody::ball_with_mass(Vec2::new(0.3, 0.0), 0.5, 1.0));

    // Detect collisions and resolve
    let collisions = world.detect_collisions_h1();
    assert!(!collisions.is_empty(), "Overlapping balls should produce collisions");

    for _ in 0..20 {
        world.satisfy_constraints(10);
    }

    let pos1 = world.bodies[ball1].position;
    let pos2 = world.bodies[ball2].position;
    let dist = pos1.distance(pos2);

    // Combined radius = 1.0 (0.5 + 0.5)
    assert!(dist >= 0.95,
        "Balls should not penetrate: distance = {} (expected >= 1.0)", dist);
}

#[test]
fn test_gravity_plus_constraint_produces_pendulum_motion() {
    reset_ids();
    let mut world = PhysicsWorld::new();

    let pivot = world.add_body(PhysicsBody::static_rect(Vec2::new(0.0, 0.0), 0.1, 0.1));
    let ball = world.add_body(PhysicsBody::ball_with_mass(Vec2::new(2.0, -2.0), 0.2, 1.0));

    world.add_constraint(distance_constraint(
        world.bodies[pivot].id,
        world.bodies[ball].id,
        3.0,
    ));

    let initial_pos = world.bodies[ball].position;

    // Run simulation
    for _ in 0..30 {
        world.apply_gravity(9.81);
        world.satisfy_constraints(15);
        world.resolve_violations();
    }

    let final_pos = world.bodies[ball].position;

    // Ball should have moved (gravity should pull it down)
    assert!((final_pos.y - initial_pos.y).abs() > 0.01,
        "Pendulum ball should move under gravity: initial y={}, final y={}",
        initial_pos.y, final_pos.y);

    // Distance constraint should still be maintained
    let pivot_pos = world.bodies[pivot].position;
    let dist = final_pos.distance(pivot_pos);
    assert!((dist - 3.0).abs() < 0.2,
        "Distance constraint violated during pendulum motion: {} (expected ~3.0)", dist);
}

#[test]
fn test_cloth_grid_converges_to_equilibrium() {
    reset_ids();
    let mut world = PhysicsWorld::new();
    let grid_size = 5;
    let spacing = 0.5;

    let mut grid: Vec<Vec<usize>> = Vec::new();
    for row in 0..grid_size {
        let mut row_indices = Vec::new();
        for col in 0..grid_size {
            let x = col as f64 * spacing;
            let y = -(row as f64 * spacing);
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

    for row in 0..grid_size {
        for col in 0..grid_size {
            let id = world.bodies[grid[row][col]].id;
            if col + 1 < grid_size {
                let right_id = world.bodies[grid[row][col + 1]].id;
                world.add_constraint(distance_constraint(id, right_id, spacing));
            }
            if row + 1 < grid_size {
                let bottom_id = world.bodies[grid[row + 1][col]].id;
                world.add_constraint(distance_constraint(id, bottom_id, spacing));
            }
        }
    }

    // Run simulation to convergence
    for _ in 0..50 {
        world.apply_gravity(2.0);
        world.satisfy_constraints(20);
        world.resolve_violations();
    }

    // Cloth should somewhat converge (holonomy should decrease)
    assert!(world.holonomy < 5.0,
        "Cloth should converge: holonomy={} (expected < 5.0)", world.holonomy);
}

#[test]
fn test_laman_under_constrained() {
    reset_ids();
    let mut world = PhysicsWorld::new();

    // Two bodies, no constraints: V=2, E=0 => E < 2V-3 = 1
    let _b1 = world.add_body(PhysicsBody::ball_with_mass(Vec2::new(0.0, 0.0), 0.2, 1.0));
    let _b2 = world.add_body(PhysicsBody::ball_with_mass(Vec2::new(1.0, 0.0), 0.2, 1.0));

    world.build_graph();
    let laman = world.laman_check();

    assert!(matches!(laman, LamanResult::UnderConstrained { .. }),
        "Two bodies without constraints should be under-constrained");
}

#[test]
fn test_laman_minimally_rigid() {
    reset_ids();
    let mut world = PhysicsWorld::new();

    // Two bodies with one distance constraint: V=2, E=1 => E = 2V-3 = 1
    let b1 = world.add_body(PhysicsBody::ball_with_mass(Vec2::new(0.0, 0.0), 0.2, 1.0));
    let b2 = world.add_body(PhysicsBody::ball_with_mass(Vec2::new(1.0, 0.0), 0.2, 1.0));

    world.add_constraint(distance_constraint(
        world.bodies[b1].id,
        world.bodies[b2].id,
        1.0,
    ));

    world.build_graph();
    let laman = world.laman_check();

    assert!(matches!(laman, LamanResult::MinimallyRigid),
        "Two bodies with one constraint should be minimally rigid: {:?}", laman);
}

#[test]
fn test_laman_over_constrained() {
    reset_ids();
    let mut world = PhysicsWorld::new();

    // Two bodies with two constraints: V=2, E=2 => E > 2V-3 = 1
    let b1 = world.add_body(PhysicsBody::ball_with_mass(Vec2::new(0.0, 0.0), 0.2, 1.0));
    let b2 = world.add_body(PhysicsBody::ball_with_mass(Vec2::new(1.0, 0.0), 0.2, 1.0));

    world.add_constraint(distance_constraint(
        world.bodies[b1].id,
        world.bodies[b2].id,
        1.0,
    ));
    world.add_constraint(distance_constraint(
        world.bodies[b1].id,
        world.bodies[b2].id,
        0.5,
    ));

    world.build_graph();
    let laman = world.laman_check();

    assert!(matches!(laman, LamanResult::OverConstrained { .. }),
        "Two bodies with two constraints should be over-constrained: {:?}", laman);
}

#[test]
fn test_h1_detection() {
    reset_ids();
    let mut world = PhysicsWorld::new();

    // Three bodies in a triangle with 3 constraints = one cycle
    let b1 = world.add_body(PhysicsBody::ball_with_mass(Vec2::new(0.0, 0.0), 0.2, 1.0));
    let b2 = world.add_body(PhysicsBody::ball_with_mass(Vec2::new(1.0, 0.0), 0.2, 1.0));
    let b3 = world.add_body(PhysicsBody::ball_with_mass(Vec2::new(0.5, 1.0), 0.2, 1.0));

    world.add_constraint(distance_constraint(world.bodies[b1].id, world.bodies[b2].id, 1.0));
    world.add_constraint(distance_constraint(world.bodies[b2].id, world.bodies[b3].id, 1.0));
    world.add_constraint(distance_constraint(world.bodies[b3].id, world.bodies[b1].id, 1.0));

    world.build_graph();
    let h1 = world.h1_detection();

    // Three edges, three vertices: B1 = E - V + C = 3 - 3 + 1 = 1
    assert_eq!(h1.b1, 1, "Triangle should have B1 = 1");
    assert!(h1.num_cycles >= 1, "Triangle should have at least 1 cycle");
}

#[test]
fn test_static_body_doesnt_move() {
    reset_ids();
    let mut world = PhysicsWorld::new();

    let static_body = world.add_body(PhysicsBody::static_rect(Vec2::new(0.0, 0.0), 1.0, 1.0));
    let ball = world.add_body(PhysicsBody::ball_with_mass(Vec2::new(2.0, 0.0), 0.2, 1.0));

    world.add_constraint(distance_constraint(
        world.bodies[static_body].id,
        world.bodies[ball].id,
        1.0,
    ));

    let static_initial = world.bodies[static_body].position;

    for _ in 0..20 {
        world.apply_gravity(9.81);
        world.satisfy_constraints(10);
        world.resolve_violations();
    }

    let static_final = world.bodies[static_body].position;
    let diff = static_initial.distance(static_final);

    assert!(diff < 1e-10,
        "Static body should not move: displacement = {}", diff);
}

#[test]
fn test_inverse_mass_distribution() {
    reset_ids();
    let mut world = PhysicsWorld::new();

    // Heavy ball and light ball connected by constraint
    let heavy = world.add_body(PhysicsBody::ball_with_mass(Vec2::new(0.0, 0.0), 0.3, 100.0));
    let light = world.add_body(PhysicsBody::ball_with_mass(Vec2::new(1.0, 0.0), 0.3, 1.0));

    world.add_constraint(distance_constraint(
        world.bodies[heavy].id,
        world.bodies[light].id,
        2.0,
    ));

    // Push the light ball away
    world.bodies[light].position = Vec2::new(3.0, 0.0);

    for _ in 0..30 {
        world.satisfy_constraints(10);
    }

    let heavy_pos = world.bodies[heavy].position;
    let light_pos = world.bodies[light].position;
    let heavy_disp = heavy_pos.distance(Vec2::new(0.0, 0.0));
    let light_disp = light_pos.distance(Vec2::new(3.0, 0.0));

    // Heavy body should move less than the light body
    assert!(heavy_disp < light_disp,
        "Heavy body ({}) should move less than light body ({})",
        heavy_disp, light_disp);
}

#[test]
fn test_constraint_graph_connectivity() {
    reset_ids();
    let mut world = PhysicsWorld::new();

    // Two separate pairs of bodies
    let p1 = world.add_body(PhysicsBody::ball_with_mass(Vec2::new(0.0, 0.0), 0.2, 1.0));
    let p2 = world.add_body(PhysicsBody::ball_with_mass(Vec2::new(1.0, 0.0), 0.2, 1.0));
    world.add_constraint(distance_constraint(world.bodies[p1].id, world.bodies[p2].id, 1.0));

    let q1 = world.add_body(PhysicsBody::ball_with_mass(Vec2::new(5.0, 0.0), 0.2, 1.0));
    let q2 = world.add_body(PhysicsBody::ball_with_mass(Vec2::new(6.0, 0.0), 0.2, 1.0));
    world.add_constraint(distance_constraint(world.bodies[q1].id, world.bodies[q2].id, 1.0));

    world.build_graph();
    let components = world.constraint_graph.connected_components();

    assert_eq!(components.len(), 2,
        "Two disconnected pairs should produce 2 components, got {}", components.len());
}
