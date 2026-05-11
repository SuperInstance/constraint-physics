# constraint-physics

**Physics Without Integration — ZHC Constraint Satisfaction Engine**

A constraint-based physics engine that replaces the traditional force-integration tick loop with **Zero-Holonomy Constraint (ZHC) detection and resolution**. Instead of computing F=ma and integrating to find positions, this engine directly measures whether topology constraints are satisfied and propagates corrections.

## The Core Insight

**Traditional physics:** F=ma → integrate → collide → resolve (tick loop, O(n²))

**Constraint physics:** C(x) = 0 → measure satisfaction → propagate violations (ZHC, O(n) per cycle)

The key insight: constraint satisfaction IS physics. A pendulum isn't a differential equation to solve — it's a distance constraint plus gravity. A cloth isn't a spring network to integrate — it's a grid of distance constraints settling toward equilibrium. Stability is a graph property (Laman rigidity), not a force balance.

## Demo

```bash
# Pendulum: a ball swinging on a constraint
cargo run -- demo pendulum

# Cloth: 10x10 grid settling under gravity
cargo run -- demo cloth

# Stack: blocks with collision constraints
cargo run -- demo stack --blocks 5

# Analyze a scene file for ZHC properties
cargo run -- analyze --file scene.json

# Benchmark
cargo run -- benchmark --bodies 1000 --steps 100
```

## Architecture

```
src/
├── vec2.rs           # 2D vector math
├── shape.rs          # Circle, Rectangle, Polygon shapes
├── body.rs           # PhysicsBody with position, velocity, mass
├── constraint.rs     # Constraint types (Distance, Hinge, Spring, Collision)
├── graph.rs          # ConstraintGraph — Laman check + H1 detection
├── world.rs          # PhysicsWorld — the constraint-based physics pipeline
├── demo.rs           # Demo runners (pendulum, cloth, stack, benchmark)
├── main.rs           # CLI entry point
└── lib.rs            # Public API
```

### Physics Pipeline

1. **Graph phase**: Build constraint graph from all bodies + constraints
2. **Laman check**: Is the system rigid? (E >= 2V-3)
   - Under-constrained → bodies can drift (need stabilization)
   - Over-constrained → emergence (collisions, contact forces)
3. **H1 detection**: β1 = E - V + C → each cycle = a constraint violation location
4. **Holonomy resolution**: Distribute corrections inversely proportional to mass
5. **Position update**: Corrections applied as position deltas (no velocity integration)

### Core Types

```rust
struct PhysicsBody {
    id: u64,
    position: Vec2,
    velocity: Vec2,
    mass: f64,
    is_static: bool,
    shape: Shape,
}

struct Constraint {
    id: u64,
    body_a: u64,
    body_b: u64,
    constraint_type: ConstraintType,
    stiffness: f64,
    rest_length: f64,
    violation: f64,
}

enum ConstraintType {
    Distance(f64),       // Fixed distance between bodies
    Hinge,               // Same position, free rotation
    Fixed,               // Same position and rotation
    Spring(f64, f64),    // Spring constant, damping
    Collision,           // Non-penetration constraint
}

struct PhysicsWorld {
    bodies: Vec<PhysicsBody>,
    constraints: Vec<Constraint>,
    constraint_graph: ConstraintGraph,
    holonomy: f64,       // Global physics consistency (0 = satisfied)
}
```

## Usage

### Library

```toml
[dependencies]
constraint-physics = { git = "https://github.com/SuperInstance/constraint-physics" }
```

```rust
use constraint_physics::*;

let mut world = PhysicsWorld::new();

// Create a pendulum
let pivot = world.add_body(PhysicsBody::static_rect(Vec2::new(0.0, 0.0), 0.1, 0.1));
let ball = world.add_body(PhysicsBody::ball(Vec2::new(2.0, -2.0), 0.2));
world.add_constraint(distance_constraint(
    world.bodies[pivot].id,
    world.bodies[ball].id,
    3.0,
));

// Run constraint-based physics
for step in 0..60 {
    world.apply_gravity(9.81);
    world.apply_velocity(0.016);
    world.satisfy_constraints(15);
    world.resolve_violations();

    println!("Step {}: ball at ({:.2}, {:.2}), holonomy={:.4}",
             step, world.bodies[ball].position.x, world.bodies[ball].position.y,
             world.holonomy);
}
```

### CLI

```bash
# Run demos
constraint-physics demo pendulum
constraint-physics demo cloth
constraint-physics demo stack --blocks 10

# Analyze a JSON scene file
constraint-physics analyze --file my_scene.json

# Benchmark
constraint-physics benchmark --bodies 500 --steps 200
```

### Scene JSON Format

```json
{
  "bodies": [
    {"id": 1, "x": 0.0, "y": 0.0, "mass": 1000000.0, "shape": "circle", "radius": 0.2},
    {"id": 2, "x": 2.0, "y": 0.0, "mass": 1.0, "shape": "circle", "radius": 0.2}
  ],
  "constraints": [
    {"id": 1, "body_a": 1, "body_b": 2, "constraint_type": "distance", "value": 3.0}
  ]
}
```

## Tests

```bash
cargo test
```

Covers: distance constraint satisfaction, collision resolution, pendulum motion, cloth convergence, Laman rigidity checks, H1 cycle detection, static bodies, inverse mass distribution, graph connectivity.

## Theory

See [docs/THEORY.md](docs/THEORY.md) for the full mathematical framework behind ZHC constraint physics.

## License

MIT
