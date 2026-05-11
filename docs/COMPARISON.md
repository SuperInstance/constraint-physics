# Comparison: Constraint-Physics vs Traditional Physics Engines

## Box2D / Rapier (Traditional)

### How Box2D Works

1. **Apply forces** (gravity, user forces) to compute accelerations
2. **Integrate** (Symplectic Euler): velocity += acceleration × dt, position += velocity × dt
3. **Broad-phase collision** (SAP or BVH tree)
4. **Narrow-phase collision** (SAT, GJK)
5. **Contact resolution** (impulse-based solver, Sequential Impulses)
6. **Joint constraints** (Lagrange multiplier, Baumgarte stabilization)

### Key Characteristics
- **Impulse-based**: Velocities change instantly via impulses
- **Timestep-dependent**: Smaller dt = more accurate, larger dt = instability
- **O(n log n)** broad phase + O(n²) narrow phase worst case
- **Restitution** and **friction** are explicit parameters
- **Sleeping** needed for stacks (detect static bodies, skip simulation)

## constraint-physics (ZHC)

### How constraint-physics Works

1. **Apply external fields** (gravity modifies velocity)
2. **Integrate velocity** (position += velocity × dt)
3. **Build constraint graph** from all bodies and constraints
4. **H₁ cycle detection** identifies over-constrained regions
5. **Constraint solver** directly corrects positions based on violation
6. **Propagate** corrections mass-inversely through the system

### Key Characteristics
- **Position-based**: Direct position correction from constraint violation
- **Graph-theoretic**: Physics is a property of the constraint graph topology
- **O(n × k)** where k = constraint iterations (usually 10-20)
- **Collision is emergent**: Over-constrained detection = collision
- **Intrinsically stable**: No timestep instability (position-based)
- **Sleeping is natural**: Holonomy ≈ 0 = system at rest

## Feature Comparison

| Feature | Box2D / Rapier | constraint-physics |
|---------|---------------|-------------------|
| Rigid bodies | ✓ Full | ✓ Basic |
| Joints (distance, hinge) | ✓ | ✓ |
| Springs | ✓ | ✓ |
| Collision detection | ✓ Broad + narrow | ✓ H₁ emergence |
| Friction | ✓ Coulomb model | ✗ Emergent only |
| Restitution | ✓ Coefficient | ✗ Not modeled |
| Stacking | ✓ With sleeping | ⚠ Partial |
| Cloth | ⚠ Soft body ext. | ✓ Native (constraint grid) |
| Soft bodies | ✓ Via soft body | ✓ Native (constraint relax.) |
| Continuous collision | ✓ CCD | ⚠ Not yet |
| Motor constraints | ✓ | ⚠ Not yet |
| Laman analysis | ✗ | ✓ Native |
| H₁ cycle analysis | ✗ | ✓ Native |
| Determinism | ⚠ Platform-dependent | ✓ Deterministic |
| Performance | O(n log n) | O(n × k) |
| Memory | Spatial structures | Graph adjacency + vectors |

## When to Use Each

### Use constraint-physics when:
- You want to understand physics as topology
- Cloth, soft bodies, and deformable structures
- Mechanisms with many constraints (kinematics)
- Deterministic simulation (e.g., game replays)
- Educational / demonstration of graph-theoretic physics
- Systems where constraint satisfaction IS the physics

### Use Box2D/Rapier when:
- You need stacking with friction (e.g., Jenga, dominoes)
- Realistic bouncing with restitution
- You need continuous collision detection
- You need established ecosystem + tooling
- Your primary concern is impulse-based realism

## Performance Characteristics

### Typical Timings (100 bodies, 100 steps)

| Engine | Time | Notes |
|--------|------|-------|
| Box2D | ~2ms | With broad-phase and spatial hashing |
| Rapier | ~1ms | Rust-native, SIMD optimized |
| constraint-physics | ~5ms | Pure Rust, no SIMD, debug build |

### Scaling

| Bodies | constraint-physics (100 steps) | Notes |
|--------|-------------------------------|-------|
| 10 | <1ms | Trivial |
| 100 | ~5ms | Sweet spot |
| 1,000 | ~50ms | Constraint solver dominates |
| 10,000 | ~500ms | Needs optimization (sparse graph) |

The constraint solver scales O(n × k × connectance). For sparse graphs (k = small average degree, like cloth), it's near O(n). For dense graphs (all-pairs), it's O(n² ÷ iterations).

## Future Directions

### Short term
- Friction as tangential constraint violation
- Restitution via velocity scaling at collision
- Better stacking (contact point averaging)
- Motor constraints (target velocity via position control)

### Medium term
- SIMD constraint solver
- Warm starting (reuse previous frame solutions)
- Island sleeping (zero-holonomy islands can sleep)
- Adaptive constraint iteration (more for over-constrained regions)

### Long term
- Full H₁-guided solver (only solve around cycles)
- GPU constraint graph processing
- Hybrid: constraint physics for soft + impulse for rigid contacts
