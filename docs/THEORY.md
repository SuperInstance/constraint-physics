# Theory: Zero-Holonomy Constraint Physics

## Overview

Traditional physics engines simulate motion by computing forces from Newton's laws, integrating accelerations to velocities, and velocities to positions. This requires O(n²) collision checks, complex solvers, and careful timestep management.

**Constraint physics asks: what if the physics IS the constraint satisfaction?**

Instead of computing what forces WOULD make constraints hold, we directly measure whether they DO hold and propagate corrections.

## Laman's Theorem and Rigidity

Laman's theorem provides the graph-theoretic foundation: a planar framework (set of points with distance constraints) is **minimally rigid** when:

E = 2V - 3

Where:
- E = number of constraints (edges)
- V = number of bodies (vertices)

### Three Regimes

| Regime | Condition | Behavior |
|--------|-----------|----------|
| **Under-constrained** | E < 2V - 3 | Bodies can drift freely; need stabilization |
| **Minimally rigid** | E = 2V - 3 | Exact constraint; system is a mechanism |
| **Over-constrained** | E > 2V - 3 | Emergence appears: stresses, collisions, contact forces |

The over-constrained regime is where interesting physics happens — collisions, contact, friction, fracture. These are all manifestations of **constraint emergence**, not separate force calculations.

## H₁ and Cycle Detection

The **first Betti number** β₁ measures the independent cycles in the constraint graph:

β₁ = E - V + C

Where C is the number of connected components. Each cycle represents a potential location of constraint violation.

### Why H₁ Maps to Physics

- A triangle (3 bodies, 3 constraints): β₁ = 3 - 3 + 1 = 1. One constraint is redundant → internal stress appears.
- Two overlapping triangles (4 bodies, 5 constraints): β₁ = 5 - 4 + 1 = 2. Two stress paths.
- A cloth grid with N×N cells: β₁ grows with grid size, representing the network of distributed stresses.

When β₁ > 0, the system has **holonomy**: you can't traverse a closed loop of constraints and return to the same configuration. The non-zero sum around any cycle IS the physics — it's the force that needs to exist for the constraint to hold.

## Holonomy Resolution

Holonomy measures the total constraint violation in the system. The resolution algorithm:

1. **Measure signed violations** for each constraint:
   - Positive: bodies too far apart (stretched)
   - Negative: bodies too close (compressed)

2. **Distribute corrections** inversely proportional to mass:
   - Each body receives Δx proportional to 1/m_i
   - Given constraints i→j with violation v, direction d⃗:
     - Body i: Δx⃗ᵢ = d⃗ · v · k · (1/mᵢ) / (1/mᵢ + 1/mⱼ)
     - Body j: Δx⃗ⱼ = -d⃗ · v · k · (1/mⱼ) / (1/mᵢ + 1/mⱼ)

3. **Propagate**: Corrections cascade through the constraint graph neighbor by neighbor, decreasing by stiffness with each hop.

4. **Convergence**: After sufficient iterations, holonomy approaches zero — all constraints are satisfied simultaneously.

### Comparison to Traditional Solver

| Aspect | Traditional | ZHC Constraint |
|--------|-------------|----------------|
| Core loop | F=ma → integrate → collide → resolve | C(x)=0 → measure → correct |
| Cost | O(n²) collision pairs | O(n) per constraint cycle |
| Stability | Requires small timesteps (CFL) | Intrinsically stable (position-based) |
| Penetration | Impulse-based resolution | Direct position correction |
| Cloth simulation | Spring networks, complex damping | Distance constraints, graph relaxation |
| Collision detection | Spatial partitioning (BVH, grid) | H₁ cycle detection on topology change |

## The Constraint-Based Pipeline

### Phase 1: Topology Update

For a pendulum:
```
Bodies: [pivot (static), ball (mass=1)]
Constraints: [distance(pivot, ball, 3.0)]
Graph: V=2, E=1 → E = 2V-3 = 1 → minimally rigid
Holonomy: 0 (no cycles)
```

### Phase 2: Gravity Application

Gravity modifies velocity as a state variable:
```
ball.velocity.y += -g × dt
```

This is the only "non-constraint" operation. Gravity is an external field, not a topology property.

### Phase 3: Velocity Integration

Position update from velocity:
```
ball.position += ball.velocity × dt
```

This moves the body off its constraint surface.

### Phase 4: Constraint Satisfaction

The constraint detects violation and corrects:
```
violation = |ball_pos - pivot_pos| - 3.0  // e.g., -0.1 (slightly compressed)
correction = d⃗ × violation × k            // pushes apart
ball.position += correction × (1/m_ball) / (1/m_ball + 1/m_pivot)
```

### Phase 5: Collision Detection via H₁

When two bodies overlap, the topology changes:
- A new collision constraint enters the graph
- E increases by 1 for each contact
- If E > 2V - 3, over-constrained → holonomy > 0
- H₁ detection identifies the cycle where violation accumulates
- Correction propagates to separate the bodies

## Convergence and Stability

The constraint satisfaction loop converges because:

1. Each correction reduces |violation| for the target constraint
2. Corrections are linear in violation (no overshoot for single constraint)
3. Chain corrections (neighbor → neighbor) decrease with distance
4. Gauss-Seidel relaxation guarantees monotonic convergence for distance constraints

Holonomy = √(Σ vᵢ²) approaches zero as constraints satisfy. The system reaches equilibrium when holonomy ≈ 0.

## Trade-offs and Limitations

### Advantages
- No integration error accumulation (position-based, not velocity-based)
- Intrinsically handles redundant constraints
- Graph-theoretic collision detection (no spatial hash needed)
- Near-linear scaling for most configurations

### Limitations
- No momentum conservation (position corrections violate physics)
- Requires more iterations for stiff systems
- Friction is emergent, not explicit
- Stacking requires specialized contact resolution

These limitations are features, not bugs — they arise from the choice of position-based constraint satisfaction over impulse-based dynamics. For many applications (cloth, soft bodies, mechanisms), the trade-off is favorable.
