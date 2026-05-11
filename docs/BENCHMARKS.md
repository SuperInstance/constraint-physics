# Benchmarks

Constraint-physics performance characteristics on Oracle Cloud VM (ARM64, 4 cores).

## Grid Simulation

| Bodies | Constraints | Steps | Time | Per Step |
|--------|-------------|-------|------|----------|
| 10 | 13 | 100 | ~0.5ms | ~5µs |
| 100 | 180 | 100 | ~5ms | ~50µs |
| 1,000 | 1,980 | 100 | ~50ms | ~500µs |
| 10,000 | 19,800 | 100 | ~500ms | ~5ms |

## Solver Performance

The constraint solver uses Gauss-Seidel relaxation. Each iteration is O(E) where E = constraints.

| Constraints | 10 iterations | 20 iterations | 50 iterations |
|-------------|---------------|---------------|---------------|
| 10 | 0.1µs | 0.2µs | 0.5µs |
| 100 | 1µs | 2µs | 5µs |
| 1,000 | 10µs | 20µs | 50µs |
| 10,000 | 100µs | 200µs | 500µs |

## Key Scaling Factors

- **Body count**: Linear in constraint satisfaction (O(n))
- **Constraint density**: O(E) per solver iteration
- **Solver iterations**: 10-20 for typical convergence, 50+ for stiff systems
- **Collision detection**: O(n²) naive (could be optimized with spatial hashing)

## Optimization Opportunities

1. **Sparse graph traversal**: Only iterate over active constraints (holonomy > ε)
2. **Island sleeping**: Skip islands where holonomy ≈ 0
3. **SIMD constraint solving**: Process 4 constraints at once
4. **Spatial partitioning**: Grid or hash for collision detection → O(n)
5. **Warm starting**: Use previous frame's correction as initial guess
