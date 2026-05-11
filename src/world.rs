use crate::body::PhysicsBody;
use crate::constraint::{Constraint, ConstraintType};
use crate::graph::{ConstraintGraph, H1Result, LamanResult};
use crate::vec2::Vec2;

/// The constraint-based physics world.
///
/// Instead of integrating forces through a tick loop, the world
/// measures constraint satisfaction and propagates corrections
/// using ZHC (Zero-Holonomy Constraint) principles.
pub struct PhysicsWorld {
    pub bodies: Vec<PhysicsBody>,
    pub constraints: Vec<Constraint>,
    pub constraint_graph: ConstraintGraph,
    pub gravity: Vec2,
    pub holonomy: f64,
    pub iteration_count: u64,
}

impl PhysicsWorld {
    /// Create a new empty physics world
    pub fn new() -> Self {
        Self {
            bodies: Vec::new(),
            constraints: Vec::new(),
            constraint_graph: ConstraintGraph::new(),
            gravity: Vec2::new(0.0, -9.81),
            holonomy: 0.0,
            iteration_count: 0,
        }
    }

    /// Add a body to the world, returns its ID
    pub fn add_body(&mut self, body: PhysicsBody) -> usize {
        let idx = self.bodies.len();
        self.bodies.push(body);
        idx
    }

    /// Add a constraint between two body IDs
    pub fn add_constraint(&mut self, constraint: Constraint) -> usize {
        let idx = self.constraints.len();
        self.constraints.push(constraint);
        idx
    }

    /// Find a body by its ID
    pub fn find_body(&self, body_id: u64) -> Option<&PhysicsBody> {
        self.bodies.iter().find(|b| b.id == body_id)
    }

    /// Find a body index by its ID
    pub fn find_body_index(&self, body_id: u64) -> Option<usize> {
        self.bodies.iter().position(|b| b.id == body_id)
    }

    /// Find a body by its ID (mutable)
    pub fn find_body_mut(&mut self, body_id: u64) -> Option<&mut PhysicsBody> {
        self.bodies.iter_mut().find(|b| b.id == body_id)
    }

    // ============================================================
    // Physics Pipeline (replaces traditional tick)
    // ============================================================

    /// 1. Build the constraint graph from bodies and constraints
    pub fn build_graph(&mut self) {
        self.constraint_graph = ConstraintGraph::build(&self.bodies, &self.constraints);
    }

    /// 2. Laman rigidity check
    pub fn laman_check(&self) -> LamanResult {
        self.constraint_graph.laman_check()
    }

    /// 3. H1 emergence detection
    pub fn h1_detection(&self) -> H1Result {
        self.constraint_graph.h1_detection()
    }

    /// Apply gravity to all dynamic bodies (turns gravity into velocity deltas)
    pub fn apply_gravity(&mut self, g: f64) {
        for body in &mut self.bodies {
            if !body.is_static {
                body.velocity.y -= g * 0.016; // Roughly ~60 FPS timestep
            }
        }
    }

    /// Apply a custom gravity vector
    pub fn apply_gravity_vec(&mut self, gravity: Vec2, dt: f64) {
        for body in &mut self.bodies {
            if !body.is_static {
                body.velocity = body.velocity + gravity * dt;
            }
        }
    }

    /// Apply velocity to position (position update from velocity)
    pub fn apply_velocity(&mut self, dt: f64) {
        for body in &mut self.bodies {
            if !body.is_static {
                body.position = body.position + body.velocity * dt;
            }
        }
    }

    /// 4. ZHC constraint satisfaction — the core of constraint physics
    ///
    /// Instead of computing forces and integrating, we directly measure
    /// constraint violations and propagate corrections through the system.
    ///
    /// This implements a Gauss-Seidel-like relaxation on position constraints:
    /// For each constraint, measure the violation and project both bodies
    /// toward satisfaction, weighted by inverse mass.
    pub fn satisfy_constraints(&mut self, iterations: usize) {
        let epsilon = 1e-10;
        let mut total_holonomy: f64 = 0.0;

        for _iter in 0..iterations {
            for constraint in &self.constraints {
                // Find body positions
                let pos_a = self.bodies.iter()
                    .find(|b| b.id == constraint.body_a)
                    .map(|b| b.position)
                    .unwrap_or(Vec2::zero());
                let pos_b = self.bodies.iter()
                    .find(|b| b.id == constraint.body_b)
                    .map(|b| b.position)
                    .unwrap_or(Vec2::zero());

                // Measure signed violation
                let signed_violation = constraint.signed_violation(pos_a, pos_b);
                if signed_violation.abs() < epsilon {
                    continue;
                }

                // Get body indices
                let Some(idx_a) = self.find_body_index(constraint.body_a) else { continue };
                let Some(idx_b) = self.find_body_index(constraint.body_b) else { continue };

                let inv_mass_a = self.bodies[idx_a].inverse_mass;
                let inv_mass_b = self.bodies[idx_b].inverse_mass;
                let total_inv = inv_mass_a + inv_mass_b;
                if total_inv < epsilon {
                    continue;
                }

                // Direction from A to B
                let dir = constraint.correction_direction(pos_a, pos_b);
                if dir.length_sq() < epsilon {
                    continue;
                }

                let stiffness = constraint.stiffness.max(0.1);

                // Distribute correction inversely proportional to mass
                // signed_violation > 0: too far apart → push together (A+B toward each other)
                // signed_violation < 0: too close → push apart (A+B away from each other)
                let correction_mag_a = signed_violation * stiffness * inv_mass_a / total_inv;
                let correction_mag_b = signed_violation * stiffness * inv_mass_b / total_inv;

                // Apply position corrections (body_a +dir, body_b -dir)
                let (body_a, body_b) = &mut get_two_mut(&mut self.bodies, idx_a, idx_b);
                if !body_a.is_static {
                    body_a.position.x += dir.x * correction_mag_a;
                    body_a.position.y += dir.y * correction_mag_a;
                }
                if !body_b.is_static {
                    body_b.position.x -= dir.x * correction_mag_b;
                    body_b.position.y -= dir.y * correction_mag_b;
                }

                total_holonomy += signed_violation * signed_violation;
            }
        }

        self.holonomy = total_holonomy.sqrt();
        self.iteration_count += iterations as u64;
    }

    /// Collision detection using H1 emergence
    ///
    /// Bodies that are close enough generate collision constraints
    pub fn detect_collisions_h1(&mut self) -> Vec<(u64, u64, f64)> {
        let mut collisions = Vec::new();

        for i in 0..self.bodies.len() {
            for j in (i + 1)..self.bodies.len() {
                let a = &self.bodies[i];
                let b = &self.bodies[j];

                // Skip static-static pairs
                if a.is_static && b.is_static {
                    continue;
                }

                let dist = a.position.distance(b.position);
                let radius_a = a.shape.bounding_radius();
                let radius_b = b.shape.bounding_radius();
                let min_dist = radius_a + radius_b;

                if dist < min_dist {
                    let penetration = min_dist - dist;
                    collisions.push((a.id, b.id, penetration));
                }
            }
        }

        // Collect info for constraint updates, separate from mutable borrow
        let collision_info: Vec<(u64, u64, f64, f64)> = collisions.iter().map(|(id_a, id_b, _pen)| {
            let body_a = self.find_body(*id_a).unwrap();
            let body_b = self.find_body(*id_b).unwrap();
            let rest_len = body_a.shape.bounding_radius() + body_b.shape.bounding_radius();
            (*id_a, *id_b, rest_len, *_pen)
        }).collect();

        // Add or update collision constraints
        for (id_a, id_b, rest_len, _penetration) in &collision_info {
            // Check if constraint already exists
            let exists = self.constraints.iter_mut().any(|c| {
                if (c.body_a == *id_a && c.body_b == *id_b)
                    || (c.body_a == *id_b && c.body_b == *id_a)
                {
                    if matches!(c.constraint_type, ConstraintType::Collision) {
                        c.rest_length = *rest_len;
                    }
                    true
                } else {
                    false
                }
            });

            if !exists {
                static NEXT_ID: std::sync::atomic::AtomicU64 =
                    std::sync::atomic::AtomicU64::new(1);
                let cid = NEXT_ID.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                let mut new_c = Constraint::new(
                    cid, *id_a, *id_b, ConstraintType::Collision, 1.0,
                );
                new_c.rest_length = *rest_len;
                self.constraints.push(new_c);
            }
        }

        collisions
    }

    /// 5. Resolve violations: propagate corrections to neighbors
    pub fn resolve_violations(&mut self) -> f64 {
        self.satisfy_constraints(5);

        // After constraint satisfaction, also damp velocity for stability
        for body in &mut self.bodies {
            if !body.is_static {
                body.velocity = body.velocity * 0.99; // Damping
            }
        }

        self.holonomy
    }

    /// Full single physics step
    pub fn step(&mut self, dt: f64) {
        // 1. Apply gravity
        self.apply_gravity_vec(self.gravity, dt);

        // 2. Apply velocity to position
        self.apply_velocity(dt);

        // 3. Build graph
        self.build_graph();

        // 4. Detect collisions via H1
        self.detect_collisions_h1();

        // 5. Satisfy constraints
        self.satisfy_constraints(10);

        // 6. Resolve remaining violations
        self.resolve_violations();
    }

    /// Get the global holonomy (measure of total constraint violation)
    pub fn holonomy(&self) -> f64 {
        self.holonomy
    }

    /// Check if system has converged (holonomy near 0)
    pub fn is_converged(&self, tolerance: f64) -> bool {
        self.holonomy < tolerance
    }
}

/// Helper to get two mutable references to elements in a slice
fn get_two_mut<T>(slice: &mut [T], i: usize, j: usize) -> (&mut T, &mut T) {
    if i < j {
        let (left, right) = slice.split_at_mut(j);
        (&mut left[i], &mut right[0])
    } else {
        let (left, right) = slice.split_at_mut(i);
        (&mut right[0], &mut left[j])
    }
}
