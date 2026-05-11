use std::collections::{HashMap, HashSet, VecDeque};
use crate::constraint::Constraint;
use crate::body::PhysicsBody;

/// The constraint graph: adjacency list of body_id -> Vec<constraint_id>
/// Used for Laman rigidity checks and H1 cycle detection.
#[derive(Debug, Clone)]
pub struct ConstraintGraph {
    /// adjacency[body_id] = (neighbor_body_id, constraint_id)
    pub adjacency: HashMap<u64, Vec<(u64, u64)>>,
    pub num_bodies: usize,
    pub num_constraints: usize,
}

impl ConstraintGraph {
    pub fn new() -> Self {
        Self {
            adjacency: HashMap::new(),
            num_bodies: 0,
            num_constraints: 0,
        }
    }

    /// Build the constraint graph from bodies and constraints
    pub fn build(bodies: &[PhysicsBody], constraints: &[Constraint]) -> Self {
        let mut graph = Self::new();
        graph.num_bodies = bodies.len();
        graph.num_constraints = constraints.len();

        // Add all body nodes
        for body in bodies {
            graph.adjacency.entry(body.id).or_default();
        }

        // Add constraint edges
        for constraint in constraints {
            let a = constraint.body_a;
            let b = constraint.body_b;
            let cid = constraint.id;
            graph.adjacency.entry(a).or_default().push((b, cid));
            graph.adjacency.entry(b).or_default().push((a, cid));
        }

        graph
    }

    /// Laman rigidity check: E >= 2*V - 3 for minimal rigidity
    /// Returns true if the system is rigid (may be over-constrained)
    pub fn laman_check(&self) -> LamanResult {
        let v = self.num_bodies;
        let e = self.num_constraints;

        if v <= 1 {
            return LamanResult::UnderConstrained {
                edges_needed: 0,
                degree: e as f64,
            };
        }

        let needed = 2 * v - 3;
        if e < needed {
            LamanResult::UnderConstrained {
                edges_needed: needed - e,
                degree: e as f64 / needed as f64,
            }
        } else if e == needed {
            LamanResult::MinimallyRigid
        } else {
            LamanResult::OverConstrained {
                excess: e - needed,
                degree: e as f64 / needed as f64,
            }
        }
    }

    /// Detect all connected components in the constraint graph
    pub fn connected_components(&self) -> Vec<HashSet<u64>> {
        let mut visited = HashSet::new();
        let mut components = Vec::new();

        for &node in self.adjacency.keys() {
            if visited.contains(&node) {
                continue;
            }

            let mut component = HashSet::new();
            let mut queue = VecDeque::new();
            queue.push_back(node);
            visited.insert(node);

            while let Some(current) = queue.pop_front() {
                component.insert(current);
                if let Some(neighbors) = self.adjacency.get(&current) {
                    for &(neighbor, _) in neighbors {
                        if !visited.contains(&neighbor) {
                            visited.insert(neighbor);
                            queue.push_back(neighbor);
                        }
                    }
                }
            }

            components.push(component);
        }

        components
    }

    /// H1 emergence detection: find independent cycles in the constraint graph
    /// Uses a spanning tree approach to find the cycle basis.
    /// Returns the identified cycle basis (each cycle is a set of body IDs).
    pub fn h1_detection(&self) -> H1Result {
        let v = self.num_bodies;
        let e = self.num_constraints;
        let laman = self.laman_check();

        // B1 = E - V + C (C = connected components)
        let components = self.connected_components();
        let c = components.len();
        let b1 = if e >= v - c { e - v + c } else { 0 };

        // Find cycle basis via DFS spanning tree
        let mut cycles = Vec::new();
        let mut visited = HashSet::new();
        let mut parent: HashMap<u64, (u64, u64)> = HashMap::new(); // child -> (parent, edge_id)

        for &start in self.adjacency.keys() {
            if visited.contains(&start) {
                continue;
            }

            // DFS spanning tree
            let mut stack = vec![(start, start, u64::MAX)];
            visited.insert(start);

            while let Some((node, par, edge_id)) = stack.pop() {
                parent.insert(node, (par, edge_id));
                if let Some(neighbors) = self.adjacency.get(&node) {
                    for &(neighbor, cid) in neighbors {
                        if neighbor == par {
                            continue;
                        }
                        if !visited.contains(&neighbor) {
                            visited.insert(neighbor);
                            stack.push((neighbor, node, cid));
                        } else if parent.contains_key(&neighbor) {
                            // Found a cycle: trace from node and neighbor to LCA
                            if let Some(cycle) = trace_cycle(&parent, node, neighbor) {
                                cycles.push(cycle);
                            }
                        }
                    }
                }
            }
        }

        H1Result {
            b1,
            num_cycles: cycles.len(),
            cycles,
            is_over_constrained: matches!(laman, LamanResult::OverConstrained { .. }),
            emergence_locations: self.find_emergence_locations(&components, b1),
        }
    }

    /// Find emergence locations: where over-constrained regions exist
    /// These correspond to collisions, contacts, and stressed regions.
    fn find_emergence_locations(&self, components: &[HashSet<u64>], b1: usize) -> Vec<EmergenceLocation> {
        let mut locations = Vec::new();
        if b1 == 0 {
            return locations;
        }

        for component in components {
            let v_c = component.len();
            let e_c: usize = component.iter()
                .filter_map(|body_id| self.adjacency.get(body_id))
                .map(|neighbors| neighbors.len())
                .sum::<usize>() / 2; // Each edge counted twice

            let needed = 2 * v_c - 3;
            if e_c > needed && v_c > 1 {
                locations.push(EmergenceLocation {
                    components: component.clone(),
                    excess: e_c - needed,
                    location_type: "over-constrained".to_string(),
                });
            }
        }

        locations
    }

    /// Get edges for a specific body
    pub fn edges_for(&self, body_id: u64) -> Vec<(u64, u64)> {
        self.adjacency.get(&body_id).cloned().unwrap_or_default()
    }
}

/// Trace a cycle between node and neighbor in the DFS tree
fn trace_cycle(parent: &HashMap<u64, (u64, u64)>, start: u64, end: u64) -> Option<HashSet<u64>> {
    let mut path_a = Vec::new();
    let mut path_b = Vec::new();

    // Trace from start upward
    let mut current = start;
    path_a.push(current);
    loop {
        if let Some(&(par, _)) = parent.get(&current) {
            if par == current {
                break;
            }
            current = par;
            path_a.push(current);
            if current == end {
                // Found intersection at end, path_a is complete
                break;
            }
        } else {
            break;
        }
    }

    // Trace from end upward
    current = end;
    path_b.push(current);
    loop {
        if let Some(&(par, _)) = parent.get(&current) {
            if par == current {
                break;
            }
            current = par;
            if current == start {
                break; // Already found via path_a
            }
            path_b.push(current);
            // Check if we've intersected path_a
            if path_a.contains(&current) {
                break;
            }
        } else {
            break;
        }
    }

    // Build cycle set
    let mut cycle: HashSet<u64> = HashSet::new();
    for &node in &path_a {
        cycle.insert(node);
    }
    for &node in &path_b {
        cycle.insert(node);
    }

    if cycle.len() > 2 {
        Some(cycle)
    } else {
        None
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum LamanResult {
    UnderConstrained { edges_needed: usize, degree: f64 },
    MinimallyRigid,
    OverConstrained { excess: usize, degree: f64 },
}

#[derive(Debug, Clone)]
pub struct H1Result {
    pub b1: usize,
    pub num_cycles: usize,
    pub cycles: Vec<HashSet<u64>>,
    pub is_over_constrained: bool,
    pub emergence_locations: Vec<EmergenceLocation>,
}

#[derive(Debug, Clone)]
pub struct EmergenceLocation {
    pub components: HashSet<u64>,
    pub excess: usize,
    pub location_type: String,
}
