use crate::vec2::Vec2;

/// Types of constraints between two bodies
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ConstraintType {
    /// Fixed distance between bodies
    Distance(f64),
    /// Same position, free rotation
    Hinge,
    /// Same position and rotation (effectively same body)
    Fixed,
    /// Spring constant, damping
    Spring(f64, f64),
    /// Non-penetration constraint (added by collision detection)
    Collision,
}

/// A constraint connecting two physics bodies
#[derive(Debug, Clone)]
pub struct Constraint {
    pub id: u64,
    pub body_a: u64,
    pub body_b: u64,
    pub constraint_type: ConstraintType,
    /// How rigid the constraint is (analogous to physical stiffness)
    pub stiffness: f64,
    /// Natural length for distance constraints
    pub rest_length: f64,
    /// Current violation magnitude (0 = satisfied)
    pub violation: f64,
}

impl Constraint {
    pub fn new(
        id: u64,
        body_a: u64,
        body_b: u64,
        constraint_type: ConstraintType,
        stiffness: f64,
    ) -> Self {
        let rest_length = match constraint_type {
            ConstraintType::Distance(l) => l,
            ConstraintType::Spring(rest, _) => rest,
            _ => 0.0,
        };
        Self {
            id,
            body_a,
            body_b,
            constraint_type,
            stiffness,
            rest_length,
            violation: 0.0,
        }
    }

    /// Measure the current violation with sign.
    ///
    /// Positive = bodies too far apart (stretched beyond rest length).
    /// Negative = bodies too close (compressed below rest length).
    /// Zero = constraint satisfied.
    pub fn signed_violation(&self, pos_a: Vec2, pos_b: Vec2) -> f64 {
        match self.constraint_type {
            ConstraintType::Distance(target) | ConstraintType::Spring(target, _) => {
                pos_a.distance(pos_b) - target
            }
            ConstraintType::Hinge | ConstraintType::Fixed => {
                pos_a.distance(pos_b)
            }
            ConstraintType::Collision => {
                let dist = pos_a.distance(pos_b);
                let pen = self.rest_length - dist;
                if pen > 0.0 { -pen } else { 0.0 }
                // Negative = penetrating; correction pushes apart
            }
        }
    }

    /// Compute the correction direction for body_a (body_b gets -direction)
    pub fn correction_direction(&self, pos_a: Vec2, pos_b: Vec2) -> Vec2 {
        let delta = pos_b - pos_a;
        let dist = delta.length();
        if dist < 1e-12 {
            return Vec2::zero();
        }
        delta.normalized()
    }
}

/// Helper to create a distance constraint
pub fn distance_constraint(body_a: u64, body_b: u64, distance: f64) -> Constraint {
    static NEXT_ID: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(1);
    let id = NEXT_ID.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    Constraint::new(id, body_a, body_b, ConstraintType::Distance(distance), 1.0)
}

/// Helper to create a spring constraint
pub fn spring_constraint(body_a: u64, body_b: u64, rest_length: f64, damping: f64) -> Constraint {
    static NEXT_ID: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(1);
    let id = NEXT_ID.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    Constraint::new(id, body_a, body_b, ConstraintType::Spring(rest_length, damping), 0.5)
}

/// Helper to create a hinge constraint
pub fn hinge_constraint(body_a: u64, body_b: u64) -> Constraint {
    static NEXT_ID: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(1);
    let id = NEXT_ID.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    Constraint::new(id, body_a, body_b, ConstraintType::Hinge, 1.0)
}
