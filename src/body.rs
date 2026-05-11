use std::sync::atomic::{AtomicU64, Ordering};
use crate::shape::Shape;
use crate::vec2::Vec2;

static NEXT_BODY_ID: AtomicU64 = AtomicU64::new(1);

/// A physics body in the constraint-based world
#[derive(Debug, Clone)]
pub struct PhysicsBody {
    pub id: u64,
    pub position: Vec2,
    pub velocity: Vec2,
    pub mass: f64,
    pub inverse_mass: f64,
    pub is_static: bool,
    pub shape: Shape,
}

impl PhysicsBody {
    pub fn new(id: u64, position: Vec2, shape: Shape, mass: f64) -> Self {
        let is_static = mass.is_infinite() || mass <= 0.0;
        Self {
            id,
            position,
            velocity: Vec2::zero(),
            mass,
            inverse_mass: if is_static { 0.0 } else { 1.0 / mass },
            is_static,
            shape,
        }
    }

    fn next_id() -> u64 {
        NEXT_BODY_ID.fetch_add(1, Ordering::Relaxed)
    }

    /// Create a ball/circle body
    pub fn ball(position: Vec2, radius: f64) -> Self {
        Self::new(Self::next_id(), position, Shape::Circle { radius }, 1.0)
    }

    /// Create a ball with custom mass
    pub fn ball_with_mass(position: Vec2, radius: f64, mass: f64) -> Self {
        Self::new(Self::next_id(), position, Shape::Circle { radius }, mass)
    }

    /// Create a static rectangle body
    pub fn static_rect(position: Vec2, width: f64, height: f64) -> Self {
        Self::new(
            Self::next_id(),
            position,
            Shape::Rectangle { half_width: width / 2.0, half_height: height / 2.0 },
            f64::INFINITY,
        )
    }

    /// Create a body with explicit ID
    pub fn with_id(id: u64, position: Vec2, shape: Shape, mass: f64) -> Self {
        Self::new(id, position, shape, mass)
    }

    /// Reset the ID counter (for deterministic tests)
    pub fn reset_id_counter() {
        NEXT_BODY_ID.store(1, Ordering::Relaxed);
    }

    /// Get current ID counter value (for test purposes)
    pub fn current_id_counter() -> u64 {
        NEXT_BODY_ID.load(Ordering::Relaxed)
    }
}
