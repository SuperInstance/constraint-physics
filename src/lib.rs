//! `constraint-physics` — a constraint-based physics engine using ZHC principles.
//!
//! Instead of integrating forces through a tick loop (F=ma → integrate → collide → resolve),
//! constraint-physics measures constraint satisfaction directly and propagates corrections
//! using Zero-Holonomy Constraint (ZHC) detection.
//!
//! ## Core concepts
//!
//! - **PhysicsBody**: Bodies with position, velocity, mass, and shape
//! - **Constraint**: Connections between bodies (distance, hinge, spring, collision)
//! - **ConstraintGraph**: The constraint topology used for Laman/H1 analysis
//! - **PhysicsWorld**: Container that runs the constraint-based physics pipeline

// Allow dead code for library API surface that's only used via CLI/examples
#![allow(dead_code)]

pub mod vec2;
pub mod shape;
pub mod body;
pub mod constraint;
pub mod graph;
pub mod world;

pub use vec2::Vec2;
pub use shape::Shape;
pub use body::PhysicsBody;
pub use constraint::{Constraint, ConstraintType, distance_constraint, spring_constraint, hinge_constraint};
pub use graph::{ConstraintGraph, LamanResult, H1Result};
pub use world::PhysicsWorld;
