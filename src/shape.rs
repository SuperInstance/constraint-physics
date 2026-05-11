use crate::vec2::Vec2;

/// Shape of a physics body
#[derive(Debug, Clone, PartialEq)]
pub enum Shape {
    Circle { radius: f64 },
    Rectangle { half_width: f64, half_height: f64 },
    Polygon { vertices: Vec<Vec2> },
}

impl Shape {
    /// Moment of inertia about center of mass
    pub fn inertia_moment(&self, mass: f64) -> f64 {
        match self {
            Shape::Circle { radius } => 0.5 * mass * radius * radius,
            Shape::Rectangle { half_width, half_height } => {
                mass * (half_width * half_width + half_height * half_height) / 3.0
            }
            Shape::Polygon { vertices } => {
                // Approximate as average triangle area moment
                let mut moment = 0.0;
                for i in 0..vertices.len() {
                    let j = (i + 1) % vertices.len();
                    let cross = vertices[i].cross(vertices[j]);
                    moment += cross * (vertices[i].length_sq()
                        + vertices[i].dot(vertices[j])
                        + vertices[j].length_sq());
                }
                moment * mass / 12.0
            }
        }
    }

    /// Bounding circle radius
    pub fn bounding_radius(&self) -> f64 {
        match self {
            Shape::Circle { radius } => *radius,
            Shape::Rectangle { half_width, half_height } => {
                (half_width * half_width + half_height * half_height).sqrt()
            }
            Shape::Polygon { vertices } => {
                vertices.iter().map(|v| v.length()).fold(0.0_f64, f64::max)
            }
        }
    }
}
