mod vec2;
mod shape;
mod body;
mod constraint;
mod graph;
mod world;
mod demo;

use clap::{Parser, Subcommand};
use std::path::PathBuf;

#[derive(Parser)]
#[command(name = "constraint-physics")]
#[command(about = "ZHC constraint-based physics engine", long_about = None)]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    /// Run a physics demo
    Demo {
        /// Which demo to run: pendulum, cloth, stack
        demo_name: String,
        /// Number of simulation steps
        #[arg(short = 'n', long, default_value_t = 60)]
        steps: usize,
        /// Number of blocks for stack demo
        #[arg(short = 'b', long, default_value_t = 5)]
        blocks: usize,
    },
    /// Analyze a physics scene from a JSON file
    Analyze {
        /// Path to the scene JSON file
        file: PathBuf,
    },
    /// Run a performance benchmark
    Benchmark {
        /// Number of bodies
        #[arg(short = 'b', long, default_value_t = 100)]
        bodies: usize,
        /// Number of simulation steps
        #[arg(short = 's', long, default_value_t = 100)]
        steps: usize,
    },
}

fn main() {
    let cli = Cli::parse();

    match &cli.command {
        Commands::Demo { demo_name, steps, blocks } => {
            match demo_name.as_str() {
                "pendulum" => demo::pendulum_demo(*steps),
                "cloth" => demo::cloth_demo(*steps),
                "stack" => demo::stack_demo(*blocks),
                other => {
                    eprintln!("Unknown demo: {}. Available: pendulum, cloth, stack", other);
                    std::process::exit(1);
                }
            }
        }
        Commands::Analyze { file } => {
            if let Err(e) = demo::analyze_scene(file.to_str().unwrap_or("")) {
                eprintln!("Analysis error: {}", e);
                std::process::exit(1);
            }
        }
        Commands::Benchmark { bodies, steps } => {
            demo::benchmark(*bodies, *steps);
        }
    }
}
