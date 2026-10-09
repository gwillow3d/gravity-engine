use std::time::{self, Instant};
use log::info;
use macroquad::input::{is_key_down, is_key_released};
use numfmt::{Formatter, Precision, Scales};
use clap::Parser;

use crate::{generator::*, renderer::RenderState};

mod simulation;
mod generator;
mod renderer;
mod version;

#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Args {
    #[arg(short='n', long="bodies", default_value_t = 1024)]
    n_bodies: usize,
    #[arg(short='t', long="tracers", default_value_t = 0)]
    n_tracers: usize,
    #[arg(short='g', long="gravity", default_value_t = 1.14)]
    gravity: f64,
    #[arg(short='d', long="delta", default_value_t = 0.01)]
    delta: f64,
    #[arg(short='e', long="epsilon", default_value_t = 1.0)]
    epsilon: f64,
    #[arg(long="seed", default_value_t = 21177)]
    seed: u64,
    #[arg(long="debug", default_value_t = false)]
    debug: bool,
    #[arg(long="f64", default_value_t = false)]
    high_precision: bool,
}

#[macroquad::main("Gravity Engine")]
async fn main() -> std::io::Result<()> {
    env_logger::init();

    let args = Args::parse();

    let n_bodies = args.n_bodies;
    let n_tracers = args.n_tracers;
    let gravity = args.gravity;
    let delta = args.delta;
    let epsilon = args.epsilon;
    let debug = args.debug;
    let seed = args.seed;
    let high_precision = args.high_precision;

    let mut f = Formatter::new()
        .separator(',').unwrap()
        .precision(Precision::Decimals(0))
        .scales(Scales::none());

    info!("Gravity Engine (Version {:?})\n", version::VERSION);

    let mut state = simulation::State::new_default(n_bodies, n_tracers, gravity, epsilon, delta);
    generator::simple_disc(&mut state, OrbitalDiskConfig {
        particles: n_bodies,
        tracers: n_tracers,
        gravity,
        star_mass: 150000.0,
        dist_range: 150.0..400.0,
        mass_range: 1.0..3.0,
        seed: seed
    });
    
    simulation::init(&mut state);

    info!("Ready to start simulation!");
    info!("Bodies: {n_bodies}");
    info!("Tracers: {n_tracers}");
    info!("Interactions: {}", f.fmt2(state.ops_per_step()));
    info!("Gravity: {gravity}");
    info!("Softening: {epsilon}m");
    info!("Timestep: {delta}s");

    // Counters
    let mut fps_ctr = 0;
    let mut ms_ctr = 0.0;

    let mut start = time::Instant::now();
    let mut fps = 0.0;

    let mut show_profiling = debug;

    loop {
        let step_start = time::Instant::now();
        simulation::step_leapfrog(&mut state);
        let step_ms = time::Instant::now().duration_since(step_start).as_micros() as f64 / 1000.0;

        let ke = state.ke();
        let gpe = state.gpe();
        let total = ke + gpe;

        fps_ctr += 1;
        ms_ctr += step_ms;

        if fps_ctr >= 15 {
            let avg_ms = Instant::now().duration_since(start).as_millis() as f64 / 15 as f64;
            fps = 1000.0f64 / avg_ms;
            start = Instant::now();
            fps_ctr = 0;
        }

        if ms_ctr >= 3000.0 && debug {
            info!("[Ge3D] Energy: {total} ({ke} KE, {gpe} GPE)");

            ms_ctr = 0.0;
        }

        if is_key_released(macroquad::input::KeyCode::F3) {
            show_profiling = !show_profiling;
        }

        renderer::render(&RenderState {sim: state.clone(), physics_time: step_ms, fps, show_profiling}).await;
    }
}