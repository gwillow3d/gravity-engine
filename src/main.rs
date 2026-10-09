use std::{fs::File, io::{BufWriter, Write}, time::{self, Instant}};
use numfmt::{Formatter, Precision, Scales};
use clap::Parser;

mod simulation;
mod pointcache2;
mod generator;
mod renderer;

#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Args {
    #[arg(short='n', default_value_t = 1024)]
    body_count: u32,
    #[arg(long="nlw", default_value_t = 0)]
    lw_count: u32,
    #[arg(short='s', long="steps", default_value_t = 0)]
    duration: u32,
    #[arg(short='g', long="gravity", default_value_t = 1.14)]
    gravity: f32,
    #[arg(short='d', long="delta", default_value_t = 0.01)]
    timestep: f32,
    #[arg(short='e', long="epsilon", default_value_t = 1.0)]
    softening: f32,
    #[arg(long="seed", default_value_t = 21177)]
    seed: u64,
    #[arg(long="debug", default_value_t = false)]
    debug: bool
}

#[macroquad::main("Gravity Engine 3D")]
async fn main() -> std::io::Result<()> {
    let args = Args::parse();

    let body_count = args.body_count;
    let lw_count = args.lw_count;
    let duration = args.duration;
    let gravity = args.gravity;
    let timestep = args.timestep;
    let softening = args.softening;
    let is_debug = args.debug;
    let seed = args.seed;

    let mut f = Formatter::new()
        .separator(',').unwrap()
        .precision(Precision::Decimals(0))
        .scales(Scales::none());

    println!("Gravity Engine 3D (Version {:?})\n", env!("CARGO_PKG_VERSION"));

    let mut state = simulation::State::new_default(body_count as usize, lw_count as usize, gravity, softening, timestep);
    generator::orbital_disk_2d(&mut state, body_count as usize, lw_count as usize, gravity, seed, 50000.0);

    let op_count = state.ops_per_step();
    let op_count_str = f.fmt2(op_count);

    println!("Ready to start simulation!");
    println!("Particles: {}", body_count + lw_count);
    println!("... massive: {body_count}");
    println!("... lightweight: {lw_count}");
    println!("Interactions: {op_count_str}");
    println!("Duration: {duration} steps");
    println!("Gravity: {gravity}");
    println!("Softening: {softening}m");
    println!("Timestep: {timestep}s");

    println!();

    let mut start = time::Instant::now();
    let mut fps_accumulator = 0;
    let mut ms_accumulator = 0.0;
    let mut fps = 0.0;
    let mut step: u64 = 0;

    loop {
        let step_start = time::Instant::now();
        simulation::step_leapfrog(&mut state);
        let step_ms = time::Instant::now().duration_since(step_start).as_micros() as f64 / 1000.0;

        let ke = state.ke();
        let gpe = state.gpe();
        let total = ke + gpe;

        fps_accumulator += 1;
        ms_accumulator += step_ms;

        if fps_accumulator >= 15 {
            let avg_ms = Instant::now().duration_since(start).as_millis() as f64 / 15 as f64;
            fps = 1000.0f64 / avg_ms;
            start = Instant::now();
            fps_accumulator = 0;
        }

        if ms_accumulator >= 3000.0 && is_debug {
            let display_step = step + 1;
            println!("[Ge3D] {display_step} / {duration} | Energy: {total} ({ke} KE, {gpe} GPE)");

            ms_accumulator = 0.0;
        }

        renderer::render(&state, step_ms, fps, step as u32, duration).await;

        step += 1;
    }
}

/*for i in 0..duration {
        if i % sample_rate == 0 {
            if write_pc2 {
                for i in 0..state.msv_body_count() {
                    let pos = Vec3A::new(state.pos_x[i], state.pos_y[i], state.pos_z[i]);
                    pointcache2::write_point(&mut writer, &pos)?;
                }
                for i in 0..state.lw_body_count() {
                    let pos = Vec3A::new(state.lw_pos_x[i], state.lw_pos_y[i], state.lw_pos_z[i]);
                    pointcache2::write_point(&mut writer, &pos)?;
                }
            }
        }
    }*/