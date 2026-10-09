use std::{fs::File, io::{BufWriter, Write}, time::{self, Instant}};
use macroquad::{camera::{Camera2D, set_camera, set_default_camera}, color::*, math::vec2, shapes::draw_poly, text::draw_text, window::{clear_background, next_frame, screen_height, screen_width}};
use numfmt::{Formatter, Precision, Scales};
use clap::Parser;

use crate::simulation::State;

mod simulation;
mod pointcache2;
mod generator;

#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Args {
    #[arg(short='n', default_value_t = 1024)]
    body_count: u32,
    #[arg(long="nlw", default_value_t = 0)]
    lw_count: u32,
    #[arg(short='s', long="steps", default_value_t = 0)]
    duration: u32,
    #[arg(short='r', long="resolution", default_value_t = 8)]
    resolution: u32,
    #[arg(short='g', long="gravity", default_value_t = 1.14)]
    gravity: f32,
    #[arg(short='d', long="delta", default_value_t = 0.01)]
    timestep: f32,
    #[arg(short='e', long="epsilon", default_value_t = 1.0)]
    softening: f32,
    #[arg(long="seed", default_value_t = 21177)]
    seed: u64,
    #[arg(long="debug", default_value_t = false)]
    debug: bool,
    #[arg(short='o', long="output", default_value_t = "output.pc2".to_string())]
    output: String,
}

#[macroquad::main("Gravity Engine 3D")]
async fn main() -> std::io::Result<()> {
    let args = Args::parse();

    let body_count = args.body_count;
    let lw_count = args.lw_count;
    let duration = args.duration;
    let sample_rate = args.resolution;
    let gravity = args.gravity;
    let timestep = args.timestep;
    let softening = args.softening;
    let is_debug = args.debug;
    let output_file = args.output;
    let seed = args.seed;
    let realtime = args.duration == 0;

    let mut f = Formatter::new()
        .separator(',').unwrap()
        .precision(Precision::Decimals(0))
        .scales(Scales::none());

    
    println!("Gravity Engine 3D (Version {:?})\n", env!("CARGO_PKG_VERSION"));

    let mut writer = BufWriter::new(File::create(output_file)?);

    let write_pc2: bool;
    let total_samples = duration / sample_rate;
    let result = pointcache2::write_header(&mut writer, 1, body_count, 0.0, sample_rate as f32, total_samples);
    if result.is_err() {
        println!("Failed to write PC2 header: {:?}", result.err());
        write_pc2 = false;
    } else { write_pc2 = true; }

    let mut state = simulation::State::new_default(body_count as usize, lw_count as usize, gravity, softening, timestep);
    //generator::generate_random(&mut state, body_count as usize, lw_count as usize, seed);
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

    let start = time::Instant::now();
    simulation::init(&mut state);
    let ms = time::Instant::now().duration_since(start).as_millis() as f32;
    println!("Initialization step took {ms:.2}ms");
    let eta_seconds = ms * duration as f32 / 1000.0;

    println!();

    if eta_seconds > 5.0 {
        println!("Estimated time: ~{}s", eta_seconds);
        if eta_seconds > 240.0 {
            println!("This might take a while.");
        }

        println!();
    }

    let init_energy = state.ke() + state.gpe();

    let mut start = time::Instant::now();
    let mut fps_accumulator = 0;
    let mut ms_accumulator = 0.0;
    let mut fps = 0.0;
    let mut last_tracked_time = start;
    let mut step: u64 = 0;

    loop {
        if step > duration as u64 && !realtime {
            break;
        }

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
            let ms_since_last_update = time::Instant::now().duration_since(last_tracked_time).as_millis() as f32 / sample_rate as f32;
            if realtime {
                print!("[Ge3D/realtime]");
            } else {
                print!("[Ge3D/offline]");
                let display_step = step + 1;
                print!(" {display_step} / {duration}")
            }
            print!(" | Energy: {total} ({ke} KE, {gpe} GPE)");

            if !realtime {
                let eta_seconds = (duration - step as u32) as f32 * ms_since_last_update / 1000.0;
                print!(" | ETA: ~{eta_seconds:.2}s");
            }
            print!("\n");

            ms_accumulator = 0.0;
            last_tracked_time = time::Instant::now();
        }

        render(&state, step_ms, fps, step as u32, duration).await;

        step += 1;
    }

    println!();

    let stop = time::Instant::now();

    let time_ms = stop.duration_since(start).as_millis();
    let time_seconds = time_ms as f32 / 1000.0;
    let ms_per_step = time_ms as f32 / duration as f32;
    println!("Took {time_seconds:.0}s to complete {duration} steps. (About {ms_per_step:.2}ms per step.)");

    let final_energy = state.ke() + state.gpe();
    let energy_delta = final_energy - init_energy;
    println!("Energy varied by {energy_delta:.2} from {init_energy:.2} to {final_energy:.2}.");

    writer.flush()?;
    Ok(())
}

async fn render(state: &State, step_ms: f64, fps: f64, step: u32, max_steps: u32) {
    clear_background(BLACK);
    set_camera(&Camera2D {
        zoom: vec2(0.0025, screen_width() / screen_height() * 0.0025),
        ..Default::default()
    });

    for i in 0..state.msv_body_count() {
        draw_poly(state.pos_x[i], state.pos_y[i], 8, state.mass[i].sqrt(), 0.0, WHITE);
    } for i in 0..state.lw_body_count() {
        draw_poly(state.lw_pos_x[i], state.lw_pos_y[i], 8, 0.2, 0.0, YELLOW);
    }

    set_default_camera();

    let ke = state.ke();
    let gpe = state.gpe();
    let total = ke + gpe;

    let mut offset: f32 = 24.0;
    draw_text("Gravity Engine", 16.0, offset, 16.0, WHITE);

    offset += 16.0 * 2.0;
    if max_steps == 0 {
        draw_text(format!("{:.2} fps", fps), 16.0, offset, 16.0, WHITE);
        offset += 16.0;
    } else {
        draw_text(format!("Step {} / {}", step, max_steps), 16.0, offset, 16.0, WHITE);
        offset += 16.0;
        //draw_text(format!("ETA {:.2}s", fps), 16.0, offset, 16.0, WHITE);
        //offset += 16.0;
    }

    draw_text(format!("{:.2}ms per step", step_ms), 16.0, offset, 16.0, WHITE);
    offset += 16.0;
    draw_text(format!("{} + {} particles", state.msv_body_count(), state.lw_body_count()), 16.0, offset, 16.0, WHITE);
    offset += 16.0;
    draw_text(format!("{} interactions", state.ops_per_step()), 16.0, offset, 16.0, WHITE);
    offset += 16.0 * 2.0;
    draw_text(format!("KE: {:.2} J", ke), 16.0, offset, 16.0, WHITE);
    offset += 16.0;
    draw_text(format!("GPE: {:.2} J", gpe), 16.0, offset, 16.0, WHITE);
    offset += 16.0;
    draw_text(format!("Total: {:.2} J", total), 16.0, offset, 16.0, WHITE);

    next_frame().await;
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