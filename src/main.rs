use std::{fs::File, io::{BufWriter, Write}, time};
use numfmt::{Formatter, Precision, Scales};
use clap::{Parser, ValueEnum};

mod simulation;
mod pc2;

const VERSION: &str = "1.0";

#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Args {
    #[arg(short='n', default_value_t = 1024)]
    body_count: u32,
    #[arg(short='s', long="steps", default_value_t = 513)]
    duration: u32,
    #[arg(short='r', long="resolution", default_value_t = 8)]
    resolution: u32,
    #[arg(short='g', long="gravity", default_value_t = 9.14)]
    gravity: f32,
    #[arg(short='d', long="delta", default_value_t = 0.05)]
    timestep: f32,
    #[arg(short='e', long="epsilon", default_value_t = 9.0)]
    softening: f32,
    #[arg(long="seed", default_value_t = 21177)]
    seed: u64,
    #[arg(long="debug", default_value_t = false)]
    debug: bool,
    #[arg(short='o', long="output", default_value_t = "output.pc2".to_string())]
    output: String,
    #[arg(short='m', long="mode", value_enum, default_value_t=Integrator::Leapfrog)]
    integrator: Integrator
}

fn main() -> std::io::Result<()> {
    let args = Args::parse();

    let body_count = args.body_count;
    let duration = args.duration;
    let sample_rate = args.resolution;
    let gravity = args.gravity;
    let timestep = args.timestep;
    let softening = args.softening;
    let is_debug = args.debug;
    let output_file = args.output;
    let seed = args.seed;
    let integrator = args.integrator;

    let mut f = Formatter::new()
        .separator(',').unwrap()
        .precision(Precision::Decimals(0))
        .scales(Scales::none());

    println!("Gravity Engine 3D (v{VERSION})\n");

    let mut buf_writer = BufWriter::new(File::create(output_file)?);

    let write_pc2: bool;
    let total_samples = duration / sample_rate;
    let result = pc2::writer_header(&mut buf_writer, 1, body_count, 0.0, sample_rate as f32, total_samples);
    if result.is_err() {
        println!("Failed to write PC2 header: {:?}", result.err());
        write_pc2 = false;
    } else { write_pc2 = true; }

    let mut state = simulation::State::new(body_count, gravity, softening, timestep);
    state.randomize_positions(seed);

    let op_count = state.ops_per_step();
    let op_count_str = f.fmt2(op_count);

    println!("Ready to start simulation!");
    println!("Duration: {duration} steps");
    println!("Gravity: {gravity}m/s");
    println!("Softening: {softening}m");
    println!("Timestep: {timestep}s");
    println!("Population: {body_count}");
    println!("Interactions: {op_count_str}");
    println!("Integrator: {:?}", integrator); 

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
    }
    
    println!();

    let init_energy = state.ke() + state.gpe();
    let start = time::Instant::now();

    let mut last_tracked_time = start;
    for i in 0..duration {
        if i % sample_rate == 0 {
            if write_pc2 {
                for pos in &state.pos_massive {
                    pc2::write_point(&mut buf_writer, pos)?;
                }
            }
        }

        if i > 0 && is_debug {
            let ke = state.ke();
            let gpe = state.gpe();
            let total = ke + gpe;

            let ms_since_last_update = time::Instant::now().duration_since(last_tracked_time).as_millis() as f32 / sample_rate as f32;
            let eta_seconds = (duration - i) as f32 * ms_since_last_update / 1000.0;

            let display_step = i + 1;
            println!("[Ge3D] {display_step} / {duration}. Energy: {total} ({ke} KE, {gpe} GPE). ETA: ~{eta_seconds:.2}s");

            last_tracked_time = time::Instant::now();
        }

        match integrator {
            Integrator::SymplecticEuler => {
                simulation::step_symplectic_euler(&mut state);
            }
            Integrator::Leapfrog => {
                simulation::step_leapfrog(&mut state);
            }
        }
    }

    println!();

    let stop = time::Instant::now();

    let time_taken_ms = stop.duration_since(start).as_millis();
    let time_taken_seconds = time_taken_ms as f32 / 1000.0;
    let avg_ms_per_step = time_taken_ms as f32 / duration as f32;
    println!("Took {time_taken_seconds:.0}s to complete {duration} steps. (About {avg_ms_per_step:.2}ms per step.)");

    let final_energy = state.ke() + state.gpe();
    let energy_delta = final_energy - init_energy;
    println!("Energy varied by {energy_delta:.2} from {init_energy:.2} to {final_energy:.2}.");

    buf_writer.flush()?;
    Ok(())
}

#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord, ValueEnum, Debug)]
enum Integrator {
    SymplecticEuler,
    Leapfrog
}