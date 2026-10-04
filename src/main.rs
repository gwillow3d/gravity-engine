use std::{fs::File, io::{BufWriter, Write}, time};

use numfmt::{Formatter, Precision, Scales};

mod simulation;

const VERSION: &str = "1.0";
const PC2_HEADER: &str = "POINTCACHE2\0";

fn main() -> std::io::Result<()> {
    println!("GRAVITY ENGINE 3D by Winkassador! (v{VERSION})\n");

    let mut f = Formatter::new()
        .separator(',').unwrap()
        .precision(Precision::Decimals(0))
        .scales(Scales::none());

    let mut buf_writer = BufWriter::new(File::create("output.pc2")?);

    let population= 512;

    let mut state = simulation::State::new(population);
    state.randomize_positions();

    let total_steps: u32 = 241;
    let sample_rate: u32 = 4;

    let total_samples = total_steps / sample_rate;

    // .pc2 header
    for c in PC2_HEADER.chars() {
        buf_writer.write_all(&(c as u8).to_le_bytes())?;
    }
    buf_writer.write_all(&1u32.to_le_bytes())?; // File version
    buf_writer.write_all(&(state.positions.len() as u32).to_le_bytes())?; // Population
    buf_writer.write_all(&0.0f32.to_le_bytes())?; // Start Frame
    buf_writer.write_all(&(sample_rate as f32).to_le_bytes())?; // Sample Rate
    buf_writer.write_all(&total_samples.to_le_bytes())?; // Num samples

    let op_count = state.get_ops_per_step();
    let op_count_str = f.fmt2(op_count);

    println!("Ready to start simulation!");
    println!("Steps: {total_steps}");
    println!("Population: {population}");
    println!("Interactions: {op_count_str}\n");

    let start = time::Instant::now();
    for i in 0..total_steps {
        if i % sample_rate == 0 {
            for j in 0..state.positions.len() {
                let pos = state.positions[j];
                buf_writer.write_all(&pos.x.to_le_bytes())?;
                buf_writer.write_all(&pos.y.to_le_bytes())?;
                buf_writer.write_all(&pos.z.to_le_bytes())?;
            }

            let kinetic = state.get_kinetic_energy();
            let potential = state.get_potential_energy();
            let total = kinetic + potential;

            let step = i + 1;
            println!("[Ge3D] Step: {step} / {total_steps}. Energy: {total} ({kinetic} KE, {potential} GPE)")
        }

        simulation::step_leapfrog(&mut state);
    }
    let stop = time::Instant::now();

    let time_taken = stop.duration_since(start).as_millis();
    let avg_step_time = time_taken as f32 / total_steps as f32;
    println!("Took {time_taken:.0}ms to complete {total_steps} steps. (About {avg_step_time:.2}ms per step.)");

    buf_writer.flush()?;
    Ok(())
}