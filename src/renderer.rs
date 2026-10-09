use macroquad::{camera::{Camera2D, set_camera, set_default_camera}, color::*, math::vec2, shapes::draw_poly, text::draw_text, window::{clear_background, next_frame, screen_height, screen_width}};

use crate::simulation::State;

pub struct RenderState {
    pub sim: State,
    pub physics_time: f64,
    pub fps: f64,
    pub show_profiling: bool,
}

pub async fn render(state: &RenderState) {
    clear_background(BLACK);
    set_camera(&Camera2D {
        zoom: vec2(0.0008, screen_width() / screen_height() * 0.0008),
        ..Default::default()
    });

    for i in 0..state.sim.body_count() {
        draw_poly(state.sim.pos_x[i] as f32, state.sim.pos_y[i] as f32, 16, state.sim.size[i] as f32, 0.0, state.sim.bdy_colour[i]);
    } for i in 0..state.sim.tracer_count() {
        draw_poly(state.sim.trc_pos_x[i] as f32, state.sim.trc_pos_y[i] as f32, 8, 1.0, 0.0, state.sim.trc_colour[i]);
    }

    set_default_camera();

    let ke = state.sim.ke();
    let gpe = state.sim.gpe();
    let total = ke + gpe;
    let init = state.sim.init_energy();
    let var = total - init;
    let var_percent = ((total / init) - 1.0) * 100.0;

    let mut offset: f32 = 24.0;
    draw_text(format!("Gravity Engine (\"{}\")", crate::version::VERSION), 16.0, offset, 16.0, WHITE);
    offset += 16.0;
    draw_text("Press F3 to show profiling", 16.0, offset, 16.0, GRAY);

    offset += 16.0 * 2.0;
    let fps_colour = appropriate_colour(state.fps, 20.0, 30.0, 40.0, 50.0, 60.0);
    draw_text(format!("{:.2} fps", state.fps), 16.0, offset, 16.0, fps_colour);
    
    if state.show_profiling {
        offset += 16.0;
        let ms_colour = appropriate_colour(state.physics_time, 1000.0 / 20.0, 1000.0 / 30.0, 1000.0 / 40.0, 1000.0 / 50.0, 1000.0 / 60.0);
        draw_text(format!("{:.2}ms per step", state.physics_time), 16.0, offset, 16.0, ms_colour);
        offset += 16.0;
        draw_text(format!("{} bodies", state.sim.body_count()), 16.0, offset, 16.0, WHITE);
        offset += 16.0;
        draw_text(format!("{} tracers", state.sim.tracer_count()), 16.0, offset, 16.0, WHITE);
        offset += 16.0;
        draw_text(format!("{} interactions", state.sim.ops_per_step()), 16.0, offset, 16.0, WHITE);
        offset += 16.0 * 2.0;   
        draw_text(format!("KE: {:.2} J", ke), 16.0, offset, 16.0, WHITE);
        offset += 16.0;
        draw_text(format!("GPE: {:.2} J", gpe), 16.0, offset, 16.0, WHITE);
        offset += 16.0;
        draw_text(format!("Total: {:.2} J", total), 16.0, offset, 16.0, WHITE);
        offset += 16.0;
        let var_colour = appropriate_colour(var_percent.abs(), 4.0, 2.0, 1.0, 0.5, 0.25);
        draw_text(format!("Variation: {:.2} J ({:.2}%)", var, var_percent), 16.0, offset, 16.0, var_colour);

        let lm = state.sim.lm();

        offset += 16.0 * 2.0;
        draw_text(format!("Linear Momentum: ({:.2}, {:.2}, {:.2})", lm.x, lm.y, lm.z), 16.0, offset, 16.0, WHITE);
    }

    next_frame().await;
}

fn appropriate_colour(number: f64, worst: f64, bad: f64, fine: f64, okay: f64, good: f64) -> Color {
    if good > worst { // Higher is better
        if number <= worst {
            return RED;
        } else if number <= bad {
            return ORANGE;
        } else if number <= fine {
            return YELLOW;
        } else if number <= okay {
            return GREEN;
        } else {
            return LIME;
        }
    }
    else {
        if number <= good { // Lower is better
            return LIME;
        } else if number <= okay {
            return GREEN;
        } else if number <= fine {
            return YELLOW;
        } else if number <= bad {
            return ORANGE;
        } else {
            return RED;
        }
    }
}