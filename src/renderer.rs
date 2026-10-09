use macroquad::{camera::{Camera2D, set_camera, set_default_camera}, color::*, math::vec2, shapes::draw_poly, text::draw_text, window::{clear_background, next_frame, screen_height, screen_width}};

use crate::simulation::State;

pub struct RenderState {
    pub sim: State,
    pub physics_time: f64,
    pub fps: f64,
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

    let mut offset: f32 = 24.0;
    draw_text(format!("Gravity Engine (\"{}\")", crate::version::VERSION), 16.0, offset, 16.0, WHITE);

    offset += 16.0 * 2.0;
    draw_text(format!("{:.2} fps", state.fps), 16.0, offset, 16.0, WHITE);
    offset += 16.0;

    draw_text(format!("{:.2}ms per step", state.physics_time), 16.0, offset, 16.0, WHITE);
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

    next_frame().await;
}