use macroquad::{camera::{Camera2D, set_camera, set_default_camera}, color::*, math::vec2, shapes::draw_poly, text::draw_text, window::{clear_background, next_frame, screen_height, screen_width}};

use crate::simulation::State;

pub async fn render(state: &State, step_ms: f64, fps: f64, step: u32, max_steps: u32) {
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