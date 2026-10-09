use std::f32::consts::TAU;

use glam::Vec3A;
use rand::{RngExt, SeedableRng, rngs::StdRng};

use crate::simulation::State;

pub fn orbital_disk_2d(state: &mut State, n: usize, nlw: usize, g: f32, seed: u64, star_mass: f32) {
    let mut rng = StdRng::seed_from_u64(seed);

    let min: f32 = 48.0;
    let max: f32 = 128.0;

    if n > 0 {
       state.mass[0] = star_mass;
    }
    for i in 1..n {
        let angle: f32 = rng.random_range(0.0..TAU);
        let dist: f32 = rng.random_range(min * min..=max * max).sqrt();
        let x = angle.cos() * dist;
        let y = angle.sin() * dist;
        state.pos_x[i] = x;
        state.pos_y[i] = y;
        state.mass[i] = rng.random_range(1.0..=3.0);
        state.vel[i] = orbital_velocity_2d(g, Vec3A::new(state.pos_x[i], state.pos_y[i], state.pos_z[i]), Vec3A::ZERO, star_mass)
    }

    for i in 0..nlw {
        let angle: f32 = rng.random_range(0.0..TAU);
        let dist: f32 = rng.random_range(min * min..=max * max).sqrt();
        let x = angle.cos() * dist;
        let y = angle.sin() * dist;
        state.lw_pos_x[i] = x;
        state.lw_pos_y[i] = y;
        state.lw_vel[i] = orbital_velocity_2d(g, Vec3A::new(state.lw_pos_x[i], state.lw_pos_y[i], state.lw_pos_z[i]), Vec3A::ZERO, star_mass)
    }
}

fn orbital_velocity_2d(g: f32, child_pos: Vec3A, parent_pos: Vec3A, parent_mass: f32) -> Vec3A {
    let d = child_pos - parent_pos;
    let r = d.length();
    let r_vec = d / r;
    let r_tan = Vec3A::new(-r_vec.y, r_vec.x, 0.0);

    let speed = (g * parent_mass / r).sqrt();
    speed * r_tan
}

pub fn generate_random(state: &mut State, n: usize, nlw: usize, seed: u64) {
    let mut rng = StdRng::seed_from_u64(seed);

    let min = -128.0;
    let max = 128.0;

    for i in 0..n {
        state.pos_x[i] = rng.random_range(min..=max);
        state.pos_y[i] = rng.random_range(min..=max);
        state.pos_z[i] = rng.random_range(min..=max);
        state.mass[i] = rng.random_range(1.0..=3.0);
    }
    for i in 0..nlw {
        state.lw_pos_x[i] = rng.random_range(min..=max);
        state.lw_pos_y[i] = rng.random_range(min..=max);
        state.lw_pos_z[i] = rng.random_range(min..=max);
    }
}