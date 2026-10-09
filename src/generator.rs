use std::{f64::consts::TAU, ops::Range};

use glam::{DVec3, Vec3A};
use rand::{RngExt, SeedableRng, rngs::StdRng};

use crate::simulation::State;

pub struct OrbitalDiskConfig {
    pub particles: usize,
    pub tracers: usize,
    pub gravity: f64,
    pub star_mass: f64,
    pub dist_range: Range<f64>,
    pub mass_range: Range<f64>,
    pub seed: u64
}

pub fn simple_disc(state: &mut State, config: OrbitalDiskConfig) {
    let mut rng = StdRng::seed_from_u64(config.seed);

    let star_r = config.star_mass.sqrt();
    let min: f64 = star_r + config.dist_range.start;
    let max: f64 = star_r + config.dist_range.end;

    if config.particles > 0 {
       state.mass[0] = config.star_mass as f64;
    }
    for i in 1..config.particles {
        let angle: f64 = rng.random_range(0.0..TAU) as f64;
        let dist: f64 = rng.random_range(min * min..=max * max).sqrt();
        let (x, y) = (angle.cos() * dist, angle.sin() * dist);
        let mass = rng.random_range(config.mass_range.clone());
        state.pos_x[i] = x as f64;
        state.pos_y[i] = y as f64;
        state.mass[i] = mass as f64;
        state.vel[i] = orbital_velocity(config.gravity as f64, DVec3::new(x, y, state.pos_z[i]), DVec3::ZERO, mass, config.star_mass as f64)
    }

    for i in 0..config.tracers {
        let angle = rng.random_range(0.0..TAU);
        let dist= rng.random_range(min * min..=max * max).sqrt();
        let x = angle.cos() * dist;
        let y = angle.sin() * dist;
        state.trc_pos_x[i] = x;
        state.trc_pos_y[i] = y;
        state.trc_vel[i] = orbital_velocity(config.gravity, DVec3::new(state.trc_pos_x[i], state.trc_pos_y[i], state.trc_pos_z[i]), DVec3::ZERO, 0.0, config.star_mass)
    }
}

pub struct StarSystemConfig {
    pub particles: usize,
    pub tracers: usize,
    pub gravity: f64,
    pub star_a_mass: f64,
    pub star_b_mass: f64,
    pub mass_range: f64,
    pub seed: u64
}

pub fn star_system(state: &mut State, config: &StarSystemConfig) {
    let mut rng = StdRng::seed_from_u64(config.seed);

    state.pos_x[0] = 300.0;
    state.pos_y[0] = 300.0;
    state.pos_z[0] = 0.0;
    state.mass[0] = config.star_a_mass;

    state.pos_x[1] = -300.0;
    state.pos_y[1] = -300.0;
    state.pos_z[1] = 0.0;
    state.mass[1] = config.star_b_mass;

    state.vel[0] = orbital_velocity(config.gravity, DVec3::new(300.0, 300.0, 0.0), DVec3::new(-300.0, -300.0, 0.0), config.star_a_mass, config.star_b_mass);
    state.vel[1] = orbital_velocity(config.gravity, DVec3::new(-300.0, -300.0, 0.0), DVec3::new(300.0, 300.0, 0.0), config.star_b_mass, config.star_a_mass);
}

fn orbital_velocity(gravity: f64, child_pos: DVec3, parent_pos: DVec3, child_mass: f64, parent_mass: f64) -> DVec3 {
    let d = child_pos - parent_pos;
    let r = d.length();
    let r_vec = d / r;
    let r_tan = DVec3::new(-r_vec.y, r_vec.x, 0.0);

    let speed = (gravity * (parent_mass - child_mass) / r).sqrt();
    speed * r_tan
}