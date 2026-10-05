use std::ops::Range;

use glam::{ Vec3A};
use rand::{RngExt, SeedableRng, rngs::StdRng};

pub struct State {
    pub gravity: f32,
    pub softening: f32,
    pub timestep: f32,
    pub pos_massive: Vec<Vec3A>,
    pub vel_massive: Vec<Vec3A>,
    pub acc_massive: Vec<Vec3A>,
    pub masses: Vec<f32>,
    potential_energy: f32,
    population: u32
}

impl State {
    pub fn new(population: u32, gravity: f32, softening: f32, timestep: f32) -> Self {
        State {
            gravity,
            softening,
            timestep,
            pos_massive: vec![Vec3A::ZERO; population as usize],
            vel_massive: vec![Vec3A::ZERO; population as usize],
            acc_massive: vec![Vec3A::ZERO; population as usize],
            masses: vec![1.0; population as usize],
            potential_energy: 0.0,
            population
        }
    }

    pub fn randomize_positions(&mut self, seed: u64) {
        let mut rng = StdRng::seed_from_u64(seed);
        for i in 0..self.body_count() {
            let x = rng.random_range(-250.0..=250.0);
            let y = rng.random_range(-250.0..=250.0);
            let z = 0.0;
            self.pos_massive[i] = Vec3A::new(x, y, z);
        }
    }

    /// Returns the simulation's gravitational potential energy. (Recomputed every step.)
    pub fn gpe(&self) -> f32 {
        self.potential_energy
    }

    /// Sums up and then returns the total kinetic energy in the simulation.
    pub fn ke(&self) -> f32 {
        let mut ke = 0.0;

        for i in 0..self.body_count() {
            let mass = self.masses[i];
            let velocity = self.vel_massive[i];
            ke += 0.5 * mass * velocity.length_squared();
        }

        ke
    }

    /// Returns the body count of the simulation.
    pub fn body_count(&self) -> usize {
        self.population as usize
    }

    /// Computes the number of pairwise interactions.
    pub fn ops_per_step(&self) -> u128 {
        (self.pos_massive.len() as u128 * (self.pos_massive.len() as u128 - 1) / 2) as u128
    }
}

pub fn init(state: &mut State) {
    accelerate(state, 0 ..state.body_count());
}

pub fn step_leapfrog(state: &mut State) {
    kick(state, 0.5, 0..state.body_count());
    drift(state, 0..state.body_count());
    accelerate(state, 0..state.body_count());
    kick(state, 0.5, 0..state.body_count());
}

pub fn step_symplectic_euler(state: &mut State) {
    accelerate(state, 0..state.body_count());
    kick(state, 1.0, 0..state.body_count());
    drift(state, 0..state.body_count());

    for i in 0..state.body_count() {
        state.acc_massive[i] = Vec3A::ZERO;
    }
}

pub fn kick(state: &mut State, mul: f32, range: Range<usize>) {
    for i in range {
        state.vel_massive[i] += state.acc_massive[i] * state.timestep * mul;
    }
}

pub fn drift(state: &mut State, range: Range<usize>) {
    for i in range {
        state.pos_massive[i] += state.vel_massive[i] * state.timestep;
    }
}

pub fn accelerate(state: &mut State, range: Range<usize>) {
    for i in 0..state.body_count() {
        state.acc_massive[i] = Vec3A::ZERO;
    }

    state.potential_energy = 0.0;

    let soft_sqr = state.softening * state.softening;

    for i in range {
        let posi = state.pos_massive[i];
        let mi = state.masses[i];

        for j in i + 1..state.body_count() {
            let mj = state.masses[j];

            let d = state.pos_massive[j] - posi;
            let dist_squared = d.length_squared();

            let r: f32 = (dist_squared + soft_sqr).sqrt();
            let acc = state.gravity * d / (r * r * r);

            state.acc_massive[i] += acc * mj;
            state.acc_massive[j] -= acc * mi;
            state.potential_energy -= state.gravity * mi * mj / r;
        }
    }
}