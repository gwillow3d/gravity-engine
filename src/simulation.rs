use glam::Vec3;
use rand::{RngExt};

pub struct State {
    pub gravity: f32,
    pub softening: f32,
    pub timestep: f32,
    pub positions: Vec<Vec3>,
    pub velocities: Vec<Vec3>,
    pub accelerations: Vec<Vec3>,
    pub masses: Vec<f32>,
    potential_energy: f32
}

impl State {
    pub fn new(population: u32) -> Self {
        State {
            gravity: 9.14,
            softening: 9.0,
            timestep: 0.05,
            positions: vec![Vec3::ZERO; population as usize],
            velocities: vec![Vec3::ZERO; population as usize],
            accelerations: vec![Vec3::ZERO; population as usize],
            masses: vec![1.0; population as usize],
            potential_energy: 0.0
        }
    }

    pub fn randomize_positions(&mut self) {
        let mut rng = rand::rng();
        for i in 0..self.positions.len() {
            let x = rng.random_range(-250.0..=250.0);
            let y = rng.random_range(-250.0..=250.0);
            let z = rng.random_range(-250.0..=250.0);
            self.positions[i] = Vec3 {x, y, z};
        }
    }

    pub fn get_potential_energy(&self) -> f32 {
        self.potential_energy
    }

    pub fn get_kinetic_energy(&self) -> f32 {
        let mut ke = 0.0;

        for i in 0..self.positions.len() {
            let mass = self.masses[i];
            let velocity = self.velocities[i];
            ke += 0.5 * mass * velocity.length_squared();
        }

        ke
    }

    pub fn get_ops_per_step(&self) -> u32 {
        (self.positions.len() * self.positions.len() - 1) as u32
    }
}

pub fn step_leapfrog(state: &mut State) {
    state.potential_energy = 0.0;

    half_kick(state);
    drift(state);
    accelerate(state);
    half_kick(state);
}

pub fn half_kick(state: &mut State) {
    for i in 0..state.positions.len() {
        state.velocities[i] += state.accelerations[i] * state.timestep * 0.5;
        state.accelerations[i] = Vec3::ZERO;
    }
}

pub fn drift(state: &mut State) {
    for i in 0..state.positions.len() {
        state.positions[i] += state.velocities[i] * state.timestep;
    }
}

pub fn accelerate(state: &mut State) {
    for i in 0..state.positions.len() {
        let i_pos = state.positions[i];
        let i_mass = state.masses[i];

        for j in i+1..state.positions.len() {
            let j_pos = state.positions[j];
            let j_mass = state.masses[j];

            let dist_squared = i_pos.distance_squared(j_pos);
            
            let d = j_pos - i_pos;
            let r: f32 = (dist_squared + state.softening * state.softening).sqrt();
            let acc = state.gravity * d / (r * r * r);

            state.accelerations[i] += acc * j_mass;
            state.accelerations[j] -= acc * i_mass;
            state.potential_energy -= state.gravity * i_mass * j_mass / r;
        }
    }
}