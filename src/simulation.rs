use glam::{ Vec3A};
use rayon::iter::{IndexedParallelIterator, IntoParallelRefMutIterator, ParallelIterator};

pub struct State {
    // Forces
    pub g: f32,
    pub eps: f32,
    pub delta: f32,

    // Massive
    pub pos_x: Vec<f32>,
    pub pos_y: Vec<f32>,
    pub pos_z: Vec<f32>,
    pub vel: Vec<Vec3A>,
    pub acc: Vec<Vec3A>,
    pub mass: Vec<f32>,
    mass_g: Vec<f32>,

    // Lightweight (massless)
    pub lw_pos_x: Vec<f32>,
    pub lw_pos_y: Vec<f32>,
    pub lw_pos_z: Vec<f32>,
    pub lw_vel: Vec<Vec3A>,
    pub lw_acc: Vec<Vec3A>,

    gpe: f64,
}

impl State {
    pub fn new_default(n_msv: usize, n_lw: usize, g: f32, eps: f32, delta: f32) -> Self {
        State {
            g,
            eps,
            delta,
            pos_x: vec![0.0f32; n_msv],
            pos_y: vec![0.0f32; n_msv],
            pos_z: vec![0.0f32; n_msv],
            vel: vec![Vec3A::ZERO; n_msv],
            acc: vec![Vec3A::ZERO; n_msv],
            mass: vec![1.0; n_msv],
            mass_g: vec![1.0; n_msv],
            lw_pos_x: vec![0.0f32; n_lw],
            lw_pos_y: vec![0.0f32; n_lw],
            lw_pos_z: vec![0.0f32; n_lw],
            lw_vel: vec![Vec3A::ZERO; n_lw],
            lw_acc: vec![Vec3A::ZERO; n_lw],
            gpe: 0.0,
        }
    }

    /// Returns the simulation's gravitational potential energy. (Recomputed every step.)
    pub fn gpe(&self) -> f64 {
        self.gpe
    }

    /// Sums up and then returns the total kinetic energy in the simulation.
    pub fn ke(&self) -> f64 {
        let mut ke = 0.0f64;

        for i in 0..self.msv_body_count() {
            let mass = self.mass[i];
            let velocity = self.vel[i];
            ke += 0.5 * mass as f64 * velocity.length_squared() as f64;
        }

        ke
    }

    /// Returns the number of massive (that have mass) particles.
    pub fn msv_body_count(&self) -> usize {
        self.pos_x.len()
    }

    /// Returns the number of lightweight (massless) particles.
    pub fn lw_body_count(&self) -> usize {
        self.lw_pos_x.len()
    }

    /// Returns the body count of the simulation.
    pub fn body_count(&self) -> usize {
        self.lw_body_count() + self.msv_body_count()
    }

    /// Computes the number of massive-massive and lightweight-massive interactions.
    pub fn ops_per_step(&self) -> u128 {
        let msv_ops = self.msv_body_count() as u128 * self.msv_body_count() as u128;
        msv_ops + self.msv_body_count() as u128 * self.lw_body_count() as u128
    }

    /// Multiplies the masses of massive particles by gravity, so it doesn't need to be computed during the accelerate pass.
    fn scale_masses(&mut self) {
        for i in 0..self.msv_body_count() {
            self.mass_g[i] = self.mass[i] * self.g;
        }
    }
}

pub fn init(state: &mut State) {
    state.scale_masses();
    accelerate_mt(state);
}

pub fn step_leapfrog(state: &mut State) {
    half_kick(state);
    drift(state);
    accelerate_mt(state);
    half_kick(state);
}

fn half_kick(state: &mut State) {
    for i in 0..state.msv_body_count() {
        state.vel[i] += state.acc[i] * state.delta * 0.5;
    }
    for i in 0..state.lw_body_count() {
        state.lw_vel[i] += state.lw_acc[i] * state.delta * 0.5;
    }
}

fn drift(state: &mut State) {
    for i in 0..state.msv_body_count() {
        let vel = state.vel[i];
        let delta = state.delta;
        state.pos_x[i] += vel.x * delta;
        state.pos_y[i] += vel.y * delta;
        state.pos_z[i] += vel.z * delta;
    }
    for i in 0..state.lw_body_count() {
        let vel = state.lw_vel[i];
        let delta = state.delta;
        state.lw_pos_x[i] += vel.x * delta;
        state.lw_pos_y[i] += vel.y * delta;
        state.lw_pos_z[i] += vel.z * delta;
    }
}

/// Calculate the acceleration between particles using multithreading.
fn accelerate_mt(state: &mut State) {
    let eps2 = state.eps * state.eps;
    let base_mass = &state.mass;
    let mass_g = &state.mass_g;

    let px = state.pos_x.as_slice();
    let py = state.pos_y.as_slice();
    let pz = state.pos_z.as_slice();

    let gpe_twice: f64 = state.acc.par_iter_mut()
        .enumerate()
        .map(|(i, a)| {
            let (ix, iy, iz) = (px[i], py[i], pz[i]);
            attract(a, ix, iy, iz, px, py, pz, mass_g, eps2) * -base_mass[i] as f64
        })
        .sum();

    let lpx = state.lw_pos_x.as_slice();
    let lpy = state.lw_pos_y.as_slice();
    let lpz = state.lw_pos_z.as_slice();

    state.lw_acc.par_iter_mut()
        .enumerate()
        .for_each(|(i, a)| {
            let (ix, iy, iz) = (lpx[i], lpy[i], lpz[i]);
            attract(a, ix, iy, iz, px, py, pz, mass_g, eps2);
        });

    state.gpe = gpe_twice * 0.5;
}

/// Computes the cumulative acceleration felt by a body `i` from all other bodies `p` and writes it to `a`.  
/// Returns the raw potential energy from the interactions.  
/// To get the actual GPE, multiply by the base mass of `i` (not multiplied by gravity).
#[inline]
fn attract(a: &mut Vec3A, ix: f32, iy: f32, iz: f32, px: &[f32], py: &[f32], pz: &[f32], m_g: &[f32], eps2: f32) -> f64 {
    let (mut ax, mut ay, mut az) = (0.0f32, 0.0f32, 0.0f32);

    let mut bgpe = 0.0f64;
    
    for j in 0..px.len() {
        let jx = px[j]; let jy = py[j]; let jz = pz[j];
        let jm = m_g[j];
        
        let (dx, dy, dz) = (jx - ix, jy - iy, jz - iz);
        let dist2 = dx * dx + dy * dy + dz * dz + eps2;
        let inv_dist = 1.0 / dist2.sqrt();
        let s = jm * (inv_dist * inv_dist * inv_dist);

        ax = dx.mul_add(s, ax);
        ay = dy.mul_add(s, ay);
        az = dz.mul_add(s, az);

        bgpe = (jm as f64).mul_add(inv_dist as f64, bgpe);
    }
    
    *a = Vec3A::new(ax, ay, az);

    bgpe
}