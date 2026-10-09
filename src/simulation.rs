use glam::{ DVec3, Vec3A};
use macroquad::color::{Color, WHITE};
use num_traits::Float;
use rayon::iter::{IndexedParallelIterator, IntoParallelRefMutIterator, ParallelIterator};

#[derive(Clone)]
pub struct State {
    // Forces
    pub g: f64,
    pub eps: f64,
    pub delta: f64,

    // Particles
    pub pos_x: Vec<f64>,
    pub pos_y: Vec<f64>,
    pub pos_z: Vec<f64>,
    pub vel: Vec<DVec3>,
    pub acc: Vec<DVec3>,
    pub mass: Vec<f64>,
    mass_g: Vec<f64>,

    // Tracers
    pub trc_pos_x: Vec<f64>,
    pub trc_pos_y: Vec<f64>,
    pub trc_pos_z: Vec<f64>,
    pub trc_vel: Vec<DVec3>,
    pub trc_acc: Vec<DVec3>,

    // Rendering
    pub size: Vec<f64>,
    pub bdy_colour: Vec<Color>,
    pub trc_colour: Vec<Color>,

    gpe: f64,
}

impl State {
    pub fn new_default(n_bdy: usize, n_trc: usize, g: f64, eps: f64, delta: f64) -> Self {
        State {
            g,
            eps,
            delta,
            pos_x: vec![0.0; n_bdy],
            pos_y: vec![0.0; n_bdy],
            pos_z: vec![0.0; n_bdy],
            vel: vec![DVec3::ZERO; n_bdy],
            acc: vec![DVec3::ZERO; n_bdy],
            mass: vec![1.0; n_bdy],
            mass_g: vec![1.0; n_bdy],
            size: vec![0.0; n_bdy],
            trc_pos_x: vec![0.0; n_trc],
            trc_pos_y: vec![0.0; n_trc],
            trc_pos_z: vec![0.0; n_trc],
            trc_vel: vec![DVec3::ZERO; n_trc],
            trc_acc: vec![DVec3::ZERO; n_trc],
            bdy_colour: vec![WHITE; n_bdy],
            trc_colour: vec![WHITE; n_trc],
            gpe: 0.0,
        }
    }

    /// Returns the simulation's gravitational potential energy. (Recomputed every step.)
    pub fn gpe(&self) -> f64 {
        self.gpe
    }

    /// Sums up and then returns the total kinetic energy in the simulation.
    pub fn ke(&self) -> f64 {
        let mut ke = 0.0;

        for i in 0..self.body_count() {
            let mass = self.mass[i];
            let velocity = self.vel[i];
            ke += 0.5 * mass * velocity.length_squared() as f64;
        }

        ke
    }

    /// Returns the number of massive (that have mass) particles.
    pub fn body_count(&self) -> usize {
        self.pos_x.len()
    }

    /// Returns the number of lightweight (massless) particles.
    pub fn tracer_count(&self) -> usize {
        self.trc_pos_x.len()
    }

    /// Returns the body count of the simulation.
    pub fn particle_count(&self) -> usize {
        self.tracer_count() + self.body_count()
    }

    /// Computes the number of massive-massive and lightweight-massive interactions.
    pub fn ops_per_step(&self) -> u128 {
        let msv_ops = self.body_count() as u128 * self.body_count() as u128;
        msv_ops + self.body_count() as u128 * self.tracer_count() as u128
    }

    /// Multiplies the masses of massive particles by gravity, so it doesn't need to be computed during the accelerate pass.
    fn scale_masses(&mut self) {
        for i in 0..self.body_count() {
            self.mass_g[i] = self.mass[i] * self.g;
        }
    }
}

pub fn init(state: &mut State) {
    state.scale_masses();
    for i in 0..state.body_count() {
        state.size[i] = state.mass[i].sqrt();
    }
    accelerate_mt(state);
}

pub fn step_leapfrog(state: &mut State) {
    half_kick(state);
    drift(state);
    accelerate_mt(state);
    half_kick(state);
}

fn half_kick(state: &mut State) {
    for i in 0..state.body_count() {
        state.vel[i] += state.acc[i] * state.delta * 0.5;
    }
    for i in 0..state.tracer_count() {
        state.trc_vel[i] += state.trc_acc[i] * state.delta * 0.5;
    }
}

fn drift(state: &mut State) {
    for i in 0..state.body_count() {
        let vel = state.vel[i];
        let delta = state.delta;
        state.pos_x[i] += vel.x * delta;
        state.pos_y[i] += vel.y * delta;
        state.pos_z[i] += vel.z * delta;
    }
    for i in 0..state.tracer_count() {
        let vel = state.trc_vel[i];
        let delta = state.delta;
        state.trc_pos_x[i] += vel.x * delta;
        state.trc_pos_y[i] += vel.y * delta;
        state.trc_pos_z[i] += vel.z * delta;
    }
}

/// Calculate the acceleration between particles using multithreading.
fn accelerate_mt(state: &mut State) {
    let eps2 = state.eps * state.eps;
    let base_mass = &state.mass;
    let mass_g = &state.mass_g;

    let bx = state.pos_x.as_slice();
    let by = state.pos_y.as_slice();
    let bz = state.pos_z.as_slice();

    let gpe_twice: f64 = state.acc.par_iter_mut()
        .enumerate()
        .map(|(i, a)| {
            let (ix, iy, iz) = (bx[i], by[i], bz[i]);
            let (acc, gpe) = attract(ix, iy, iz, bx, by, bz, mass_g, eps2);
            *a = DVec3::new(acc[0], acc[1], acc[2]);
            gpe * -base_mass[i] as f64
        })
        .sum();

    let trc_x = state.trc_pos_x.as_slice();
    let trc_y = state.trc_pos_y.as_slice();
    let trc_z = state.trc_pos_z.as_slice();

    state.trc_acc.par_iter_mut()
        .enumerate()
        .for_each(|(i, a)| {
            let (ix, iy, iz) = (trc_x[i], trc_y[i], trc_z[i]);
            let (acc, _gpe) = attract::<f64>(ix, iy, iz, bx, by, bz, mass_g, eps2);
            *a = DVec3::new(acc[0], acc[1], acc[2]);
        });

    state.gpe = gpe_twice * 0.5;
}

/// Computes the cumulative acceleration felt by a body `i` from all other bodies `p` and writes it to `a`.  
/// Returns the raw potential energy from the interactions.  
/// To get the actual GPE, multiply by the base mass of `i` (not multiplied by gravity).
#[inline]
fn attract<T: Float>(ix: T, iy: T, iz: T, px: &[T], py: &[T], pz: &[T], m_g: &[T], eps2: T) -> ([T; 3], f64) {
    let (mut ax , mut ay, mut az) = (T::zero(), T::zero(), T::zero());

    let mut bgpe = 0.0;
    
    for j in 0..px.len() {
        let jx = px[j]; let jy = py[j]; let jz = pz[j];
        let jm = m_g[j];
        
        let (dx, dy, dz) = (jx - ix, jy - iy, jz - iz);
        let dist2 = dx * dx + dy * dy + dz * dz + eps2;
        let inv_dist = T::one() / dist2.sqrt();
        let s = jm * (inv_dist * inv_dist * inv_dist);

        ax = dx.mul_add(s, ax);
        ay = dy.mul_add(s, ay);
        az = dz.mul_add(s, az);

        bgpe = jm.to_f64().unwrap().mul_add(inv_dist.to_f64().unwrap(), bgpe);
    }

    ([ax, ay, az], bgpe)
}