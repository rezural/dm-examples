#[cfg(feature = "profile")]
extern crate coarse_prof;
extern crate dm_examples;
extern crate nalgebra as na;
extern crate rapier3d;
extern crate rapier_testbed3d;

use dm_examples::external_structs::fib_sphere::StateHistory;
use dm_examples::io::ply::output_particles_to_file;
use dm_examples::run::Manager;
use na::{Isometry3, Point3, Vector3};
use rapier3d::dynamics::{JointSet, RigidBodySet};
use rapier3d::geometry::ColliderSet;
use rapier_testbed3d::harness::{Harness, RunState};
use salva3d::object::Fluid;
use salva3d::solver::XSPHViscosity;
use salva3d::{
    integrations::rapier::{FluidsHarnessPlugin, FluidsPipeline},
    object::Boundary,
};
use std::time::Instant;
use std::{f32, path::PathBuf};

use salva3d::solver::NonPressureForce;

const PARTICLE_RADIUS: f32 = 0.12;
const SMOOTHING_FACTOR: f32 = 2.0;

pub fn init_world(harness: &mut Harness) {
    let gravity = Vector3::y() * 0.;
    let bodies = RigidBodySet::new();
    let colliders = ColliderSet::new();
    let joints = JointSet::new();
    let mut fluids_pipeline = FluidsPipeline::new(PARTICLE_RADIUS, SMOOTHING_FACTOR);

    // Initialize the fluid.
    let _viscosity = XSPHViscosity::new(0.2, 0.0);
    let mut plugin = FluidsHarnessPlugin::new();
    // fluids.
    let history = std::fs::read_to_string(
        std::env::args()
            .collect::<Vec<String>>()
            .get(1)
            .unwrap()
            .as_str(),
    );
    let history: StateHistory = serde_json::from_str(history.unwrap().as_str()).unwrap();

    let custom_force = CustomForceField {
        history,
        current_state: 0,
        start: Instant::now(),
    };
    // let custom_force2 = CustomForceField {
    //     origin: Point3::new(-1.0, 0.0, 0.0),
    // };
    let extents = Vector3::new(3., 1., 3.);
    let pose = Isometry3::translation(0., 0., 0.);

    let volume = dm_examples::generators::fluid::volume_of_liquid(
        extents.x * 2.,
        extents.z * 2.,
        extents.y * 2.,
        PARTICLE_RADIUS,
        pose.translation,
        |_| true,
    );

    let mut fluid = Fluid::new(volume, PARTICLE_RADIUS, 1000.);
    fluid.nonpressure_forces.push(Box::new(custom_force));
    // fluid.nonpressure_forces.push(Box::new(custom_force2));
    // fluid.nonpressure_forces.push(Box::new(viscosity));

    let fluid_handle = fluids_pipeline.liquid_world.add_fluid(fluid);
    let run = Manager::new(None, PathBuf::from("./runs"));
    run.create_path(run.particles_full_path());

    plugin.add_callback(
        move |_, _, fluids_pipeline: &mut FluidsPipeline, run_state: &RunState| {
            let fluid = fluids_pipeline
                .liquid_world
                .fluids_mut()
                .get(fluid_handle)
                .unwrap();

            // println!("saving particles");
            let to_file = run
                .particles_full_path()
                .join(format!("particles-{}.ply", run_state.timestep_id));
            let to_file = PathBuf::from(to_file);
            output_particles_to_file(fluid, &to_file);
            // println!("done");
        },
    );

    /*
     * Set up the harness.
     */
    plugin.set_pipeline(fluids_pipeline);
    harness.add_plugin(plugin);
    harness.set_world_with_params(bodies, colliders, joints, gravity, ());
    harness.integration_parameters_mut().set_dt(1.0 / 60.0);
}

fn main() {
    let harness = &mut Harness::new_empty();
    harness.set_max_steps(5000);
    init_world(harness);
    harness.run();

    #[cfg(feature = "profile")]
    coarse_prof::write(&mut std::io::stdout()).unwrap();
}

struct CustomForceField {
    start: Instant,
    history: StateHistory,
    current_state: usize,
}

impl NonPressureForce for CustomForceField {
    fn solve(
        &mut self,
        _timestep: &salva3d::TimestepManager,
        _kernel_radius: f32,
        _fluid_fluid_contacts: &salva3d::geometry::ParticlesContacts,
        _fluid_boundaries_contacts: &salva3d::geometry::ParticlesContacts,
        fluid: &mut Fluid,
        _boundaries: &[Boundary],
        _densities: &[f32],
    ) {
        let ts = Instant::now() - self.start;
        let mut state = self.history.history.get(self.current_state).unwrap();
        let next_state = self.history.history.get(self.current_state + 1);

        if ts > state.time_offset {
            if let Some(next_state) = next_state {
                self.current_state += 1;
                state = next_state;
            }
        }

        for (pos, acc) in fluid.positions.iter().zip(fluid.accelerations.iter_mut()) {
            let points =
                fibonacci_sphere::fib_sphere::points_on_sphere_fib(state.state.npoints, 1.0);
            let points: Vec<Point3<f32>> = points
                .iter()
                .map(|&point| {
                    let a = point * state.state.radius;
                    Point3::new(a.x, a.y, a.z)
                })
                .collect();
            for point in points {
                let vec = point - pos;
                *acc = *acc + vec;
            }
        }
    }

    fn apply_permutation(&mut self, _permutation: &[usize]) {}
}
