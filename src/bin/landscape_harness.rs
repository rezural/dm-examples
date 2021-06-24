extern crate nalgebra as na;
extern crate dm_examples;
extern crate rapier3d;
extern crate rapier_testbed3d;
#[cfg(feature = "profile")]
extern crate coarse_prof;

use na::{Isometry3, Vector3};
use ncollide3d::query::Ray;
use rapier3d::dynamics::{JointSet, RigidBodySet};
use rapier3d::geometry::{ColliderSet};
use rapier_testbed3d::harness::{Harness, RunState};
use salva3d::integrations::rapier::{FluidsHarnessPlugin, FluidsPipeline};
use salva3d::object::{Fluid};
use salva3d::solver::{XSPHViscosity};
use std::{f32, path::PathBuf};

use dm_examples::generators::heightfield;

const PARTICLE_RADIUS: f32 = 0.04;
const SMOOTHING_FACTOR: f32 = 2.0;
const CULL_PARTICLES_BELOW: f32 = -0.6;

pub fn init_world(harness: &mut Harness) {
    let gravity = Vector3::y() * -9.81;
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let joints = JointSet::new();
    let mut fluids_pipeline = FluidsPipeline::new(PARTICLE_RADIUS, SMOOTHING_FACTOR);

    // Initialize the fluid.
    let viscosity = XSPHViscosity::new(0.2, 0.0);
    // let tension = Akinci2013SurfaceTension::new(0.2, 0.2);
    let mut fluid = Fluid::new(Vec::new(), PARTICLE_RADIUS, 1000.0);
    fluid.nonpressure_forces.push(Box::new(viscosity));
    // fluid.nonpressure_forces.push(Box::new(tension));
    let fluid_handle = fluids_pipeline.liquid_world.add_fluid(fluid);
    // plugin.set_fluid_color(fluid_handle, Point3::new(0.5, 1.0, 1.0));

    let height_multiplier = 1.;
    let ground_size = Vector3::new(20., 10., 20.);

    let heightfield_file = String::from("assets/heightmaps/basicmulti.json");
    let heightfield_data = std::fs::read_to_string(heightfield_file).unwrap();
    let heightfield: Vec<Vec<f32>> = serde_json::from_str(heightfield_data.as_str()).unwrap();
    

    let mut heightfield_flattened: Vec<f32> = heightfield.clone()
        .iter()
        .flatten()
        .cloned()
        .collect();
    
    heightfield_flattened.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let highest_point = heightfield_flattened.last().unwrap().clone();
    let lowest_point = heightfield_flattened.first().unwrap().clone();

    let heightfield_matrix = heightfield::dmatrix_from_heightfield(heightfield.clone(), lowest_point, height_multiplier);

    let (_, ground_shape) = heightfield::generate_ground(
        ground_size,
        heightfield_matrix,
        PARTICLE_RADIUS * 1.2,
        &mut fluids_pipeline,
        &mut bodies,
        &mut colliders);


    let fg_extents = Vector3::new(3., 1., 3.);
    let fluid_generator_pose = Isometry3::translation(-ground_size.x * 0.29, highest_point * ground_size.y - 5.5, -ground_size.z * 0.34);
    // Callback that will be executed on the main loop to generate new particles every second.
    let mut last_t = -0.7;
    
    let mut plugin = FluidsHarnessPlugin::new();

    plugin.add_callback(move |_, _, fluids_pipeline: &mut FluidsPipeline, run_state: &RunState, _ | {
        let fluid = fluids_pipeline
            .liquid_world
            .fluids_mut()
            .get_mut(fluid_handle)
            .unwrap();

        for i in 0..fluid.num_particles() {
            if fluid.positions[i].y < CULL_PARTICLES_BELOW {
                fluid.delete_particle_at_next_timestep(i);
            }
        }

        let t = run_state.time;
        if t - last_t < 0.03 {
            return;
        }

        println!("{}", run_state.timestep_id);
        last_t = t;
        let mut particle_radius = PARTICLE_RADIUS;
        let mut afg_extents = fg_extents.clone();
        let pose = fluid_generator_pose.clone();
        if run_state.timestep_id > 1 {
            particle_radius = PARTICLE_RADIUS * 2.;
            afg_extents.y /= 2.;
        }

        let volume = dm_examples::generators::fluid::volume_of_liquid(
            afg_extents.x * 2., 
            afg_extents.z * 2., 
            afg_extents.y * 2., 
            particle_radius, 
            pose.translation, 
            |point| {
                let ray = Ray::new(point - Vector3::y() * (PARTICLE_RADIUS * 4.), -Vector3::y());
                ground_shape.intersects_ray(&Isometry3::identity(), &ray, 10000.)
            });

        fluid.add_particles(&volume, None);

        let to_file = format!("tmp/particles-{}.ply", run_state.timestep_id);
        let to_file = PathBuf::from(to_file);
        dm_examples::io::ply::output_particles_to_file(fluid, &to_file);

    });

    /*
     * Set up the harness.
     */
    plugin.set_pipeline(fluids_pipeline);
    harness.add_plugin(plugin);
    harness.set_world_with_gravity(bodies, colliders, joints, gravity);
    harness.integration_parameters_mut().set_dt(1.0 / 300.0);
}

fn main() {
    let harness = &mut Harness::new_empty();
    harness.set_max_steps(1000);
    init_world(harness);
    harness.run();

    #[cfg(feature = "profile")]
    coarse_prof::write(&mut std::io::stdout()).unwrap();
}


// fn main() {

//     let mut testbed = Testbed::from_builders(0, vec![("Landscape", init_world)]);
//     {
//         // #[cfg(feature = "profile")]
//         testbed.harness_mut().set_max_steps(10);
//         testbed.run();

//         println!("profiling")
//         #[cfg(feature = "profile")]
//         coarse_prof::write(&mut std::io::stdout()).unwrap();

//     }
// }