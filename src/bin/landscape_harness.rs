#[cfg(feature = "profile")]
extern crate coarse_prof;
extern crate dm_examples;
extern crate nalgebra as na;
extern crate rapier3d;
// extern crate rapier_testbed3d;

use dm_examples::generators::fluid::volume_of_liquid;
use dm_examples::io::ply::{output_fluid_to_file, output_particles_to_file};
use dm_examples::run::Manager;
use na::{Isometry3, Vector3};
use parry3d::query::Ray;
use rapier3d::harness::{Harness, RunState};
use rapier3d::prelude::RigidBodyHandle;
// use rapier_testbed3d::{Testbed, TestbedApp};
use salva3d::integrations::rapier::{FluidsHarnessPlugin, FluidsPipeline};
use salva3d::object::Fluid;
use salva3d::solver::XSPHViscosity;
use std::{f32, path::PathBuf};

use dm_examples::generators::heightfield;
use salva3d::rapier::prelude::JointSet;
use salva3d::rapier::{dynamics::RigidBodySet, geometry::ColliderSet};

const PARTICLE_RADIUS: f32 = 0.4;
const SMOOTHING_FACTOR: f32 = 2.0;
const CULL_PARTICLES_BELOW: f32 = -10.;

pub fn init_world(harness: &mut Harness) -> RigidBodyHandle {
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

    let height_multiplier = 1.;

    let heightfield_file = String::from("assets/heightmaps/basicmulti.json");
    let heightfield_data = std::fs::read_to_string(heightfield_file).unwrap();
    let heightfield: Vec<Vec<f32>> = serde_json::from_str(heightfield_data.as_str()).unwrap();

    let mut heightfield_flattened: Vec<f32> =
        heightfield.clone().iter().flatten().cloned().collect();

    heightfield_flattened.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let highest_point = heightfield_flattened.last().unwrap().clone();
    let lowest_point = heightfield_flattened.first().unwrap().clone();

    let ground_size = Vector3::new(100., 40., 100.);

    let fg_extents = Vector3::new(1., 1., 1.);
    let fluid_generator_pose = Isometry3::translation(
        -ground_size.x * 0.29,
        highest_point * ground_size.y - 5.5,
        -ground_size.z * 0.34,
    );
    let pose = fluid_generator_pose.clone();

    let afg_extents = fg_extents.clone();
    // let volume = volume_of_liquid(
    //     afg_extents.x * 4.,
    //     afg_extents.z * 4.,
    //     afg_extents.y * 4.,
    //     PARTICLE_RADIUS,
    //     pose.translation,
    //     |point| {
    //         let ray = Ray::new(point - Vector3::y() * (PARTICLE_RADIUS * 4.), -Vector3::y());
    //         // ground_shape.intersects_ray(&Isometry3::identity(), &ray, 10000.)
    //         true
    //     },
    // );
    // fluid.add_particles(&volume, None);

    let fluid_handle = fluids_pipeline.liquid_world.add_fluid(fluid);
    // println!("{:?}", fluid_handle);
    // println!("{:?}", fluids_pipeline.liquid_world.fluids().len());

    // plugin.set_fluid_color(fluid_handle, Point3::new(0.5, 1.0, 1.0));

    let heightfield_matrix =
        heightfield::dmatrix_from_heightfield(heightfield.clone(), lowest_point, height_multiplier);

    let (ground_handle, samples) = heightfield::generate_ground(
        ground_size,
        heightfield_matrix,
        PARTICLE_RADIUS * 1.2,
        &mut fluids_pipeline,
        &mut bodies,
        &mut colliders,
    );

    // let ground_collider = colliders.get(collider_handle).clone();
    // let ground_shape = ground_collider.unwrap().shape().clone();

    // Callback that will be executed on the main loop to generate new particles every second.
    let mut last_t = -0.7;

    let mut plugin = FluidsHarnessPlugin::new();

    let run = Manager::new(None, PathBuf::from("./runs"));
    run.create_path(run.particles_full_path());

    let to_file = run.run_path().join("ground-samples.ply");
    let to_file = PathBuf::from(to_file);
    output_particles_to_file(&samples, None, &to_file);

    plugin.add_callback(
        move |_, _, fluids_pipeline: &mut FluidsPipeline, run_state: &RunState| {
            if let Some(fluid) = fluids_pipeline
                .liquid_world
                .fluids_mut()
                .get_mut(fluid_handle)
            {
                for i in 0..fluid.num_particles() {
                    if fluid.positions[i].y < CULL_PARTICLES_BELOW {
                        fluid.delete_particle_at_next_timestep(i);
                    }
                }

                let t = run_state.time;
                if t - last_t > 0.5 {
                    last_t = t;
                    let mut particle_radius = PARTICLE_RADIUS;
                    let mut afg_extents = fg_extents.clone();
                    let pose = fluid_generator_pose.clone();
                    if run_state.timestep_id > 1 {
                        particle_radius = PARTICLE_RADIUS * 2.;
                        afg_extents.y /= 2.;
                    }

                    let volume = volume_of_liquid(
                        afg_extents.x * 8.,
                        afg_extents.z * 2.,
                        particle_radius * 8.,
                        particle_radius,
                        pose.translation,
                        |point| {
                            let ray = Ray::new(
                                point - Vector3::y() * (PARTICLE_RADIUS * 4.),
                                -Vector3::y(),
                            );
                            // ground_shape.intersects_ray(&Isometry3::identity(), &ray, 10000.)
                            true
                        },
                    );
                    let velocity = Vector3::new(0., -PARTICLE_RADIUS * 4., 0.);
                    let velocities: Vec<Vector3<f32>> =
                        volume.iter().map(|_| velocity.clone()).collect();
                    fluid.add_particles(&volume, Some(&velocities));
                }

                // println!("{}", run_state.timestep_id);

                let to_file = run
                    .particles_full_path()
                    .join(format!("particles-{}.ply", run_state.timestep_id));
                let to_file = PathBuf::from(to_file);
                output_fluid_to_file(fluid, &to_file);
            }
        },
    );

    plugin.set_pipeline(fluids_pipeline);
    harness.add_plugin(plugin);
    harness.set_world_with_params(bodies, colliders, joints, gravity, ());
    harness.integration_parameters_mut().dt = 1.0 / 300.0;
    harness.set_max_steps(1000000);

    ground_handle
}

// pub fn init_testbed(_: &mut Testbed) {}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    // println!("{:?}", args);
    let mut running_testbed = false;
    // if let Some(arg) = args.last() {
    //     if arg == "--testbed" {
    //         running_testbed = true;
    //         let mut testbed = TestbedApp::from_builders(0, vec![("Landscape", init_testbed)]);
    //         let ground_handle = init_world(testbed.harness_mut());
    //         testbed.set_body_wireframe(ground_handle, false);
    //         testbed.run();
    //     }
    // }
    if !running_testbed {
        let mut harness = Harness::new_empty();
        init_world(&mut harness);
        harness.run();
    }

    #[cfg(feature = "profile")]
    coarse_prof::write(&mut std::io::stdout()).unwrap();
}
