extern crate nalgebra as na;

use dm_examples::generators;
use dm_examples::io::ply::output_particles_to_file;
use dm_examples::run::Manager;
use na::{Isometry3, Point3, Translation3, Vector3};
use parry3d::shape::SharedShape;
use rapier_testbed3d::harness::RunState;
use rapier_testbed3d::{Testbed, TestbedApp};
use salva3d::integrations::rapier::{ColliderSampling, FluidsPipeline, FluidsTestbedPlugin};
use salva3d::object::{Boundary, Fluid};
use salva3d::rapier::prelude::JointSet;
use salva3d::solver::ArtificialViscosity;
use std::f32;
use std::path::PathBuf;

use salva3d::rapier::{
    dynamics::{RigidBodyBuilder, RigidBodySet},
    geometry::{ColliderBuilder, ColliderSet},
};

const PARTICLE_RADIUS: f32 = 0.1;
const SMOOTHING_FACTOR: f32 = 2.0;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let gravity = Vector3::y() * -9.81;
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let joints = JointSet::new();
    let mut fluids_pipeline = FluidsPipeline::new(PARTICLE_RADIUS, SMOOTHING_FACTOR);

    // Parameters of the ground.
    let ground_thickness = 0.2;
    let ground_half_width = 3.;
    let ground_half_height = 2.;

    let dims = Vector3::new(2., 50., 2.);
    // fluids.
    let nparticles = 15;
    let particles = generators::fluid::volume_of_liquid(
        dims.x,
        dims.z,
        dims.y,
        PARTICLE_RADIUS,
        Translation3::new(0., 1., 0.),
        |p| true,
    );
    let mut fluid = Fluid::new(particles, PARTICLE_RADIUS, 1000.);
    fluid.transform_by(&Isometry3::translation(
        dims.x * 0.7,
        ground_thickness + dims.y * 0.51,
        dims.z * 0.7,
    ));
    let viscosity = ArtificialViscosity::new(1.0, 0.0);
    fluid.nonpressure_forces.push(Box::new(viscosity));
    let fluid_handle = fluids_pipeline.liquid_world.add_fluid(fluid);

    /*
     * Ground.
     */
    let ground_shape = SharedShape::cuboid(ground_half_width, ground_thickness, ground_half_width);
    let wall_shape = SharedShape::cuboid(ground_thickness, ground_half_height, ground_half_width);

    let ground_body = RigidBodyBuilder::new_static().build();
    let ground_handle = bodies.insert(ground_body);

    let wall_poses = [
        Isometry3::new(
            Vector3::new(0.0, ground_half_height, ground_half_width),
            Vector3::y() * (f32::consts::PI / 2.0),
        ),
        Isometry3::new(
            Vector3::new(0.0, ground_half_height, -ground_half_width),
            Vector3::y() * (f32::consts::PI / 2.0),
        ),
        Isometry3::translation(ground_half_width, ground_half_height, 0.0),
        Isometry3::translation(-ground_half_width, ground_half_height, 0.0),
    ];

    for pose in wall_poses.iter() {
        let samples =
            salva3d::sampling::shape_surface_ray_sample(&*wall_shape, PARTICLE_RADIUS).unwrap();
        let co = ColliderBuilder::new(wall_shape.clone())
            .position(*pose)
            .build();
        let co_handle = colliders.insert_with_parent(co, ground_handle, &mut bodies);
        let bo_handle = fluids_pipeline
            .liquid_world
            .add_boundary(Boundary::new(Vec::new()));

        fluids_pipeline.coupling.register_coupling(
            bo_handle,
            co_handle,
            ColliderSampling::StaticSampling(samples),
        );
    }

    let samples =
        salva3d::sampling::shape_surface_ray_sample(&*ground_shape, PARTICLE_RADIUS).unwrap();
    let co = ColliderBuilder::new(ground_shape).build();
    let co_handle = colliders.insert_with_parent(co, ground_handle, &mut bodies);
    let bo_handle = fluids_pipeline
        .liquid_world
        .add_boundary(Boundary::new(Vec::new()));

    let run = Manager::new(None, PathBuf::from("./runs"));
    run.create_path(run.particles_full_path());

    fluids_pipeline.coupling.register_coupling(
        bo_handle,
        co_handle,
        ColliderSampling::StaticSampling(samples),
    );
    let mut plugin = FluidsTestbedPlugin::new();

    plugin.add_callback(move |harness, fluids_pipeline: &mut FluidsPipeline| {
        let fluid = fluids_pipeline
            .liquid_world
            .fluids_mut()
            .get(fluid_handle)
            .unwrap();

        println!("saving particles");
        let to_file = run
            .particles_full_path()
            .join(format!("particles-{}.ply", harness.state.timestep_id));
        let to_file = PathBuf::from(to_file);
        println!("{}", fluid.num_particles());
        output_particles_to_file(fluid, &to_file);
        // println!("done");
    });
    /*
     * Set up the testbed.
     */
    plugin.render_boundary_particles = true;
    plugin.set_pipeline(fluids_pipeline);
    plugin.set_fluid_color(fluid_handle, Point3::new(0.8, 0.7, 1.0));
    testbed.add_plugin(plugin);
    testbed.set_body_wireframe(ground_handle, true);
    testbed.set_world_with_params(bodies, colliders, joints, gravity, ());
    testbed.integration_parameters_mut().dt = 1.0 / 200.0;
    testbed.look_at(Point3::new(3.0, 3.0, 3.0), Point3::origin());
}

fn main() {
    let testbed = TestbedApp::from_builders(0, vec![("Basic", init_world)]);
    testbed.run()
}
