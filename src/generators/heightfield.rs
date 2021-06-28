use na::{DMatrix, Vector3};
use rapier3d::{
    dynamics::{RigidBodyBuilder, RigidBodyHandle, RigidBodySet},
    geometry::{Collider, ColliderBuilder, ColliderHandle, ColliderSet, Shape},
};
use salva3d::{
    integrations::rapier::{ColliderSampling, FluidsPipeline},
    object::Boundary,
};

pub fn dmatrix_from_heightfield(
    heightfield: Vec<Vec<f32>>,
    wall_height: f32,
    height_multiplier: f32,
) -> DMatrix<f32> {
    let hf_width = heightfield.len().clone();
    let hf_length = heightfield.get(0).unwrap().len().clone();

    DMatrix::from_fn(hf_width + 1, hf_length + 1, |i, j| {
        if i == 0 || i == hf_width || j == 0 || j == hf_length {
            wall_height
        } else {
            heightfield[i][j] * height_multiplier
        }
    })
}

pub fn generate_ground<'a>(
    ground_size: Vector3<f32>,
    heights: DMatrix<f32>,
    sample_radius: f32,
    fluids_pipeline: &mut FluidsPipeline,
    bodies: &mut RigidBodySet,
    colliders: &'a mut ColliderSet,
) -> (RigidBodyHandle, ColliderHandle) {
    // Setup the ground.
    let ground_handle = bodies.insert(RigidBodyBuilder::new_static().build());
    let ground_shape = ColliderBuilder::heightfield(heights, ground_size).build();
    let ball_samples =
        salva3d::sampling::shape_surface_ray_sample(ground_shape.shape(), sample_radius).unwrap();

    let co_handle = colliders.insert_with_parent(ground_shape, ground_handle, bodies);
    let bo_handle = fluids_pipeline
        .liquid_world
        .add_boundary(Boundary::new(Vec::new()));

    fluids_pipeline.coupling.register_coupling(
        bo_handle,
        co_handle,
        ColliderSampling::StaticSampling(ball_samples),
    );

    (ground_handle, co_handle)
}
