use na::{Point3, Vector3, Translation3};

pub fn volume_of_liquid<F>(
    width: f32,
    length: f32,
    height: f32,
    particle_rad: f32,
    translation: Translation3<f32>,
    include_particle_fn: F
)
    -> Vec<Point3<f32>> where
    F: Fn(&Point3<f32>) -> bool
{
    let particle_diam = particle_rad * 2.0;
    let size_x = (width / particle_diam).ceil() as i32;
    let size_y = (height / particle_diam).ceil() as i32;
    let size_z = (length / particle_diam).ceil() as i32;

    let half_extents = Vector3::new(size_x as f32, size_y as f32, size_z as f32) * particle_rad;

    let mut points = Vec::new();

    for i in 0..size_x {
        for j in 0..size_y {
            for k in 0..size_z {
                let x = (i as f32) * particle_diam;
                let y = (j as f32) * particle_diam;
                let z = (k as f32) * particle_diam;
                let point = Point3::new(x, y, z)
                    + Vector3::repeat(particle_rad)
                    - half_extents
                    + translation.vector.clone();
                if include_particle_fn(&point) {
                    points.push(point);
                }
            }
        }
    }
    points
}