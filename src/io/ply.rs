extern crate ply_rs;

use na::{Point3, Vector3};
use ply_rs::ply::{
    Addable, DefaultElement, ElementDef, Encoding, Ply, Property, PropertyDef, PropertyType,
    ScalarType,
};
use ply_rs::writer::Writer;
use salva3d::object::Fluid;
use std::io::Write;
use std::path;
use std::{fs::File, io::BufWriter};

pub fn output_particles_to_file(
    particles: &Vec<Point3<f32>>,
    velocities: Option<&Vec<Vector3<f32>>>,
    to_file: &path::PathBuf,
) {
    let file: File = File::create(to_file).unwrap();
    let mut buf = Vec::<u8>::new();

    let mut ply = create_ply();

    let particles = particles
        .iter()
        .enumerate()
        .map(|(i, particle)| {
            let mut point = DefaultElement::new();

            let velocity: Vector3<f32> = if let Some(velocities) = velocities {
                velocities[i]
            } else {
                Vector3::y()
            };

            point.insert("x".to_string(), Property::Float(particle.x));
            point.insert("y".to_string(), Property::Float(particle.y));
            point.insert("z".to_string(), Property::Float(particle.z));

            point.insert("nx".to_string(), Property::Float(velocity.x));
            point.insert("ny".to_string(), Property::Float(velocity.y));
            point.insert("nz".to_string(), Property::Float(velocity.z));
            point
        })
        .collect();

    ply.payload.insert("vertex".to_string(), particles);

    let w = Writer::new();
    let _written = w.write_ply(&mut buf, &mut ply).unwrap();

    let mut buf_writer = BufWriter::new(file);
    match buf_writer.write_all(&buf) {
        Ok(r) => r,
        _ => (),
    }
}

pub fn output_fluid_to_file(fluid: &Fluid, to_file: &path::PathBuf) {
    output_particles_to_file(&fluid.positions, Some(&fluid.velocities), to_file)
}

fn create_ply() -> Ply<DefaultElement> {
    let mut ply = Ply::<DefaultElement>::new();
    ply.header.encoding = Encoding::BinaryBigEndian;
    let mut point_element = ElementDef::new("vertex".to_string());
    let p = PropertyDef::new("x".to_string(), PropertyType::Scalar(ScalarType::Float));
    point_element.properties.add(p);
    let p = PropertyDef::new("y".to_string(), PropertyType::Scalar(ScalarType::Float));
    point_element.properties.add(p);
    let p = PropertyDef::new("z".to_string(), PropertyType::Scalar(ScalarType::Float));
    point_element.properties.add(p);
    let p = PropertyDef::new("nx".to_string(), PropertyType::Scalar(ScalarType::Float));
    point_element.properties.add(p);
    let p = PropertyDef::new("ny".to_string(), PropertyType::Scalar(ScalarType::Float));
    point_element.properties.add(p);
    let p = PropertyDef::new("nz".to_string(), PropertyType::Scalar(ScalarType::Float));
    point_element.properties.add(p);
    ply.header.elements.add(point_element);

    ply
}
