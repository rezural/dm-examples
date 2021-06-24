extern crate ply_rs;

use na::Vector3;
use ply_rs::ply::{
    Addable, DefaultElement, ElementDef, Encoding, Ply, Property, PropertyDef, PropertyType, ScalarType,
};
use ply_rs::writer::Writer;
use salva3d::object::Fluid;
use std::{fs::File, io::BufWriter};
use std::io::Write;
use std::path;

pub fn output_particles_to_file(fluid: &Fluid, to_file: &path::PathBuf) {
    let mut file: File = File::create(to_file).unwrap();
    let mut buf = Vec::<u8>::new();

    let mut ply = create_ply();

    let points = fluid
        .positions
        .iter()
        .enumerate()
        .map(|(i, particle)| {
            let mut point = DefaultElement::new();
            let velocity = fluid.velocities[i];

            // first frame will not have velocities
            // FIXME: what does this actually do? it looks wrong
            let velocity = if velocity.x == 0. {
                velocity
            } else {
                Vector3::from(velocity)
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

    ply.payload.insert("vertex".to_string(), points);

    let w = Writer::new();
    let _written = w.write_ply(&mut buf, &mut ply).unwrap();

    let mut buf_writer = BufWriter::new(file);
    match buf_writer.write_all(&buf) {
        Ok(r) => r,
        _ => ()
    }
}

fn create_ply() -> Ply<DefaultElement> {
    let mut ply = Ply::<DefaultElement>::new();
    ply.header.encoding = Encoding::BinaryBigEndian;
    ply.header.comments.push("A beautiful comment!".to_string());
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
