extern crate vtkio;
extern crate anyhow;

use std::path::PathBuf;

use na::Vector3;
use splashsurf_lib::{Real, mesh::TriMesh3d};
use vtkio::{IOBuffer, model::{DataSet, VertexNumbers}};
use anyhow::{Context, anyhow};

pub fn particles_from_coords_with_stride<RealOut: Real, RealIn: Real>(
    coords: &Vec<RealIn>,
    stride: usize,
    x_index: usize,
    y_index: usize,
    z_index: usize,
) -> Result<Vec<Vector3<RealOut>>, anyhow::Error> {
    if coords.len() % stride != 0 {
        anyhow!(format!("The number of values in the particle data point buffer is not divisible by stride: {}", stride));
    }

    let num_points = coords.len() / stride;
    let mut positions = Vec::with_capacity(num_points);
    for i in 0..num_points {
        positions.push(Vector3::new(
            RealOut::from_f32(coords[stride * i + x_index].to_f32().unwrap()).unwrap(),
            RealOut::from_f32(coords[stride * i + y_index].to_f32().unwrap()).unwrap(),
            RealOut::from_f32(coords[stride * i + z_index].to_f32().unwrap()).unwrap(),
        ))
    }

    Ok(positions)
}


pub fn trimesh_from_dataset<R: Real>(dataset: DataSet) -> Result<TriMesh3d<R>, anyhow::Error> {
    if let DataSet::UnstructuredGrid { pieces, .. } = dataset {
        if let Some(piece) = pieces.into_iter().next() {
            let piece_data = piece
                .load_piece_data()
                .context("Failed to load unstructured grid piece")?;

            let points: Result<Vec<Vector3<R>>, anyhow::Error> = match piece_data.points {
                IOBuffer::F32(coords) => {particles_from_coords_with_stride(&coords, 3, 0, 1, 2)}
                // IOBuffer::F64(coords) => {particles_from_coords_with_stride(&coords, 3, 0, 1, 2)}
                _ => Err(anyhow!(
                    "Point coordinate IOBuffer does not contain f32 or f64 values"
                ))
            };

            let points = points.unwrap();

            let cells = piece_data.cells;

            let cell_verts =  match cells.cell_verts {
                VertexNumbers::Legacy { num_cells, vertices } => Ok(vertices),
                _ => Err(anyhow!(
                    "Can only load from Legacy VTK files"
                ))
            }.unwrap();
            
            let trimesh: TriMesh3d<R> = {
                //FIXME: This converts from Vec<u32> -> Vec<[usize, 3]>, but there must be a better way
                let chunks: Vec<Vec<u32>> = cell_verts
                    .chunks(4)
                    .map(|x| x.to_vec())
                    .collect();

                let mut cvertices: Vec<[usize; 3]> = Vec::new();
                for chunk in chunks {
                    // println!("doing chunk");
                    // println!("{:?}", chunk);
                    cvertices.push([chunk[1] as usize, chunk[2] as usize, chunk[3] as usize]);
                }
                TriMesh3d { vertices: points, triangles: cvertices }
            };

            Ok(trimesh)
        } else {
            Err(anyhow!(
                "Loaded dataset does not contain an unstructured grid piece"
            ))
        }
    } else {
        Err(anyhow!(
            "Loaded dataset does not contain an unstructured grid"
        ))
    }
}