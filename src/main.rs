use clap::{Parser, ValueEnum};
use gdal::{raster::ResampleAlg, Dataset};
use std::fs::File;
use std::io::{stdout, Write};
use std::path::Path;
use std::process::ExitCode;
use stl_io::{Normal, Triangle, Vertex};

#[derive(Parser, PartialEq, Clone, Debug)]
struct Args {
    /// Path to the input GeoTIFF image. Must be a file.
    #[arg(long)]
    geotiff: String,
    /// Path to write the output STL mesh. Specify - to write to STDOUT.
    #[arg(long)]
    stl: String,
    /// Scale of the horizontal (X and Y) axes
    #[arg(long, default_value_t = 0.016)]
    horizontal_scale: f64,
    /// Scale of the vertical (Z) axis
    #[arg(long, default_value_t = 0.016)]
    vertical_scale: f32,
    /// Reduce the size of output meshes by not creating vertices closer than this many mm
    #[arg(long, default_value_t = 0.05)]
    minimum_level_of_detail: f64,
    /// Optionally thicken the model by extending the base downwards by this many mm
    #[arg(long, default_value_t = 0.0)]
    base_height: f32,
    /// Optionally centre the coordinates of the output mesh about this X coordinate,
    /// specified in the coordinate system of the GeoTIFF. Default is to centre the mesh
    /// at the middle of the GeoTIFF.
    #[arg(long)]
    centre_x: Option<f64>,
    /// Optionally centre the coordinates of the output mesh about this Y coordinate,
    /// specified in the coordinate system of the GeoTIFF. Default is to centre the mesh
    /// at the middle of the GeoTIFF.
    #[arg(long)]
    centre_y: Option<f64>,
    /// What to use as `0` in the Z (vertical) axis. Either sea level or the minimum height
    /// in the GeoTIFF.
    #[arg(long, value_enum, default_value_t = ZOffset::SeaLevel)]
    z_offset: ZOffset,
    /// Optionally round Z heights to multiples of this many mm. Creates a contouring effect.
    #[arg(long)]
    quantise_z_layer_height: Option<f32>,
    /// Optionally fillet (round) the edges of the top surface by this many mm
    #[arg(long)]
    edge_fillet: Option<f32>,
}

#[derive(ValueEnum, PartialEq, Clone, Debug)]
enum ZOffset {
    SeaLevel,
    DataMinimum,
}

impl Args {
    fn validate(&self) -> Result<(), String> {
        if !Path::new(&self.geotiff).is_file() {
            return Err(format!(
                "--geotiff {:?} did not exist or was not a file",
                self.geotiff
            ));
        }
        if self.stl.is_empty() {
            return Err("--stl argument cannot be empty".to_string());
        }

        if self.horizontal_scale <= 0.0 {
            return Err(format!(
                "--horizontal-scale must be positive, but was {}",
                self.horizontal_scale
            ));
        }
        if self.minimum_level_of_detail <= 0.0 {
            return Err(format!(
                "--minimum-level-of-detail must be positive, but was {}",
                self.minimum_level_of_detail
            ));
        }
        if self.base_height < 0.0 {
            return Err(format!(
                "--base-height cannot be negative, but was {}",
                self.base_height
            ));
        }
        if let Some(quantise_z_layer_height) = self.quantise_z_layer_height {
            if quantise_z_layer_height <= 0.0 {
                return Err(format!(
                    "--quantise-z-layer-height must be positive, but was {}",
                    quantise_z_layer_height
                ));
            }
        }
        if let Some(edge_fillet) = self.edge_fillet {
            if edge_fillet <= 0.0 {
                return Err(format!(
                    "--edge-fillet must be positive, but was {}",
                    edge_fillet
                ));
            }
            if let Some(edge_fillet) = self.edge_fillet {
                if edge_fillet > self.base_height {
                    return Err(format!(
                        "--edge-fillet cannot exceed --base-height, but {} > {}",
                        edge_fillet, self.base_height
                    ));
                }
            }
        }
        Ok(())
    }
}

fn main() -> ExitCode {
    let args = Args::parse();
    if let Err(e) = args.validate() {
        eprintln!("ERROR: Arguments invalid: {}", e);
        return ExitCode::FAILURE;
    }

    if let Err(e) = geotiff2stl(args) {
        eprintln!("ERROR: Could not generate STL: {}", e);
        return ExitCode::FAILURE;
    }

    ExitCode::SUCCESS
}

fn geotiff2stl(args: Args) -> Result<(), String> {
    eprintln!("Reading from {:?}", args.geotiff);
    eprintln!();

    let dataset = Dataset::open(&args.geotiff).unwrap();

    if dataset.driver().long_name() != "GeoTIFF" {
        return Err(format!(
            "Input was not parsed as a GeoTIFF: {:?}",
            dataset.driver().long_name()
        ));
    }

    eprintln!("GEOTIFF");
    let spatial_ref = dataset.spatial_ref().unwrap().name().unwrap();
    eprintln!("Spatial ref is {:?}", spatial_ref);
    let geo_transform = dataset.geo_transform().unwrap();
    eprintln!("Geo transform is {:?}", geo_transform);
    let raster_size = dataset.raster_size();
    eprintln!("Pixel size (x, y) is {:?}", raster_size);

    if dataset.raster_count() != 1 {
        return Err(format!(
            "only 1 raster band is supported, but had {}",
            dataset.raster_count()
        ));
    }
    // FIXME: Come up with more generic way to only accept British National Grid, or do conversions
    if spatial_ref != "OSGB36 / British National Grid" && spatial_ref != "British_National_Grid" {
        return Err(format!(
            "only 'OSGB36 / British National Grid' spatial ref is supported, but had {:?}",
            spatial_ref
        ));
    }
    // Look up index numbers against https://docs.rs/gdal/latest/gdal/type.GeoTransform.html
    if geo_transform[2] != 0.0 || geo_transform[4] != 0.0 {
        return Err(format!(
            "GeoTIFF rotations are not supported: {} {}",
            geo_transform[2], geo_transform[4]
        ));
    }
    if geo_transform[5] >= 0.0 {
        return Err(format!(
            "GeoTIFF y-axis must be north-up but was non-negative: {}",
            geo_transform[5]
        ));
    }
    if geo_transform[1] <= 0.0 {
        return Err(format!(
            "GeoTIFF x-axis must not be inverted: {}",
            geo_transform[1]
        ));
    }

    let geotiff_top_left_xy = (geo_transform[0], geo_transform[3]);
    let geotiff_pixel_resolution_xy_m = (geo_transform[1], -geo_transform[5]);
    let geotiff_geo_dimensions = (
        geotiff_pixel_resolution_xy_m.0 * (raster_size.0 as f64),
        geotiff_pixel_resolution_xy_m.1 * (raster_size.1 as f64),
    );
    eprintln!(
        "Geospatial area (x, y) dimensions are {:?}",
        geotiff_geo_dimensions
    );
    let geotiff_bottom_right_xy = (
        geotiff_top_left_xy.0 + geotiff_geo_dimensions.0,
        geotiff_top_left_xy.1 - geotiff_geo_dimensions.1,
    );
    eprintln!(
        "Geospatial area top-left (x, y) is at {:?}",
        geotiff_top_left_xy
    );
    eprintln!(
        "Geospatial area bottom-right (x, y) is at {:?}",
        geotiff_bottom_right_xy
    );
    eprintln!(
        "Geospatial area per pixel (x, y) is {:?}/m",
        geotiff_pixel_resolution_xy_m
    );
    eprintln!();

    eprintln!("MESH DIMENSIONS");
    eprintln!(
        "GeoTIFF represents an (x, y) area of {:?}m",
        geotiff_geo_dimensions
    );
    eprintln!("Horizontal scale is set to {}x", args.horizontal_scale);
    eprintln!("Vertical scale is set to {}x", args.vertical_scale);
    eprintln!(
        "Vertical scale is {}x the horizontal scale",
        args.vertical_scale / args.horizontal_scale as f32
    );
    let mesh_dimensions = (
        geotiff_geo_dimensions.0 * args.horizontal_scale,
        geotiff_geo_dimensions.1 * args.horizontal_scale,
    );
    eprintln!(
        "Mesh dimensions (x, y) will be {:?}mm = ({}, {})meters",
        mesh_dimensions,
        mesh_dimensions.0 / 1000.0,
        mesh_dimensions.1 / 1000.0
    );
    let base_z = Some(-args.base_height);
    eprintln!("Mesh base height will be {}mm", args.base_height);
    eprintln!();

    eprintln!("MESH LEVEL OF DETAIL");
    let available_resolution = (
        (1.0 / geotiff_pixel_resolution_xy_m.0) * args.horizontal_scale,
        (1.0 / geotiff_pixel_resolution_xy_m.1) * args.horizontal_scale,
    );
    eprintln!(
        "GeoTIFF provides an (x, y) level of detail of {:?}mm",
        available_resolution
    );
    eprintln!(
        "Minimum level of detail is set to {}mm",
        args.minimum_level_of_detail
    );
    let resolution = (
        available_resolution.0.max(args.minimum_level_of_detail),
        available_resolution.1.max(args.minimum_level_of_detail),
    );
    eprintln!("Mesh level of detail (x, y) will be {:?}mm", resolution);
    let mut centering_offsets = (mesh_dimensions.0 / 2.0, mesh_dimensions.1 / 2.0);
    if let Some(centre_x) = args.centre_x {
        centering_offsets.0 =
            mesh_dimensions.0 * (centre_x - geotiff_top_left_xy.0) / geotiff_geo_dimensions.0;
        eprintln!(
            "Mesh is set to put X=0 at real-world coordinate {}",
            centre_x
        );
    }
    if let Some(centre_y) = args.centre_y {
        centering_offsets.1 =
            mesh_dimensions.1 * (centre_y - geotiff_bottom_right_xy.1) / geotiff_geo_dimensions.1;
        eprintln!(
            "Mesh is set to put Y=0 at real-world coordinate {}",
            centre_y
        );
    }
    if let Some(quantise_z_layer_height) = args.quantise_z_layer_height {
        eprintln!(
            "Mesh is set to quantize Z heights into steps of {}mm",
            quantise_z_layer_height
        );
    }
    if let Some(edge_fillet) = args.edge_fillet {
        eprintln!(
            "Mesh edges will be filleted (rounded) with a radius of {}mm",
            edge_fillet
        );
    }
    eprintln!();

    eprintln!("MESH SIZE");
    let vertex_counts = (
        (mesh_dimensions.0 * 1.0 / resolution.0).floor() as usize,
        (mesh_dimensions.1 * 1.0 / resolution.1).floor() as usize,
    );
    eprintln!("Mesh vertex counts (x, y) are {:?}", vertex_counts);
    eprintln!(
        "Mesh will have {} vertices",
        vertex_counts.0 * vertex_counts.1
    );
    eprintln!();

    let rasterband = dataset.rasterband(1).unwrap();
    if rasterband.size() != dataset.raster_size() {
        return Err(format!(
            "GeoTIFF scaled rasterbands are not supported: {:?} was not {:?}",
            rasterband.size(),
            dataset.raster_size()
        ));
    }
    if rasterband.color_interpretation() != gdal::raster::ColorInterpretation::GrayIndex {
        return Err(format!(
            "GeoTIFF non-grayindex rasterband color tables are not supported: {:?}",
            rasterband.color_interpretation()
        ));
    }
    if rasterband.band_type().name() != "Float32" {
        return Err(format!(
            "GeoTIFF non-float32 rasterband types are not supported: {:?}",
            rasterband.band_type().name()
        ));
    }

    let buf = rasterband
        .read_as::<f32>(
            (0, 0),
            rasterband.size(),
            vertex_counts,
            Some(ResampleAlg::Bilinear),
        )
        .unwrap();

    let mut z_offset = 0.0;
    if args.z_offset == ZOffset::DataMinimum {
        z_offset = f32::INFINITY;
        for y in 0..(buf.size.1 - 1) {
            for x in 0..(buf.size.0 - 1) {
                let y_flipped = buf.size.1 - y - 1;
                let z_value = buf.data[y_flipped * buf.size.0 + x] * args.vertical_scale;
                if z_value < z_offset {
                    z_offset = z_value;
                }
            }
        }
    }

    let buf_z = |x: usize, y: usize| {
        let y_flipped = buf.size.1 - y - 1;
        let mut z_value = buf.data[y_flipped * buf.size.0 + x] * args.vertical_scale - z_offset;

        if let Some(quantise_z_layer_height) = args.quantise_z_layer_height {
            let quantization_multiplier = 1.0 / quantise_z_layer_height;
            z_value = (z_value * quantization_multiplier).round() / quantization_multiplier;
        }

        if let Some(edge_fillet) = args.edge_fillet {
            let edge_fillet_vertex_count = (
                edge_fillet / resolution.0 as f32,
                edge_fillet / resolution.1 as f32,
            );
            let mut fillet_z_value_decrease: f32 = 0.0;
            if (x as f32) < edge_fillet_vertex_count.0 {
                let adjacent = edge_fillet_vertex_count.0 - x as f32;
                let hypotenuse = edge_fillet_vertex_count.0;
                let opposite = (hypotenuse.powf(2.0) - adjacent.powf(2.0)).sqrt()
                    * resolution.0 as f32
                    - edge_fillet;
                fillet_z_value_decrease = fillet_z_value_decrease.min(opposite);
            } else if (x as f32) >= vertex_counts.0 as f32 - edge_fillet_vertex_count.0 {
                let adjacent = edge_fillet_vertex_count.0 - (vertex_counts.0 - x) as f32;
                let hypotenuse = edge_fillet_vertex_count.0;
                let opposite = (hypotenuse.powf(2.0) - adjacent.powf(2.0)).sqrt()
                    * resolution.0 as f32
                    - edge_fillet;
                fillet_z_value_decrease = fillet_z_value_decrease.min(opposite);
            }
            if (y as f32) < edge_fillet_vertex_count.1 {
                let adjacent = edge_fillet_vertex_count.1 - y as f32;
                let hypotenuse = edge_fillet_vertex_count.1;
                let opposite = (hypotenuse.powf(2.0) - adjacent.powf(2.0)).sqrt()
                    * resolution.1 as f32
                    - edge_fillet;
                fillet_z_value_decrease = fillet_z_value_decrease.min(opposite);
            } else if (y as f32) >= vertex_counts.1 as f32 - edge_fillet_vertex_count.1 {
                let adjacent = edge_fillet_vertex_count.1 - (vertex_counts.1 - y) as f32;
                let hypotenuse = edge_fillet_vertex_count.1;
                let opposite = (hypotenuse.powf(2.0) - adjacent.powf(2.0)).sqrt()
                    * resolution.1 as f32
                    - edge_fillet;
                fillet_z_value_decrease = fillet_z_value_decrease.min(opposite);
            }
            z_value += fillet_z_value_decrease;
        }

        z_value
    };

    let vertex = |x: usize, y: usize, z: Option<f32>| {
        Vertex::new([
            trunc((x as f64 * resolution.0 - centering_offsets.0) as f32),
            trunc((y as f64 * resolution.1 - centering_offsets.1) as f32),
            trunc(z.unwrap_or_else(|| buf_z(x, y))),
        ])
    };

    let mut triangles = vec![];
    for y in 0..(buf.size.1 - 1) {
        for x in 0..(buf.size.0 - 1) {
            let v1 = vertex(x, y, None);
            let v2 = vertex(x + 1, y, None);
            let v3 = vertex(x + 1, y + 1, None);
            let v4 = vertex(x, y + 1, None);
            triangles.push(triangle(v1, v2, v3));
            triangles.push(triangle(v3, v4, v1));
        }
    }

    for y in 0..(buf.size.1 - 1) {
        let x = 0;
        let v1 = vertex(x, y + 1, base_z);
        let v2 = vertex(x, y + 1, None);
        let v3 = vertex(x, y, None);
        let v4 = vertex(x, y, base_z);
        triangles.push(triangle(v3, v2, v1));
        triangles.push(triangle(v1, v4, v3));

        let x = buf.size.0 - 1;
        let v1 = vertex(x, y + 1, base_z);
        let v2 = vertex(x, y + 1, None);
        let v3 = vertex(x, y, None);
        let v4 = vertex(x, y, base_z);
        triangles.push(triangle(v1, v2, v3));
        triangles.push(triangle(v3, v4, v1));
    }

    for x in 0..(buf.size.0 - 1) {
        let y = 0;
        let v1 = vertex(x, y, base_z);
        let v2 = vertex(x, y, None);
        let v3 = vertex(x + 1, y, None);
        let v4 = vertex(x + 1, y, base_z);
        triangles.push(triangle(v3, v2, v1));
        triangles.push(triangle(v1, v4, v3));

        let y = buf.size.1 - 1;
        let v1 = vertex(x, y, base_z);
        let v2 = vertex(x, y, None);
        let v3 = vertex(x + 1, y, None);
        let v4 = vertex(x + 1, y, base_z);
        triangles.push(triangle(v1, v2, v3));
        triangles.push(triangle(v3, v4, v1));
    }

    let v1 = vertex(0, buf.size.1 - 1, base_z);
    let v2 = vertex(0, 0, base_z);
    let v3 = vertex(buf.size.0 - 1, 0, base_z);
    let v4 = vertex(buf.size.0 - 1, buf.size.1 - 1, base_z);
    triangles.push(triangle(v3, v2, v1));
    triangles.push(triangle(v1, v4, v3));

    let x_max = triangles
        .iter()
        .flat_map(|t| t.vertices)
        .map(|v| v[0])
        .fold(0.0, |a: f32, b: f32| a.max(b));
    let x_min = triangles
        .iter()
        .flat_map(|t| t.vertices)
        .map(|v| v[0])
        .fold(f32::INFINITY, |a: f32, b: f32| a.min(b));
    let y_max = triangles
        .iter()
        .flat_map(|t| t.vertices)
        .map(|v| v[1])
        .fold(0.0, |a: f32, b: f32| a.max(b));
    let y_min = triangles
        .iter()
        .flat_map(|t| t.vertices)
        .map(|v| v[1])
        .fold(f32::INFINITY, |a: f32, b: f32| a.min(b));
    let z_max = triangles
        .iter()
        .flat_map(|t| t.vertices)
        .map(|v| v[2])
        .fold(0.0, |a: f32, b: f32| a.max(b));
    let z_min = triangles
        .iter()
        .flat_map(|t| t.vertices)
        .map(|v| v[2])
        .fold(f32::INFINITY, |a: f32, b: f32| a.min(b));

    eprintln!("RESULT");
    eprintln!("Mesh has {} triangles", triangles.len());
    eprintln!("Mesh X ranges between {}mm and {}mm", x_max, x_min);
    eprintln!("Mesh Y ranges between {}mm and {}mm", y_max, y_min);
    eprintln!("Mesh height ranges between {}mm and {}mm", z_max, z_min);
    eprintln!(
        "Horizontal scale is 1:{:.2}",
        1000.0 / args.horizontal_scale
    );
    eprintln!("Vertical scale is 1:{:.2}", 1000.0 / args.vertical_scale);
    eprintln!("Import the mesh to other software using millimeters");
    eprintln!();

    eprintln!("Writing to {:?}", args.stl);
    let mut f: Box<dyn Write> = if args.stl == "-" {
        Box::new(stdout().lock())
    } else {
        Box::new(File::create(args.stl).unwrap())
    };
    if let Err(e) = stl_io::write_stl(&mut f, triangles.iter()) {
        return Err(format!("error writing STL: {}", e));
    }
    Ok(())
}

fn triangle(v1: Vertex, v2: Vertex, v3: Vertex) -> Triangle {
    let u = [v2[0] - v1[0], v2[1] - v1[1], v2[2] - v1[2]];
    let v = [v3[0] - v1[0], v3[1] - v1[1], v3[2] - v1[2]];
    Triangle {
        vertices: [v1, v2, v3],
        normal: Normal::new([
            trunc(u[1] * v[2] - u[2] * v[1]),
            trunc(u[2] * v[0] - u[0] * v[2]),
            trunc(u[0] * v[1] - u[1] * v[0]),
        ]),
    }
}

fn trunc(before: f32) -> f32 {
    f32::trunc(before * 1000.0) / 1000.0
}

// fn triangles_bounds(triangles: &Vec<Triangle>) -> [(f32, f32); 3] {
//     let vertices = triangles.iter().flat_map(|t| t.vertices);
//     let x = f32_bounds(&vertices.map(|v| v[0]));
//     let y = f32_bounds(&vertices.map(|v| v[1]));
//     let z = f32_bounds(&vertices.map(|v| v[2]));
//     [x, y, z]
// }

// fn f32_bounds<I: Iterator<Item = f32>>(iter: &I) -> (f32, f32) {
//     let max = iter.fold(0.0, |a: f32, b: f32| a.max(b));
//     let min = iter.fold(0.0, |a: f32, b: f32| a.min(b));
//     (min, max)
// }
