use clap::Parser;
use gdal::{raster::ResampleAlg, Dataset};
use std::fs::File;
use stl_io::{Normal, Triangle, Vertex};

#[derive(Parser)]
struct Args {
    #[arg(long)]
    geotiff: String,
    #[arg(long)]
    stl: String,
    #[arg(long, default_value_t = 0.016)]
    horizontal_scale: f64,
    #[arg(long, default_value_t = 0.016)]
    vertical_scale: f32,
    #[arg(long, default_value_t = 0.05)]
    minimum_level_of_detail: f64,
    #[arg(long, default_value_t = 0.0)]
    base_height: f32,
    #[arg(long)]
    centre_x: Option<f64>,
    #[arg(long)]
    centre_y: Option<f64>,
    #[arg(long)]
    quantise_z_layer_height: Option<f32>,
    #[arg(long)]
    edge_fillet: Option<f32>,
}

fn main() {
    let args = Args::parse();

    if args.geotiff == "-" {
        panic!("stdin not yet supported");
    }
    if args.stl == "-" {
        panic!("stdout not yet supported");
    }
    if args.horizontal_scale <= 0.0 {
        panic!("horizontal_scale must be positive")
    }
    if args.minimum_level_of_detail <= 0.0 {
        panic!("minimum_level_of_detail must be positive")
    }
    if args.base_height < 0.0 {
        panic!("minimum_level_of_detail cannot be negative")
    }
    if let Some(quantise_z_layer_height) = args.quantise_z_layer_height {
        if quantise_z_layer_height <= 0.0 {
            panic!("quantise_z_layer_height must be positive");
        }
    }
    if let Some(edge_fillet) = args.edge_fillet {
        if edge_fillet <= 0.0 {
            panic!("edge_fillet must be positive");
        }
        if let Some(edge_fillet) = args.edge_fillet {
            if edge_fillet > args.base_height {
                panic!("edge_fillet cannot exceed base_height");
            }
        }
    }

    geotiff2stl(args);
}

fn geotiff2stl(args: Args) {
    println!("Reading from {:?}", args.geotiff);
    println!();

    let dataset = Dataset::open(&args.geotiff).unwrap();

    if dataset.driver().long_name() != "GeoTIFF" {
        panic!("dataset was not parsed as a GeoTIFF");
    }

    println!("GEOTIFF");
    let spatial_ref = dataset.spatial_ref().unwrap().name().unwrap();
    println!("Spatial ref is {:?}", spatial_ref);
    let geo_transform = dataset.geo_transform().unwrap();
    println!("Geo transform is {:?}", geo_transform);
    let raster_size = dataset.raster_size();
    println!("Pixel size (x, y) is {:?}", raster_size);

    if dataset.raster_count() != 1 {
        panic!("only 1 raster band is supported");
    }
    if spatial_ref != "OSGB36 / British National Grid" && spatial_ref != "British_National_Grid" {
        panic!("only 'OSGB36 / British National Grid' spatial ref is supported");
    }
    // Look up index numbers against https://docs.rs/gdal/latest/gdal/type.GeoTransform.html
    if geo_transform[2] != 0.0 || geo_transform[4] != 0.0 {
        panic!("geo rotations are not supported");
    }
    if geo_transform[5] >= 0.0 {
        panic!("y-axis must be north-up");
    }
    if geo_transform[1] <= 0.0 {
        panic!("x-axis must not be inverted");
    }

    let geotiff_top_left_xy = (geo_transform[0], geo_transform[3]);
    let geotiff_pixel_resolution_xy_m = (geo_transform[1], -geo_transform[5]);
    let geotiff_geo_dimensions = (
        geotiff_pixel_resolution_xy_m.0 * (raster_size.0 as f64),
        geotiff_pixel_resolution_xy_m.1 * (raster_size.1 as f64),
    );
    println!(
        "Geospatial area (x, y) dimensions are {:?}",
        geotiff_geo_dimensions
    );
    let geotiff_bottom_right_xy = (
        geotiff_top_left_xy.0 + geotiff_geo_dimensions.0,
        geotiff_top_left_xy.1 - geotiff_geo_dimensions.1,
    );
    println!(
        "Geospatial area top-left (x, y) is at {:?}",
        geotiff_top_left_xy
    );
    println!(
        "Geospatial area bottom-right (x, y) is at {:?}",
        geotiff_bottom_right_xy
    );
    println!(
        "Geospatial area per pixel (x, y) is {:?}/m",
        geotiff_pixel_resolution_xy_m
    );
    println!();

    println!("MESH DIMENSIONS");
    println!(
        "GeoTIFF represents an (x, y) area of {:?}m",
        geotiff_geo_dimensions
    );
    println!("Horizontal scale is set to {}x", args.horizontal_scale);
    println!("Vertical scale is set to {}x", args.vertical_scale);
    println!(
        "Vertical scale is {}x the horizontal scale",
        args.vertical_scale / args.horizontal_scale as f32
    );
    let mesh_dimensions = (
        geotiff_geo_dimensions.0 * args.horizontal_scale,
        geotiff_geo_dimensions.1 * args.horizontal_scale,
    );
    println!(
        "Mesh dimensions (x, y) will be {:?}mm = ({}, {})meters",
        mesh_dimensions,
        mesh_dimensions.0 / 1000.0,
        mesh_dimensions.1 / 1000.0
    );
    let base_z = Some(-args.base_height);
    println!("Mesh base height will be {}mm", args.base_height);
    println!();

    println!("MESH LEVEL OF DETAIL");
    let available_resolution = (
        (1.0 / geotiff_pixel_resolution_xy_m.0) * args.horizontal_scale,
        (1.0 / geotiff_pixel_resolution_xy_m.1) * args.horizontal_scale,
    );
    println!(
        "GeoTIFF provides an (x, y) level of detail of {:?}mm",
        available_resolution
    );
    println!(
        "Minimum level of detail is set to {}mm",
        args.minimum_level_of_detail
    );
    let resolution = (
        available_resolution.0.max(args.minimum_level_of_detail),
        available_resolution.1.max(args.minimum_level_of_detail),
    );
    println!("Mesh level of detail (x, y) will be {:?}mm", resolution);
    let mut centering_offsets = (mesh_dimensions.0 / 2.0, mesh_dimensions.1 / 2.0);
    if let Some(centre_x) = args.centre_x {
        centering_offsets.0 =
            mesh_dimensions.0 * (centre_x - geotiff_top_left_xy.0) / geotiff_geo_dimensions.0;
        println!(
            "Mesh is set to put X=0 at real-world coordinate {}",
            centre_x
        );
    }
    if let Some(centre_y) = args.centre_y {
        centering_offsets.1 =
            mesh_dimensions.1 * (centre_y - geotiff_bottom_right_xy.1) / geotiff_geo_dimensions.1;
        println!(
            "Mesh is set to put Y=0 at real-world coordinate {}",
            centre_y
        );
    }
    if let Some(quantise_z_layer_height) = args.quantise_z_layer_height {
        println!(
            "Mesh is set to quantize Z heights into steps of {}mm",
            quantise_z_layer_height
        );
    }
    println!();

    println!("MESH SIZE");
    let vertex_counts = (
        (mesh_dimensions.0 as f64 * 1.0 / resolution.0).floor() as usize,
        (mesh_dimensions.1 as f64 * 1.0 / resolution.1).floor() as usize,
    );
    println!("Mesh vertex counts (x, y) are {:?}", vertex_counts);
    println!(
        "Mesh will have {} vertices",
        vertex_counts.0 * vertex_counts.1
    );
    println!();

    let rasterband = dataset.rasterband(1).unwrap();
    if rasterband.size() != dataset.raster_size() {
        panic!("scaled rasterbands is not supported");
    }
    if rasterband.color_interpretation() != gdal::raster::ColorInterpretation::GrayIndex {
        panic!("non-grayindex rasterband color tables are not supported");
    }
    if rasterband.band_type().name() != "Float32" {
        panic!("non-float32 rasterband types are not supported");
    }

    let buf = rasterband
        .read_as::<f32>(
            (0, 0),
            rasterband.size(),
            vertex_counts,
            Some(ResampleAlg::Bilinear),
        )
        .unwrap();
    let buf_z = |x: usize, y: usize| {
        let y_flipped = buf.size.1 - y - 1;
        let mut z_value = buf.data[y_flipped * buf.size.0 + x] * args.vertical_scale;

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
                // z_value += (hypotenuse.powf(2.0) - adjacent.powf(2.0)).sqrt() * resolution.0 as f32
                //     - edge_fillet;
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

        let y = buf.size.0 - 1;
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
        .map(|t| t.vertices)
        .flatten()
        .map(|v| v[0])
        .fold(0.0, |a: f32, b: f32| a.max(b));
    let x_min = triangles
        .iter()
        .map(|t| t.vertices)
        .flatten()
        .map(|v| v[0])
        .fold(f32::INFINITY, |a: f32, b: f32| a.min(b));
    let y_max = triangles
        .iter()
        .map(|t| t.vertices)
        .flatten()
        .map(|v| v[1])
        .fold(0.0, |a: f32, b: f32| a.max(b));
    let y_min = triangles
        .iter()
        .map(|t| t.vertices)
        .flatten()
        .map(|v| v[1])
        .fold(f32::INFINITY, |a: f32, b: f32| a.min(b));
    let z_max = triangles
        .iter()
        .map(|t| t.vertices)
        .flatten()
        .map(|v| v[2])
        .fold(0.0, |a: f32, b: f32| a.max(b));
    let z_min = triangles
        .iter()
        .map(|t| t.vertices)
        .flatten()
        .map(|v| v[2])
        .fold(f32::INFINITY, |a: f32, b: f32| a.min(b));

    println!("RESULT");
    println!("Mesh has {} triangles", triangles.len());
    println!("Mesh X ranges between {}mm and {}mm", x_max, x_min);
    println!("Mesh Y ranges between {}mm and {}mm", y_max, y_min);
    println!("Mesh height ranges between {}mm and {}mm", z_max, z_min);
    println!(
        "Horizontal scale is 1:{:.2}",
        1000.0 / args.horizontal_scale
    );
    println!("Vertical scale is 1:{:.2}", 1000.0 / args.vertical_scale);
    println!("Import the mesh to other software using millimeters");
    println!();

    println!("Writing to {:?}", args.stl);
    let mut f = File::create(args.stl).unwrap();
    stl_io::write_stl(&mut f, triangles.iter()).unwrap();
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
