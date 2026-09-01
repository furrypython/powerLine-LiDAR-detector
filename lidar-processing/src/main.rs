use las::Read;
use std::{fs::metadata, fs::create_dir, fs::read_dir, env, time, path::Path, ffi::OsStr};

mod io; 
mod histogram;
mod morph;
mod graph;
mod ground;
mod grid;
mod pca;

//----------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------

fn execute_algorithms(input: &String, output: &String, all_files: &Vec<(String, las::Bounds)>, buffer_zone: f64) {
    let mut readed_items = io::read_las(input); 
    let original_bounds = readed_items.0.header().bounds();

    println!("----------Processing file: {}----------", input);

    //Ground Reduction
    let now = time::Instant::now();
    let mut combined_points = ground::ground_reduction(&mut readed_items.0, 0.01);
    
    let min_x = original_bounds.min.x - buffer_zone;
    let min_y = original_bounds.min.y - buffer_zone;
    let max_x = original_bounds.max.x + buffer_zone;
    let max_y = original_bounds.max.y + buffer_zone;

    for (neighbor_path, neighbor_bounds) in all_files {
        if neighbor_path == input { continue; }
        // Check intersection
        if neighbor_bounds.min.x <= max_x && neighbor_bounds.max.x >= min_x &&
           neighbor_bounds.min.y <= max_y && neighbor_bounds.max.y >= min_y {
            let mut neighbor_items = io::read_las(neighbor_path);
            let neighbor_points = ground::ground_reduction(&mut neighbor_items.0, 0.01);
            for p in neighbor_points {
                if p.x >= min_x && p.x <= max_x && p.y >= min_y && p.y <= max_y {
                    combined_points.push(p);
                }
            }
        }
    }

    println!("Ground Reduction & Buffer Loading time: {:?} millisecs.", now.elapsed().as_millis());
    
    // Height-Above-Ground Filter
    let now = time::Instant::now();
    let elevated_points = ground::filter_by_height(&combined_points, 3.0);
    println!("Height Filter time: {:?} millisecs. Extracted {} points from {} total.", now.elapsed().as_millis(), elevated_points.len(), combined_points.len());

    // Ensemble Filtering (PCA + Thin Structure + Orientation + Junctions + Droppers + Length)
    let now = time::Instant::now();
    let final_points = pca::extract_powerlines(&elevated_points, 0.3, 0.85); // radius=0.3m, threshold=0.85
    println!("Ensemble Filtering time: {:?} millisecs. Extracted {} points.", now.elapsed().as_millis(), final_points.len());

    // Skip the old heuristic pipeline and write the mathematically identified wires directly!
    //Writing output file
    let now = time::Instant::now();
    io::write_las_flat(&final_points, readed_items.1, output, original_bounds);
    println!("Writing time: {:?} millisecs.", now.elapsed().as_millis());
}

fn main() {
    let total_time = time::Instant::now();
    let args: Vec<String> = env::args().collect();

    if args.len() != 3 {
        panic!("Error: Program must be executes with command \"./lidar-processing <input> <output>\"");
    }

    let input = &args[1];
    let output = &args[2];
    let mut num_cells = 0;

    let md_input = metadata(input).unwrap();

    let mut all_files = Vec::new();

    if md_input.is_dir() {
        let paths = read_dir(input).unwrap();
        for path in paths {
            let path = path.unwrap().path();
            let extension = path.extension().and_then(OsStr::to_str);
            if extension == Some("las") || extension == Some("laz") {
                let filename = path.file_name().unwrap().to_str().unwrap();
                let input_path = format!("{}/{}", input, filename);
                let bounds = {
                    let items = io::read_las(&input_path);
                    items.0.header().bounds()
                };
                all_files.push((input_path, bounds));
            }
        }
        
        if !(Path::new(output).exists()) {
            create_dir(output).unwrap();
        }
        
        let paths2 = read_dir(input).unwrap();
        for path in paths2 {
            let path = path.unwrap().path();
            let extension = path.extension().and_then(OsStr::to_str);
            if extension == Some("las") || extension == Some("laz") {
                let filename = path.file_name().unwrap().to_str().unwrap();
                let input_path = format!("{}/{}", input, filename);
                let output_filename = format!("{}/FILTERED_{}.{}", output, &filename[0..filename.len()-4], extension.unwrap());
                num_cells += 1;
                execute_algorithms(&input_path, &output_filename, &all_files, 5.0);
            }
        }
        
    } else {
        if !(Path::new(output).exists()) {
            create_dir(output).unwrap();
        }

        let filename = Path::new(&input).file_name().unwrap().to_str().unwrap();
        let extension = Path::new(&input)
                        .extension()
                        .and_then(OsStr::to_str);
        let output_filename = format!("{}/FILTERED_{}.{}", output, &filename[0..filename.len()-4], extension.unwrap());
        num_cells += 1;
        
        let bounds = {
            let items = io::read_las(input);
            items.0.header().bounds()
        };
        all_files.push((input.to_string(), bounds));
        
        execute_algorithms(input, &output_filename, &all_files, 5.0);
    }
    println!("Total time for {:?} cells: {:?} milisecs.", num_cells, total_time.elapsed().as_millis());
}