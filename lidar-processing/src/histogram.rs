use las::Point;

//---------------------------------------Histograms-------------------------------------------

//Function to get histogram of points hegihts in each cell
#[allow(dead_code)]
pub fn get_histogram(points: &Vec<Point>) -> Vec<(f64, usize)>{
    let min_z = points.iter().map(|p| p.z).fold(f64::NAN, f64::min);
    let max_z = points.iter().map(|p| p.z).fold(f64::NAN, f64::max);
    
    let diff = max_z - min_z;
    let mut histogram = Vec::new();

    for i in (0..diff.ceil() as usize).step_by(2) {
        histogram.push((min_z + i as f64, 0));
    }
    
    for point in points {
        for i in 0..histogram.len() {
            if point.z > histogram[i].0  && point.z <= histogram[i].0 + 2. {
                histogram[i].1 += 1;
                break;
            }
        }
    }
    
    histogram
}

//-------------------------------------------FILTER HEIGHT DENSITYS-------------------------------------------------

//Local Histogram based filter
fn process_cell(point_cell: &Vec<Point>, min_density: usize, empty: usize, non_empty_cont: usize, _non_empty: usize, _end: usize) -> Vec<Point> {
    let mut cell = Vec::new();
    if point_cell.is_empty() {
        return cell;
    }

    let max_z = point_cell.iter().map(|p| p.z).fold(std::f64::NAN, |a,b| a.max(b));
    let min_z = point_cell.iter().map(|p| p.z).fold(std::f64::NAN, |a,b| a.min(b));
    
    let diff = max_z - min_z;
    if diff.is_nan() || diff <= 0.0 {
        return cell;
    }

    let num_bins = diff.ceil() as usize;
    if num_bins == 0 {
        return cell;
    }

    // density for each 1m bin, top-down
    let mut bins = vec![0; num_bins];
    for p in point_cell {
        let mut bin_idx = (max_z - p.z).floor() as usize;
        if bin_idx >= num_bins {
            bin_idx = num_bins - 1;
        }
        bins[bin_idx] += 1;
    }

    let mut i = 0;
    while i < num_bins {
        // Skip empty bins
        if bins[i] <= min_density {
            i += 1;
            continue;
        }

        let cluster_start = i;
        let mut cluster_end = i;
        
        while i < num_bins && bins[i] > min_density {
            cluster_end = i;
            i += 1;
        }
        
        let cluster_height = cluster_end - cluster_start + 1;
        
        let gap_start = i;
        let mut gap_end = i;
        while i < num_bins && bins[i] <= min_density {
            gap_end = i;
            i += 1;
        }
        
        let gap_size = if gap_start < num_bins { gap_end - gap_start + 1 } else { 0 };
        
        if cluster_height <= non_empty_cont && gap_size >= empty {
            let cluster_top_z = max_z - (cluster_start as f64);
            let cluster_bottom_z = max_z - (cluster_end as f64) - 1.0 - 1.0; // 1m bin size + 1m buffer

            for p in point_cell {
                if p.z <= cluster_top_z && p.z > cluster_bottom_z {
                    cell.push(p.clone());
                }
            }
        }
    }
    
    cell
}

//Filter all cells
pub fn histogram_filter(point_cloud: &Vec<Vec<Vec<Point>>>, min_density: usize, empty:usize , non_empty_cont: usize, _non_empty:usize, _end: usize) -> Vec<Vec<Vec<Point>>> {   
    let mut filtered_point_cloud = Vec::new();
    for i in 0..point_cloud.len(){
        filtered_point_cloud.push(Vec::new());
        for j in 0..point_cloud[i].len(){
            filtered_point_cloud[i].push(process_cell(&point_cloud[i][j], min_density, empty, non_empty_cont, _non_empty, _end));
        }
    }
    filtered_point_cloud
}