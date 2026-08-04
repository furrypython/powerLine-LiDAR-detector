use core::panic;

use las::{Header, Point};

pub fn grid_division(point_cloud: Vec<Point>, cell_size: f64) -> (Vec<Vec<Vec<Point>>>, f64, f64, f64, f64) {
    if cell_size <= 0. {
        panic!("Grid division error. Cell_size must be greater than 0");
    }
    if point_cloud.is_empty() {
        return (Vec::new(), 0., 0., 0., 0.);
    }
    let min_x = point_cloud.iter().map(|p| p.x).fold(f64::INFINITY, f64::min);
    let min_y = point_cloud.iter().map(|p| p.y).fold(f64::INFINITY, f64::min);
    let max_x = point_cloud.iter().map(|p| p.x).fold(f64::NEG_INFINITY, f64::max);
    let max_y = point_cloud.iter().map(|p| p.y).fold(f64::NEG_INFINITY, f64::max);

    let mut grid_size_x = ((max_x - min_x)/cell_size).ceil() as usize;
    let mut grid_size_y = ((max_y - min_y)/cell_size).ceil() as usize;
    
    println!("Grid sizes computed: {}x{}", grid_size_x, grid_size_y);
    println!("X range: {} to {}, diff: {}", min_x, max_x, max_x - min_x);
    println!("Y range: {} to {}, diff: {}", min_y, max_y, max_y - min_y);
    
    if grid_size_x == 0 { grid_size_x = 1; }
    if grid_size_y == 0 { grid_size_y = 1; }
    
    let mut grids:Vec<Vec<Vec<Point>>> = Vec::new();
    for i in 0..grid_size_x{
        grids.push(Vec::new());
        for _ in 0..grid_size_y{
            grids[i].push(Vec::new());
        }
    }

    for point in point_cloud {
        let mut i = if point.x < min_x { 0 } else { ((point.x - min_x) / cell_size).floor() as usize };
        let mut j = if point.y < min_y { 0 } else { ((point.y - min_y) / cell_size).floor() as usize };
        
        if i >= grid_size_x { i = grid_size_x - 1; }
        if j >= grid_size_y { j = grid_size_y - 1; }
        
        grids[i][j].push(point);
    }
    (grids, min_x, min_y, max_x, max_y)
}

pub fn create_result(cells_list: &Vec<(usize, usize)>, point_cloud: &Vec<Vec<Vec<las::Point>>>, density_thres: usize) -> Vec<Vec<Vec<las::Point>>> {
    let mut result = Vec::new();
    for i in 0..point_cloud.len() {
        result.push(Vec::new());
        for _ in 0..point_cloud[i].len() {
            result[i].push(Vec::new());
        }
    }
    for i in 0..cells_list.len() {
        result[cells_list[i].0][cells_list[i].1] = point_cloud[cells_list[i].0][cells_list[i].1].clone();
    }
    let mut mean_density = 0;
    let mut count = 0;
    for i in 0..point_cloud.len() {
        for j in 0..point_cloud[i].len() {
            if point_cloud[i][j].len() > 0 {
                let max_z = point_cloud[i][j].iter().map(|p| p.z).fold(f64::NAN, f64::max);
                let min_z = point_cloud[i][j].iter().map(|p| p.z).fold(f64::NAN, f64::min);
                for x in min_z as i32..=max_z as i32 {
                    let density = point_cloud[i][j].iter().filter          //Number of points in height range
                    (|p| p.z >= x as f64 && p.z < x as f64 + 1.).count();
                    if density > 0 {
                        mean_density += density;
                        count += 1;
                    }
                }
            }
        }
    }
    if count > 0 {
        mean_density = mean_density/count;
        for i in 0..result.len() {
            for j in 0..result[i].len() {
                let max_z = result[i][j].iter().map(|p| p.z).fold(f64::NAN, f64::max);
                let min_z = result[i][j].iter().map(|p| p.z).fold(f64::NAN, f64::min);
                for w in min_z as i32..=max_z as i32 {
                    if result[i][j].iter().filter
                    (|p| p.z >= w as f64 && p.z < w as f64 + 1.).count() > mean_density + density_thres {
                        result[i][j].retain(|x| x.z < w as f64 || x.z >= w as f64 + 1.);
                    }
                }
            }
        }
    }
    result
}