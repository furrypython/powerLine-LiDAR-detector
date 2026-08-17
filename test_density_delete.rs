fn main() {
    let mean_density = 3000;
    let density_thres = 100;
    let mut cell_points = vec![10.5, 10.6, 10.7, 11.5, 11.6]; // z values
    
    let w = 10;
    let slice_density = 5000; // Simulate high density
    
    if slice_density > mean_density + density_thres {
        cell_points.retain(|&z| z > w as f64 + 1.0);
    }
    
    println!("Remaining points: {:?}", cell_points);
}
