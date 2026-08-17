fn main() {
    let max_z = vec![1.0, 2.0, 3.0].into_iter().fold(std::f64::NAN, f64::max);
    println!("max_z: {}", max_z);
    
    // Wait, what if the vector is Point structs?
    struct Point { z: f64 }
    let pts = vec![Point{z: 1.0}, Point{z: 2.0}, Point{z: 3.0}];
    let max_z_pts = pts.iter().map(|p| p.z).fold(std::f64::NAN, f64::max);
    println!("max_z_pts: {}", max_z_pts);
}
