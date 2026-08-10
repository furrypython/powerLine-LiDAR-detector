fn main() {
    let max_z = vec![1.0, 2.0, 3.0].into_iter().fold(std::f64::NAN, f64::max);
    println!("max_z: {}", max_z);
}
