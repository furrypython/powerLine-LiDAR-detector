fn main() {
    let max1 = f64::max(std::f64::NAN, 1.0);
    let max2 = f64::max(1.0, std::f64::NAN);
    println!("max(NaN, 1.0) = {}", max1);
    println!("max(1.0, NaN) = {}", max2);
}
