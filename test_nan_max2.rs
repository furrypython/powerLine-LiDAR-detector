fn main() {
    let a = std::f64::NAN;
    let b = 1.0;
    println!("a.max(b) = {}", a.max(b));
    println!("b.max(a) = {}", b.max(a));
}
