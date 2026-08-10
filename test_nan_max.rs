fn main() {
    let max = vec![1.0, 2.0, 3.0].into_iter().fold(std::f64::NAN, |a,b| a.max(b));
    println!("Max is: {}", max);
}
