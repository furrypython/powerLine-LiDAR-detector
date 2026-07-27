use las::{Read, Reader};
use std::env;

fn main() {
    let input = env::args().nth(1).unwrap_or("/Users/omori_rei/workspace/dataset/senro_pl/no_grounds/G_senro.las".to_string());
    let mut reader = match Reader::from_path(&input) {
        Ok(r) => r,
        Err(e) => {
            println!("Error opening {}: {:?}", input, e);
            return;
        }
    };
    let header = reader.header();
    let min_x = header.bounds().min.x;
    let min_y = header.bounds().min.y;
    let max_x = header.bounds().max.x;
    let max_y = header.bounds().max.y;
    
    let mut actual_min_x = std::f64::MAX;
    let mut actual_min_y = std::f64::MAX;
    let mut actual_max_x = std::f64::MIN;
    let mut actual_max_y = std::f64::MIN;
    
    let mut out_of_bounds = 0;
    let mut total = 0;
    
    for p in reader.points() {
        let p = p.unwrap();
        total += 1;
        if p.x < min_x || p.x > max_x || p.y < min_y || p.y > max_y {
            out_of_bounds += 1;
        }
        if p.x < actual_min_x { actual_min_x = p.x; }
        if p.x > actual_max_x { actual_max_x = p.x; }
        if p.y < actual_min_y { actual_min_y = p.y; }
        if p.y > actual_max_y { actual_max_y = p.y; }
    }
    
    println!("Header X: {} to {} (diff {})", min_x, max_x, max_x - min_x);
    println!("Header Y: {} to {} (diff {})", min_y, max_y, max_y - min_y);
    println!("Actual X: {} to {} (diff {})", actual_min_x, actual_max_x, actual_max_x - actual_min_x);
    println!("Actual Y: {} to {} (diff {})", actual_min_y, actual_max_y, actual_max_y - actual_min_y);
    println!("Out of bounds: {} / {}", out_of_bounds, total);
}
