use las::{Read, Reader};
use std::env;

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() > 1 {
        let mut reader = Reader::from_path(&args[1]).unwrap();
        let header = reader.header();
        println!("Bounds: {:?}", header.bounds());
        let min_x = header.bounds().min.x;
        let min_y = header.bounds().min.y;
        let max_x = header.bounds().max.x;
        let max_y = header.bounds().max.y;
        println!("min_x: {}, max_x: {}, diff: {}", min_x, max_x, max_x - min_x);
        println!("min_y: {}, max_y: {}, diff: {}", min_y, max_y, max_y - min_y);
    }
}
