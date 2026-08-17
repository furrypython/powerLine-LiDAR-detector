use std::f64;

#[derive(Clone)]
struct Point { x: f64, y: f64, z: f64 }

fn process_cell(point_cell: &Vec<Point>, min_density: usize, empty:usize , non_empty_cont:usize, non_empty: usize, end: usize) -> Vec<Point> {
    let mut cell = Vec::new();
    let mut candidate = false;
    let mut count_non_empty = 0;
    let mut count_empty = 0;
    let mut total_count_non_empty = 0;
    
    let max_z = point_cell.iter().map(|p| p.z).fold(std::f64::NAN, |a,b| a.max(b));
    let min_z = point_cell.iter().map(|p| p.z).fold(std::f64::NAN, |a,b| a.min(b));
    for i in 0..end {
        if (max_z - (i as f64)) > min_z{
            let density = point_cell.iter().filter
                    (|p| p.z <= max_z - (i as f64) && p.z > max_z - (i as f64 + 1.)).count();
            
            if density > min_density {
                count_non_empty += 1;
                total_count_non_empty += 1;
                count_empty = 0;
            } else {
                count_empty += 1;
                count_non_empty = 0;
            }

            if count_non_empty >= non_empty_cont || total_count_non_empty >= non_empty {
                break;
            }

            if count_empty >= empty {
                candidate = true;
                break;
            }
        }
        else {
            break;
        }
    }

    if candidate {
        for point in point_cell.iter().filter(|p| p.z > max_z - 20.){
            cell.push(point.clone());
        }
    }
    cell
}

fn main() {
    let mut points = Vec::new();
    // ONLY Ground points at z=0 to z=1 (20 points)
    for i in 0..20 { points.push(Point { x: 0., y: 0., z: (i as f64) * 0.05 }); }
    
    let res = process_cell(&points, 6, 4, 4, 7, 30);
    println!("Candidate with ONLY ground points: {} points", res.len());
    
    let mut points2 = Vec::new();
    // Trees at z=0 to z=10 (100 points)
    for i in 0..100 { points2.push(Point { x: 0., y: 0., z: (i as f64) * 0.1 }); }
    
    let res2 = process_cell(&points2, 6, 4, 4, 7, 30);
    println!("Candidate with trees: {} points", res2.len());
}
