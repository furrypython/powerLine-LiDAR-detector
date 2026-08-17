fn main() {
    let min_density = 2;
    let empty = 2;
    let non_empty_cont = 4;
    let non_empty = 7;
    let end = 30;
    
    // Simulate a cell with a pylon (continuous points from z=20 down to z=0)
    let max_z = 20.0;
    let min_z = 0.0;
    
    let mut candidate = false;
    let mut count_non_empty = 0;
    let mut count_empty = 0;
    let mut total_count_non_empty = 0;
    
    for i in 0..end {
        if (max_z - (i as f64)) > min_z {
            let density = 5; // Pylon has high density everywhere
            
            if density > min_density {
                count_non_empty += 1;
                total_count_non_empty += 1;
                count_empty = 0;
            } else {
                count_empty += 1;
                count_non_empty = 0;
            }

            if count_non_empty >= non_empty_cont || total_count_non_empty >= non_empty {
                println!("Broke due to non_empty_cont at i={}", i);
                break;
            }

            if count_empty >= empty {
                candidate = true;
                break;
            }
        } else {
            break;
        }
    }
    println!("Pylon cell candidate: {}", candidate);
}
