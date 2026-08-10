fn main() {
    let padding = 5;
    let size = 20;
    let mut padded = vec![vec![false; size]; size];
    
    // Draw a diagonal line
    for i in 0..size {
        padded[i][i] = true;
    }
    
    let i = 10;
    let j = 10;
    
    let mut neighbours_per_level = vec![0; padding];
    for k in i-padding..i {
        for l in j-padding..j+padding+1 {
            if padded[k][l] == true  {
                neighbours_per_level[k-(i-padding)] += 1;
            }
            if padded[i+padding-(i-k)][l] == true {
                neighbours_per_level[k-(i-padding)] += 1;
            }
        }
    }
    println!("Vertical check (rows): {:?}", neighbours_per_level);
    
    let mut neighbours_per_level_h = vec![0; padding];
    for l in j-padding..j {
        // Wait, the original code for horizontal check:
        // for l in j-padding..j {
        //     if padded[i][l] == true { ... }
        //     if padded[i][j+padding-(j-l)] == true { ... }
        // }
        // Wait! The original code for horizontal check ONLY checks row `i`!
        // Let's look at the code:
        // for l in j-padding..j {
        //     if padded[i][l] == true  {
        //         neighbours_per_level[l-(j-padding)] += 1;
        //     }
        //     if padded[i][j+padding-(j-l)] == true  {
        //         neighbours_per_level[l-(j-padding)] += 1;
        //     }
        // }
        // It ONLY checks row `i`! That means it only checks if there is a perfectly horizontal line on row `i`!
        // It doesn't check `k` from `i-padding` to `i+padding`!
    }
}
