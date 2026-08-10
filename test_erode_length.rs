fn main() {
    let padding = 5;
    let size = 30;
    let mut padded = vec![vec![false; size]; size];
    
    // Draw a horizontal line of length 15
    for x in 10..25 {
        padded[15][x] = true;
    }
    
    let mut eroded = vec![vec![false; size]; size];
    
    for i in padding..size-padding {
        for j in padding..size-padding {
            if padded[i][j] {
                let mut neighbours_per_level = vec![0; padding];
                let mut has_neighbour = false;
                for k in i-padding..i {
                    for l in j-padding..j+padding+1 {
                        if padded[k][l] { neighbours_per_level[k-(i-padding)] += 1; }
                        if padded[i+padding-(i-k)][l] { neighbours_per_level[k-(i-padding)] += 1; }
                    }
                }
                for x in 0..padding {
                    if neighbours_per_level[x] >= 2 { has_neighbour = true; }
                    else { has_neighbour = false; break; }
                }
                if !has_neighbour {
                    neighbours_per_level = vec![0; padding];
                    for l in j-padding..j {
                        for k in i-padding..i+padding+1 {
                            if padded[k][l] { neighbours_per_level[l-(j-padding)] += 1; }
                            if padded[k][j+padding-(j-l)] { neighbours_per_level[l-(j-padding)] += 1; }
                        }
                    }
                    for x in 0..padding {
                        if neighbours_per_level[x] >= 2 { has_neighbour = true; }
                        else { has_neighbour = false; break; }
                    }
                }
                eroded[i][j] = has_neighbour;
            }
        }
    }
    
    let count = eroded.iter().flatten().filter(|&&x| x).count();
    println!("Eroded kept: {} cells", count);
}
