fn main() {
    let padding = 5;
    let size = 50;
    let mut padded = vec![vec![false; size]; size];
    
    // Draw a line with slope 0.2
    for x in 0..size {
        let y = 10 + x / 5;
        padded[y][x] = true;
    }
    
    let mut eroded_bug = vec![vec![false; size]; size];
    let mut eroded_fix = vec![vec![false; size]; size];
    
    for i in padding..size-padding {
        for j in padding..size-padding {
            if padded[i][j] {
                // BUGGY VERSION
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
                        if padded[i][l] { neighbours_per_level[l-(j-padding)] += 1; }
                        if padded[i][j+padding-(j-l)] { neighbours_per_level[l-(j-padding)] += 1; }
                    }
                    for x in 0..padding {
                        if neighbours_per_level[x] >= 2 { has_neighbour = true; }
                        else { has_neighbour = false; break; }
                    }
                }
                eroded_bug[i][j] = has_neighbour;
                
                // FIXED VERSION
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
                eroded_fix[i][j] = has_neighbour;
            }
        }
    }
    
    let bug_count = eroded_bug.iter().flatten().filter(|&&x| x).count();
    let fix_count = eroded_fix.iter().flatten().filter(|&&x| x).count();
    
    println!("Buggy erosion kept: {} cells", bug_count);
    println!("Fixed erosion kept: {} cells", fix_count);
}
