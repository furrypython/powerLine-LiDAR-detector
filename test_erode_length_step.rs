fn main() {
    let padding = 5;
    let size = 20;
    let mut padded = vec![vec![false; size]; size];
    
    // Draw a line with slope 0.2
    for x in 0..size {
        let y = x / 5;
        padded[y][x] = true;
    }
    
    let i = 2; // Let's change this to 10 for the test but wait, the line is at y = 0..4
    // Let's shift the line down to y = 10
    for x in 0..size {
        let y = 10 + x / 5;
        padded[y][x] = true;
    }
    let i = 12;
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
        if padded[i][l] == true  {
            neighbours_per_level_h[l-(j-padding)] += 1;
        }
        if padded[i][j+padding-(j-l)] == true  {
            neighbours_per_level_h[l-(j-padding)] += 1;
        }
    }
    println!("Horizontal check (columns): {:?}", neighbours_per_level_h);
}
