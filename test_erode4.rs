fn main() {
    let size = 20;
    let mut padded = vec![vec![false; size]; size];
    
    // Draw a line with slope 0.5
    for x in 0..size {
        let y = x / 2;
        padded[y][x] = true;
    }
    
    let mut eroded = vec![vec![false; size]; size];
    
    for i in 1..size-1 {
        for j in 1..size-1 {
            if padded[i][j] {
                let min_neighbours = if i == 1 || j == 1 || i == size-2 || j == size-2 { 1 } else { 2 };
                let count = (padded[i+1][j] as i32) + (padded[i-1][j] as i32) 
                    + (padded[i][j+1] as i32) + (padded[i][j-1] as i32) 
                    + (padded[i+1][j+1] as i32) + (padded[i-1][j-1] as i32) 
                    + (padded[i-1][j+1] as i32) + (padded[i+1][j-1] as i32);
                if count >= min_neighbours {
                    eroded[i][j] = true;
                }
            }
        }
    }
    
    let orig_count = padded.iter().flatten().filter(|&&x| x).count();
    let count = eroded.iter().flatten().filter(|&&x| x).count();
    println!("Original: {}, Eroded4 kept: {}", orig_count, count);
}
