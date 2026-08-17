fn main() {
    let size = 30;
    let mut padded = vec![vec![false; size]; size];
    
    // Draw a horizontal line of length 6
    for x in 10..16 {
        padded[15][x] = true;
    }
    
    let mut dilated = padded.clone();
    
    for i in 1..size-1 {
        for j in 1..size-1 {
            if padded[i][j] {
                dilated[i][j] = true;
                continue;
            }
            
            let mut fill_gap = false;
            if padded[i-1][j] && padded[i+1][j] { fill_gap = true; }
            else if padded[i][j-1] && padded[i][j+1] { fill_gap = true; }
            else if padded[i-1][j-1] && padded[i+1][j+1] { fill_gap = true; }
            else if padded[i-1][j+1] && padded[i+1][j-1] { fill_gap = true; }

            if fill_gap {
                dilated[i][j] = true;
            }
        }
    }
    
    let count = dilated.iter().flatten().filter(|&&x| x).count();
    println!("Dilated kept: {} cells", count);
}
