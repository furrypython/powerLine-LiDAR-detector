fn main() {
    let size = 20;
    let mut padded = vec![vec![false; size]; size];
    
    // Draw a horizontal line with a 1-cell gap
    for x in 5..10 { padded[10][x] = true; }
    // gap at x=10
    for x in 11..16 { padded[10][x] = true; }
    
    let mut eroded = vec![vec![false; size]; size];
    
    for i in 1..size-1 {
        for j in 1..size-1 {
            if padded[i][j] {
                let count = (padded[i+1][j] as i32) + (padded[i-1][j] as i32) 
                    + (padded[i][j+1] as i32) + (padded[i][j-1] as i32) 
                    + (padded[i+1][j+1] as i32) + (padded[i-1][j-1] as i32) 
                    + (padded[i-1][j+1] as i32) + (padded[i+1][j-1] as i32);
                if count >= 2 {
                    eroded[i][j] = true;
                }
            }
        }
    }
    
    // Print the line
    for x in 5..16 {
        print!("{}", if eroded[10][x] { "X" } else { "." });
    }
    println!("");
}
