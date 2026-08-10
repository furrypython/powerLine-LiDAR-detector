fn empty_vector(num: usize) -> Vec<bool> { vec![false; num] }
fn create_padded_point_cloud(point_cloud: &Vec<Vec<bool>>, padding: usize) -> Vec<Vec<bool>> {
    let mut padded_point_cloud = Vec::new();
    let empty_cells = empty_vector(padding);
    let cols = if point_cloud.is_empty() { 0 } else { point_cloud[0].len() };

    for _ in 0..padding { padded_point_cloud.push(vec![false; cols + padding*2]); }
    for i in 0..point_cloud.len(){
        let mut row = empty_cells.clone();
        row.extend_from_slice(&point_cloud[i]);
        row.extend_from_slice(&empty_cells);
        padded_point_cloud.push(row);
    }
    for _ in 0..padding { padded_point_cloud.push(vec![false; cols + padding*2]); }
    padded_point_cloud
}

fn erode_neighborhood(point_cloud: &Vec<Vec<bool>>, padding: usize) -> Vec<Vec<bool>> {
    let mut eroded = point_cloud.clone();
    let padded = create_padded_point_cloud(&eroded, padding);
    for i in padding..padded.len() - padding {
        for j in padding..padded[i].len() - padding {
            let mut neighbours_per_level = vec![0; padding];
            let mut has_neighbour = false;
            if padded[i][j] {
                let border_cell = if i == padding || j == padding || i == (padded.len() - (padding + 1)) || j == (padded[i].len() - (padding + 1)) { true } else { false };
                for k in i-padding..i {
                    for l in j-padding..j+padding+1 {
                        if padded[k][l] { neighbours_per_level[k-(i-padding)] += 1; }
                        if padded[i+padding-(i-k)][l] { neighbours_per_level[k-(i-padding)] += 1; }
                    }
                }
                for x in 0..neighbours_per_level.len() {
                    if border_cell && neighbours_per_level[x] >= 1 { has_neighbour = true; }
                    else if !border_cell && i-x >= padding && neighbours_per_level[x] >= 1 { has_neighbour = true; }
                    else if !border_cell && neighbours_per_level[x] >= 2 { has_neighbour = true; }
                    else { has_neighbour = false; break; }
                }
                if !has_neighbour {
                    for x in 0..neighbours_per_level.len() { neighbours_per_level[x] = 0; }
                    for l in j-padding..j {
                        if padded[i][l] { neighbours_per_level[l-(j-padding)] += 1; }
                        if padded[i][j+padding-(j-l)] { neighbours_per_level[l-(j-padding)] += 1; }
                    }
                    for x in 0..neighbours_per_level.len() {
                        if border_cell && neighbours_per_level[x] >= 1 { has_neighbour = true; }
                        else if !border_cell && i-x >= padding && neighbours_per_level[x] >= 1 { has_neighbour = true; }
                        else if !border_cell && neighbours_per_level[x] >= 2 { has_neighbour = true; }
                        else { has_neighbour = false; break; }
                    }
                }
                if has_neighbour { eroded[i-padding][j-padding] = true } else { eroded[i-padding][j-padding] = false }
            }
        }
    }
    eroded
}


fn print_pc(pc: &Vec<Vec<bool>>) {
    for i in 0..20 {
        for j in 0..20 {
            if pc[i][j] { print!("1"); } else { print!("."); }
        }
        println!("");
    }
}

fn main() {
    let size = 20;
    let mut pc = vec![vec![false; size]; size];
    for i in 5..13 { pc[i][10] = true; }
    let eroded = erode_neighborhood(&pc, 5);
    for i in 0..20 {
        for j in 0..20 { if eroded[i][j] { print!("1"); } else { print!("."); } }
        println!();
    }
}
