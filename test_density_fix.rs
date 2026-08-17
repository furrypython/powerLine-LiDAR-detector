fn main() {
    // Simulate calculating mean_density on ALL filtered cells vs only result cells
    let power_line_densities = vec![1000, 1200, 900, 1100, 1300];
    let tree_densities = vec![5000, 6000, 5500];
    
    // Original logic: mean_density on result (only power lines survived morph)
    let mean_result: i32 = power_line_densities.iter().sum::<i32>() / power_line_densities.len() as i32;
    println!("Mean density (result only): {}", mean_result);
    
    let mut deleted_power_lines = 0;
    for &d in &power_line_densities {
        if d > mean_result + 100 {
            deleted_power_lines += 1;
        }
    }
    println!("Deleted power lines (original logic): {}", deleted_power_lines);
    
    // Fixed logic: mean_density on ALL filtered cells
    let mut all_densities = power_line_densities.clone();
    all_densities.extend(tree_densities.iter());
    let mean_all: i32 = all_densities.iter().sum::<i32>() / all_densities.len() as i32;
    println!("Mean density (all filtered): {}", mean_all);
    
    let mut deleted_power_lines_fix = 0;
    for &d in &power_line_densities {
        if d > mean_all + 100 {
            deleted_power_lines_fix += 1;
        }
    }
    println!("Deleted power lines (fixed logic): {}", deleted_power_lines_fix);
}
