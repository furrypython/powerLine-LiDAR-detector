use las::Point;
use kdtree::KdTree;
use kdtree::distance::squared_euclidean;
use nalgebra::{Matrix3, SymmetricEigen};

// Ensemble Filter: Extracts power lines using a combination of geometric metrics.
// - Detector A: Main Wire (Linearity via PCA with radius search)
// - Detector B: Thin Structure (Variance threshold)
// - Detector C: Orientation (Horizontal check)
// - Detector D: Junctions (Planarity/Scattering + Proximity to Main Wires)
pub fn extract_powerlines(points: &Vec<Point>, radius: f64, linearity_threshold: f64) -> Vec<Point> {
    if points.is_empty() {
        return Vec::new();
    }

    let mut tree = KdTree::new(3);
    for (i, p) in points.iter().enumerate() {
        tree.add([p.x, p.y, p.z], i).unwrap();
    }

    let radius_sq = radius * radius;
    let mut main_wires = Vec::new();
    let mut junction_candidates = Vec::new();

    for p in points.iter() {
        let neighbors = match tree.within(&[p.x, p.y, p.z], radius_sq, &squared_euclidean) {
            Ok(n) => n,
            Err(_) => continue,
        };
        
        // Need enough points to form a reliable covariance matrix
        if neighbors.len() < 5 {
            continue; 
        }

        let mut mean_x = 0.0;
        let mut mean_y = 0.0;
        let mut mean_z = 0.0;

        for &(_dist, &idx) in &neighbors {
            let neighbor_p = &points[idx];
            mean_x += neighbor_p.x;
            mean_y += neighbor_p.y;
            mean_z += neighbor_p.z;
        }

        let n_f64 = neighbors.len() as f64;
        mean_x /= n_f64;
        mean_y /= n_f64;
        mean_z /= n_f64;

        let mut cov = Matrix3::zeros();
        for &(_dist, &idx) in &neighbors {
            let np = &points[idx];
            let dx = np.x - mean_x;
            let dy = np.y - mean_y;
            let dz = np.z - mean_z;

            cov[(0, 0)] += dx * dx;
            cov[(0, 1)] += dx * dy;
            cov[(0, 2)] += dx * dz;
            cov[(1, 0)] += dy * dx;
            cov[(1, 1)] += dy * dy;
            cov[(1, 2)] += dy * dz;
        }
        cov[(2, 0)] = cov[(0, 2)];
        cov[(2, 1)] = cov[(1, 2)];

        cov /= n_f64;

        let eig = SymmetricEigen::new(cov);
        
        // Pair eigenvalues with their corresponding eigenvectors and sort descending
        let mut eigen_pairs = vec![
            (eig.eigenvalues[0], eig.eigenvectors.column(0).into_owned()),
            (eig.eigenvalues[1], eig.eigenvectors.column(1).into_owned()),
            (eig.eigenvalues[2], eig.eigenvectors.column(2).into_owned()),
        ];
        
        eigen_pairs.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal));

        let e1 = eigen_pairs[0].0;
        let e2 = eigen_pairs[1].0;
        let e3 = eigen_pairs[2].0;
        let v1 = &eigen_pairs[0].1;

        if e1 < 1e-6 {
            continue;
        }

        let linearity = (e1 - e2) / e1;
        let planarity = (e2 - e3) / e1;
        let scattering = e3 / e1;

        // Detector B: Thin Structure Filter
        // e2 is the variance perpendicular to the main axis.
        // A variance of 0.02 corresponds to a standard deviation of ~0.14m.
        let max_thickness_variance = 0.02;

        // Detector C: Orientation Filter
        // v1.z is the cosine of the angle with the Z-axis.
        // |v1.z| < 0.75 means the angle is > ~41 degrees from vertical (mostly horizontal).
        let is_horizontal = v1.z.abs() < 0.75; 

        // Detector A: Main Wire
        if linearity >= linearity_threshold && e2 < max_thickness_variance && is_horizontal {
            main_wires.push(p.clone());
        } 
        // Detector D: Junction Candidates
        else if planarity > 0.4 || scattering > 0.4 {
            junction_candidates.push(p.clone());
        }
    }

    // Ensemble Merger: Combine Main Wires and Validated Junctions
    let mut final_points = main_wires.clone();
    
    if !main_wires.is_empty() && !junction_candidates.is_empty() {
        let mut wire_tree = KdTree::new(3);
        for (i, p) in main_wires.iter().enumerate() {
            wire_tree.add([p.x, p.y, p.z], i).unwrap();
        }

        let proximity_sq = 0.5 * 0.5; // 0.5 meters proximity check
        for p in junction_candidates {
            if let Ok(neighbors) = wire_tree.within(&[p.x, p.y, p.z], proximity_sq, &squared_euclidean) {
                if !neighbors.is_empty() {
                    final_points.push(p);
                }
            }
        }
    }

    final_points
}
