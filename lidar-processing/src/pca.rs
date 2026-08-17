use las::Point;
use kdtree::KdTree;
use kdtree::distance::squared_euclidean;
use nalgebra::{Matrix3, SymmetricEigen};

// Computes linearity for a given point cloud.
// Returns a new vector containing only points that have a high linearity score.
pub fn filter_by_linearity(points: &Vec<Point>, k: usize, threshold: f64) -> Vec<Point> {
    if points.is_empty() {
        return Vec::new();
    }

    // Build KD-Tree
    // We use [f64; 3] as the point type and usize as the index
    let mut tree = KdTree::new(3);
    for (i, p) in points.iter().enumerate() {
        tree.add([p.x, p.y, p.z], i).unwrap();
    }

    let mut linear_points = Vec::new();

    // Compute linearity for each point
    for p in points {
        // Find nearest neighbors
        let neighbors = match tree.nearest(&[p.x, p.y, p.z], k, &squared_euclidean) {
            Ok(n) => n,
            Err(_) => continue,
        };
        
        if neighbors.len() < k {
            continue; // Not enough points to form a reliable covariance matrix
        }

        // Calculate the centroid (mean) of the neighbors
        let mut mean_x = 0.0;
        let mut mean_y = 0.0;
        let mut mean_z = 0.0;

        for &(_dist, &idx) in &neighbors {
            let neighbor_p = &points[idx];
            mean_x += neighbor_p.x;
            mean_y += neighbor_p.y;
            mean_z += neighbor_p.z;
        }

        mean_x /= neighbors.len() as f64;
        mean_y /= neighbors.len() as f64;
        mean_z /= neighbors.len() as f64;

        // Calculate the covariance matrix
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
            cov[(2, 0)] += dz * dx;
            cov[(2, 1)] += dz * dy;
            cov[(2, 2)] += dz * dz;
        }

        cov /= neighbors.len() as f64;

        // Compute eigenvalues using SymmetricEigen
        let eig = SymmetricEigen::new(cov);
        
        // Eigenvalues in nalgebra are not guaranteed to be sorted.
        // Let's collect and sort them in descending order.
        let mut evs = [eig.eigenvalues[0], eig.eigenvalues[1], eig.eigenvalues[2]];
        evs.sort_by(|a, b| b.partial_cmp(a).unwrap_or(std::cmp::Ordering::Equal));

        let e1 = evs[0];
        let e2 = evs[1];

        // Linearity calculation: (e1 - e2) / e1
        // If e1 is 0, the points are completely coincident, linearity is 0.
        let linearity = if e1 > 1e-6 {
            (e1 - e2) / e1
        } else {
            0.0
        };

        if linearity >= threshold {
            linear_points.push(p.clone());
        }
    }

    linear_points
}
