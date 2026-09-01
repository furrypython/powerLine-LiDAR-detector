use las::Point;
use kdtree::KdTree;
use kdtree::distance::squared_euclidean;
use nalgebra::{Matrix3, SymmetricEigen, Vector3};
use rand::{thread_rng, seq::SliceRandom, Rng};

/// RANSAC based Macro-Plane Detection
/// Removes massive flat surfaces (buildings, walls, roofs)
pub fn remove_macro_planes(points: &Vec<Point>, distance_threshold: f64, min_inliers: usize, iterations: usize) -> Vec<Point> {
    if points.len() < 3 {
        return points.clone();
    }

    let mut remaining_points = points.clone();
    let mut rng = thread_rng();

    loop {
        if remaining_points.len() < min_inliers {
            break;
        }

        let mut best_inliers = Vec::new();
        let mut best_inlier_count = 0;

        for _ in 0..iterations {
            // Randomly select 3 points
            let sample: Vec<&Point> = remaining_points.choose_multiple(&mut rng, 3).collect();
            if sample.len() < 3 { break; }

            let p1 = Vector3::new(sample[0].x, sample[0].y, sample[0].z);
            let p2 = Vector3::new(sample[1].x, sample[1].y, sample[1].z);
            let p3 = Vector3::new(sample[2].x, sample[2].y, sample[2].z);

            let v1 = p2 - p1;
            let v2 = p3 - p1;
            let normal = v1.cross(&v2);

            // Check if points are collinear (normal is zero)
            if normal.norm() < 1e-6 {
                continue;
            }
            let normal = normal.normalize();

            // Plane equation: ax + by + cz + d = 0
            let d = -normal.dot(&p1);

            let mut current_inliers = Vec::new();
            let mut current_count = 0;

            for (i, p) in remaining_points.iter().enumerate() {
                let pt = Vector3::new(p.x, p.y, p.z);
                let distance = (normal.dot(&pt) + d).abs();
                
                if distance < distance_threshold {
                    current_inliers.push(i);
                    current_count += 1;
                }
            }

            if current_count > best_inlier_count {
                best_inlier_count = current_count;
                best_inliers = current_inliers;
            }
        }

        if best_inlier_count >= min_inliers {
            // Remove the inliers from remaining_points (working backwards to preserve indices)
            best_inliers.sort_unstable();
            for &idx in best_inliers.iter().rev() {
                remaining_points.swap_remove(idx);
            }
        } else {
            break; // No more massive planes found
        }
    }

    remaining_points
}

/// Volumetric Vegetation Filter
/// Removes fluffy, volumetric points like trees using PCA scattering
pub fn remove_vegetation(points: &Vec<Point>, radius: f64, scattering_threshold: f64) -> Vec<Point> {
    if points.is_empty() {
        return Vec::new();
    }

    let mut tree = KdTree::new(3);
    for (i, p) in points.iter().enumerate() {
        tree.add([p.x, p.y, p.z], i).unwrap();
    }

    let radius_sq = radius * radius;
    let mut filtered_points = Vec::new();

    for p in points.iter() {
        let neighbors = match tree.within(&[p.x, p.y, p.z], radius_sq, &squared_euclidean) {
            Ok(n) => n,
            Err(_) => continue,
        };
        
        if neighbors.len() < 5 {
            filtered_points.push(p.clone());
            continue; 
        }

        let mut mean_x = 0.0;
        let mut mean_y = 0.0;
        let mut mean_z = 0.0;

        for &(_dist, &idx) in &neighbors {
            let np = &points[idx];
            mean_x += np.x;
            mean_y += np.y;
            mean_z += np.z;
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
        let mut evs = [eig.eigenvalues[0], eig.eigenvalues[1], eig.eigenvalues[2]];
        evs.sort_by(|a, b| b.partial_cmp(a).unwrap_or(std::cmp::Ordering::Equal));

        let e1 = evs[0];
        let e3 = evs[2];

        if e1 < 1e-6 {
            filtered_points.push(p.clone());
            continue;
        }

        let scattering = e3 / e1;

        // If scattering is high, it's volumetric (vegetation). Only keep if it's low.
        if scattering <= scattering_threshold {
            filtered_points.push(p.clone());
        }
    }

    filtered_points
}
