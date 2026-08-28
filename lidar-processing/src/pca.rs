use las::Point;
use kdtree::KdTree;
use kdtree::distance::squared_euclidean;
use nalgebra::{Matrix3, SymmetricEigen};
use std::collections::HashSet;

// Ensemble Filter: Extracts power lines using a combination of geometric metrics.
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
    let mut main_wire_indices = HashSet::new();
    let mut rejected_indices = Vec::new();

    // ----------------------------------------------------------------
    // PASS 1: Main Wires (Detector A, B, C)
    // ----------------------------------------------------------------
    for (i, p) in points.iter().enumerate() {
        let neighbors = match tree.within(&[p.x, p.y, p.z], radius_sq, &squared_euclidean) {
            Ok(n) => n,
            Err(_) => continue,
        };
        
        // Lowered minimum points to 3 to capture sparse upper catenary wires
        if neighbors.len() < 3 {
            rejected_indices.push(i);
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
        
        let mut eigen_pairs = vec![
            (eig.eigenvalues[0], eig.eigenvectors.column(0).into_owned()),
            (eig.eigenvalues[1], eig.eigenvectors.column(1).into_owned()),
            (eig.eigenvalues[2], eig.eigenvectors.column(2).into_owned()),
        ];
        
        eigen_pairs.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal));

        let e1 = eigen_pairs[0].0;
        let e2 = eigen_pairs[1].0;
        let v1 = &eigen_pairs[0].1;

        if e1 < 1e-6 {
            rejected_indices.push(i);
            continue;
        }

        let linearity = (e1 - e2) / e1;

        // Detector B: Thin Structure Filter
        let max_thickness_variance = 0.02;

        // Detector C: Orientation Filter
        let is_horizontal = v1.z.abs() < 0.75; 

        // Detector A: Main Wire
        if linearity >= linearity_threshold && e2 < max_thickness_variance && is_horizontal {
            main_wires.push(p.clone());
            main_wire_indices.insert(i);
        } else {
            rejected_indices.push(i);
        }
    }

    // ----------------------------------------------------------------
    // PASS 2: Droppers (Detector F - Hybrid Approach)
    // ----------------------------------------------------------------
    let mut droppers = Vec::new();
    let mut dropper_indices = HashSet::new();
    let mut remaining_rejected = Vec::new();
    
    if !main_wires.is_empty() {
        let mut wire_tree = KdTree::new(3);
        for (i, p) in main_wires.iter().enumerate() {
            wire_tree.add([p.x, p.y, p.z], i).unwrap();
        }
        
        // Find seeds: rejected points within 0.5m of a main wire
        let seed_proximity_sq = 0.5 * 0.5;
        let mut seeds = Vec::new();
        for &idx in &rejected_indices {
            let p = &points[idx];
            if let Ok(neighbors) = wire_tree.within(&[p.x, p.y, p.z], seed_proximity_sq, &squared_euclidean) {
                if !neighbors.is_empty() {
                    seeds.push(idx);
                }
            }
        }
        
        // Project spatial cylinder vertically from these seeds
        let mut xy_tree = KdTree::new(2);
        for &idx in &rejected_indices {
            let p = &points[idx];
            xy_tree.add([p.x, p.y], idx).unwrap();
        }
        
        let cylinder_radius_sq = 0.5 * 0.5;
        let mut dropper_candidates = HashSet::new();
        
        for &seed_idx in &seeds {
            let seed_p = &points[seed_idx];
            if let Ok(neighbors) = xy_tree.within(&[seed_p.x, seed_p.y], cylinder_radius_sq, &squared_euclidean) {
                // Density check: if < 200 points in this vertical cylinder, it's a dropper, not a mast
                if neighbors.len() < 200 {
                    for &(_dist, &idx) in &neighbors {
                        dropper_candidates.insert(idx);
                    }
                }
            }
        }
        
        for &idx in &dropper_candidates {
            droppers.push(points[idx].clone());
            dropper_indices.insert(idx);
        }
        
        for &idx in &rejected_indices {
            if !dropper_indices.contains(&idx) {
                remaining_rejected.push(idx);
            }
        }
    } else {
        remaining_rejected = rejected_indices;
    }

    // ----------------------------------------------------------------
    // PASS 3: Hardware & Junctions (Detector D)
    // ----------------------------------------------------------------
    let mut junctions = Vec::new();
    if !main_wires.is_empty() || !droppers.is_empty() {
        let mut combined_tree = KdTree::new(3);
        let mut combined_idx = 0;
        for p in &main_wires {
            combined_tree.add([p.x, p.y, p.z], combined_idx).unwrap();
            combined_idx += 1;
        }
        for p in &droppers {
            combined_tree.add([p.x, p.y, p.z], combined_idx).unwrap();
            combined_idx += 1;
        }
        
        // Tightened proximity to 0.15m to avoid bleeding into masts
        let junction_proximity_sq = 0.15 * 0.15; 
        
        for &idx in &remaining_rejected {
            let p = &points[idx];
            
            let neighbors = match tree.within(&[p.x, p.y, p.z], radius_sq, &squared_euclidean) {
                Ok(n) => n,
                Err(_) => continue,
            };
            
            if neighbors.len() < 3 { continue; }
            
            let mut mean_x = 0.0;
            let mut mean_y = 0.0;
            let mut mean_z = 0.0;

            for &(_dist, &n_idx) in &neighbors {
                let np = &points[n_idx];
                mean_x += np.x;
                mean_y += np.y;
                mean_z += np.z;
            }

            let n_f64 = neighbors.len() as f64;
            mean_x /= n_f64;
            mean_y /= n_f64;
            mean_z /= n_f64;

            let mut cov = Matrix3::zeros();
            for &(_dist, &n_idx) in &neighbors {
                let np = &points[n_idx];
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
            let e2 = evs[1];
            let e3 = evs[2];

            if e1 < 1e-6 { continue; }

            let planarity = (e2 - e3) / e1;
            let scattering = e3 / e1;

            if planarity > 0.4 || scattering > 0.4 {
                if let Ok(close_wires) = combined_tree.within(&[p.x, p.y, p.z], junction_proximity_sq, &squared_euclidean) {
                    if !close_wires.is_empty() {
                        junctions.push(p.clone());
                    }
                }
            }
        }
    }

    // Combine all valid points
    let mut merged_points = main_wires;
    merged_points.extend(droppers);
    merged_points.extend(junctions);

    if merged_points.is_empty() {
        return merged_points;
    }

    // ----------------------------------------------------------------
    // POST-PROCESSING: Detector E (Length / Continuity Filter)
    // ----------------------------------------------------------------
    let mut merged_tree = KdTree::new(3);
    for (i, p) in merged_points.iter().enumerate() {
        merged_tree.add([p.x, p.y, p.z], i).unwrap();
    }

    let mut visited = vec![false; merged_points.len()];
    let mut final_points = Vec::new();
    let cluster_radius_sq = 1.0 * 1.0; // 1.0m to connect adjacent points in a wire

    for i in 0..merged_points.len() {
        if visited[i] { continue; }

        let mut cluster = Vec::new();
        let mut stack = vec![i];
        visited[i] = true;

        let mut min_x = merged_points[i].x;
        let mut max_x = merged_points[i].x;
        let mut min_y = merged_points[i].y;
        let mut max_y = merged_points[i].y;
        let mut min_z = merged_points[i].z;
        let mut max_z = merged_points[i].z;

        while let Some(curr_idx) = stack.pop() {
            let p = &merged_points[curr_idx];
            cluster.push(p.clone());

            if p.x < min_x { min_x = p.x; }
            if p.x > max_x { max_x = p.x; }
            if p.y < min_y { min_y = p.y; }
            if p.y > max_y { max_y = p.y; }
            if p.z < min_z { min_z = p.z; }
            if p.z > max_z { max_z = p.z; }

            if let Ok(neighbors) = merged_tree.within(&[p.x, p.y, p.z], cluster_radius_sq, &squared_euclidean) {
                for &(_dist, &n_idx) in &neighbors {
                    if !visited[n_idx] {
                        visited[n_idx] = true;
                        stack.push(n_idx);
                    }
                }
            }
        }

        // Calculate the bounding box diagonal length of the cluster
        let dx = max_x - min_x;
        let dy = max_y - min_y;
        let dz = max_z - min_z;
        let length = (dx * dx + dy * dy + dz * dz).sqrt();

        // If the cluster is at least 5.0 meters long, it's a valid wire structure
        if length >= 5.0 {
            final_points.extend(cluster);
        }
    }

    final_points
}
