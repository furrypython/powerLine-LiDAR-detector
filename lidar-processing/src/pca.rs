use las::Point;
use kdtree::KdTree;
use kdtree::distance::squared_euclidean;
use nalgebra::{Matrix3, SymmetricEigen, Vector3};
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
    let mut candidate_main_wires = Vec::new();
    let mut candidate_main_wire_indices = HashSet::new();
    let mut wire_directions = Vec::new();
    let mut rejected_indices = Vec::new();

    // ----------------------------------------------------------------
    // PASS 1: Base Wire Detection (Detector A, B, C)
    // ----------------------------------------------------------------
    for (i, p) in points.iter().enumerate() {
        let neighbors = match tree.within(&[p.x, p.y, p.z], radius_sq, &squared_euclidean) {
            Ok(n) => n,
            Err(_) => continue,
        };
        
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
        let max_thickness_variance = 0.02;
        let is_horizontal = v1.z.abs() < 0.75; 

        if linearity >= linearity_threshold && e2 < max_thickness_variance && is_horizontal {
            candidate_main_wires.push((i, p.clone()));
            candidate_main_wire_indices.insert(i);
            wire_directions.push(Vector3::new(v1.x, v1.y, v1.z));
        } else {
            rejected_indices.push(i);
        }
    }

    // ----------------------------------------------------------------
    // PASS 2: Directional "Free Space" Check (Quadrant Density)
    // ----------------------------------------------------------------
    let mut main_wires = Vec::new();
    let mut main_wire_indices = HashSet::new();
    let isolation_radius_sq = 2.0 * 2.0;

    for (idx, (orig_i, p)) in candidate_main_wires.iter().enumerate() {
        let neighbors = match tree.within(&[p.x, p.y, p.z], isolation_radius_sq, &squared_euclidean) {
            Ok(n) => n,
            Err(_) => {
                main_wires.push(p.clone());
                main_wire_indices.insert(*orig_i);
                continue;
            }
        };

        let mut non_wire_points = Vec::new();
        for &(_dist, &n_idx) in &neighbors {
            if !candidate_main_wire_indices.contains(&n_idx) {
                non_wire_points.push(n_idx);
            }
        }

        let n_count = non_wire_points.len();

        if n_count > 500 {
            rejected_indices.push(*orig_i);
            continue; // Rule 1: Too many non-wire points -> Reject
        }

        if n_count < 20 {
            main_wires.push(p.clone());
            main_wire_indices.insert(*orig_i);
            continue; // Clean free space
        }

        // Rule 2: Directional quadrant check
        let v1 = wire_directions[idx];
        let mut u = v1.cross(&Vector3::z());
        if u.norm() < 1e-6 {
            u = v1.cross(&Vector3::x());
        }
        u = u.normalize();
        let v = v1.cross(&u).normalize();

        let mut q1 = 0;
        let mut q2 = 0;
        let mut q3 = 0;
        let mut q4 = 0;

        for &n_idx in &non_wire_points {
            let np = &points[n_idx];
            let d = Vector3::new(np.x - p.x, np.y - p.y, np.z - p.z);
            let x = d.dot(&u);
            let y = d.dot(&v);

            if x >= 0.0 && y >= 0.0 { q1 += 1; }
            else if x < 0.0 && y >= 0.0 { q2 += 1; }
            else if x < 0.0 && y < 0.0 { q3 += 1; }
            else { q4 += 1; }
        }

        let max_q = q1.max(q2).max(q3).max(q4);
        if (max_q as f64) > 0.8 * (n_count as f64) {
            rejected_indices.push(*orig_i); // Attached to a solid object on one side
        } else {
            main_wires.push(p.clone());
            main_wire_indices.insert(*orig_i);
        }
    }

    // ----------------------------------------------------------------
    // PASS 3: Droppers (Detector F - Hybrid Approach)
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
    // PASS 4: Hardware & Junctions (Detector D)
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
    // POST-PROCESSING: Streetlight Killer (Collinearity Check)
    // ----------------------------------------------------------------
    let mut merged_tree = KdTree::new(3);
    for (i, p) in merged_points.iter().enumerate() {
        merged_tree.add([p.x, p.y, p.z], i).unwrap();
    }

    let mut visited = vec![false; merged_points.len()];
    let mut final_points = Vec::new();
    let cluster_radius_sq = 1.0 * 1.0; // 1.0m to connect adjacent points in a wire

    let mut clusters = Vec::new();

    for i in 0..merged_points.len() {
        if visited[i] { continue; }

        let mut cluster = Vec::new();
        let mut stack = vec![i];
        visited[i] = true;

        while let Some(curr_idx) = stack.pop() {
            let p = &merged_points[curr_idx];
            cluster.push(curr_idx);

            if let Ok(neighbors) = merged_tree.within(&[p.x, p.y, p.z], cluster_radius_sq, &squared_euclidean) {
                for &(_dist, &n_idx) in &neighbors {
                    if !visited[n_idx] {
                        visited[n_idx] = true;
                        stack.push(n_idx);
                    }
                }
            }
        }
        clusters.push(cluster);
    }

    for cluster in clusters {
        // Find bounding box to estimate length
        let mut min_x = merged_points[cluster[0]].x;
        let mut max_x = min_x;
        let mut min_y = merged_points[cluster[0]].y;
        let mut max_y = min_y;
        let mut min_z = merged_points[cluster[0]].z;
        let mut max_z = min_z;

        for &idx in &cluster {
            let p = &merged_points[idx];
            if p.x < min_x { min_x = p.x; }
            if p.x > max_x { max_x = p.x; }
            if p.y < min_y { min_y = p.y; }
            if p.y > max_y { max_y = p.y; }
            if p.z < min_z { min_z = p.z; }
            if p.z > max_z { max_z = p.z; }
        }

        let dx = max_x - min_x;
        let dy = max_y - min_y;
        let dz = max_z - min_z;
        let length = (dx * dx + dy * dy + dz * dz).sqrt();

        if length >= 5.0 {
            for &idx in &cluster {
                final_points.push(merged_points[idx].clone());
            }
        } else {
            // Short cluster (< 5m). Check for collinearity to save fragmented wires.
            if cluster.len() < 2 { continue; }

            // Find endpoints (max distance pair)
            let mut max_dist_sq = 0.0;
            let mut p1_idx = cluster[0];
            let mut p2_idx = cluster[0];

            // Simple O(N^2) is fine for small clusters
            for &i1 in &cluster {
                for &i2 in &cluster {
                    let pt1 = &merged_points[i1];
                    let pt2 = &merged_points[i2];
                    let d_sq = (pt1.x - pt2.x).powi(2) + (pt1.y - pt2.y).powi(2) + (pt1.z - pt2.z).powi(2);
                    if d_sq > max_dist_sq {
                        max_dist_sq = d_sq;
                        p1_idx = i1;
                        p2_idx = i2;
                    }
                }
            }

            let p1 = &merged_points[p1_idx];
            let p2 = &merged_points[p2_idx];
            
            let dir = Vector3::new(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
            if dir.norm() < 1e-6 { continue; }
            let dir = dir.normalize();

            let mut has_collinear_neighbor = false;
            let cluster_set: HashSet<usize> = cluster.iter().cloned().collect();

            // Check rays from p1 and p2
            for (i, p) in merged_points.iter().enumerate() {
                if cluster_set.contains(&i) { continue; }

                // Check ray from p2 along dir
                let v2 = Vector3::new(p.x - p2.x, p.y - p2.y, p.z - p2.z);
                let proj2 = v2.dot(&dir);
                if proj2 > 0.0 && proj2 < 10.0 {
                    let dist_sq = v2.norm_squared() - proj2 * proj2;
                    if dist_sq < 0.5 * 0.5 {
                        has_collinear_neighbor = true;
                        break;
                    }
                }

                // Check ray from p1 along -dir
                let v1 = Vector3::new(p.x - p1.x, p.y - p1.y, p.z - p1.z);
                let proj1 = v1.dot(&-dir);
                if proj1 > 0.0 && proj1 < 10.0 {
                    let dist_sq = v1.norm_squared() - proj1 * proj1;
                    if dist_sq < 0.5 * 0.5 {
                        has_collinear_neighbor = true;
                        break;
                    }
                }
            }

            if has_collinear_neighbor {
                for &idx in &cluster {
                    final_points.push(merged_points[idx].clone());
                }
            }
        }
    }

    final_points
}
