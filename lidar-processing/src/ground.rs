use las::{Point, point::Classification, Reader, Read};
use rand::{thread_rng, seq::SliceRandom};
use kdtree::KdTree;
use kdtree::distance::squared_euclidean;

//Reduces ground points to ground_percentage
pub fn ground_reduction(point_cloud: &mut Reader, ground_percentage: f64) -> Vec<Point> {
    let mut points_vector = Vec::new();
    let remove = [(true, 1.-ground_percentage), (false, ground_percentage)]; //Porbability of deleting ground point (1-ground_percentage)

    for point in point_cloud.points() {
        let point = point.unwrap();
        if point.classification == Classification::Ground {
            let mut rng = thread_rng();
            if !remove.choose_weighted(&mut rng, |item| item.1).unwrap().0 {
                points_vector.push(point);
            }
        }
        else{
            points_vector.push(point);
        }
    }
    points_vector
}

// Filters out points that are too close to the ground
pub fn filter_by_height(points: &Vec<Point>, min_height: f64) -> Vec<Point> {
    let mut ground_tree = KdTree::new(2); // 2D tree (X, Y)
    let mut ground_points = Vec::new();
    let mut non_ground_points = Vec::new();

    for p in points.iter() {
        if p.classification == Classification::Ground {
            ground_points.push(p.clone());
        } else {
            non_ground_points.push(p.clone());
        }
    }

    if ground_points.is_empty() {
        return non_ground_points;
    }

    for (i, p) in ground_points.iter().enumerate() {
        ground_tree.add([p.x, p.y], i).unwrap();
    }

    let mut filtered_points = Vec::new();

    for p in non_ground_points {
        if let Ok(nearest) = ground_tree.nearest(&[p.x, p.y], 1, &squared_euclidean) {
            if let Some(&(_dist, &idx)) = nearest.first() {
                let ground_p = &ground_points[idx];
                let height = p.z - ground_p.z;
                if height >= min_height {
                    filtered_points.push(p);
                }
            }
        }
    }

    filtered_points
}