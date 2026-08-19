import os
import argparse
import numpy as np
import laspy
import random
from scipy.spatial import cKDTree
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import minimum_spanning_tree, connected_components, dijkstra

def generate_random_color():
    """Generates a random RGB color."""
    return [random.randint(50, 255) for _ in range(3)]

def voxel_downsample(points, voxel_size):
    """Downsamples a point cloud using a voxel grid, returning the centroid of each voxel."""
    voxel_indices = np.floor(points / voxel_size).astype(np.int32)
    unique_voxels, inverse_indices = np.unique(voxel_indices, axis=0, return_inverse=True)
    
    downsampled_points = np.zeros((len(unique_voxels), 3))
    counts = np.zeros(len(unique_voxels))
    
    np.add.at(downsampled_points, inverse_indices, points)
    np.add.at(counts, inverse_indices, 1)
    
    downsampled_points /= counts[:, np.newaxis]
    return downsampled_points

def compute_tangents(nodes, k=15):
    """Computes the local tangent direction for each node using PCA."""
    tree = cKDTree(nodes)
    tangents = np.zeros_like(nodes)
    
    # Restrict distance so PCA doesn't pull points from across large gaps
    distances, indices = tree.query(nodes, k=k, distance_upper_bound=2.0)
    
    for i, neighbors in enumerate(indices):
        valid_neighbors = neighbors[distances[i] != np.inf]
        pts = nodes[valid_neighbors]
        if len(pts) < 3:
            tangents[i] = np.array([1.0, 0.0, 0.0]) # Fallback
            continue
            
        # Center the points
        pts_centered = pts - np.mean(pts, axis=0)
        
        # Covariance matrix
        cov = np.cov(pts_centered, rowvar=False)
        
        # Eigen decomposition
        evals, evecs = np.linalg.eigh(cov)
        
        # The eigenvector corresponding to the largest eigenvalue is the tangent
        tangents[i] = evecs[:, np.argmax(evals)]
        
    return tangents

def extract_wires_anisotropic(points, voxel_size=0.5, max_dist=2.0, max_perp_dist=0.4):
    """
    Extracts individual wires by building a graph that only permits 
    connections where the perpendicular distance to the local tangent is small.
    This physically prevents jumping sideways to parallel wires.
    """
    if len(points) < 2:
        return np.zeros(len(points), dtype=int), 0
        
    # 1. Downsample points to create graph nodes
    nodes = voxel_downsample(points, voxel_size)
    if len(nodes) < 2:
        return np.zeros(len(points), dtype=int), 0

    # 2. Compute local tangents 
    tangents = compute_tangents(nodes, k=15)

    # 3. Build a spatial graph based on distance
    tree = cKDTree(nodes)
    pairs = tree.query_pairs(r=max_dist)
    
    row, col, data = [], [], []
    
    # 4. Filter edges using Perpendicular Distance logic
    for i, j in pairs:
        v = nodes[j] - nodes[i]
        dist = np.linalg.norm(v)
        if dist == 0: continue
            
        # Find perpendicular distance from i's tangent to j
        proj_i = np.dot(v, tangents[i]) * tangents[i]
        perp_i = np.linalg.norm(v - proj_i)
        
        # Find perpendicular distance from j's tangent to i
        proj_j = np.dot(v, tangents[j]) * tangents[j]
        perp_j = np.linalg.norm(v - proj_j)
        
        # STRICT PRUNE: If the connection jumps sideways by more than the wire's 
        # physical radius (e.g. 0.4m), it is jumping to a parallel wire! Drop it.
        if perp_i <= max_perp_dist and perp_j <= max_perp_dist:
            row.extend([i, j])
            col.extend([j, i])
            data.extend([dist, dist])
            
    if not data:
        return np.zeros(len(points), dtype=int), 0

    # 5. Extract the individual wires using Connected Components
    graph = csr_matrix((data, (row, col)), shape=(len(nodes), len(nodes)))
    num_components, node_labels = connected_components(csgraph=graph, directed=False)
    
    # 6. Map raw points back to their nearest node's cluster label
    _, closest_node_idx = tree.query(points)
    point_labels = node_labels[closest_node_idx]
    
    return point_labels, num_components

def extract_longest_path(mst_graph):
    """Finds the longest path (diameter) in a Minimum Spanning Tree to remove branches."""
    mst_undirected = mst_graph + mst_graph.T
    n_components, labels = connected_components(mst_undirected, directed=False)
    
    # Find the largest connected component to start Dijkstra
    unique, counts = np.unique(labels, return_counts=True)
    largest_comp_label = unique[np.argmax(counts)]
    start_node = np.where(labels == largest_comp_label)[0][0]
    
    # Run BFS/Dijkstra to find the furthest node from start_node (End 1)
    dist1, pred1 = dijkstra(mst_undirected, directed=False, indices=start_node, return_predecessors=True)
    dist1[dist1 == np.inf] = -1
    end1 = np.argmax(dist1)
    
    # Run BFS/Dijkstra from End 1 to find the other furthest node (End 2)
    dist2, pred2 = dijkstra(mst_undirected, directed=False, indices=end1, return_predecessors=True)
    dist2[dist2 == np.inf] = -1
    end2 = np.argmax(dist2)
    
    # Backtrack from End 2 to End 1 to get the pure, unbranched path
    path = []
    curr = end2
    while curr != -9999 and curr != end1:
        path.append(curr)
        curr = pred2[curr]
    if curr == end1:
        path.append(end1)
        
    return path[::-1]

def smooth_polyline(points, window_size=5):
    """Applies a moving average to smooth a polyline's vertices."""
    if len(points) < window_size:
        return points
    
    smoothed = np.zeros_like(points)
    
    # Pad the ends so the wire doesn't shrink
    pad_front = np.repeat(points[0:1], window_size//2, axis=0)
    pad_back = np.repeat(points[-1:], window_size//2, axis=0)
    padded = np.vstack((pad_front, points, pad_back))
    
    for i in range(3): # Smooth X, Y, Z independently
        smoothed[:, i] = np.convolve(padded[:, i], np.ones(window_size)/window_size, mode='valid')
        
    return smoothed

def extract_skeleton(points, bin_size=1.0, max_dist=2.0):
    """
    Extracts a perfectly smooth, unbranched 3D polyline from an isolated wire cluster.
    """
    if len(points) < 2:
        return points, []

    # 1. Downsample points to create graph nodes
    nodes = voxel_downsample(points, bin_size)
    if len(nodes) < 2:
        return nodes, []

    # 2. Build nearest neighbor graph (generous radius to jump small gaps)
    tree = cKDTree(nodes)
    pairs = tree.query_pairs(r=max_dist + 2.0)
    
    if not pairs:
        return nodes, []

    # Build sparse matrix for graph
    row, col, data = [], [], []
    for i, j in pairs:
        dist = np.linalg.norm(nodes[i] - nodes[j])
        row.append(i); col.append(j); data.append(dist)
        
    graph = csr_matrix((data, (row, col)), shape=(len(nodes), len(nodes)))
    
    # 3. Compute Euclidean Minimum Spanning Tree
    mst = minimum_spanning_tree(graph)
    
    if mst.nnz == 0:
        return nodes, []
    
    # 4. Extract the Longest Path to remove all MST branches/spikes
    path_indices = extract_longest_path(mst)
    
    if len(path_indices) < 2:
        return nodes, []
        
    ordered_nodes = nodes[path_indices]
    
    # 5. Apply Moving Average Smoothing to remove voxel zigzag artifacts
    window = max(3, int(5.0 / bin_size)) # E.g., 5-node window for 1.0m bins (5m smoothing)
    smoothed_nodes = smooth_polyline(ordered_nodes, window_size=window)
    
    # 6. Create linear edges [0, 1], [1, 2], ...
    edges = [[i, i+1] for i in range(len(smoothed_nodes)-1)]
    
    return smoothed_nodes, edges

def write_ply(filepath, vertices, edges, colors=None):
    """Writes vertices and edges to an ASCII .ply file."""
    num_vertices = len(vertices)
    num_edges = len(edges)
    
    has_colors = colors is not None
    
    with open(filepath, 'w') as f:
        # Header
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {num_vertices}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        if has_colors:
            f.write("property uchar red\n")
            f.write("property uchar green\n")
            f.write("property uchar blue\n")
        f.write(f"element edge {num_edges}\n")
        f.write("property int vertex1\n")
        f.write("property int vertex2\n")
        f.write("end_header\n")
        
        # Body - Vertices
        for i, v in enumerate(vertices):
            if has_colors:
                c = colors[i]
                f.write(f"{v[0]:.4f} {v[1]:.4f} {v[2]:.4f} {int(c[0])} {int(c[1])} {int(c[2])}\n")
            else:
                f.write(f"{v[0]:.4f} {v[1]:.4f} {v[2]:.4f}\n")
                
        # Body - Edges
        for e in edges:
            f.write(f"{e[0]} {e[1]}\n")

def main():
    parser = argparse.ArgumentParser(description="Convert filtered LiDAR wires to PLY polylines using Anisotropic Graph Separation.")
    parser.add_argument("input_las", help="Path to the filtered .las/.laz file")
    parser.add_argument("output_dir", help="Directory to save the resulting .ply files")
    parser.add_argument("--max_dist", type=float, default=2.0, help="Max distance to connect points during clustering (meters)")
    parser.add_argument("--max_perp_dist", type=float, default=0.4, help="Max perpendicular distance to local tangent to connect points. Solves parallel bridging. (meters)")
    parser.add_argument("--min_samples", type=int, default=10, help="Min points per wire cluster")
    parser.add_argument("--bin_size", type=float, default=1.0, help="Step size for polyline vertices (meters)")
    
    args = parser.parse_args()
    
    input_las = args.input_las
    output_dir = args.output_dir
    
    # Setup Output Directories
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        
    individual_dir = os.path.join(output_dir, "individual_wires")
    if not os.path.exists(individual_dir):
        os.makedirs(individual_dir)

    # 1. Load LAS File
    print(f"Reading {input_las}...")
    las = laspy.read(input_las)
    points = np.vstack((las.x, las.y, las.z)).transpose()
    
    if len(points) == 0:
        print("No points found in the input file.")
        return

    # 2. Cluster Points using Anisotropic Directional Graph
    print(f"Clustering {len(points)} points directionally...")
    
    graph_voxel_size = min(args.bin_size, 0.5)
    
    labels, num_components = extract_wires_anisotropic(
        points, 
        voxel_size=graph_voxel_size, 
        max_dist=args.max_dist, 
        max_perp_dist=args.max_perp_dist
    )
    
    unique_labels = set(labels)
    print(f"Found {len(unique_labels)} potential wires.")
    
    consolidated_vertices = []
    consolidated_edges = []
    consolidated_colors = []
    
    current_vertex_offset = 0
    valid_wires_count = 0
    
    # 3. Process each isolated cluster/wire
    for wire_id in unique_labels:
        wire_points = points[labels == wire_id]
        
        if len(wire_points) < args.min_samples:
            continue
            
        # Extract skeleton finding the longest, unbranched path and smoothing it
        vertices, wire_edges = extract_skeleton(wire_points, bin_size=args.bin_size, max_dist=args.max_dist)
        
        if len(vertices) < 2 or len(wire_edges) == 0:
            continue
            
        valid_wires_count += 1
        wire_color = generate_random_color()
        
        ind_filepath = os.path.join(individual_dir, f"wire_{wire_id}.ply")
        write_ply(ind_filepath, vertices, wire_edges)
        
        for v in vertices:
            consolidated_vertices.append(v)
            consolidated_colors.append(wire_color)
            
        for e in wire_edges:
            consolidated_edges.append([e[0] + current_vertex_offset, e[1] + current_vertex_offset])
            
        current_vertex_offset += len(vertices)
        
    # 4. Save consolidated multi-colored file
    print(f"Successfully processed {valid_wires_count} wires.")
    print(f"Saving consolidated file...")
    cons_filepath = os.path.join(output_dir, "consolidated_colored_wires.ply")
    write_ply(cons_filepath, consolidated_vertices, consolidated_edges, consolidated_colors)
    
    print("Done! Check the output directory.")

if __name__ == "__main__":
    main()