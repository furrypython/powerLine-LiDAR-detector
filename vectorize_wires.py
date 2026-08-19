import os
import argparse
import numpy as np
import laspy
from sklearn.cluster import DBSCAN
import random
from scipy.spatial import cKDTree
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import minimum_spanning_tree, connected_components

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

def compute_tangents(nodes, k=10):
    """Computes the local tangent direction for each node using PCA."""
    tree = cKDTree(nodes)
    tangents = np.zeros_like(nodes)
    
    # Find k nearest neighbors for PCA
    distances, indices = tree.query(nodes, k=min(k, len(nodes)))
    
    for i, neighbors in enumerate(indices):
        pts = nodes[neighbors]
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

def extract_wires_anisotropic(points, voxel_size=0.5, max_dist=2.0, alignment_threshold=0.85):
    """
    Extracts individual wires by building a graph that only permits 
    connections along the local tangent direction. Replaces DBSCAN.
    """
    if len(points) < 2:
        return np.zeros(len(points), dtype=int), 0
        
    # 1. Downsample points to create graph nodes
    # Use a finer voxel size (0.2-0.5m) to ensure parallel wires aren't merged into a single voxel
    nodes = voxel_downsample(points, voxel_size)
    if len(nodes) < 2:
        return np.zeros(len(points), dtype=int), 0

    # 2. Compute local tangents using your existing PCA function
    tangents = compute_tangents(nodes, k=15)

    # 3. Build a spatial graph based on distance
    tree = cKDTree(nodes)
    pairs = tree.query_pairs(r=max_dist)
    
    row, col, data = [], [], []
    
    # 4. Filter edges using Anisotropic (Directional) logic
    for i, j in pairs:
        v = nodes[j] - nodes[i]
        dist = np.linalg.norm(v)
        if dist == 0: continue
            
        v_norm = v / dist
        
        # How well does the connection align with the wire direction?
        align_i = abs(np.dot(v_norm, tangents[i]))
        align_j = abs(np.dot(v_norm, tangents[j]))
        
        # PRUNE: If connection jumps sideways, alignment is low
        if align_i > alignment_threshold and align_j > alignment_threshold:
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

def extract_skeleton(points, voxel_size=1.0, max_edge_length=5.0, alpha=15.0):
    """
    Extracts a 3D skeleton from a cluster of points using a Directional Minimum Spanning Tree.
    This penalizes perpendicular connections to preserve parallel wires.
    """
    if len(points) < 2:
        return points, []

    # 1. Downsample points to create graph nodes
    nodes = voxel_downsample(points, voxel_size)
    if len(nodes) < 2:
        return nodes, []

    # 2. Compute local tangents for directional awareness
    tangents = compute_tangents(nodes, k=min(10, len(nodes)))

    # 3. Build nearest neighbor graph
    tree = cKDTree(nodes)
    pairs = tree.query_pairs(r=max_edge_length)
    
    if not pairs:
        return nodes, []

    # Build sparse matrix for graph
    row = []
    col = []
    data = []
    for i, j in pairs:
        v = nodes[j] - nodes[i]
        dist = np.linalg.norm(v)
        
        if dist == 0:
            continue
            
        v_norm = v / dist
        
        # Calculate alignment with local tangents (dot product)
        dot_i = abs(np.dot(v_norm, tangents[i]))
        dot_j = abs(np.dot(v_norm, tangents[j]))
        
        # Average alignment (1.0 = perfectly parallel, 0.0 = perfectly perpendicular)
        alignment = (dot_i + dot_j) / 2.0
        
        # Apply penalty for perpendicular connections
        penalty = alpha * (1.0 - alignment)
        weight = dist * (1.0 + penalty)
        
        row.append(i)
        col.append(j)
        data.append(weight)
        # Undirected graph
        row.append(j)
        col.append(i)
        data.append(weight)
        
    if not data:
        return nodes, []
        
    graph = csr_matrix((data, (row, col)), shape=(len(nodes), len(nodes)))
    
    # 4. Compute Minimum Spanning Tree
    mst = minimum_spanning_tree(graph)
    
    # 5. Extract edges and prune bad connections
    mst_coo = mst.tocoo()
    edges = []
    
    for i, j, w in zip(mst_coo.row, mst_coo.col, mst_coo.data):
        dist = np.linalg.norm(nodes[j] - nodes[i])
        # If the weight is significantly higher than the distance, it means 
        # it was heavily penalized (i.e., a sideways jump). We prune these.
        if w > dist * 3.0: 
            continue
        edges.append([i, j])
    
    return nodes, edges

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
    parser.add_argument("--alignment", type=float, default=0.85, help="Directional alignment threshold (0.0 to 1.0, higher is stricter/more parallel)")
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
    
    # We use a finer voxel size (0.5m) specifically for the separation graph 
    # to ensure parallel lines don't get merged into a single voxel before we even start.
    graph_voxel_size = min(args.bin_size, 0.5)
    
    labels, num_components = extract_wires_anisotropic(
        points, 
        voxel_size=graph_voxel_size, 
        max_dist=args.max_dist, 
        alignment_threshold=args.alignment
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
        
        # Skip noise / tiny clusters
        if len(wire_points) < args.min_samples:
            continue
            
        # Now we use the original logic on the ALREADY SEPARATED wire points.
        # Because this cluster ONLY contains points from one line, the MST 
        # cannot jump to a neighbor (the neighbor points are in a different cluster).
        vertices, wire_edges = extract_skeleton(wire_points, voxel_size=args.bin_size, max_edge_length=max(args.max_dist, 4.0))
        
        if len(vertices) < 2 or len(wire_edges) == 0:
            continue # Skip if it fails to form a line
            
        valid_wires_count += 1
        wire_color = generate_random_color()
        
        # Write individual wire to its own PLY file
        ind_filepath = os.path.join(individual_dir, f"wire_{wire_id}.ply")
        write_ply(ind_filepath, vertices, wire_edges)
        
        # Append to consolidated data
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