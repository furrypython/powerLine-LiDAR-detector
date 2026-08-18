import os
import argparse
import numpy as np
import laspy
from sklearn.cluster import DBSCAN
import random
from scipy.spatial import cKDTree
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import minimum_spanning_tree

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

def extract_skeleton(points, voxel_size=1.0, max_edge_length=5.0):
    """
    Extracts a 3D skeleton from a cluster of points using a Minimum Spanning Tree.
    This preserves parallel wires and droppers.
    """
    if len(points) < 2:
        return points, []

    # 1. Downsample points to create graph nodes
    nodes = voxel_downsample(points, voxel_size)
    if len(nodes) < 2:
        return nodes, []

    # 2. Build nearest neighbor graph
    tree = cKDTree(nodes)
    pairs = tree.query_pairs(r=max_edge_length)
    
    if not pairs:
        return nodes, []

    # Build sparse matrix for graph
    row = []
    col = []
    data = []
    for i, j in pairs:
        dist = np.linalg.norm(nodes[i] - nodes[j])
        row.append(i)
        col.append(j)
        data.append(dist)
        # Undirected graph
        row.append(j)
        col.append(i)
        data.append(dist)
        
    graph = csr_matrix((data, (row, col)), shape=(len(nodes), len(nodes)))
    
    # 3. Compute Minimum Spanning Tree
    mst = minimum_spanning_tree(graph)
    
    # Extract edges from MST
    mst_coo = mst.tocoo()
    edges = np.vstack((mst_coo.row, mst_coo.col)).T
    
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
    parser = argparse.ArgumentParser(description="Convert filtered LiDAR wires to PLY polylines.")
    parser.add_argument("input_las", help="Path to the filtered .las/.laz file")
    parser.add_argument("output_dir", help="Directory to save the resulting .ply files")
    parser.add_argument("--eps", type=float, default=2.0, help="DBSCAN clustering distance (meters)")
    parser.add_argument("--min_samples", type=int, default=10, help="DBSCAN min points per cluster")
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

    # 2. Cluster Points using DBSCAN
    print(f"Clustering {len(points)} points using DBSCAN (eps={args.eps}m)...")
    clustering = DBSCAN(eps=args.eps, min_samples=args.min_samples).fit(points)
    labels = clustering.labels_
    
    unique_labels = set(labels)
    unique_labels.discard(-1) # Remove noise label (-1)
    
    print(f"Found {len(unique_labels)} wires.")
    
    consolidated_vertices = []
    consolidated_edges = []
    consolidated_colors = []
    
    current_vertex_offset = 0
    
    # 3. Process each cluster/wire
    for wire_id in unique_labels:
        wire_points = points[labels == wire_id]
        
        # Extract skeleton preserving topology (parallel lines and droppers)
        # Use DBSCAN eps * 2 as a safe max_edge_length to connect components
        vertices, wire_edges = extract_skeleton(wire_points, voxel_size=args.bin_size, max_edge_length=args.eps * 2)
        
        if len(vertices) < 2 or len(wire_edges) == 0:
            continue # Skip noise clusters too small to form a line
            
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
    print(f"Saving consolidated file...")
    cons_filepath = os.path.join(output_dir, "consolidated_colored_wires.ply")
    write_ply(cons_filepath, consolidated_vertices, consolidated_edges, consolidated_colors)
    
    print("Done! Check the output directory.")

if __name__ == "__main__":
    main()
