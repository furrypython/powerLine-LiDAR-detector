import os
import argparse
import numpy as np
import laspy
from sklearn.cluster import DBSCAN
import random

def generate_random_color():
    """Generates a random RGB color."""
    return [random.randint(50, 255) for _ in range(3)]

def extract_polyline(points, bin_size=1.0):
    """
    Fits a smooth 1D polyline through a cluster of 3D LiDAR points.
    It uses PCA to find the principal direction (wire length), projects
    the points, and averages them inside evenly spaced bins.
    """
    if len(points) < 2:
        return points

    # 1. PCA to find the main direction of the wire
    centroid = np.mean(points, axis=0)
    centered_points = points - centroid
    
    # Use SVD to compute PCA
    U, S, Vt = np.linalg.svd(centered_points, full_matrices=False)
    main_axis = Vt[0]

    # 2. Project points onto the main axis
    projections = np.dot(centered_points, main_axis)
    
    # 3. Sort and bin points along the axis to create a smooth single line
    min_proj = np.min(projections)
    max_proj = np.max(projections)
    
    # Create bins along the wire
    bins = np.arange(min_proj, max_proj + bin_size, bin_size)
    
    polyline_vertices = []
    for i in range(len(bins) - 1):
        mask = (projections >= bins[i]) & (projections < bins[i+1])
        pts_in_bin = points[mask]
        if len(pts_in_bin) > 0:
            bin_centroid = np.mean(pts_in_bin, axis=0)
            polyline_vertices.append(bin_centroid)
            
    return np.array(polyline_vertices)

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
        
        # Collapse the thick point cloud into a clean, ordered 1D polyline
        polyline = extract_polyline(wire_points, bin_size=args.bin_size)
        
        if len(polyline) < 2:
            continue # Skip noise clusters too small to form a line
            
        wire_color = generate_random_color()
        
        # Create edge index connections (0 -> 1, 1 -> 2, etc.)
        wire_edges = []
        for i in range(len(polyline) - 1):
            wire_edges.append([i, i + 1])
            
        # Write individual wire to its own PLY file
        ind_filepath = os.path.join(individual_dir, f"wire_{wire_id}.ply")
        write_ply(ind_filepath, polyline, wire_edges)
        
        # Append to consolidated data
        for v in polyline:
            consolidated_vertices.append(v)
            consolidated_colors.append(wire_color)
            
        for e in wire_edges:
            consolidated_edges.append([e[0] + current_vertex_offset, e[1] + current_vertex_offset])
            
        current_vertex_offset += len(polyline)
        
    # 4. Save consolidated multi-colored file
    print(f"Saving consolidated file...")
    cons_filepath = os.path.join(output_dir, "consolidated_colored_wires.ply")
    write_ply(cons_filepath, consolidated_vertices, consolidated_edges, consolidated_colors)
    
    print("Done! Check the output directory.")

if __name__ == "__main__":
    main()
