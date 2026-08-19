import os
import argparse
import numpy as np
import laspy
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

def compute_tangents(nodes, k=15, radius=0.6):
    """Computes the local tangent direction for each node using PCA."""
    tree = cKDTree(nodes)
    tangents = np.zeros_like(nodes)
    
    distances, indices = tree.query(nodes, k=k, distance_upper_bound=radius)
    
    for i, neighbors in enumerate(indices):
        valid_neighbors = neighbors[distances[i] != np.inf]
        pts = nodes[valid_neighbors]
        if len(pts) < 3:
            tangents[i] = np.array([1.0, 0.0, 0.0]) # Fallback
            continue
            
        pts_centered = pts - np.mean(pts, axis=0)
        cov = np.cov(pts_centered, rowvar=False)
        evals, evecs = np.linalg.eigh(cov)
        tangents[i] = evecs[:, np.argmax(evals)]
        
    return tangents

def prune_short_branches(nodes, edges, min_length=0.5):
    """Removes short dangling branches (noise) while preserving long branches (droppers)."""
    adj = {i: set() for i in range(len(nodes))}
    for u, v in edges:
        adj[u].add(v)
        adj[v].add(u)
        
    edge_len = {}
    for u, v in edges:
        d = np.linalg.norm(nodes[u] - nodes[v])
        edge_len[(u, v)] = d
        edge_len[(v, u)] = d

    while True:
        leaves = [i for i, neighbors in adj.items() if len(neighbors) == 1]
        removed_any = False
        
        for leaf in leaves:
            if leaf not in adj or len(adj[leaf]) != 1:
                continue
                
            curr = leaf
            prev = None
            branch_len = 0.0
            branch_nodes = []
            
            while True:
                branch_nodes.append(curr)
                neighbors = list(adj[curr])
                
                if len(neighbors) > 2:
                    break # Reached junction
                if len(neighbors) == 1 and curr != leaf:
                    break # Reached another leaf (isolated path)
                    
                next_node = neighbors[0] if neighbors[0] != prev else (neighbors[1] if len(neighbors) > 1 else None)
                if next_node is None:
                    break
                    
                branch_len += edge_len[(curr, next_node)]
                prev = curr
                curr = next_node
                
            if branch_len < min_length:
                if len(list(adj[curr])) > 2:
                    # Keep the junction node
                    nodes_to_remove = branch_nodes[:-1]
                else:
                    # Remove the entire isolated short path
                    nodes_to_remove = branch_nodes
                    
                for n in nodes_to_remove:
                    if n in adj:
                        for neighbor in list(adj[n]):
                            adj[neighbor].remove(n)
                        del adj[n]
                removed_any = True
                
        if not removed_any:
            break
            
    new_edges = []
    for u, neighbors in adj.items():
        for v in neighbors:
            if u < v:
                new_edges.append([u, v])
                
    return list(adj.keys()), new_edges

def smooth_graph(nodes, edges, iterations=3):
    """Applies Laplacian smoothing to the graph to remove zigzags."""
    adj = {i: [] for i in range(len(nodes))}
    for u, v in edges:
        adj[u].append(v)
        adj[v].append(u)
        
    smoothed = np.copy(nodes)
    for _ in range(iterations):
        new_nodes = np.copy(smoothed)
        for i in range(len(nodes)):
            if len(adj[i]) > 0:
                avg_neighbor = np.mean(smoothed[adj[i]], axis=0)
                new_nodes[i] = 0.5 * smoothed[i] + 0.5 * avg_neighbor
        smoothed = new_nodes
    return smoothed

def extract_wire_network(points, voxel_size=0.2, max_dist=1.0, align_threshold=0.85, vertical_thresh=0.5):
    """
    Builds a wire network graph that preserves droppers and parallel wires correctly.
    """
    if len(points) < 2:
        return [], []
        
    # 1. Downsample
    nodes = voxel_downsample(points, voxel_size)
    if len(nodes) < 2:
        return [], []

    # 2. Tangents
    tangents = compute_tangents(nodes, k=15, radius=0.6)
    tree = cKDTree(nodes)
    pairs = tree.query_pairs(r=max_dist)
    
    row, col, data = [], [], []
    valid_vertical_edges = []
    
    # 3. Anisotropic & Vertical Filter
    for i, j in pairs:
        v = nodes[j] - nodes[i]
        dist = np.linalg.norm(v)
        if dist == 0: continue
            
        u = v / dist
        
        # Check if the connection is a Dropper (Vertical)
        is_vertical = abs(u[2]) > vertical_thresh
        
        # Check if the connection aligns with the horizontal wire path
        align_i = abs(np.dot(u, tangents[i]))
        align_j = abs(np.dot(u, tangents[j]))
        is_aligned = align_i > align_threshold and align_j > align_threshold
        
        if is_vertical or is_aligned:
            row.extend([i, j])
            col.extend([j, i])
            data.extend([dist, dist])
            
            if is_vertical:
                valid_vertical_edges.append((i, j, dist))
                
    if not data:
        return [], []

    graph = csr_matrix((data, (row, col)), shape=(len(nodes), len(nodes)))
    
    # 4. MST extracts the thin, zigzag-free backbone of the graph
    mst = minimum_spanning_tree(graph)
    mst_coo = mst.tocoo()
    
    final_edges = set()
    for i, j in zip(mst_coo.row, mst_coo.col):
        final_edges.add((min(i, j), max(i, j)))
        
    # 5. Restore droppers! (MST breaks cycles, so it deletes droppers. We add them back.)
    for i, j, dist in valid_vertical_edges:
        final_edges.add((min(i, j), max(i, j)))
        
    final_edges_list = [list(e) for e in final_edges]
    
    # 6. Prune short noise branches (keeps long droppers and main wires)
    valid_node_indices, pruned_edges = prune_short_branches(nodes, final_edges_list, min_length=0.5)
    
    if len(valid_node_indices) == 0:
        return [], []
        
    # 7. Smooth the network
    smoothed_nodes = smooth_graph(nodes, pruned_edges, iterations=3)
    
    # Re-index
    index_map = {old_idx: new_idx for new_idx, old_idx in enumerate(valid_node_indices)}
    final_nodes = smoothed_nodes[valid_node_indices]
    final_edges_remapped = [[index_map[u], index_map[v]] for u, v in pruned_edges]
    
    return final_nodes, final_edges_remapped

def write_ply(filepath, vertices, edges, colors=None):
    """Writes vertices and edges to an ASCII .ply file."""
    num_vertices = len(vertices)
    num_edges = len(edges)
    has_colors = colors is not None
    
    with open(filepath, 'w') as f:
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
        
        for i, v in enumerate(vertices):
            if has_colors:
                c = colors[i]
                f.write(f"{v[0]:.4f} {v[1]:.4f} {v[2]:.4f} {int(c[0])} {int(c[1])} {int(c[2])}\n")
            else:
                f.write(f"{v[0]:.4f} {v[1]:.4f} {v[2]:.4f}\n")
                
        for e in edges:
            f.write(f"{e[0]} {e[1]}\n")

def main():
    parser = argparse.ArgumentParser(description="Convert filtered LiDAR wires to PLY networks (preserving droppers).")
    parser.add_argument("input_las", help="Path to the filtered .las/.laz file")
    parser.add_argument("output_dir", help="Directory to save the resulting .ply files")
    parser.add_argument("--voxel_size", type=float, default=0.2, help="Voxel downsampling resolution. Keep small (0.1-0.2) to separate parallel wires.")
    parser.add_argument("--max_dist", type=float, default=1.0, help="Max distance to connect points (meters)")
    parser.add_argument("--alignment", type=float, default=0.85, help="Directional alignment threshold for horizontal wires (0.0-1.0)")
    
    args = parser.parse_args()
    
    input_las = args.input_las
    output_dir = args.output_dir
    
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        
    individual_dir = os.path.join(output_dir, "individual_wires")
    if not os.path.exists(individual_dir):
        os.makedirs(individual_dir)

    print(f"Reading {input_las}...")
    las = laspy.read(input_las)
    points = np.vstack((las.x, las.y, las.z)).transpose()
    
    if len(points) == 0:
        print("No points found in the input file.")
        return

    print("Building anisotropic wire network (separating parallel wires and preserving droppers)...")
    
    final_nodes, final_edges = extract_wire_network(
        points, 
        voxel_size=args.voxel_size, 
        max_dist=args.max_dist, 
        align_threshold=args.alignment
    )
    
    if len(final_nodes) == 0:
        print("Failed to extract any wires.")
        return
        
    print("Separating network into distinct physical components...")
    
    row = [e[0] for e in final_edges] + [e[1] for e in final_edges]
    col = [e[1] for e in final_edges] + [e[0] for e in final_edges]
    data = np.ones(len(row))
    graph_final = csr_matrix((data, (row, col)), shape=(len(final_nodes), len(final_nodes)))
    num_comps, labels = connected_components(graph_final, directed=False)
    
    print(f"Found {num_comps} connected wire structures.")
    
    consolidated_vertices = []
    consolidated_edges = []
    consolidated_colors = []
    current_vertex_offset = 0
    valid_wires_count = 0
    
    for comp_id in range(num_comps):
        comp_nodes_idx = np.where(labels == comp_id)[0]
        
        # Skip tiny isolated fragments
        if len(comp_nodes_idx) < 10:
            continue
            
        valid_wires_count += 1
        comp_color = generate_random_color()
        
        # Extract edges for this component
        comp_nodes_set = set(comp_nodes_idx)
        comp_edges = [e for e in final_edges if e[0] in comp_nodes_set and e[1] in comp_nodes_set]
        
        # Re-index for the individual file
        local_map = {old: new for new, old in enumerate(comp_nodes_idx)}
        local_edges = [[local_map[e[0]], local_map[e[1]]] for e in comp_edges]
        local_nodes = final_nodes[comp_nodes_idx]
        
        ind_filepath = os.path.join(individual_dir, f"wire_{comp_id}.ply")
        write_ply(ind_filepath, local_nodes, local_edges)
        
        # Append to consolidated
        for v in local_nodes:
            consolidated_vertices.append(v)
            consolidated_colors.append(comp_color)
            
        for e in local_edges:
            consolidated_edges.append([e[0] + current_vertex_offset, e[1] + current_vertex_offset])
            
        current_vertex_offset += len(local_nodes)
        
    print(f"Successfully processed {valid_wires_count} valid wire structures.")
    print(f"Saving consolidated file...")
    cons_filepath = os.path.join(output_dir, "consolidated_colored_wires.ply")
    write_ply(cons_filepath, consolidated_vertices, consolidated_edges, consolidated_colors)
    
    print("Done! Check the output directory.")

if __name__ == "__main__":
    main()