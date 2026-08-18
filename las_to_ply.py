import laspy
import numpy as np
import os

def convert_las_to_ply(input_las_path, output_ply_path):
    print(f"Reading {input_las_path}...")
    
    # 1. Read the LAS file
    las = laspy.read(input_las_path)
    
    # Extract X, Y, Z coordinates
    points = np.vstack((las.x, las.y, las.z)).transpose()
    
    # 2. Center the point cloud (Fixes the Blender/MeshLab zooming issue)
    centroid = np.mean(points, axis=0)
    points_centered = points - centroid
    print(f"Original center was at {centroid}. Shifted to (0,0,0).")

    # 3. Handle Colors
    has_color = hasattr(las, 'red') and hasattr(las, 'green') and hasattr(las, 'blue')
    if has_color:
        print("Extracting colors...")
        # LAS colors are 16-bit (0-65535). PLY expects 8-bit (0-255).
        # We bit-shift by 8 to efficiently convert 16-bit to 8-bit.
        r = np.right_shift(las.red, 8).astype(np.uint8)
        g = np.right_shift(las.green, 8).astype(np.uint8)
        b = np.right_shift(las.blue, 8).astype(np.uint8)
    else:
        print("No color data found in LAS file.")

    # 4. Save as PLY directly (Bypassing Open3D to prevent macOS crashes)
    print(f"Saving to {output_ply_path}...")
    num_points = len(points_centered)
    
    with open(output_ply_path, 'wb') as f:
        # Write PLY Header (ASCII)
        header = "ply\n"
        header += "format binary_little_endian 1.0\n"
        header += f"element vertex {num_points}\n"
        header += "property float x\n"
        header += "property float y\n"
        header += "property float z\n"
        if has_color:
            header += "property uchar red\n"
            header += "property uchar green\n"
            header += "property uchar blue\n"
        header += "end_header\n"
        f.write(header.encode('ascii'))
        
        # Write PLY Data (Binary)
        if has_color:
            # Create a structured numpy array to interleave XYZ and RGB perfectly
            vertex_data = np.empty(num_points, dtype=[
                ('x', 'f4'), ('y', 'f4'), ('z', 'f4'),
                ('red', 'u1'), ('green', 'u1'), ('blue', 'u1')
            ])
            vertex_data['red'] = r
            vertex_data['green'] = g
            vertex_data['blue'] = b
        else:
            vertex_data = np.empty(num_points, dtype=[
                ('x', 'f4'), ('y', 'f4'), ('z', 'f4')
            ])

        vertex_data['x'] = points_centered[:, 0]
        vertex_data['y'] = points_centered[:, 1]
        vertex_data['z'] = points_centered[:, 2]
            
        f.write(vertex_data.tobytes())
        
    print("Done! File saved successfully.")

if __name__ == "__main__":
    # Use the exact paths from your terminal output
    input_file = "<INPUT_LAS_FILE>.las"
    output_file = "<OUTPUT_PLY_FILE>.ply"
    
    if os.path.exists(input_file):
        convert_las_to_ply(input_file, output_file)
    else:
        print(f"Error: Could not find {input_file}")