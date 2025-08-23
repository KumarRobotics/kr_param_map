import numpy as np
from PIL import Image
import open3d as o3d
import os

def read_img(path, map_size, resolution, occ_th=0.5, negate=False):
    """
    Convert 2D image to 3D point cloud map.
    
    Args:
        path (str): Path to the input image.
        map_size (tuple): (x_size, y_size, z_size) in meters.
        resolution (float): Grid resolution (size of each voxel).
        occ_th (float): Occupancy threshold (0-1).
        negate (bool): If True, invert pixel meaning.
    
    Returns:
        o3d.geometry.PointCloud: Point cloud representation.
    """
    print(f"Reading image map from: {path}")
    
    # Load image as RGBA
    img = Image.open(path).convert("RGB")
    img_data = np.array(img)

    print(f"Image data shape: {img_data.shape}")

    # Grid dimensions
    dim_x = int(map_size[0] / resolution)
    dim_y = int(map_size[1] / resolution)
    dim_z = int(map_size[2] / resolution)

    print(f"Map size (m): {map_size}")
    print(f"Grid dimensions (cells): {dim_x}, {dim_y}, {dim_z}")

    # Ratios for mapping image pixels to grid
    ratio_x = img.width / dim_x
    ratio_y = img.height / dim_y
    
    points = []
    
    for m in range(dim_z):
        for j in range(dim_y):
            for i in range(dim_x):
                j_px = int(j * ratio_y)
                i_px = int(i * ratio_x)
                
                # Extract pixel RGBA
                r, g, b = img_data[j_px, i_px]
                
                # Average channels
                color_avg = (r + g + b) / 3.0

                if negate:
                    color_avg = 255 - color_avg
                
                occ = (255 - color_avg) / 255.0
                
                if occ > occ_th:
                    # Convert voxel index to metric coordinates
                    x = i * resolution
                    y = (dim_y - j - 1) * resolution  # flip Y
                    z = m * resolution
                    points.append([x, y, z])
    
    # Build point cloud
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)
    
    return cloud



# Example usage
if __name__ == "__main__":
    map_size = np.array([20.0, 10.0, 0.1])   # world dimensions in meters
    resolution = 0.1                          # grid resolution
    
    input_file = "maze.png"
    cloud = read_img(input_file, map_size, resolution, occ_th=0.7, negate=False)

    # Save with same basename as input (replace extension with .pcd)
    base_name = os.path.splitext(input_file)[0]
    output_file = f"{base_name}.pcd"
    
    o3d.io.write_point_cloud(output_file, cloud)
    print(f"[INFO] Saved point cloud to {output_file}")

    # Optional: visualize
    o3d.visualization.draw_geometries([cloud])