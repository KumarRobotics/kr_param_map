import open3d as o3d
import numpy as np
import os
from collections import defaultdict

def pcd_to_voxel_grid(pcd_file, voxel_size=0.2):
    """Load a PCD and return (voxels, voxel_size, origin) where
       voxels is an array of integer (x,y,z) grid indices,
       origin is the 3D world origin of the voxel grid."""
    pcd = o3d.io.read_point_cloud(pcd_file)
    if pcd.is_empty():
        raise ValueError("Input PCD is empty!")

    vg = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=voxel_size)
    idxs = [v.grid_index for v in vg.get_voxels()]            # integer (x,y,z)
    voxels = np.asarray(idxs, dtype=int)
    origin = np.array(vg.origin, dtype=float)                 # world-space origin of voxel grid
    return voxels, voxel_size, origin

def runs_in_row(row):
    """Given a 1D boolean array, return inclusive [x1, x2] runs of True."""
    runs = []
    in_run = False
    start = 0
    for x, val in enumerate(row):
        if val and not in_run:
            in_run = True
            start = x
        elif not val and in_run:
            runs.append((start, x - 1))
            in_run = False
    if in_run:
        runs.append((start, len(row) - 1))
    return runs

def greedy_xy_merge_rectangles(binary_grid):
    """
    Maximal rectangle merging along X then Y for a 2D boolean grid.
    Returns list of rectangles as ((x1,y1),(x2,y2)) in grid coords (inclusive).
    This preserves shape exactly (no dilation): rectangles cover all True cells.
    """
    H, W = binary_grid.shape
    active = {}  # key=(x1,x2) -> [y_start, y_end_current]
    rects = []

    for y in range(H):
        row_runs = runs_in_row(binary_grid[y])
        next_active = {}
        used_prev = set()

        # Try to extend existing rects vertically when run matches exactly
        for (x1, x2) in row_runs:
            if (x1, x2) in active:
                y0, y1_cur = active[(x1, x2)]
                next_active[(x1, x2)] = [y0, y + 1]  # extend
                used_prev.add((x1, x2))
            else:
                # start a new rectangle
                next_active[(x1, x2)] = [y, y]

        # finalize any rectangles that didn't continue
        for key, (y0, y1_cur) in active.items():
            if key not in used_prev:
                rects.append(((key[0], y0), (key[1], y1_cur)))  # inclusive y_end

        active = next_active

    # finalize remaining active after last row
    for key, (y0, y1_cur) in active.items():
        rects.append(((key[0], y0), (key[1], y1_cur)))

    return rects

def rectangles_per_z(voxels):
    """
    Build rectangles per Z layer using greedy XY merging.
    Returns dict: z -> list of ((x1,y1),(x2,y2)) rects (inclusive).
    """
    # Group occupied (x,y) per z
    by_z = defaultdict(list)
    for x, y, z in voxels:
        by_z[z].append((x, y))

    rects_z = {}
    for z, xy_list in by_z.items():
        xs = [x for x, _ in xy_list]
        ys = [y for _, y in xy_list]
        x_min, x_max = min(xs), max(xs)
        y_min, y_max = min(ys), max(ys)

        W = x_max - x_min + 1
        H = y_max - y_min + 1

        grid = np.zeros((H, W), dtype=bool)
        for x, y in xy_list:
            grid[y - y_min, x - x_min] = True  # note: rows are Y

        # Greedy rectangle merge on this slice
        rects = greedy_xy_merge_rectangles(grid)

        # Map back to absolute voxel indices
        rects_abs = []
        for (x1, y1), (x2, y2) in rects:
            rects_abs.append(((x1 + x_min, y1 + y_min), (x2 + x_max - (x_max - x_min), y2 + y_max - (y_max - y_min))))
        rects_z[z] = rects_abs
    return rects_z

def boxes_to_mesh_from_rectangles(rects_z, voxel_size, origin):
    """
    Convert per-Z rectangles into a single Open3D mesh of axis-aligned boxes.
    Each rectangle becomes one box of size ((x2+1 - x1)*vx, (y2+1 - y1)*vy, 1*vz).
    """
    mesh = o3d.geometry.TriangleMesh()
    vz = voxel_size

    for z, rects in rects_z.items():
        for (x1, y1), (x2, y2) in rects:
            # size in voxels (inclusive -> +1)
            nx = (x2 - x1 + 1)
            ny = (y2 - y1 + 1)
            nz = 1

            size = np.array([nx * voxel_size, ny * voxel_size, nz * voxel_size], dtype=float)

            # world-space min corner = origin + (index * voxel_size)
            min_corner = origin + np.array([x1 * voxel_size, y1 * voxel_size, z * voxel_size], dtype=float)

            box = o3d.geometry.TriangleMesh.create_box(*size)
            box.translate(min_corner)
            mesh += box

    mesh.remove_duplicated_vertices()
    mesh.remove_unreferenced_vertices()
    mesh.remove_duplicated_triangles()
    mesh.remove_degenerate_triangles()
    return mesh

def save_gazebo_model(mesh, model_name, output_dir):
    """Write OBJ + model.sdf + model.config + world file."""
    model_path = os.path.join(output_dir, model_name)
    mesh_dir = os.path.join(model_path, "meshes")
    os.makedirs(mesh_dir, exist_ok=True)

    mesh_file = os.path.join(mesh_dir, "map.obj")
    o3d.io.write_triangle_mesh(mesh_file, mesh, write_vertex_normals=False)

    with open(os.path.join(model_path, "model.config"), "w") as f:
        f.write(f"""<?xml version="1.0" ?>
<model>
  <name>{model_name}</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author><name>Auto</name></author>
  <description>Greedy XY merged voxel mesh</description>
</model>
""")
    with open(os.path.join(model_path, "model.sdf"), "w") as f:
        f.write(f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{model_name}">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry><mesh><uri>model://{model_name}/meshes/map.obj</uri></mesh></geometry>
      </collision>
      <visual name="visual">
        <geometry><mesh><uri>model://{model_name}/meshes/map.obj</uri></mesh></geometry>
      </visual>
    </link>
  </model>
</sdf>
""")
    world_file = os.path.join(output_dir, f"{model_name}.world")
    with open(world_file, "w") as f:
        f.write(f"""<sdf version="1.6">
<world name="default">
  <include><uri>model://{model_name}</uri></include>
</world>
</sdf>
""")
    return mesh_file, world_file

def pcd_to_greedy_xy_mesh(pcd_file, model_name="map", output_dir=".", voxel_size=0.2, write_gazebo=True):
    output_dir = os.path.expanduser(output_dir)

    # 1) Voxelize
    voxels, vs, origin = pcd_to_voxel_grid(pcd_file, voxel_size=voxel_size)
    if voxels.size == 0:
        raise ValueError("No occupied voxels after voxelization. Try a larger voxel_size.")

    # 2) Rectangles per Z via greedy X+Y merging
    rects_z = rectangles_per_z(voxels)

    # 3) Build mesh
    mesh = boxes_to_mesh_from_rectangles(rects_z, vs, origin)

    # 4) Save OBJ (+ Gazebo descriptors)
    if write_gazebo:
        os.makedirs(os.path.expanduser(output_dir), exist_ok=True)
        mesh_path, world_path = save_gazebo_model(mesh, model_name, output_dir)
        print(f"[INFO] OBJ saved: {mesh_path}")
        print(f"[INFO] World:     {world_path}")
    else:
        o3d.io.write_triangle_mesh("map.obj", mesh, write_vertex_normals=False)
        print("[INFO] OBJ saved: map.obj")

    return mesh

if __name__ == "__main__":
    # Tune voxel_size to control detail vs. number of boxes (e.g., 0.1 for fine, 0.3 for coarse)
    pcd_file = "maze.pcd"
    pcd_to_greedy_xy_mesh(pcd_file, model_name="maze", output_dir=".", voxel_size=0.1)

