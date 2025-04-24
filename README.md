# Kumar Lab Parameterized Map


All in one! It's a revolution in map parameterization and representation for motion planning.


## About

This repo is related to our evaluation research. If this repo helps your research, please cite our paper at:

```
@INPROCEEDINGS{10610207,
  author={Shao, Yifei Simon and Wu, Yuwei and Jarin-Lipschitz, Laura and Chaudhari, Pratik and Kumar, Vijay},
  booktitle={2024 IEEE International Conference on Robotics and Automation (ICRA)}, 
  title={Design and Evaluation of Motion Planners for Quadrotors in Environments with Varying Complexities}, 
  year={2024},
  volume={},
  number={},
  pages={10033-10039},
  keywords={Measurement;Navigation;Software algorithms;Kinematics;Planning;Complexity theory;Trajectory},
  doi={10.1109/ICRA57147.2024.10610207}}

```



## 0. Setup


### 0.1 general map annotation


<p align="center">
  <img src="docs/map_annotation.png" width = "400" height = "400"/>
</p>


## 1. Usage


### 1.1 Grid Map Reader

Set the map mode and file path in "read_grid_map.launch" and launch 
```
roslaunch param_env read_grid_map.launch
```

We support file format as:

- image map: .png 
- rosbag: sensor_msgs::PointCloud
- rosbag: sensor_msgs::PointCloud2
- pcd

In the launch file, set "map/mode" to switch to a different mode, and use the following examples to load the map.
```
<!-- map mode 
       0 randomly generate
       1 read the image
       2 read the ros bag poind cloud 1
       3 read the ros bag poind cloud 2
       4 read pcd file -->
<param name="map/mode"       value="4" />
```

We also enable multi-map loading, set folder path, and use_folder as true. If you only want to load a single file, please comment the following:

```
<param name="folder_path" value="$(find param_env)/data/img/maze/"/>
<param name="use_folder"  value="false"/>
```


- The 2D image map is converted into 3d, with z-axis having the same content. 

- Maze images are generated with the help of [Multi Solution Maze Generator](https://github.com/shaoyifei96/multi_solution_mazegenerator)

<p align="center">
  <img src="docs/img_map1.png" width = "390" height = "390"/>
  <img src="docs/img_maze1.png" width = "390" height = "390"/>
</p>

The point cloud in bags is shown as:

<p align="center">
  <img src="docs/pc1.png" width = "390" height = "390"/>
  <img src="docs/pc2.png" width = "390" height = "390"/>
</p>


You can also set the inflation (m) to inflate the grid map

```
<param name="map/inflate_radius" value="0.3"/>
```

<p align="center">
  <img src="docs/pcd.png" width = "390" height = "390"/>
  <img src="docs/pcd_inf.png" width = "390" height = "390"/>
</p>


The topics:

- /read_grid_map/global_cloud: publish the point clouds 
- /read_grid_map/global_gridmap: publish the center points of the grid map with inflation


### 1.2 Sctructed Map Generator


```
roslaunch param_env structure_map.launch
```

You can adjust the approximate ratio of each element (overlapping is also counting now) in the launch file 

```
<param name="map/cylinder_ratio" value="0.10" type="double"/>
<param name="map/circle_ratio"   value="0.02" type="double"/>
<param name="map/gate_ratio"     value="0.02" type="double"/>
<param name="map/ellip_ratio"    value="0.02" type="double"/>
<param name="map/poly_ratio"     value="0.01" type="double"/>
```

Examples:

<p align="center">
  <img src="docs/exp_gate1.png" width = "390" height = "390"/>
  <img src="docs/exp_gate2.png" width = "390" height = "390"/>
  <img src="docs/exp_all2.png" width = "390" height = "390"/>
  <img src="docs/cluttered.png" width = "390" height = "390"/>
</p>

By increasing the occupied ratios, it's harder to generate feasible trajectories.

<p align="center">
  <img src="docs/exp_cy1.png" width = "280" height = "280"/>
  <img src="docs/exp_cy2.png" width = "280" height = "280"/>
  <img src="docs/exp_cy3.png" width = "280" height = "280"/>
</p>

You can also enable noise around the obstacles by setting:

```
  <param name="params/add_noise" value="true"/>
```


The topics:

- /structure_map/global_cloud: publish the point clouds 


### 1.3 Change resolution or change map


You can also change the resolution online by publishing the resolution value to:

```
/structure_map/change_res
```
or

```
/read_grid_map/change_res
```

<p align="center">
  <img src="docs/res_01.png" width = "390" height = "390"/>
  <img src="docs/res_02.png" width = "390" height = "390"/>
  <img src="docs/res_05.png" width = "390" height = "390"/>
  <img src="docs/res_10.png" width = "390" height = "390"/>
</p>


For read maps, you can publish all the maps by setting the folder path in the launch file.

```
 <param name="folder_path" value="$(find param_env)/data/img/maze/"/>

```


Publish true in topic.

```
/structure_map/change_map
```
or

```
/read_grid_map/change_map
```

to trigger the function.


### 1.4 Dataset generation 

If you want to save map, set "dataset/save_map" to true, and set the data number in "dataset/samples_num".

```
    <param name="dataset/save_map"    value="true"/>
    <param name="dataset/samples_num" value="100"/>
    <param name="dataset/start_index" value="5080900"/>
    <param name="dataset/path"        value="$(find param_env)/dataset/"/>

```

It will be saved into the path you set.
