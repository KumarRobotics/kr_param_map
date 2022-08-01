# Kumar Lab Parameterized Map


All in one! It's a revolution of map parameterization and representation for motion planning! Let's see.



## 0. Setup




### 0.1 general map annotation


<p align="center">
  <img src="docs/map_annotation.png" width = "400" height = "400"/>
</p>


## 1. Usage




### 1.1 Grid Map Reader

It support file format as:

- rosbag: sensor_msgs::PointCloud
- rosbag: sensor_msgs::PointCloud2
- pcd

set the map mode and file path in "read_grid_map.launch" and launch 
```
roslaunch param_env read_grid_map.launch
```


<p align="center">
  <img src="docs/pc1.png" width = "390" height = "390"/>
  <img src="docs/pc2.png" width = "390" height = "390"/>
</p>


You can also set the inflation ratio to inflate the grid map

```
<param name="map/inflate_ratio" value="0.3"/>
```


<p align="center">
  <img src="docs/pcd.png" width = "390" height = "390"/>
  <img src="docs/pcd_inf.png" width = "390" height = "390"/>
</p>


The topics:

- /read_grid_map/global_cloud: publish the point clouds 
- /read_grid_map/global_gridmap: publish the center points of the grid map


### 1.2 Sctructed Map Generator


```
roslaunch param_env structure_map.launch
```

You can adjust the apprximate ratio of each element (overlapping is also counting now) in the launch file 

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

By increasing the occupied ratios, it's harder to generate feasible trajectories

<p align="center">
  <img src="docs/exp_cy1.png" width = "280" height = "280"/>
  <img src="docs/exp_cy2.png" width = "280" height = "280"/>
  <img src="docs/exp_cy3.png" width = "280" height = "280"/>
</p>


You can also change the resolution online by publishing the resolution value to:

```
/structure_map/change_res
```


<p align="center">
  <img src="docs/res_01.png" width = "390" height = "390"/>
  <img src="docs/res_02.png" width = "390" height = "390"/>
  <img src="docs/res_05.png" width = "390" height = "390"/>
  <img src="docs/res_10.png" width = "390" height = "390"/>
</p>
