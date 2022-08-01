# Kumar Lab Parameterized Map


All in one! It's a revolution of map parameterization and representation for motion planning! Let's see.



## 0. Setup




### general map annotation


<p align="center">
  <img src="docs/map_annotation.png" width = "390" height = "390"/>
</p>


## 1. Usage




### 1.1 Grid Map Reader

It support:

- 
- 
- 

<p align="center">
  <img src="docs/pcd.png" width = "390" height = "390"/>
  <img src="docs/pcd_inf.png" width = "390" height = "390"/>
</p>




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
  <img src="docs/exp_gate2.png" width = "390" height = "390"/>
  <img src="docs/exp_cy2.png" width = "390" height = "390"/>
  <img src="docs/exp_all2.png" width = "390" height = "390"/>
  <img src="docs/cluttered.png" width = "390" height = "390"/>
</p>


