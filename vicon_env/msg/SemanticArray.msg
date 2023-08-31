std_msgs/Header header
int32 mav_id #-1 for global map, 0 + for the mav_id
Point3d mav_pos

#2d semantics
Ellipse[] ellipses
Circle[]  circles
Polygon[] polygons

#3d semantics
Cylinder[] cylinders
Polyhedron[] polyhedrons
