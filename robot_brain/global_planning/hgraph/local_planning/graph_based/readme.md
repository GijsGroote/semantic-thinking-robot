**Occupancy graph**

## variable convention
cart_2d = (position_x, position_y): 2-dimensional carthesian coordinate. 
c_idx = (x_idx, y_idx): index in occupancy grid map corresponding to the 2-dimensional cartesian coordinates.

pose_2d = (position_x, position_y, orientation): 2-dimensional pose.
p_idx = (x_idx, y_idx, orien_idx): index in occupancy grid map corresponding to the 2-dimensional poses.

## Occupancy grid sketch
![Occupancy grid sketch](../../../images/occupancy_grid_sketch.jpeg)
The origin is in the center, with the positive x-axis facing south and the positive y-axis facing east.

# TODO: 
- convert a shape + orientation to points on a 2-dimensional map, the shadow if ligth would be shining perpendicular to the ground plane 
- there is a potential bug, to find if a point is in a rectangle. a check varifies is a point can be projected to 2 edges, (that would work for rectangular shapes and diamond shapes could be created with a shadow function)
- testing, test the grid, how should it look after initialisation, after the shortest path algorithm ran?


