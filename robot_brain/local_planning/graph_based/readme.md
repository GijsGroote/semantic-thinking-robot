**Configuration Grid Map**

Stores the configuration space of an object in the robot environment with all other objects and obstacles. Configuration space includes _free space_ where the object can move, _obstacle space_ where the object is in collision with obstacles, _movable_ space where an object is that can be moved to another location and _unknown space_ for objects for which it is unknown if they can be moved or not.

## variable convention
cart_2d = (position_x, position_y): 2-dimensional carthesian coordinate. 
c_idx = (x_idx, y_idx): index in occupancy grid map corresponding to the 2-dimensional cartesian coordinates.

pose_2d = (position_x, position_y, orientation): 2-dimensional pose.
p_idx = (x_idx, y_idx, orien_idx): index in occupancy grid map corresponding to the 2-dimensional poses.

## Occupancy grid axis
The origin is in the center, with the positive x-axis facing south and the positive y-axis facing east.
