# Occupancy graph

## variable convention

position
position_2d

idx: the index in the grid_map, this can be either 2 dimensional (x_idx, y_idx) or 3-dimenional (adding orien_idx)



# TODO: 
- convert a shape + orientation to points on a 2-dimensional map, the shadow if ligth would be shining perpendicular to the ground plane 
- there is a potential bug, to find if a point is in a rectangle. a check varifies is a point can be projected to 2 edges, (that would work for rectangular shapes and diamond shapes could be created with a shadow function)
- testing, test the grid, how should it look after initialisation, after the shortest path algorithm ran?


