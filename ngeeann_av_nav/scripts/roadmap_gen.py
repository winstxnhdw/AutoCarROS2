#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
inner = 99
road_width = 7.5

origin_x, origin_y = -130, -130
width, height = 520, 520
resolution = 0.5
angle_inc = 0.001

grid = np.ones((width, height))

def set_cell(x, y, val):
    """ Set the value of a cell in the grid. 

    Arguments: 
        x, y  - This is a point in the map coordinate frame.
        val   - This is the value that should be assigned to the
                grid cell that contains (x,y).
    """

    ix = int((x - origin_x) / resolution)
    iy = int((y - origin_y) / resolution)
    if ix < 0 or iy < 0 or ix >= width or iy >= height:
        pass
    grid[iy, ix] = val

theta = 0

for r in np.arange(inner, inner+road_width, resolution):

    for theta in np.arange(0, 2*np.pi, angle_inc):

        x = r * np.cos(theta)
        y = r * np.sin(theta)

        try:
            set_cell(x, y, 0)
        except:
            pass

plt.imshow(grid)
plt.show()