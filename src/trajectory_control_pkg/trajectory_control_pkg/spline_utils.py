import numpy as np
from scipy.interpolate import splprep, splev

def smooth_path(waypoints, num_points=200):
    waypoints = np.array(waypoints)
    x = waypoints[:, 0]
    y = waypoints[:, 1]

    tck, _ = splprep([x, y], s=0.5)
    u_fine = np.linspace(0, 1, num_points)
    x_smooth, y_smooth = splev(u_fine, tck)

    return list(zip(x_smooth, y_smooth))

