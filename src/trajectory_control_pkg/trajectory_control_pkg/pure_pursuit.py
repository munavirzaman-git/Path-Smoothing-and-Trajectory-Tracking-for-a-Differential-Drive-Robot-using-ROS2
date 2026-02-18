import numpy as np

def pure_pursuit_control(robot_pose, trajectory, lookahead_dist=0.4, v=0.1):
    x, y, theta = robot_pose

    goal_x, goal_y, _ = trajectory[-1]
    if np.hypot(goal_x - x, goal_y - y) < 0.15:
        return 0.0, 0.0, None

    target = None

    for px, py, _ in trajectory:
        dx = px - x
        dy = py - y

        x_r = np.cos(theta)*dx + np.sin(theta)*dy
        dist = np.hypot(dx, dy)

        if dist >= lookahead_dist and x_r > 0:
            target = (px, py)
            break

    if target is None:
        return 0.0, 0.0, None

    dx = target[0] - x
    dy = target[1] - y

    x_r = np.cos(theta)*dx + np.sin(theta)*dy
    y_r = -np.sin(theta)*dx + np.cos(theta)*dy

    curvature = 2 * y_r / (lookahead_dist**2)
    omega = v * curvature
    omega = max(min(omega, 1.5), -1.5)

    return v, omega, target

