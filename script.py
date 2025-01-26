import numpy as np
import matplotlib.pyplot as plt
from robot import Robot

def plot_working_area(reached_points, robot: Robot):
    plt.figure(figsize=(10,10))
    plt.xlabel("x")
    plt.ylabel("z")
    plt.axis("equal")
    plt.plot(reached_points[:, 0], reached_points[:, 2], marker='', linestyle='-', markersize=3)
    base, j1, j2 = robot.get_points(0, np.deg2rad(50), np.deg2rad(-50))
    plt.plot(np.array([base[0], j1[0], j2[0]]), np.array([base[2], j1[2], j2[2]]), 'ro-')
    plt.savefig("plots/work_area.png")


if __name__ == '__main__':
    # Create plots with regular execution. Everything else should be run within tests.
    robot = Robot(x_r=0.0, y_r=0.0, theta=0, l=0, a=0, r=0, h=0, b=0)

    points = []
    for p1 in np.linspace(robot.beta_1_limit[0], robot.beta_1_limit[1], 100):
        for p2 in np.linspace(0, np.deg2rad(360), 50):
            points.append((p1, p2))
    reachedPoints = np.array([robot.forward_kinematics(0.0, beta_1, beta_2) for beta_1, beta_2 in points])
    plot_working_area(reachedPoints, robot)

