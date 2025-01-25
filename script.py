import numpy as np
import matplotlib.pyplot as plt
from transformations import *
from robot import Robot

# Exercise 2.3
def joint_angles_for_circle_motion(robot, km_r=(1, 0, 0.55, 1), radius=0.25, positions_in_circle=100):
    # positions_in_circle specifies how many points of the circle should be approached by the arm.
    # This function is not intended for plotting.
    # Instead, it calculates the progression of the joint angles.
    theta = np.linspace(0, 2*np.pi, positions_in_circle)
    y = radius * np.sin(theta)
    z = radius * np.cos(theta)
    alphas, beta_1s, beta_2s = zip(*[robot.inverse_kinematics(km_r + np.array([0, y_i, z_i, 1])) for y_i, z_i in zip(y, z)])
    return alphas, beta_1s, beta_2s


def plot_circle(reached_points):
    plt.figure(figsize=(10,10))
    plt.xlabel("x")
    plt.ylabel("z")
    plt.axis("equal")
    plt.plot(reached_points[:, 0], reached_points[:, 2], marker='', linestyle='-', markersize=3)
    plt.scatter([0], [0], color='r')
    plt.savefig("plots/kreis.png")


if __name__ == '__main__':
    # Create plots with regular execution. Everything else should be run within tests.
    robot = Robot()


    points = []
    for p1 in np.linspace(robot.beta_1_limit[0], robot.beta_1_limit[1], 500):
        for p2 in np.linspace(0, np.deg2rad(360), 100):
            points.append((p1, p2))
    reachedPoints = np.array([robot.forward_kinematics(0.0, beta_1, beta_2) for beta_1, beta_2 in points])
    plot_circle(reachedPoints)

