import numpy as np
import matplotlib.pyplot as plt
import pickle
from robot import Robot
from shapely.geometry import shape


def plot_alpha_shape(filename: str, robot: Robot):
    with open(filename, 'rb') as file:
        data = pickle.load(file)
        alpha_shape_geom = shape(data['alpha_shape'])
        # reached_points = data['reached_points']

    plt.figure(figsize=(10, 10))
    plt.xlabel("x")
    plt.ylabel("z")
    plt.axis("equal")

    x, y = alpha_shape_geom.exterior.xy
    plt.fill(x, y, fc="gray", ec="black", alpha=0.4, label="Working Area")

    target = [0.5, 0.0, 0.8, 1]
    # target = [0.0, 0.0, 0.2, 1]

    try:
        alpha, beta_1, beta_2 = robot.inverse_kinematics(np.array(target), elbow_up=True)
    except OutOfWorkspace:
        print("Using elbow down config")
        alpha, beta_1, beta_2 = robot.inverse_kinematics(np.array(target), elbow_up=False)

    base, j1, j2 = robot.get_points(alpha, beta_1, beta_2)
    plt.plot(np.array([base[0], j1[0], j2[0]]), np.array([base[2], j1[2], j2[2]]), 'ro-', label="Example Pose")

    plt.legend()
    plt.savefig("plots/work_area.png")


if __name__ == '__main__':
    robot = Robot(x_r=0.0, y_r=0.0, theta=0, l=0, a=0, r=0, h=0, b=0)

    plot_alpha_shape("alpha_shape_data.pkl", robot)
