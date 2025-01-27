import numpy as np
import matplotlib.pyplot as plt
import pickle
from robot import Robot, OutOfWorkspace
from shapely.geometry import shape


def plot_sim(robot: Robot):
    plt.figure(figsize=(20, 10))
    plt.xlabel("x")
    plt.ylabel("z")
    plt.axis("equal")

    target_1 = [0.9, 0.0, -0.05, 1]
    target_2 = [-0.2, -0.3, 0.25, 1]

    alpha_auf, beta_1_auf, beta_2_auf = robot.inverse_kinematics(np.array(target_1), elbow_up=True)

    base, j1, j2 = robot.get_points(alpha_auf, beta_1_auf, beta_2_auf)
    ax = plt.subplot(1, 2, 1)
    plt.axis("equal")
    plt.xticks(np.array([base[0], j1[0], j2[0]]))
    plt.yticks(np.array([base[2], j1[2], j2[2]]))
    plt.scatter(np.array([base[0], j1[0], j2[0]]), np.array([base[2], j1[2], j2[2]]), c=[6,3,1], cmap="rainbow", label="Base")
    plt.plot(np.array([base[0], j1[0], j2[0]]), np.array([base[2], j1[2], j2[2]]), 'r-', label="Pauf Pose")
    plt.legend()

    alpha_ab, beta_1_ab, beta_2_ab = robot.inverse_kinematics(np.array(target_2), elbow_up=True)

    base, j1, j2 = robot.get_points(alpha_ab, beta_1_ab, beta_2_ab)

    plt.subplot(1, 2, 2)
    plt.axis("equal")
    plt.yticks(np.array([base[2], j1[2], j2[2]]))
    plt.scatter(np.array([base[0], j1[0], j2[0]]), np.array([base[2], j1[2], j2[2]]), c=[6,3,1], cmap="rainbow", label="Base")
    plt.plot(np.array([base[0], j1[0], j2[0]]), np.array([base[2], j1[2], j2[2]]), 'r-', label="Pab Pose")
    

    plt.legend()
    plt.savefig("plots/pick_up_sim.png")

    plt.figure(figsize=(10, 10))
    # alpha_ab = 1.4
    alpha_interp = np.linspace(alpha_auf, alpha_ab - np.deg2rad(360), 100)
    alpha_left_interp = np.linspace(alpha_auf, alpha_ab, 100)
    print(f"{np.rad2deg(alpha_ab)} {np.rad2deg(alpha_ab - np.deg2rad(360))}")
    beta_1_interp = np.linspace(beta_1_auf, beta_1_ab, 100)
    beta_2_interp = np.linspace(beta_2_auf, beta_2_ab, 100)
    plt.plot(np.rad2deg(alpha_interp), "gray" if np.abs(alpha_ab - np.deg2rad(360) * np.sign(360)) >= np.pi else "r", label="alpha right rotation")
    plt.plot(np.rad2deg(alpha_left_interp), "gray" if np.abs(alpha_ab) > np.pi else "r", label="alpha left rotation")
    plt.plot(np.rad2deg(beta_1_interp), label="beta 1")
    plt.plot(np.rad2deg(beta_2_interp), label="beta 2")
    plt.legend()
    plt.xlabel("Percent (%)")
    plt.ylabel("Joint Angle (Grad)")
    plt.title("Vollsynchrone PTP von P_auf zu P_ab")
    plt.savefig("plots/sim.png")


if __name__ == '__main__':
    robot = Robot(x_r=0.0, y_r=0.0, theta=0, l=0, a=0, r=0, h=0, b=0)
    plot_sim(robot)