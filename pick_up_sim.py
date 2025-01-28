import numpy as np
import matplotlib.pyplot as plt
import pickle
from robot import Robot, OutOfWorkspace
from shapely.geometry import shape


def plot_sim(robot: Robot):
    fig, axes = plt.subplots(1, 2, figsize=(20, 10))

    target_1 = [0.9, 0.0, -0.05, 1]
    target_2 = [-0.2, 0.3, 0.25, 1]




    alpha_auf, beta_1_auf, beta_2_auf = robot.inverse_kinematics(np.array(target_1), elbow_up=True)
    base_auf, j1_auf, j2_auf = robot.get_points(alpha_auf, beta_1_auf, beta_2_auf)

    ax1 = axes[0]
    ax1.set_title("Target 1 (Pauf Pose)")
    ax1.set_xlabel("x")
    ax1.set_ylabel("z")
    ax1.axis("equal")
    ax1.scatter([base_auf[0], j1_auf[0], j2_auf[0]], [base_auf[2], j1_auf[2], j2_auf[2]], c=[6, 3, 1], cmap="rainbow", label="Base")
    ax1.plot([base_auf[0], j1_auf[0], j2_auf[0]], [base_auf[2], j1_auf[2], j2_auf[2]], 'r-', label="Pauf Pose")
    ax1.legend()




    alpha_ab, beta_1_ab, beta_2_ab = robot.inverse_kinematics(np.array(target_2), elbow_up=True)
    base_ab, j1_ab, j2_ab = robot.get_points(alpha_ab, beta_1_ab, beta_2_ab)

    ax2 = axes[1]
    ax2.set_title("Target 2 (Pab Pose)")
    ax2.set_xlabel("x")
    ax2.set_ylabel("z")
    ax2.axis("equal")
    ax2.scatter([base_ab[0], j1_ab[0], j2_ab[0]], [base_ab[2], j1_ab[2], j2_ab[2]], c=[6, 3, 1], cmap="rainbow", label="Base")
    ax2.plot([base_ab[0], j1_ab[0], j2_ab[0]], [base_ab[2], j1_ab[2], j2_ab[2]], 'r-', label="Pab Pose")
    ax2.legend()

    plt.tight_layout()
    plt.savefig("plots/pick_up_sim.png")

    plt.figure(figsize=(10, 10))
    # alpha_ab = 1.4
    alpha_interp = np.linspace(alpha_auf, alpha_ab - np.deg2rad(360), 100)
    alpha_left_interp = np.linspace(alpha_auf, alpha_ab, 100)
    # print(f"{np.rad2deg(alpha_ab)} {np.rad2deg(alpha_ab - np.deg2rad(360))}")



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
    robot = Robot(x_r=0.0, y_r=0.0, theta=0)
    plot_sim(robot)