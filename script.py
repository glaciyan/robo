import numpy as np
import matplotlib.pyplot as plt
from transformations import *

class Robot:
    def __init__(self, x_r=2, y_r=1, theta=np.radians(30), l=0.6, h=0.2, r=0.1, a=0.0, b=0.0, l_1=0.6, l_2=0.4):
        self.x_r = x_r
        self.y_r = y_r
        self.theta = theta
        self.l = l
        self.h = h
        self.r = r
        self.a = a
        self.b = b
        self.l_1 = l_1
        self.l_2 = l_2
        self.beta_1_limit = [np.deg2rad(10.0), np.deg2rad(170.0)]

    # Exercise 2.1
    def forward_kinematics(self, alpha=np.radians(0), beta_1=np.radians(0), beta_2=np.radians(0)):
        p_a2 = (0, 0, 0, 1)
        TR_0 = trans(np.array([self.x_r, self.y_r, self.r])) @ rot2trans(rotz(self.theta))
        TR_D = trans(np.array([self.l/2 - self.a/2, 0, self.h + self.b])) @ rot2trans(rotz(alpha)) 
        TR_R = rot2trans(rotx(np.radians(90)))
        TR_A1 = rot2trans(rotz(beta_1)) @ trans(np.array([self.l_1, 0, 0]))
        TR_A2 = rot2trans(rotz(beta_2)) @ trans(np.array([self.l_2, 0, 0]))
        # TR = TR_0 @ TR_D @ TR_R @ TR_A1 @ TR_A2
        TR = TR_R @ TR_A1 @ TR_A2
        p_a0 = TR @ p_a2
        return p_a0

    def forward_kinematics_j1(self, alpha=np.radians(0), beta_1=np.radians(0), beta_2=np.radians(0)):
        p_a2 = (0, 0, 0, 1)
        TR_0 = trans(np.array([self.x_r, self.y_r, self.r])) @ rot2trans(rotz(self.theta))
        TR_D = trans(np.array([self.l/2 - self.a/2, 0, self.h + self.b])) @ rot2trans(rotz(alpha)) @ rot2trans(rotx(np.radians(90)))
        TR_A1 = rot2trans(rotz(beta_1)) @ trans(np.array([self.l_1, 0, 0]))
        TR = TR_0 @ TR_D @ TR_A1
        p_a0 = TR @ p_a2
        return p_a0

    # Exercise 2.2
    def inverse_kinematics(self, p_r):
        p_a2 = (0, 0, 0, 1)
        TR_D = trans(np.array([self.l/2 - self.a/2, 0, self.h + self.b]))
        base = TR_D @ p_a2

        cc = p_r[:3] - base[:3]

        a = np.sqrt(cc[0]**2 + cc[1]**2 + cc[2]**2)
        if a > self.l_1+self.l_2:
            raise ValueError

        e = -1
        c = (a**2 - self.l_1**2 - self.l_2**2) / (2 * self.l_1)
        b = e * np.sqrt(self.l_2**2 - c**2)

        beta_2 = np.arctan2(b,c)

        sphere_radius = a
        azimuth = np.arctan2(cc[1], cc[0])
        polar = np.arcsin(cc[2] / sphere_radius)

        beta_1 = polar - np.arctan2(b, self.l_1 + c)

        alpha = azimuth

        return alpha, beta_1, beta_2

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

