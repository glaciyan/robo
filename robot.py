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
        TR = TR_0 @ TR_D @ TR_R @ TR_A1 @ TR_A2
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
        # TODO throw error when outside of working space

        alpha = azimuth

        return alpha, beta_1, beta_2

    def get_points(self, alpha, beta_1, beta_2):
        p_a2 = (0, 0, 0, 1)
        TR_0 = trans(np.array([self.x_r, self.y_r, self.r])) @ rot2trans(rotz(self.theta))
        TR_D = trans(np.array([self.l/2 - self.a/2, 0, self.h + self.b])) @ rot2trans(rotz(alpha)) 
        TR_R = rot2trans(rotx(np.radians(90)))
        T_BASE = TR_0 @ TR_D @ TR_R

        base = T_BASE @ p_a2

        TR_A1 = rot2trans(rotz(beta_1)) @ trans(np.array([self.l_1, 0, 0]))
        j1 = T_BASE @ TR_A1 @ p_a2

        TR_A2 = rot2trans(rotz(beta_2)) @ trans(np.array([self.l_2, 0, 0]))
        TR = T_BASE @ TR_A1 @ TR_A2
        j2 = TR @ p_a2 # tcp
        return base, j1, j2
