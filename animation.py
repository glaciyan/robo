from manim import *
import numpy as np
from manim.utils.color.X11 import CYAN1, MAGENTA

from robot import Robot, OutOfWorkspace


class RobotAnim(ThreeDScene):
    def construct(self):
        robot = Robot(x_r=0.0, y_r=0.0, theta=0, l=0, a=0, r=0, h=0, b=0)

        size_multiplier = 4
        # Set up 3D camera

        self.set_camera_orientation(phi=75 * DEGREES, theta=30 * DEGREES, zoom=3)

        # Parameters for the robot arm
        link1_length = robot.l_1
        link2_length = robot.l_2
        base_position = ORIGIN

        # Create the spinning plate
        plate = Circle(radius=0.1, color=BLUE).set_fill(BLUE, opacity=0.3)
        # plate.rotate(PI / 2, axis=RIGHT)  # Rotate plate to lie on the Z-plane
        # plate_center_dot = Dot(base_position, color=WHITE)

        # Robot arm components
        joint1 = Dot3D(base_position, color=RED)
        link1 = Line(start=base_position, end=base_position + RIGHT * link1_length, color=GREEN)
        joint2 = Dot3D(link1.get_end(), color=RED)
        link2 = Line(start=link1.get_end(), end=link1.get_end() + RIGHT * link2_length, color=YELLOW)
        end_effector = Dot3D(link2.get_end(), color=WHITE)

        target1 = [0.9, 0.0, -0.05, 1]
        target2 = [-0.2, 0.0, 0.25, 1]

        target_point_1 = Dot3D(target1[0:3], color=CYAN1)
        target_point_2 = Dot3D(target2[0:3], color=MAGENTA)

        axes = ThreeDAxes(x_range=(-3, 3, 1), y_range=(-2, 2, 1), z_range=(-2, 2, 1))

        # Add components to the scene
        self.add(axes, axes.get_axis_labels(), plate, link1, link2, joint1, joint2, end_effector, target_point_1, target_point_2)


        try:
            robot_config_Pauf = robot.inverse_kinematics(np.array(target1), elbow_up=True)
        except OutOfWorkspace:
            print("Using elbow down config for target 1")
            robot_config_Pauf = robot.inverse_kinematics(np.array(target1), elbow_up=False)


        try:
            robot_config_Pab = robot.inverse_kinematics(np.array(target2), elbow_up=True)
        except OutOfWorkspace:
            print("Using elbow down config for target 2")
            robot_config_Pab = robot.inverse_kinematics(np.array(target2), elbow_up=False)

        run_time = 5 # seconds


        def update_arm(arm_parts, alpha):
            cc = robot_config_Pauf
            base, j1, j2 = robot.get_points(cc[0], cc[1], cc[2])
            joint1.move_to(base)
            link1.put_start_and_end_on(base, j1)
            joint2.move_to(j1)
            link2.put_start_and_end_on(j1, j2)
            end_effector.move_to(j2)

        # Robot arm movement (controlled with alpha from 0 to 1)
        self.play(UpdateFromAlphaFunc(
            VGroup(link1, joint2, link2, end_effector), update_arm,
            run_time=run_time, rate_func=linear
        )   )

        # Pause to admire the scene
        self.wait(2)
