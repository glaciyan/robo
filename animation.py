from manim import *
import numpy as np
from robot import Robot

class Robo(ThreeDScene):
    def construct(self):
        # Initialize the robot
        robot = Robot(x_r=0.0, y_r=0.0, theta=0)

        # Set up 3D axes
        self.set_camera_orientation(phi=75 * DEGREES, theta=-45 * DEGREES)
        axes = ThreeDAxes(
            x_range=[-1, 1, 0.2], y_range=[-1, 1, 0.2], z_range=[-1, 1, 0.2],
            x_length=8, y_length=8, z_length=8
        )
        self.add(axes)

        # Define target points
        target_1 = [0.9, 0.0, -0.05, 1]  # Pauf target
        target_2 = [-0.2, 0.3, 0.25, 1]  # Pab target

        # Compute joint angles and positions for Target 1 (Pauf)
        alpha_auf, beta_1_auf, beta_2_auf = robot.inverse_kinematics(np.array(target_1), elbow_up=True)
        base_auf, j1_auf, j2_auf = robot.get_points(alpha_auf, beta_1_auf, beta_2_auf)

        # Compute joint angles and positions for Target 2 (Pab)
        alpha_ab, beta_1_ab, beta_2_ab = robot.inverse_kinematics(np.array(target_2), elbow_up=True)
        base_ab, j1_ab, j2_ab = robot.get_points(alpha_ab, beta_1_ab, beta_2_ab)

        # Interpolate between Pauf and Pab
        steps = 100  # You can adjust this if needed for smoother transitions
        alpha_interp = np.linspace(alpha_auf, alpha_ab - np.deg2rad(360), steps)
        beta_1_interp = np.linspace(beta_1_auf, beta_1_ab, steps)
        beta_2_interp = np.linspace(beta_2_auf, beta_2_ab, steps)

        # Create the initial robotic arm configuration for Pauf
        arm = VGroup(
            Line3D(base_auf, j1_auf, color=BLUE),
            Line3D(j1_auf, j2_auf, color=GREEN),
            Line3D(j2_auf, target_1[:3], color=RED),
        )
        self.add(arm)

        # Label Pauf pose
        label_pauf = Text("Pauf Pose").to_edge(UL).set_color(BLUE)
        self.add_fixed_in_frame_mobjects(label_pauf)

        # Desired total duration of the animation in seconds (e.g., 5 seconds)
        total_duration = 5
        # Calculate the run_time per frame
        run_time_per_frame = total_duration / steps

        # Create a list of transformations
        transforms = []
        for alpha, beta1, beta2 in zip(alpha_interp, beta_1_interp, beta_2_interp):
            base, j1, j2 = robot.get_points(alpha, beta1, beta2)
            new_arm = VGroup(
                Line3D(base, j1, color=BLUE),
                Line3D(j1, j2, color=GREEN),
                Line3D(j2, target_2[:3], color=RED),
            )
            transforms.append(ApplyMethod(arm.become, new_arm, run_time=run_time_per_frame))

        # Play the sequence of transformations
        self.play(*transforms)

        # Add label for Pab pose after the transition
        label_pab = Text("Pab Pose").to_edge(UR).set_color(RED)
        self.add_fixed_in_frame_mobjects(label_pab)

        # Hold the final pose for a moment
        self.wait(2)
