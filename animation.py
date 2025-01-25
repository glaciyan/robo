from manim import *
import numpy as np

from robot import Robot

class RobotAnim(ThreeDScene):
    def construct(self):
        robot = Robot()
        size_multiplier = 4
        # Set up 3D camera
        self.set_camera_orientation(phi=75 * DEGREES, theta=30 * DEGREES, focal_distance=18)

        # Parameters for the robot arm
        link1_length = robot.l_1 * size_multiplier
        link2_length = robot.l_2 * size_multiplier
        base_position = ORIGIN

        # Create the spinning plate
        plate = Circle(radius=0.3 * size_multiplier, color=BLUE).set_fill(BLUE, opacity=0.3)
        # plate.rotate(PI / 2, axis=RIGHT)  # Rotate plate to lie on the Z-plane
        # plate_center_dot = Dot(base_position, color=WHITE)

        # Robot arm components
        joint1 = Dot3D(base_position, color=RED)
        link1 = Line(start=base_position, end=base_position + RIGHT * link1_length, color=GREEN)
        joint2 = Dot3D(link1.get_end(), color=RED)
        link2 = Line(start=link1.get_end(), end=link1.get_end() + RIGHT * link2_length, color=YELLOW)
        end_effector = Dot3D(link2.get_end(), color=WHITE)

        axes = ThreeDAxes()

        # Add components to the scene
        self.add(axes, plate, link1, joint1, link2, joint2, end_effector)

        def update_arm(arm_parts, alpha):
            # Define angles for the joints (sine wave for smooth motion)
            theta1 = np.pi / 4 * np.sin(alpha * 2 * PI)
            theta2 = np.pi / 3 * np.sin(alpha * 2 * PI + PI / 2)

            # Update positions of link1 and link2
            link1_end = base_position + np.array([
                link1_length * np.cos(theta1),
                0,  # Fixed on the Z-plane for link1
                link1_length * np.sin(theta1)
            ])

            link2_end = link1_end + np.array([
                link2_length * np.cos(theta1 + theta2),
                0,  # Fixed on the Z-plane for link2
                link2_length * np.sin(theta1 + theta2)
            ])

            # Update mobjects
            link1.put_start_and_end_on(base_position, link1_end)
            joint2.move_to(link1_end)
            link2.put_start_and_end_on(link1_end, link2_end)
            end_effector.move_to(link2_end)

        # Robot arm movement (controlled with alpha from 0 to 1)
        self.play(UpdateFromAlphaFunc(
            VGroup(link1, joint2, link2, end_effector), update_arm,
            run_time=5, rate_func=linear
        ))

        # Pause to admire the scene
        self.wait(2)
