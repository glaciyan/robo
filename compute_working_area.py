import numpy as np
import alphashape
import pickle
from robot import Robot
from shapely.geometry import mapping


def compute_reachable_points(robot: Robot):
    points = []
    for beta_1 in np.linspace(robot.beta_1_limit[0], robot.beta_1_limit[1], 100):
        for beta_2 in np.linspace(0, np.deg2rad(360), 50):
            points.append((beta_1, beta_2))
    return np.array([robot.forward_kinematics(0.0, beta_1, beta_2) for beta_1, beta_2 in points])


def compute_reachable_points_with_alpha(robot: Robot):
    points = []
    for beta_1 in np.linspace(robot.beta_1_limit[0], robot.beta_1_limit[1], 100):
        for beta_2 in np.linspace(0, np.deg2rad(360), 50):
            for alpha in np.linspace(0, np.deg2rad(380), 50):
                points.append((beta_1, beta_2, alpha))
    return np.array([robot.forward_kinematics(alpha, beta_1, beta_2) for beta_1, beta_2, alpha in points])

def compute_and_save_alpha_shape(robot: Robot, filename: str, alpha: float):
    reached_points = compute_reachable_points(robot)

    points_2d = reached_points[:, [0, 2]]
    alpha_shape = alphashape.alphashape(points_2d, alpha)

    with open(filename, 'wb') as file:
        pickle.dump({
            'alpha_shape': mapping(alpha_shape),
            'reached_points': reached_points
        }, file)


if __name__ == '__main__':
    robot = Robot(x_r=0.0, y_r=0.0, theta=0)

    compute_and_save_alpha_shape(robot, "alpha_shape_data.pkl", alpha=7)
