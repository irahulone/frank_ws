##### This class is developed to produce the kinematic equations for the rover mobility. #####
import time
import numpy as np

from . import RobotDefinition

class RoverDescription(RobotDefinition.RobotDefinition):
    # Physical parameters of the robot.
    def __init__(self, name, robot_measurement):
        super().__init__(name, robot_measurement) 
        self.link_length = robot_measurement[0]
        self.wheel_radius = robot_measurement[1]

    # Kinematic equation to define the indoor rover.
    def kinematic_equation(self, velocity_vector):
        # Link lengths.
        l1 = self.link_length[0]
        l2 = self.link_length[1]
        radius = self.wheel_radius
        
        # Kinematic matrix.
        kineMatrix = np.array([[1, 1, -(l1+l2)],
                              [1, -1, (l1+l2)],
                              [1, -1, -(l1+l2)],
                              [1, 1, (l1+l2)]])

        # Kinematic equation to retrieve angular velocity vector.
        angular_vel_vector = 1/radius * np.dot(kineMatrix, velocity_vector)
        
        return angular_vel_vector

    # Convert velocity to rpm value.
    def vel_to_rpm(self, angular_vel_vector, weight=1):
        return (1/6)*weight*angular_vel_vector