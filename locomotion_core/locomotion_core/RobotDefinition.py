##### Abstract class to define the physical parameters of a robot. #####

from abc import (
  ABC,
  abstractmethod,
)

class RobotDefinition(ABC):

    def __init__(self, name, robot_measurement):
        self.name = name
        self.robot_measurement = robot_measurement

    @abstractmethod
    def kinematic_equation(self, velocity_vector):
        pass