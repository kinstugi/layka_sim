from math import atan2
from robot_control.supervisor_controller_interface import SupervisorControllerInterface
from utils import linalg2_util as linalg



class SearchRobotsController:
    def __init__(self, supervisor: SupervisorControllerInterface) -> None:
        self.supervisor = supervisor

        # gains
        self.kP = 5.0
        self.kI = 0.0
        self.kD = 0.0

        # stored values - for computing next results
        self.prev_time = 0.0
        self.prev_eP = 0.0
        self.prev_eI = 0.0

        # key vectors and data (initialize to any non-zero vector)
        self.search_heading_vector = [1.0, 0.0]
    
    def update_heading(self):
        self.search_heading_vector = self.calculate_search_heading_vector()
    
    def execute(self):
        current_time = self.supervisor.time()
        dt = current_time - self.prev_time

        # calculate the error terms
        theta_d = atan2(self.gtg_heading_vector[1], self.gtg_heading_vector[0])
        eP = theta_d
        eI = self.prev_eI + eP * dt
        eD = (eP - self.prev_eP) / dt

        omega = self.kP * eP + self.kI * eI + self.kD * eD

        v = self.supervisor.v_max() / (abs(omega) + 1) ** 0.5

        self.prev_time = current_time
        self.prev_eP = eP
        self.prev_eI = eI

        self.supervisor.set_outputs(v, omega)

    def calculate_search_heading_vector(self):
        robot_inv_pos, robot_inv_theta = (
            self.supervisor.estimated_pose().inverse().vunpack()
        )

        goal = self.supervisor.goal()
        goal = linalg.rotate_and_translate_vector(goal, robot_inv_theta, robot_inv_pos)
