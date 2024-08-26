import numpy as np

from aiml_virtual.trajectory import trajectory


class DummyDroneTrajectory(trajectory.Trajectory):
    def __init__(self, target_pos: np.ndarray = np.array([0, 0, 1])):
        super().__init__()
        self.output["load_mass"] = 0.0
        self.output["target_pos"] = target_pos
        self.output["target_rpy"] = np.zeros(3)
        self.output["target_vel"] = np.zeros(3)
        self.output["target_acc"] = np.zeros(3)
        self.output["target_ang_vel"] = np.zeros(3)
        self.output["target_quat_vel"] = np.zeros(4)

    def evaluate(self, state, i, step, time) -> dict:
        return self.output
