from abc import ABC, abstractmethod


class Trajectory(ABC):  # move this to a separate file, and make it abstract base
    def __init__(self):
        self.output = {}

    @abstractmethod
    def evaluate(self, state, i, step, time) -> dict:
        pass

