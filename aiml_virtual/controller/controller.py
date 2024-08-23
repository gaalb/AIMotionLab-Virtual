from abc import ABC, abstractmethod
from typing import Any


class Controller(ABC):
    def __init__(self):
        pass

    @abstractmethod
    def compute_control(self, *args, **kwargs) -> Any:  # Todo: any?
        pass
