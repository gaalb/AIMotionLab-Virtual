from abc import ABC, abstractmethod
from typing import Any


class Controller(ABC):
    def __init__(self):
        pass

    # All controllers will have different inputs and outputs as per the nature of controllers. For this reason, in this
    # controller, I think I'm leaving the signature flexible: might change it later.
    # It bears mentioning that an actual controller makes sense, instead of a simple controller function, as
    # by making it an object, we can assign state to it, such as an integrator.
    @abstractmethod
    def compute_control(self, *args, **kwargs) -> Any:  # Todo: any?
        pass
