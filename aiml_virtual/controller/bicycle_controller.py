from aiml_virtual.controller import controller


class BicycleController(controller.Controller):
    def __init__(self):
        super().__init__()

    def compute_control(self, *args, **kwargs) -> float:
        return 0.1
