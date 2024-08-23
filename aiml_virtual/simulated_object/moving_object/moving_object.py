import xml.etree.ElementTree as ET
from abc import abstractmethod
import mujoco
from typing import Optional

from aiml_virtual.simulated_object import simulated_object


class MovingObject(simulated_object.SimulatedObject):
    def __init_subclass__(cls, **kwargs):
        super().__init_subclass__(**kwargs)

    def __init__(self):
        super().__init__()
        pass

    @classmethod
    def get_identifiers(cls) -> Optional[list[str]]:
        return None

    @abstractmethod
    def bind_to_model(self, model: mujoco.MjModel):
        pass

    @abstractmethod
    def bind_to_data(self, data: mujoco.MjData):
        pass

    @abstractmethod
    def update(self, i: int, step: float) -> None:
        pass

    @abstractmethod
    def create_xml_element(self, pos: str, quat: str, color: str) -> dict[str, list[ET.Element]]:
        pass

