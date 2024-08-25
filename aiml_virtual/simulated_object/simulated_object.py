import xml.etree.ElementTree as ET
from abc import ABC, abstractmethod
from typing import Optional, Type

import mujoco


class SimulatedObject(ABC):
    # We will use this registry to parse XML files for objects to add to scene registries.
    xml_registry: dict[str, Type['SimulatedObject']] = {}
    instance_count: dict[Type['SimulatedObject'], int] = {}

    @classmethod
    @abstractmethod
    def get_identifiers(cls) -> Optional[list[str]]:
        """
        Must return None or a list of identifiers (strings) that will be used to identify the class in XML.
        """
        pass

    # The __init_subclass__ function gets called when a subclass is defined. A subclass of SimulatedObject will be
    # (for example) MovingObject, meaning that when the interpeter encounters MovingObject, it calls __init_subclass__,
    # with the argument cls set to MovingObject. Since MovingObject is a subclass of SimulatedObject, it inherits this
    # function, meaning that if MovingObject has a subclass Drone, then whenever Drone is defined, this function gets
    # called, even though Drone is not a direct subclass of SimulatedObject. If MovingObject also implements its own
    # __init_subclass__, it will overload the inherited __init_subclass__, therefore, if we want to call it, then
    # MovingObject.__init_subclass__ shall include a call super().__init_subclass__, that way it calls the parent's
    # respective function in addition to its own.
    def __init_subclass__(cls, **kwargs):
        super().__init_subclass__(**kwargs)
        #  A class descended from SimulatedObject *must* specify whether he is a candidate for parsing. If the user
        #  doesn't want the class to be a candidate for parsing (because the class is abstract, or for any other
        #  reason), the get_identifiers() function shall return None
        identifiers: Optional[str] = cls.get_identifiers()
        if identifiers is None:
            return
        else:
            for identifier in identifiers:
                if identifier in SimulatedObject.xml_registry:
                    raise ValueError(f"identifier {identifier} is already registered to "
                                     f"{SimulatedObject.xml_registry[identifier].__name__}")
                else:
                    SimulatedObject.xml_registry[identifier] = cls
        SimulatedObject.instance_count[cls] = 0

    def __init__(self):
        cls = self.__class__
        self.name = f"{cls.__name__}_{SimulatedObject.instance_count[cls]}"
        SimulatedObject.instance_count[cls] += 1
        self.model: Optional[mujoco.MjModel] = None
        self.data: Optional[mujoco.MjData] = None

    def bind_to_model(self, model: mujoco.MjModel) -> None:
        self.model = model

    @abstractmethod
    def bind_to_data(self, data: mujoco.MjData):
        pass

    # TODO: this probably won't be control step: rename i and control step to something more representative
    @abstractmethod
    def update(self, i: int, step: float) -> None:
        pass

    @abstractmethod
    def create_xml_element(self, pos: str, quat: str, color: str) -> dict[str, list[ET.Element]]:
        pass



