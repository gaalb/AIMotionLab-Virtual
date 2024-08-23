import xml.etree.ElementTree as ET
import mujoco
from typing import Optional, Any
from enum import Enum
import numpy as np

from aiml_virtual.simulated_object.moving_object import moving_object
from aiml_virtual.controller import controller, drone_geom_controller
from aiml_virtual.trajectory import trajectory, dummy_drone_trajectory

PROP_COLOR = "0.1 0.1 0.1 1.0"
PROP_LARGE_COLOR = "0.1 0.02 0.5 1.0"
SITE_NAME_END = "_cog"


class CRAZYFLIE_PROP(Enum):
    #OFFSET = "0.047"
    OFFSET = "0.03275"
    OFFSET_Z = "0.0223"
    MOTOR_PARAM = "0.02514"
    MAX_THRUST = "0.16"
    MASS = "0.028"
    DIAGINERTIA = "1.4e-5 1.4e-5 2.17e-5"
    COG = "0.0 0.0 0.0"


class Drone(moving_object.MovingObject):
    def __init__(self):  # in addition to typing, also check if all of these variables are necessary
        super().__init__()
        self.controller: Optional[controller.Controller] = None  # TODO
        self.actr: Any = None  # TODO: type and explanation, for all under this too (got too lazy to write :) )
        self.ctrl: Any = None  # TODO: type
        self.sensor: Any = None  # TODO: type
        self.xquat: Any = None  # TODO: type
        self.qpos: Any = None  # TODO: type
        self.mass: Any = None  # TODO: type
        self.inertia: Any = None  # TODO: type
        self.prop_qpos: list[Any] = [None, None, None, None]
        self.prop_joint: list[Any] = [None, None, None, None]
        self.ctrl: list[Any] = [None, None, None, None]
        self.prop_angle: list[Any] = [None, None, None, None]
        self.actr_force: list[Any] = [None, None, None, None]
        self.qvel: Any = None
        self.qacc: Any = None
        self.sensor_gyro: Any = None
        self.sensor_velocimeter: Any = None
        self.sensor_accelerometer: Any = None
        self.sensor_posimeter: Any = None
        self.sensor_orimeter: Any = None
        self.sensor_ang_accelerometer: Any = None
        self.state: dict[str, Any] = {}
        self.ctrl_input = np.zeros(4)
        self.trajectory: Optional[trajectory.Trajectory] = None  # TODO: move this to moving_object

        self._create_input_matrix(float(CRAZYFLIE_PROP.OFFSET.value), float(CRAZYFLIE_PROP.OFFSET.value),
                                  float(CRAZYFLIE_PROP.OFFSET.value), float(CRAZYFLIE_PROP.MOTOR_PARAM.value))

    @classmethod
    def get_identifiers(cls) -> Optional[list[str]]:
        # the identifiers to look for in the XML
        return ["Drone", "drone"]

    def _create_input_matrix(self, Lx1, Lx2, Ly, motor_param):
        self.input_mtx = np.array([[1/4, -1/(4*Ly), -1/(4*Lx2),  1 / (4*motor_param)],
                                  [1/4, -1/(4*Ly),   1/(4*Lx1), -1 / (4*motor_param)],
                                  [1/4,  1/(4*Ly),   1/(4*Lx1),  1 / (4*motor_param)],
                                  [1/4,  1/(4*Ly),  -1/(4*Lx2), -1 / (4*motor_param)]])

    def spin_propellers(self):
        if self.sensor_posimeter[2] > 0.015:
            self.prop_angle[0] += self.ctrl[0][0] * 100
            self.prop_angle[1] -= self.ctrl[1][0] * 100
            self.prop_angle[2] += self.ctrl[2][0] * 100
            self.prop_angle[3] -= self.ctrl[3][0] * 100
        for i in range(4):
            self.prop_qpos[i][0] = self.prop_angle[i]

    # todo: types
    def update(self, i: int, step: float) -> None:
        # todo: check this as compared to the original when cleaning up
        self.spin_propellers()
        setpoint = self.trajectory.evaluate(self.state, i, step, self.data.time)
        ctrl = self.controller.compute_control(state=self.state, setpoint=setpoint, time=self.data.time)
        self.ctrl_input = ctrl
        motor_thrusts = self.input_mtx @ ctrl
        self.set_ctrl(motor_thrusts)

    def set_ctrl(self, ctrl):
        for i in range(4):
            self.ctrl[i][0] = ctrl[i]

    def bind_to_model(self, model: mujoco.MjModel):
        self.model = model
        self.mass = self.model.body(self.name).mass
        self.inertia = self.model.body(self.name).inertia

    def bind_to_data(self, data: mujoco.MjData):
        self.data = data
        free_joint = self.data.joint(self.name)
        self.xquat = self.data.body(self.name).xquat
        self.qpos = free_joint.qpos
        self.sensor_gyro = self.data.sensor(self.name + "_gyro").data
        self.sensor_velocimeter = self.data.sensor(self.name + "_velocimeter").data
        self.sensor_accelerometer = self.data.sensor(self.name + "_accelerometer").data
        self.sensor_posimeter = self.data.sensor(self.name + "_posimeter").data
        self.sensor_orimeter = self.data.sensor(self.name + "_orimeter").data
        self.sensor_ang_accelerometer = self.data.sensor(self.name + "_ang_accelerometer").data
        self.state: dict[str, Any] = {
            "pos": self.sensor_posimeter,
            "vel": self.sensor_velocimeter,
            "acc": self.sensor_accelerometer,
            "quat": self.sensor_orimeter,
            "ang_vel": self.sensor_gyro,
            "ang_acc": self.sensor_ang_accelerometer
        }
        for i in range(4):
            self.prop_joint[i] = self.data.joint(f"{self.name}_prop{i}")
            self.prop_qpos[i] = self.prop_joint[i].qpos
            self.prop_angle[i] = self.prop_qpos[i][0]  # ?????????????
            self.ctrl[i] = self.data.actuator(f"{self.name}_actr{i}").ctrl
            self.actr_force[i] = self.data.actuator(f"{self.name}_actr{i}").force
        self.controller = drone_geom_controller.GeomControl(self.mass, self.inertia, self.model.opt.gravity)
        self.trajectory = dummy_drone_trajectory.DummyDroneTrajectory()

    def create_xml_element(self, pos: str, quat: str, color: str) -> dict[str, list[ET.Element]]:
        # TODO: separate crazyflie and bumblebee elements and common parts (for simplicity, just doing crazyflie for now)
        # NOTE: In the original version, the mass of the drone and the mass of the prop are getting confused and
        # weirdly overwritten. The add_drone_common_parts function wants a 'mass' parameter, and uses this mass
        # parameter to set the inertial element of the XML, which makes sense. HOWEVER, the actual value passed to
        # this mass parameter is CRAZYFLIE_PROP.MASS.value, which doesn't make sense. It is then overwritten to be
        # "0.00001", which is what is used to set the mass of the propellers, instead of CRAZYFLIE_PROP.MASS.value
        name = self.name
        mass = CRAZYFLIE_PROP.MASS.value
        diaginertia = CRAZYFLIE_PROP.DIAGINERTIA.value
        Lx1 = CRAZYFLIE_PROP.OFFSET.value
        Lx2 = CRAZYFLIE_PROP.OFFSET.value
        Ly = CRAZYFLIE_PROP.OFFSET.value
        Lz = CRAZYFLIE_PROP.OFFSET_Z.value
        motor_param = CRAZYFLIE_PROP.MOTOR_PARAM.value
        max_thrust = CRAZYFLIE_PROP.MAX_THRUST.value
        cog = CRAZYFLIE_PROP.COG.value
        drone = ET.Element("body", name=name, pos=pos, quat=quat)  # this is the parent element
        ET.SubElement(drone, "geom", name=name + "_body", type="mesh", mesh="crazyflie_body", rgba=color)
        ET.SubElement(drone, "geom", name=name + "_4_motormounts", type="mesh", mesh="crazyflie_4_motormounts",
                      rgba=color)
        ET.SubElement(drone, "geom", name=name + "_4_motors", type="mesh", mesh="crazyflie_4_motors", rgba=color)

        ret = {"worldbody": [drone],
               "actuator": [],
               "sensor": []}
        # TODO: safety sphere?
        # ET.SubElement(drone, "geom", type="sphere", name=name + "_sphere", size="1.0", rgba=color, contype="0",
        #               conaffinity="0")
        # we give the inertia by hand instead of it auto-computing based on geoms
        ET.SubElement(drone, "inertial", pos=cog, diaginertia=diaginertia, mass=mass)
        ET.SubElement(drone, "joint", name=name, type="free")  # the free joint that allows this to move freely
        site_name = name + "_cog"
        ET.SubElement(drone, "site", name=site_name, pos="0 0 0", size="0.005")  # center of gravity
        prop_site_size = "0.0001"
        prop_mass = "0.00001"
        prop_pos = [f"{Lx2} -{Ly} {Lz}",
                    f"-{Lx1} -{Ly} {Lz}",
                    f"-{Lx1} {Ly} {Lz}",
                    f"{Lx2} {Ly} {Lz}"]
        for i in range(4):
            prop_name = f"{name}_prop{i}"
            prop_body = ET.SubElement(drone, "body", name=prop_name)
            ET.SubElement(prop_body, "joint", name=prop_name, axis="0 0 1", pos=prop_pos[i])
            ET.SubElement(drone, "site", name=prop_name, pos=prop_pos[i], size=prop_site_size)
            if i % 2 == 0:
                mesh = "crazyflie_ccw_prop"
                actuator = ET.Element("general", site=prop_name, name=f"{name}_actr{i}",
                                      gear=f" 0 0 1 0 0 {motor_param}",
                                      ctrllimited="true", ctrlrange=f"0 {max_thrust}")
                ret["actuator"].append(actuator)
            else:
                mesh = "crazyflie_cw_prop"
                actuator = ET.Element("general", site=prop_name, name=f"{name}_actr{i}",
                                      gear=f" 0 0 1 0 0 -{motor_param}",
                                      ctrllimited="true", ctrlrange=f"0 {max_thrust}")
                ret["actuator"].append(actuator)
            ET.SubElement(prop_body, "geom", name=prop_name, type="mesh", mesh=mesh, mass=prop_mass,
                          pos=prop_pos[i], rgba=PROP_COLOR)
        ret["sensor"].append(ET.Element("gyro", site=site_name, name=name + "_gyro"))
        ret["sensor"].append(ET.Element("framelinvel", objtype="site", objname=site_name, name=name + "_velocimeter"))
        ret["sensor"].append(ET.Element("accelerometer", site=site_name, name=name + "_accelerometer"))
        ret["sensor"].append(ET.Element("framepos", objtype="site", objname=site_name, name=name + "_posimeter"))
        ret["sensor"].append(ET.Element("framequat", objtype="site", objname=site_name, name=name + "_orimeter"))
        ret["sensor"].append(ET.Element("frameangacc", objtype="site", objname=site_name, name=name + "_ang_accelerometer"))
        return ret


