import os
import time
from aiml_virtual.simulator import ActiveSimulator
from aiml_virtual.xml_generator import SceneXmlGenerator
from aiml_virtual.trajectory import CarTrajectory, HookedDroneNLTrajectory, HookedDroneNLAdaptiveTrajectory
from aiml_virtual.controller import CarLPVController, LtvLqrLoadControl
from aiml_virtual.util import mujoco_helper, carHeading2quaternion
from aiml_virtual.object.drone import DRONE_TYPES
from aiml_virtual.object.payload import PAYLOAD_TYPES, TeardropPayload, BoxPayload
from aiml_virtual.object.car import Fleet1Tenth
import numpy as np
import matplotlib.pyplot as plt
from aiml_virtual.object import parseMovingObjects
from scipy.spatial.transform import Rotation
from aiml_virtual.trajectory.car_path_point_generator import paperclip, dented_paperclip
from aiml_virtual.trajectory.trailer_predictor import TrailerPredictor



RED = "0.85 0.2 0.2 1.0"
BLUE = "0.2 0.2 0.85 1.0"
BLACK = "0.1 0.1 0.1 1.0"


abs_path = os.path.dirname(os.path.abspath(__file__))
xml_path = os.path.join(abs_path, "..", "xml_models")
xml_base_filename = "scene_base_indoor_hfield.xml"
save_filename = "built_scene.xml"

car_trajectory = CarTrajectory()
# define path points and build the path

# Scenario #1
path_points = np.roll(paperclip(), shift=13, axis=0)
drone_init_pos = np.array([0*-1, 0*-1.13, 1.1, 0])  # initial drone position and yaw angle
load_target_pos = np.array([1, 1.13, 0.25])

# Scenario #2
# path_points = dented_paperclip()
# drone_init_pos = np.array([1, 1.13, 1.1, 0])  # initial drone position and yaw angle
# load_target_pos = np.array([1, 1.13, 0.19])
car_trajectory.build_from_points_const_speed(path_points=path_points, path_smoothing=1e-4, path_degree=5,
                                             const_speed=0.6)

car_pos = np.array([car_trajectory.pos_tck[1][0][0], car_trajectory.pos_tck[1][1][0], 0.17])
heading_smoothing_index = 5
car_heading = np.arctan2(car_trajectory.pos_tck[1][1][heading_smoothing_index] - car_trajectory.pos_tck[1][1][0],
                            car_trajectory.pos_tck[1][0][heading_smoothing_index] - car_trajectory.pos_tck[1][0][0])
car_quat = carHeading2quaternion(car_heading)
car_rot = np.array([[np.cos(car_heading), -np.sin(car_heading), 0],
                    [np.sin(car_heading), np.cos(car_heading), 0],
                    [0, 0, 1]])
payload_offset = np.array([-0.5, 0, 0.082])
payload_pos = car_pos + car_rot @ payload_offset
payload_quat = carHeading2quaternion(car_heading+np.deg2rad(0))


# create xml with car, trailer, payload, and bumblebee
scene = SceneXmlGenerator(xml_base_filename)
car_name = scene.add_car(pos=np.array2string(car_pos)[1:-1], quat=car_quat, color=RED,
                            is_virtual=True, has_rod=False, has_trailer=True)
trailer_name = car_name + "_trailer"
load_mass = 0.01
payload_name = scene.add_payload(pos=np.array2string(payload_pos)[1:-1], size="0.05 0.05 0.05", mass=str(load_mass),
                                    quat=payload_quat, color=BLACK, type=PAYLOAD_TYPES.Teardrop)

bb_name = scene.add_drone(np.array2string(drone_init_pos[0:3])[1:-2], "1 0 0 0", RED, DRONE_TYPES.BUMBLEBEE_HOOKED, 2)

terrain0 = scene.add_moving_terrain("0 0 0")
terrain1 = scene.add_moving_terrain("0 0 0")
terrain2 = scene.add_moving_terrain("0 0 0")
terrain3 = scene.add_moving_terrain("0 0 0")
terrain4 = scene.add_moving_terrain("0 0 0")
terrain5 = scene.add_moving_terrain("0 0 0")

# saving the scene as xml so that the simulator can load it
scene.save_xml(os.path.join(xml_path, save_filename))

# create list of parsers 
virt_parsers = [parseMovingObjects]

control_step, graphics_step = 0.025, 0.05
xml_filename = os.path.join(xml_path, save_filename)

replanning_timestep = 1.5
bb_trajectory = HookedDroneNLAdaptiveTrajectory(replanning_timestep=replanning_timestep, plot_trajs=False)
bb_trajectory.set_control_step(control_step)
bb_controller = LtvLqrLoadControl([0.605], [1.5e-3, 1.5e-3, 2.6e-3], [0, 0, -9.81])

# Simulate car + trailer to get payload trajectory
predictor = TrailerPredictor(car_trajectory, with_graphics=False, payload_type=PAYLOAD_TYPES.Teardrop,
                             with_terrain=True)
predictor.trailer_top_plate_height = car_pos[2] + 0.067 
init_state = np.hstack((car_pos, np.fromstring(car_quat, sep=" "),
                                     np.zeros(6), 0, 0, 0, 0,
                                     payload_pos, np.fromstring(payload_quat, sep=" "), np.zeros(3)))
load_init_pos, load_init_vel, load_init_yaw, load_yaw_rel = predictor.simulate(init_state, 0, 15, num_ter=0)

# Plan trajectory
bb_trajectory.construct(drone_init_pos[0:3]-np.array([0, 0, 0.4]), drone_init_pos[3], [load_init_pos, load_init_vel],
                        [load_init_yaw, load_yaw_rel], load_target_pos, 0, load_mass, grasp_speed=1.3)

# Compute control gains
bb_controller.setup_hook_up(bb_trajectory, hook_mass=0.02, payload_mass=load_mass)

# recording interval for automatic video capture
#rec_interval=[0,12]
rec_interval = None # no video capture

simulator = ActiveSimulator(xml_filename, rec_interval, control_step, graphics_step, window_size=[3840, 2160],
                            with_graphics=True)
#simulator.cam.azimuth = -45
#simulator.cam.elevation = -25
#simulator.cam.lookat, simulator.cam.distance = [0, 0, .5], 4
simulator.onBoard_elev_offset = 20

# grabbing the drone and the car
car = simulator.get_MovingObject_by_name_in_xml(car_name)

car_controller = CarLPVController(car.mass, car.inertia, disturbed=False)

car_controllers = [car_controller]

# setting trajectory and controller for car
car.set_trajectory(car_trajectory)
car.set_controllers(car_controllers)

bb = simulator.get_MovingObject_by_name_in_xml(bb_name)

bb.set_controllers([bb_controller])
bb.set_trajectory(bb_trajectory)

payload = simulator.get_MovingObject_by_name_in_xml(payload_name)

replanning_cycle = int(replanning_timestep / control_step)
num_traj_to_compute = 1

scenario_duration = bb_trajectory.segment_times[-1]
print(scenario_duration)
while not simulator.should_close(scenario_duration):
    simulator.update()
    # Update terrain
    num_ter = 0
    if simulator.time > 1:
        num_ter = 1
    if simulator.time > 2:
        num_ter = 2
    if simulator.time > 4:
        num_ter = 3
    if simulator.time > 6:
        num_ter = 4
    if simulator.time > 8:
        num_ter = 5
    for i in range(6):
        if i == num_ter:
            simulator.data.mocap_pos[i][-1] = 0
        else:
            simulator.data.mocap_pos[i][-1] = -10
    #if simulator.i == int(5 / control_step):
    #    simulator.pause()
    # Replan trajectory
    if simulator.i % replanning_cycle == 0 and num_traj_to_compute < len(bb_trajectory.replanner.planners):
        simulator.pause()
        # This part should be executed in parallel
        # get qpos and qvel of car + trailer
        #car_to_rod = car.data.joint("car_to_rod")  # ball joint
        #rod_yaw = Rotation.from_quat(np.roll(car_to_rod.qpos, -1)).as_euler('xyz')[2]
        #rod_yaw_rate = car_to_rod.qvel[2]
        #front_to_rear = car.data.joint("front_to_rear")  # hinge joint
        #car_trailer_state = np.hstack((car.joint.qpos, car.joint.qvel, rod_yaw, rod_yaw_rate, front_to_rear.qpos, front_to_rear.qvel, 
        #                               payload.sensor_posimeter, payload.sensor_orimeter, payload.sensor_velocimeter))
        # simulate car + trailer
        pred_start = time.time()
        load_init_pos_new, load_init_vel_new, load_init_yaw_new, _ = predictor.simulate(init_state, 0, 15, num_ter=num_ter)
        pred_end = time.time()
        print(f"Prediction time: {pred_end-pred_start:.3f} s")
        
        bb_trajectory.replan(num_traj_to_compute, load_init_pos_new, load_init_vel_new, load_init_yaw_new)
        replan_end = time.time()
        print(f"Replan time: {replan_end-pred_end:.3f} s")
        # output: bb_trajectory.replanner.planners[num_traj] is optimized
        num_traj_to_switch = num_traj_to_compute
        num_traj_to_compute += 1
    #if simulator.i == activate_timestep:
        bb_trajectory.switch(num_traj_to_switch)
        bb_controller.setup_hook_up(bb_trajectory, hook_mass=0.001, payload_mass=load_mass)
        lqr_end = time.time()
        print(f"LQR time: {lqr_end-replan_end:.3f} s")
        simulator.unpause()

simulator.close()
