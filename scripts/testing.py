"""
testing.py
========================

Docstring needs to be written!
"""

import os
import pathlib

import aiml_virtual.scene as scene
import aiml_virtual.simulated_object.moving_object.bicycle as bicycle
import aiml_virtual.simulator as simulator
import aiml_virtual.simulated_object.moving_object.drone as drone

if __name__ == "__main__":
    project_root = pathlib.Path(__file__).parents[1].resolve().as_posix()
    xml_directory = os.path.join(project_root, "xml_models")
    # scene = scene.Scene(os.path.join(xml_directory, "scene_base.xml"))
    scene = scene.Scene(os.path.join(xml_directory, "bicycle.xml"))
    bike1 = bicycle.Bicycle()
    scene.add_object(bike1, "0 1 0", "0 0 0 1", "0.5 0.5 0.5 1")
    bike2 = bicycle.Bicycle()
    scene.add_object(bike2, "0 -1 0", "0 0 0 1", "0.5 0.5 0.5 1")
    drone = drone.Drone()
    scene.add_object(drone, "0 0 1", "0 0 0 1", "0.5 0.5 0.5 1")

    sim = simulator.Simulator(scene, control_freq=500, target_fps=100)
    with sim.launch_viewer():
        while sim.viewer.is_running():
            sim.step()

