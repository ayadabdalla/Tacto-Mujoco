# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import logging
import os
import sys
import time

import cv2
import hydra
from omegaconf import OmegaConf
import pybullet as p
import pybulletX as px
import mujoco as mj
import pyrender
import tacto  # Import TACTO
from mujoco.viewer import launch
import glfw
import threading

CONTROL_INCREMENT = 0.0005  # Amount to move joints per keypress


def key_callback(window, key, scancode, action, mods):
    """
    GLFW key callback to interactively control joint positions.
    """
    if action != glfw.PRESS and action != glfw.REPEAT:
        return  # Ignore key releases

    if key == glfw.KEY_PAGE_UP:
        data.qpos[model.joint("x").qposadr] += CONTROL_INCREMENT
    elif key == glfw.KEY_PAGE_DOWN:
        data.qpos[model.joint("x").qposadr] -= CONTROL_INCREMENT
    elif key == glfw.KEY_RIGHT:
        data.qpos[model.joint("y").qposadr] -= CONTROL_INCREMENT
    elif key == glfw.KEY_LEFT:
        data.qpos[model.joint("y").qposadr] += CONTROL_INCREMENT
    elif key == glfw.KEY_UP:
        data.qpos[model.joint("z").qposadr] += CONTROL_INCREMENT
    elif key == glfw.KEY_DOWN:
        data.qpos[model.joint("z").qposadr] -= CONTROL_INCREMENT
    # add more keys to control joints of another object
    elif key == glfw.KEY_W:
        data.qpos[model.joint("can_x").qposadr] += CONTROL_INCREMENT
    elif key == glfw.KEY_S:
        data.qpos[model.joint("can_x").qposadr] -= CONTROL_INCREMENT
    elif key == glfw.KEY_D:
        data.qpos[model.joint("can_z").qposadr] += CONTROL_INCREMENT
    elif key == glfw.KEY_A:
        data.qpos[model.joint("can_z").qposadr] -= CONTROL_INCREMENT
    elif key == glfw.KEY_R:
        data.qpos[model.joint("can_y").qposadr] += CONTROL_INCREMENT
    elif key == glfw.KEY_F:
        data.qpos[model.joint("can_y").qposadr] -= CONTROL_INCREMENT


log = logging.getLogger(__name__)


# Load the config YAML file from examples/conf/digit.yaml
# OmegaConf.register_new_resolver("base_dir", script_dir)
# @hydra.main(config_path="conf", config_name="digit")
# script meta data, configuration and constants
# remove leading slash from string
def main(cfg):
    # change base_dir using omegaconf library to override the default value
    # Initialize World
    global model, data, camera, scene
    import glfw


    # Initialize digits
    bg = cv2.imread("conf/bg_digit_240_320.jpg")
    digits = tacto.Sensor(**cfg.tacto, background=bg)

    current_dir = os.path.dirname(os.path.abspath(__file__))
    model = mj.MjModel.from_xml_path(
        f"{current_dir}/../playground_data/touch_playground.xml"
    )
    data = mj.MjData(model)
    model.opt.timestep = 0.001

    # log.info("Initializing world")
    # Create GLFW window
    if not glfw.init():
       raise Exception("Failed to initialize GLFW")
    window = glfw.create_window(1200, 800, "MuJoCo Interactive Viewer", None, None)
    if not window:
        glfw.terminate()
        raise RuntimeError("GLFW window creation failed")

    glfw.make_context_current(window)
    glfw.set_key_callback(window, key_callback)
    options = mj.MjvOption()
    scene = mj.MjvScene(model, maxgeom=1000)
    context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150)
    camera = mj.MjvCamera()

    # Add objects from mujoco to renderer
    # digits.add_body_mujoco("digit",model,data) # Create and initialize DIGIT
    mj.mj_step(model, data)
    digits.add_body_mujoco("can", model, data)  # get body in a smart way from mujoco
    digits.add_camera_mujoco(
        "touch", model, data
    )  # get camera in a smart way from mujoco

    # Render the DIGIT sensor in a separate thread
    def render_digit():
        while not glfw.window_should_close(window):
            color, depth = digits.render(model, data)
            digits.updateGUI(color, depth)
            time.sleep(0.01)
    render_thread = threading.Thread(target=render_digit)
    render_thread.start()

    while not glfw.window_should_close(window):
        # step simulation of mujoco model
        mj.mj_step(model, data)
        # visualize the simulation
        # Render the scene
        viewport_width, viewport_height = glfw.get_framebuffer_size(window)
        mj.mjv_updateScene(
            model, data, options, None, camera, mj.mjtCatBit.mjCAT_ALL.value, scene
        )
        mj.mjr_render(mj.MjrRect(0, 0, viewport_width, viewport_height), scene, context)

        glfw.swap_buffers(window)
        glfw.poll_events()


if __name__ == "__main__":
    hydra.initialize("conf", version_base=None)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    cfg = hydra.compose("digit.yaml", overrides=[f"base_dir={script_dir}"])
    main(cfg)
