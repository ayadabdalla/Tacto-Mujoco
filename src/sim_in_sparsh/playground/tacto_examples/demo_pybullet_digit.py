# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import logging
import os

import cv2
import hydra
from omegaconf import OmegaConf
import pybullet as p
import pybulletX as px
import tacto  # Import TACTO

log = logging.getLogger(__name__)

# Load the config YAML file from examples/conf/digit.yaml
# OmegaConf.register_new_resolver("base_dir", script_dir)
# @hydra.main(config_path="conf", config_name="digit")
# script meta data, configuration and constants
# remove leading slash from string
def main(cfg):
    # change base_dir using omegaconf library to override the default value

    # Initialize digits
    bg = cv2.imread("conf/bg_digit_240_320.jpg")
    digits = tacto.Sensor(**cfg.tacto, background=bg)

    # Initialize World
    log.info("Initializing world")
    px.init()

    p.resetDebugVisualizerCamera(**cfg.pybullet_camera)

    # Create and initialize DIGIT
    digit_body = px.Body(**cfg.digit)
    digits.add_camera(digit_body.id, [-1])

    # Add object to pybullet and tacto simulator
    obj = px.Body(**cfg.object)
    digits.add_body(obj)

    # Create control panel to control the 6DoF pose of the object
    panel = px.gui.PoseControlPanel(obj, **cfg.object_control_panel)
    panel.start()
    log.info("Use the slides to move the object until in contact with the DIGIT")

    # run p.stepSimulation in another thread
    t = px.utils.SimulationThread(real_time_factor=1.0)
    t.start()

    while True:
        color, depth = digits.render()
        digits.updateGUI(color, depth)

    t.stop()


if __name__ == "__main__":
    hydra.initialize("conf", version_base=None)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    cfg=hydra.compose("digit.yaml", overrides=[f"base_dir={script_dir}"])
    main(cfg)
