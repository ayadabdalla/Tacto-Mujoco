import logging
import os
import mujoco as mj



logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

from mujoco.viewer import launch


def main():
    # get path of parent directory
    current_dir = os.path.dirname(os.path.abspath(__file__))
    model = mj.MjModel.from_xml_path(f"{current_dir}/assets/touch_playground.xml")
    launch(model)


if __name__ == "__main__":
    main()
