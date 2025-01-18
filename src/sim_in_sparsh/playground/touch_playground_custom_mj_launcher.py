import mujoco as mj
import glfw
import numpy as np
import time

# Path to your XML model
MODEL_PATH = "/home/abdullah/utn/ontouch/robot-control-stack/python/examples/playground_data/touch_playground.xml"  # Update with your model path

# Joint control parameters
CONTROL_INCREMENT = 0.05  # Amount to move joints per keypress

def key_callback(window, key, scancode, action, mods):
    """
    GLFW key callback to interactively control joint positions.
    """
    if action != glfw.PRESS and action != glfw.REPEAT:
        return  # Ignore key releases

    # Define joint controls
    if key == glfw.KEY_X:
        data.qpos[model.joint("x").qposadr] += CONTROL_INCREMENT
    elif key == glfw.KEY_Z:
        data.qpos[model.joint("x").qposadr] -= CONTROL_INCREMENT
    elif key == glfw.KEY_Y:
        data.qpos[model.joint("y").qposadr] += CONTROL_INCREMENT
    elif key == glfw.KEY_A:
        data.qpos[model.joint("y").qposadr] -= CONTROL_INCREMENT

def main():
    # Initialize GLFW
    if not glfw.init():
        raise RuntimeError("GLFW initialization failed")

    # Load MuJoCo model and data
    global model, data
    model = mj.MjModel.from_xml_path(MODEL_PATH)
    data = mj.MjData(model)

    # Create GLFW window
    window = glfw.create_window(1200, 800, "MuJoCo Interactive Viewer", None, None)
    if not window:
        glfw.terminate()
        raise RuntimeError("GLFW window creation failed")

    glfw.make_context_current(window)

    # Set up GLFW key callback
    glfw.set_key_callback(window, key_callback)

    # Initialize MuJoCo visualization
    options = mj.MjvOption()
    scene = mj.MjvScene(model, maxgeom=1000)
    context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150)
    camera = mj.MjvCamera()

    # Main simulation loop
    prev_time = time.time()
    while not glfw.window_should_close(window):
        # Compute time since last frame
        current_time = time.time()
        dt = current_time - prev_time
        prev_time = current_time

        # Step the simulation
        mj.mj_step(model, data)

        # Render the scene
        viewport_width, viewport_height = glfw.get_framebuffer_size(window)
        mj.mjv_updateScene(model, data, options,None, camera, mj.mjtCatBit.mjCAT_ALL.value,scene)
        mj.mjr_render(mj.MjrRect(0, 0, viewport_width, viewport_height), scene, context)

        # Swap buffers and poll events
        glfw.swap_buffers(window)
        glfw.poll_events()

    # Cleanup
    glfw.terminate()

if __name__ == "__main__":
    main()
