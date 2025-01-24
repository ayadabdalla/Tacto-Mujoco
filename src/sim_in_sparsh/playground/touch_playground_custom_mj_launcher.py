import inspect
import os
import sys
import mujoco as mj
import glfw
import numpy as np
import time

# Path to your XML model
current_dir = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = f"{current_dir}/playground_data/touch_playground.xml"  # Update with your model path

CONTROL_INCREMENT = 0.05  # Amount to move joints per keypress
output_data = []  # List to collect sensor data for saving

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
        data.qpos[model.joint("z").qposadr] += CONTROL_INCREMENT
    elif key == glfw.KEY_LEFT:
        data.qpos[model.joint("z").qposadr] -= CONTROL_INCREMENT
    elif key == glfw.KEY_UP:
        data.qpos[model.joint("y").qposadr] += CONTROL_INCREMENT
    elif key == glfw.KEY_DOWN:
        data.qpos[model.joint("y").qposadr] -= CONTROL_INCREMENT
    # control rx and ry joints
    elif key == glfw.KEY_W:
        data.qpos[model.joint("rx").qposadr] += CONTROL_INCREMENT
    elif key == glfw.KEY_S:
        data.qpos[model.joint("rx").qposadr] -= CONTROL_INCREMENT
    elif key == glfw.KEY_A:
        data.qpos[model.joint("ry").qposadr] += CONTROL_INCREMENT
    elif key == glfw.KEY_D:
        data.qpos[model.joint("ry").qposadr] -= CONTROL_INCREMENT
    elif key == glfw.KEY_Q:
        data.qpos[model.joint("rz").qposadr] += CONTROL_INCREMENT
    elif key == glfw.KEY_E:
        data.qpos[model.joint("rz").qposadr] -= CONTROL_INCREMENT
    # add keys for zooming in and out
        # Zoom in and out
    elif key == glfw.KEY_EQUAL:  # Zoom In (e.g., `=` key)
        data.cam.distance -= CONTROL_INCREMENT
    elif key == glfw.KEY_MINUS:  # Zoom Out (e.g., `-` key)
        data.cam.distance += CONTROL_INCREMENT


def main():
    # Initialize GLFW
    if not glfw.init():
        raise RuntimeError("GLFW initialization failed")
    # Print the location of the MuJoCo Python bindings
    print("MuJoCo Python bindings location:", mj.__file__)

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
    glfw.set_key_callback(window, key_callback)

    # Initialize MuJoCo visualization
    options = mj.MjvOption()
    scene = mj.MjvScene(model, maxgeom=1000)
    context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150)
    camera = mj.MjvCamera()
    sensor_id = model.sensor("touch").id
    print(model.sensor_dim[sensor_id])

    # Main simulation loop
    prev_time = time.time()
    while not glfw.window_should_close(window):
        current_time = time.time()
        dt = current_time - prev_time
        prev_time = current_time

        # Step the simulation
        mj.mj_step(model, data)

        # Fetch touch grid data
        sensor_id = model.sensor("touch").id
        touch_data = data.sensordata[sensor_id:sensor_id + model.sensor_dim[sensor_id]].reshape((120, 160, 3))  # Adjust shape as per your config
        # print("Touch Data (Center Pixel):", touch_data[3, 3])  # Example: Print the center pixel
        print("Touch Data (All Pixels):", touch_data)  # Example: Print all pixels
        print(touch_data.shape)
        output_data.append(touch_data)  # Collect data for saving

        #get the position of the object
        object_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "a")
        object_pos = data.geom_xpos[object_id]

        # Get position (3D) and rotation matrix (3x3)
        position = data.geom_xpos[object_id]        # Shape: (3,)
        rotation = data.geom_xmat[object_id].reshape(3, 3)  # Reshape flattened 9-element array to 3x3

        # Construct 4x4 pose matrix
        pose_matrix = np.eye(4)
        pose_matrix[:3, :3] = rotation  # Rotation component
        pose_matrix[:3, 3] = position   # Translation component

        print("4x4 Pose Matrix:")
        print(pose_matrix)

        # Render the scene
        viewport_width, viewport_height = glfw.get_framebuffer_size(window)
        mj.mjv_updateScene(model, data, options, None, camera, mj.mjtCatBit.mjCAT_ALL.value, scene)
        mj.mjr_render(mj.MjrRect(0, 0, viewport_width, viewport_height), scene, context)

        glfw.swap_buffers(window)
        glfw.poll_events()

    # Save collected sensor data to a file
    print("Touch grid data saved to touch_grid_data.npy")

    glfw.terminate()

if __name__ == "__main__":
    main()
