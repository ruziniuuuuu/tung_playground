import glfw
from tung_playground import renderer
from tung_playground.renderer import Renderer
import numpy as np
import mujoco

score = 0
look_sahur = True
renderer = None

def key_callback(window, key, scancode, action, mods, model, data):
    global score, look_sahur, renderer
    if action == glfw.PRESS:
        if key == glfw.KEY_BACKSPACE:
            mujoco.mj_resetData(model, data)
            mujoco.mj_forward(model, data)
            score = 0
        elif key == glfw.KEY_W:
            data.qfrc_applied[0] = 500000
            score += 1
        elif key == glfw.KEY_S:
            data.qfrc_applied[0] = -500000
        elif key == glfw.KEY_A:
            data.qfrc_applied[1] = 500000
        elif key == glfw.KEY_D:
            data.qfrc_applied[1] = -500000
        elif key == glfw.KEY_SPACE:
            data.qfrc_applied[2] = 500000
        elif key == glfw.KEY_CAPS_LOCK:
            look_sahur = not look_sahur
        elif key == glfw.KEY_LEFT:
            renderer.cam.azimuth -= 5
        elif key == glfw.KEY_RIGHT:
            renderer.cam.azimuth += 5
        elif key == glfw.KEY_UP:
            renderer.cam.elevation -= 5
        elif key == glfw.KEY_DOWN:
            renderer.cam.elevation += 5

def main():
    global renderer
    # Initialize glfw
    if not glfw.init():
        return

    # Create a window
    window = glfw.create_window(1200, 900, "Tung Playground", None, None)
    if not window:
        glfw.terminate()
        return

    # Make the window's context current
    glfw.make_context_current(window)
    glfw.swap_interval(1)

    # Load the model
    model = mujoco.MjModel.from_xml_path("tung_playground/world.xml")
    data = mujoco.MjData(model)

    # Create the renderer
    renderer = Renderer(model, data, window)

    # Set key callback
    glfw.set_key_callback(renderer.window, lambda window, key, scancode, action, mods: key_callback(window, key, scancode, action, mods, model, data))

    while not glfw.window_should_close(renderer.window):
        # Clear forces

        # AI controller for villain
        data.qfrc_applied[9] = np.random.uniform(-10, 10) * 250
        data.qfrc_applied[10] = np.random.uniform(-10, 10) * 250
        data.qfrc_applied[11] = np.random.uniform(-10, 10) * 250

        # Step the world
        mujoco.mj_step(model, data)

        data.qfrc_applied[:] = 0
        # Render the world
        if look_sahur:
            renderer.cam.lookat[:] = [data.qpos[0], data.qpos[1], data.qpos[2]]
        else:
            renderer.cam.lookat[:] = [data.qpos[9], data.qpos[10], data.qpos[11]]

        renderer.render(score)

    glfw.terminate()

if __name__ == "__main__":
    main()
