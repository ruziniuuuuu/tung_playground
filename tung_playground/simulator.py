import glfw
from tung_playground.renderer import Renderer
import numpy as np
import mujoco

score = 0

def key_callback(window, key, scancode, action, mods, model, data):
    global score
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

def main():
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
        data.qfrc_applied[6] = np.random.uniform(-10, 10)
        data.qfrc_applied[7] = np.random.uniform(-10, 10)

        # Step the world
        mujoco.mj_step(model, data)

        data.qfrc_applied[:] = 0
        # Render the world
        renderer.render(score)

    glfw.terminate()

if __name__ == "__main__":
    main()
