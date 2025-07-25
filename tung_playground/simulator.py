import mujoco
import glfw
import numpy as np

def key_callback(window, key, scancode, action, mods, model, data):
    if action == glfw.PRESS and key == glfw.KEY_BACKSPACE:
        mujoco.mj_resetData(model, data)
        mujoco.mj_forward(model, data)

def main():
    # Load the hero
    from tung_playground.hero import Hero
    hero = Hero("tralalero_tralala")

    # Load the model
    model = mujoco.MjModel.from_xml_path(hero.model_path)
    data = mujoco.MjData(model)

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

    # Initialize mujoco visualization
    cam = mujoco.MjvCamera()
    opt = mujoco.MjvOption()
    scn = mujoco.MjvScene(model, maxgeom=10000)
    con = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)

    # Set key callback
    glfw.set_key_callback(window, lambda window, key, scancode, action, mods: key_callback(window, key, scancode, action, mods, model, data))

    # Set camera configuration
    cam.azimuth = 90
    cam.elevation = -45
    cam.distance = 5.0
    cam.lookat[:] = [0.0, 0.0, 0.0]

    while not glfw.window_should_close(window):
        step_start = data.time

        while (data.time - step_start) < 1/60.0:
            mujoco.mj_step(model, data)

        # Apply random force
        data.qfrc_applied[0] = np.random.uniform(-1, 1)
        data.qfrc_applied[1] = np.random.uniform(-1, 1)
        data.qfrc_applied[2] = np.random.uniform(-1, 1)

        # Update scene and render
        viewport_width, viewport_height = glfw.get_framebuffer_size(window)
        viewport = mujoco.MjrRect(0, 0, viewport_width, viewport_height)
        mujoco.mjv_updateScene(model, data, opt, None, cam, mujoco.mjtCatBit.mjCAT_ALL, scn)
        mujoco.mjr_render(viewport, scn, con)

        # Swap buffers
        glfw.swap_buffers(window)

        # Poll for and process events
        glfw.poll_events()

    glfw.terminate()

if __name__ == "__main__":
    main()
