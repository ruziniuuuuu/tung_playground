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
            renderer.cam.azimuth -= 15
        elif key == glfw.KEY_RIGHT:
            renderer.cam.azimuth += 15
        elif key == glfw.KEY_UP:
            renderer.cam.elevation -= 15
        elif key == glfw.KEY_DOWN:
            renderer.cam.elevation += 15

def main():
    global renderer, score
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
        # data.qfrc_applied[9] = np.random.uniform(-10, 10) * 250
        # data.qfrc_applied[10] = np.random.uniform(-10, 10) * 250
        # data.qfrc_applied[11] = np.random.uniform(-10, 10) * 250

        hero_pos = data.qpos[0:2]
        villain_pos = data.qpos[9:11]
        direction = hero_pos - villain_pos
        direction = direction / (np.linalg.norm(direction) + 1e-9)
        # print(score)
        data.qfrc_applied[8] = direction[0] * 500
        data.qfrc_applied[9] = direction[1] * 500

        # Check if hero reached target
        hero_pos = data.qpos[0:2]
        target_pos = model.body("target").pos[0:2]
        dist = np.linalg.norm(hero_pos - target_pos)
        if dist < 0.75:
            score += 1
            # Move target to a new random position
            model.body("target").pos[0] = np.random.uniform(-5, 5)
            model.body("target").pos[1] = np.random.uniform(-5, 5)

        # Check if villain caught hero
        villain_pos = data.qpos[9:11]
        dist = np.linalg.norm(hero_pos - villain_pos)
        if dist < 0.9:
            score -= 0.001
        # print(score)
        if score <= -1 or score >=5:
            mujoco.mj_resetData(model, data)
            mujoco.mj_forward(model, data)
            score = 0

        # Step the world
        mujoco.mj_step(model, data)

        data.qfrc_applied[:] = 0
        # Render the world
        if look_sahur:
            renderer.cam.lookat[:] = [data.qpos[0], data.qpos[1], data.qpos[2]]
        else:
            renderer.cam.lookat[:] = [data.qpos[9], data.qpos[10], data.qpos[11]]

        renderer.render(round(score, 2))

    glfw.terminate()

if __name__ == "__main__":
    main()
