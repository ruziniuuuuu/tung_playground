import mujoco
import glfw
from mujoco import viewer
import numpy as np

def key_callback(window, key, scancode, action, mods):
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

    # Create the viewer
    with viewer.launch_passive(model, data) as v:
        # Set camera configuration
        v.cam.azimuth = 90
        v.cam.elevation = -45
        v.cam.distance = 5.0
        v.cam.lookat[:] = [0.0, 0.0, 0.0]

        # Set key callback
        glfw.set_key_callback(v.window, key_callback)

        while v.is_running():
            step_start = data.time

            while (data.time - step_start) < 1/60.0:
                mujoco.mj_step(model, data)

            # Apply random force
            data.qfrc_applied[0] = np.random.uniform(-1, 1)
            data.qfrc_applied[1] = np.random.uniform(-1, 1)
            data.qfrc_applied[2] = np.random.uniform(-1, 1)

            v.sync()

if __name__ == "__main__":
    main()
