import mujoco
import glfw
from mujoco import viewer

def main():
    # Load the hero
    from tung_playground.hero import Hero
    hero = Hero("tralalero_tralala")

    # Load the model
    model = mujoco.MjModel.from_xml_path(hero.model_path)
    data = mujoco.MjData(model)

    # Create the viewer
    with viewer.launch_passive(model, data) as v:
        while v.is_running():
            v.sync()

if __name__ == "__main__":
    main()
