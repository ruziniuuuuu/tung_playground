import glfw
from tung_playground.hero import Hero
from tung_playground.world import World
from tung_playground.renderer import Renderer
import numpy as np

score = 0

def key_callback(window, key, scancode, action, mods, world):
    global score
    if action == glfw.PRESS:
        if key == glfw.KEY_BACKSPACE:
            for i in range(len(world.models)):
                mujoco.mj_resetData(world.models[i], world.datas[i])
                mujoco.mj_forward(world.models[i], world.datas[i])
            score = 0
        elif key == glfw.KEY_W:
            world.datas[0].qfrc_applied[0] = 100
            score += 1
        elif key == glfw.KEY_S:
            world.datas[0].qfrc_applied[0] = -100
        elif key == glfw.KEY_A:
            world.datas[0].qfrc_applied[1] = 100
        elif key == glfw.KEY_D:
            world.datas[0].qfrc_applied[1] = -100

def main():
    # Create the world
    world = World()

    # Add the heroes
    hero = Hero("complexo")
    villain = Hero("villain")
    world.add_hero(hero)
    world.add_hero(villain)

    # Create the renderer
    renderer = Renderer(world)

    # Set key callback
    glfw.set_key_callback(renderer.window, lambda window, key, scancode, action, mods: key_callback(window, key, scancode, action, mods, world))

    while not glfw.window_should_close(renderer.window):
        # Step the world
        world.step()

        # AI controller for villain
        world.datas[1].qfrc_applied[0] = np.random.uniform(-10, 10)
        world.datas[1].qfrc_applied[1] = np.random.uniform(-10, 10)

        # Render the world
        renderer.render(score)

        # Clear forces
        for data in world.datas:
            data.qfrc_applied[:] = 0


    renderer.close()

if __name__ == "__main__":
    main()
