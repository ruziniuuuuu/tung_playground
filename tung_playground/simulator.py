import glfw
from tung_playground import renderer
from tung_playground.renderer import Renderer
import numpy as np
import mujoco

score = 0
look_sahur = True
renderer = None

def reset_positions(model, data):
    """
    Resets the simulation, placing the hero at the center and the villain
    and target at random positions a safe distance away.
    """
    global score
    # Reset all simulation data to defaults
    mujoco.mj_resetData(model, data)
    
    # Hero starts at the center
    data.qpos[0:2] = [0, 0]
    hero_pos = data.qpos[0:2]
    min_spawn_distance = 4.0  # A safe distance

    # Set random position for the villain, ensuring it's not too close to the hero
    while True:
        villain_pos_x = np.random.uniform(-10, 10)
        villain_pos_y = np.random.uniform(-10, 10)
        villain_pos = np.array([villain_pos_x, villain_pos_y])
        if np.linalg.norm(hero_pos - villain_pos) > min_spawn_distance:
            # Corrected: Villain's x,y position qpos indices are 8 and 9
            data.qpos[8:10] = villain_pos
            break

    # Set random position for the target, ensuring it's not too close to the hero
    while True:
        target_pos_x = np.random.uniform(-10, 10)
        target_pos_y = np.random.uniform(-10, 10)
        target_pos = np.array([target_pos_x, target_pos_y])
        if np.linalg.norm(hero_pos - target_pos) > min_spawn_distance:
            model.body("target").pos[0:2] = target_pos
            break
    
    # Reset score
    score = 0
    
    # Forward the simulation to apply changes
    mujoco.mj_forward(model, data)

def key_callback(window, key, scancode, action, mods, model, data):
    global score, look_sahur, renderer
    if action == glfw.PRESS:
        if key == glfw.KEY_BACKSPACE:
            reset_positions(model, data)
            score = 0
        else:
            # Get camera azimuth in radians
            azimuth_rad = np.deg2rad(renderer.cam.azimuth)
            
            # Calculate forward and right vectors based on camera direction
            forward_x = np.cos(azimuth_rad)
            forward_y = np.sin(azimuth_rad)
            right_x = -np.sin(azimuth_rad)
            right_y = np.cos(azimuth_rad)

            force_magnitude = 500000

            if key == glfw.KEY_W:
                data.qfrc_applied[0] = force_magnitude * forward_x
                data.qfrc_applied[1] = force_magnitude * forward_y
            elif key == glfw.KEY_S:
                data.qfrc_applied[0] = -force_magnitude * forward_x
                data.qfrc_applied[1] = -force_magnitude * forward_y
            elif key == glfw.KEY_A:
                data.qfrc_applied[0] = force_magnitude * right_x
                data.qfrc_applied[1] = force_magnitude * right_y
            elif key == glfw.KEY_D:
                data.qfrc_applied[0] = -force_magnitude * right_x
                data.qfrc_applied[1] = -force_magnitude * right_y
            elif key == glfw.KEY_SPACE:
                data.qfrc_applied[2] = 500000
            elif key == glfw.KEY_CAPS_LOCK:
                look_sahur = not look_sahur
            elif key == glfw.KEY_RIGHT:
                renderer.cam.azimuth -= 15
            elif key == glfw.KEY_LEFT:
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

    # Initial spawn of hero, villain, and target
    reset_positions(model, data)

    while not glfw.window_should_close(renderer.window):
        # --- Villain AI ---

        # AI controller for villain
        hero_pos = data.qpos[0:2]
        villain_pos = data.qpos[9:11]
        
        # Calculate distance and direction from villain to hero
        direction_vector = hero_pos - villain_pos
        distance = np.linalg.norm(direction_vector)
        direction_normalized = direction_vector / (distance + 1e-9)

        # Make villain faster when far away
        base_force = 500  # The villain's base speed
        distance_gain = 500  # How much faster the villain gets with distance
        dynamic_force = base_force + distance * distance_gain

        # Apply force to move the villain towards the hero
        data.qfrc_applied[8] = direction_normalized[0] * dynamic_force
        data.qfrc_applied[9] = direction_normalized[1] * dynamic_force

        # --- Game Logic ---

        # Check if hero reached target
        target_pos = model.body("target").pos[0:2]
        dist_to_target = np.linalg.norm(hero_pos - target_pos)
        if dist_to_target < 1:
            score += 1
            # Move target to a new random position
            model.body("target").pos[0] = np.random.uniform(-10, 10)
            model.body("target").pos[1] = np.random.uniform(-10, 10)

        # Check if villain caught hero
        dist_to_villain = np.linalg.norm(hero_pos - villain_pos)
        if dist_to_villain < 1:
            score -= 0.005
        
        # Reset if score is out of bounds
        if score < 0 or score >= 5:
            reset_positions(model, data)

        # Step the world
        mujoco.mj_step(model, data)

        data.qfrc_applied[:] = 0
        
        # Render the world
        if look_sahur:
            renderer.cam.lookat[:] = [data.qpos[0], data.qpos[1], data.qpos[2]]
        else:
            renderer.cam.lookat[:] = [data.qpos[8], data.qpos[9], data.qpos[10]]

        renderer.render(round(score, 2))

    glfw.terminate()

if __name__ == "__main__":
    main()