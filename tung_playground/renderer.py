import mujoco
import glfw

class Renderer:
    def __init__(self, world):
        self.world = world

        # Initialize glfw
        if not glfw.init():
            return

        # Create a window
        self.window = glfw.create_window(1200, 900, "Tung Playground", None, None)
        if not self.window:
            glfw.terminate()
            return

        # Make the window's context current
        glfw.make_context_current(self.window)
        glfw.swap_interval(1)

        # Initialize mujoco visualization
        self.cam = mujoco.MjvCamera()
        self.opt = mujoco.MjvOption()
        self.scn = mujoco.MjvScene(self.world.models[0], maxgeom=10000)
        self.con = mujoco.MjrContext(self.world.models[0], mujoco.mjtFontScale.mjFONTSCALE_150)

        # Set camera configuration
        self.cam.azimuth = 90
        self.cam.elevation = -45
        self.cam.distance = 5.0
        self.cam.lookat[:] = [0.0, 0.0, 0.0]

    def render(self, score):
        # Update scene and render
        viewport_width, viewport_height = glfw.get_framebuffer_size(self.window)
        viewport = mujoco.MjrRect(0, 0, viewport_width, viewport_height)

        for i in range(len(self.world.models)):
            mujoco.mjv_updateScene(self.world.models[i], self.world.datas[i], self.opt, None, self.cam, mujoco.mjtCatBit.mjCAT_ALL, self.scn)
            mujoco.mjr_render(viewport, self.scn, self.con)

        # Add score to GUI
        mujoco.mjr_text(self.con, mujoco.mjtFont.mjFONT_NORMAL, f"Score: {score}", 10, 10)

        # Swap buffers
        glfw.swap_buffers(self.window)

        # Poll for and process events
        glfw.poll_events()

    def close(self):
        glfw.terminate()
