import mujoco
import glfw

class Renderer:
    def __init__(self, model, data, window):
        self.model = model
        self.data = data
        self.window = window

        # Initialize mujoco visualization
        self.cam = mujoco.MjvCamera()
        self.opt = mujoco.MjvOption()
        self.scn = mujoco.MjvScene(self.model, maxgeom=10000)
        self.con = mujoco.MjrContext(self.model, mujoco.mjtFontScale.mjFONTSCALE_150)

        # Set camera configuration
        self.cam.azimuth = 90
        self.cam.elevation = -45
        self.cam.distance = 5.0
        self.cam.lookat[:] = [0.0, 0.0, 0.0]

        # Enable the headlight for better illumination
        self.model.vis.headlight.active = 1
        self.model.vis.headlight.ambient = [0.4, 0.4, 0.4]
        self.model.vis.headlight.diffuse = [0.8, 0.8, 0.8]
        self.model.vis.headlight.specular = [0.1, 0.1, 0.1]

        # Enable shadows and reflections for a more realistic scene
        self.scn.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 1
        self.scn.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = 1


    def render(self, score):
        # Update scene and render
        viewport_width, viewport_height = glfw.get_framebuffer_size(self.window)
        viewport = mujoco.MjrRect(0, 0, viewport_width, viewport_height)

        mujoco.mjv_updateScene(self.model, self.data, self.opt, None, self.cam, mujoco.mjtCatBit.mjCAT_ALL, self.scn)
        mujoco.mjr_render(viewport, self.scn, self.con)

        # Add score to GUI
        # mujoco.mjr_text(mujoco.mjtFont.mjFONT_NORMAL, f"Score: {score}", self.con, 10, 10, 1, 1, 1)
        mujoco.mjr_text(mujoco.mjtFont.mjFONT_NORMAL, f"Score: {score}", self.con, 0, 0, 1, 1, 1)


        # Swap buffers
        glfw.swap_buffers(self.window)

        # Poll for and process events
        glfw.poll_events()