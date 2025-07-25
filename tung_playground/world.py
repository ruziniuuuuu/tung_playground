import mujoco

class World:
    def __init__(self):
        self.models = []
        self.datas = []

    def add_hero(self, hero):
        model = mujoco.MjModel.from_xml_path(hero.model_path)
        data = mujoco.MjData(model)
        self.models.append(model)
        self.datas.append(data)

    def step(self):
        for i in range(len(self.models)):
            mujoco.mj_step(self.models[i], self.datas[i])
