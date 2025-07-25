import os

class Hero:
    def __init__(self, name):
        self.name = name
        self.model_path = os.path.join("heroes", name, "model.xml")
