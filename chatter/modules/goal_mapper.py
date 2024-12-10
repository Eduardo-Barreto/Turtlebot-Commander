import json

class GoalMapper:
    def __init__(self, config_path):
        with open(config_path, 'r') as file:
            self.locations = json.load(file)

    def get_coordinates(self, destination):
        return self.locations.get(destination, None)

