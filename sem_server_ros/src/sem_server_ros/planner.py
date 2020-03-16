from jena_models.planner import Planner


class JenaSempyPlanner:

    def __init__(self):
        self.jena_planner = Planner()

    def create_plan(self, skill):
        print("ROS plan: {}".format(skill))
        try:
            self.jena_planner.create_plan(skill)
        except:
            print(self.jena_planner.base_solution)
            print("Error during planning")

    def init_time(self):
        pass

    def find_available_steps(self):
        pass

    def apply_timestamp(self, action):
        pass
