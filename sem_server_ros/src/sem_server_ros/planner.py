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
        try:
            self.jena_planner.init_time()
        except:
            print("Could not initialize time")

    def find_available_steps(self):
        try:
            return self.jena_planner.find_available_steps()
        except:
            print("Could not find next steps")

    def find_next_action(self):
        try:
            return self.jena_planner.find_next_action()
        except:
            print("Failed to compute next action")

    def apply_timestamp(self, action):
        try:
            self.jena_planner.apply_timestamp(action)
        except:
            print("Something went wrong with the graph timebounds")

    def print_plan(self):
        self.jena_planner.print_plan()
