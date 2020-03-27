from jena_models.planner import Planner


class RosPlanner:

    def __init__(self):
        self.jena_planner = Planner()

    def create_plan(self, action_sem, object_sem):
        print("ROS plan: {} - {}".format(action_sem, object_sem))
        try:
            return self.jena_planner.create_plan(action_sem, object_sem)
        except:
            print(self.jena_planner.base_solution)
            print("Error during planning")

    def print_plan(self):
        self.jena_planner.print_plan()
