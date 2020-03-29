from jena_models.planner import Planner


class RosPlanner:

    def __init__(self):
        self.jena_planner = Planner()

    def create_plan(self, sem_collection):
        print("ROS plan: {}".format(sem_collection.list_triples))
        try:
            return self.jena_planner.create_plan(sem_collection.list_triples)
        except:
            print(self.jena_planner.base_solution)
            print("Error during planning")

    def print_plan(self):
        self.jena_planner.print_plan()
