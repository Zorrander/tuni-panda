from simple_net.planner import Planner


class RosPlanner:

    def __init__(self):
        self.planner = Planner()

    def retrieve_plan(self):
        try:
            return self.planner.base_solution
        except Exception as e:
            print(e)

    def find_available_steps(self, time):
        try:
            return self.planner.find_available_steps(time)
        except Exception as e:
            print(e)

    def create_plan(self, sem_collection):
        # print("ROS plan: {}".format(sem_collection))
        try:
            return self.planner.create_plan(sem_collection)
        except:
            print(self.planner.base_solution)
            print("Error during planning")

    def print_plan(self):
        self.planner.print_plan()
