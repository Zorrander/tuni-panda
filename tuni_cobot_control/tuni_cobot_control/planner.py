import rclpy
from rclpy.node import Node
import time
import copy
from tuni_cobot_control.world import DigitalWorld

class Planner(Node):

    def __init__(self):
        super().__init__('planner')
        self.final_plan = []
        self.decomp_history = []

    def record_decomposition_of_task(self, current_task, method):
        self.decomp_history.insert(0, (current_task, self.final_plan, method))

    def restore_to_last_decomposed_task(self):
        last_record = self.decomp_history.pop(0)
        self.tasks_to_process.append(last_record[0])
        self.final_plan = last_record[1]

    def create_plan(self, current_world):
        try:
            world = DigitalWorld(current_world)
            print(world)
            self.final_plan = []
            self.decomp_history = []
            self.tasks_to_process = world.root_task
            print("TASKS TO PROCESS: {}".format(self.tasks_to_process))
            while self.tasks_to_process:
                current_task = self.tasks_to_process.pop(0)
                print("CURRENT TASK: {}".format(current_task))
                if world.find_type(current_task) == "CompoundTask":
                    method = world.find_satisfied_method(current_task)
                    test = True if method else False
                    print("if method {}".format(test))
                    if method:
                        self.record_decomposition_of_task(current_task, method)
                        new_tasks = [task for task in world.find_subtasks(method) if task not in self.final_plan]
                        print("NEW TASKS TO ADD {}".format(new_tasks))
                        self.tasks_to_process.extend(new_tasks)
                    else:
                        self.restore_to_last_decomposed_task()
                else:  # Primitive task
                    if world.are_preconditions_met(current_task):
                        print("Preconditions of {} are met".format(current_task))
                        world.apply_effects(current_task)

                        self.final_plan.append(current_task)
                    else:
                        self.restore_to_last_decomposed_task()
        except Exception as e:
            print(e)

    def execute(self, primitive):
        self.sem_controller.interpret(primitive)

    def run(self):
        try:
            while self.final_plan:
                primitive = self.final_plan.pop(0)
                if self.are_preconditions_met(primitive):
                    self.execute(primitive)
                else:
                    raise DispatchingError(primitive)
        except DispatchingError as e:
            self.final_plan.insert(0, e.primitive)
            self.create_plan(self.final_plan)
            self.run()

def main(args=None):
    rclpy.init(args=args)
    world = DigitalWorld()
    planner = Planner()
    #world.send_command()
    planner.create_plan(world)
    print("Final plan: {}".format(planner.final_plan))
    time.sleep(1)


    rclpy.spin(planner)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
