import rclpy
from rclpy.node import Node
import time
import copy
from tuni_cobot_control.world import DigitalWorld

class Planner(Node):

    def __init__(self):
        super().__init__('planner')

    def explore_primitive_task(self, current_task):
        print("Explore {}".format(current_task))
        return True if self.world.are_preconditions_met(current_task) else False

    def sort_priority(self, elem):
        return elem[1]

    def explore_compound_task(self, current_task):
        print("Explore {}".format(current_task))
        method = self.world.find_satisfied_method(current_task)
        if method:
            method.sort(key=self.sort_priority)
            new_tasks = [task for task in self.world.find_subtasks(method[0][0])]
            print("NEW TASKS TO ADD {}".format(new_tasks))
            return new_tasks
        else:
            self.restore_to_last_decomposed_task(decomp_history)

    def search(self, final_plan, tasks_to_process):
        print("TASKS TO PROCESS: {}".format(tasks_to_process))
        if not tasks_to_process:
            return final_plan
        else:
            current_task = tasks_to_process.pop(0)
            if self.world.find_type(current_task) == "CompoundTask":
                new_tasks = self.explore_compound_task(current_task)
                if new_tasks:
                    tasks_to_process.extend(new_tasks)
                    self.search(final_plan, tasks_to_process)
                    #del tasks_to_process[:-len(new_tasks)]
                else:
                    tasks_to_process.append(current_task)
                    self.search(final_plan, tasks_to_process)
            else:  # Primitive task
                if self.explore_primitive_task(current_task):
                    self.world.apply_effects(current_task)
                    self.search(final_plan, tasks_to_process)
                    final_plan.insert(0, current_task)
                else:
                    tasks_to_process.append(current_task)
                    self.search(final_plan, tasks_to_process)
            return final_plan


    def create_plan(self, current_world):
        try:
            self.world = DigitalWorld(current_world)
            final_plan = []
            tasks_to_process = self.world.root_task
            final_plan = self.search(final_plan, tasks_to_process)
            print("EXECUTE: {}".format(final_plan))
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
    world.send_command()
    planner.create_plan(world)

    rclpy.spin(planner)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
