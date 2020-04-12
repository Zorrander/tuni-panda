import rclpy
from rclpy.node import Node
import time
import copy
from tuni_cobot_control.world import DigitalWorld

class DispatchingError(Exception):
   def __init__(self, primitive):
      self.primitive = primitive

class Planner(Node):

    def __init__(self, current_world):
        super().__init__('planner')
        self.current_world = current_world

    def explore_primitive_task(self, current_task):
        return True if self.planning_world.are_preconditions_met(current_task) else False

    def explore_compound_task(self, current_task):
        method = self.planning_world.find_satisfied_method(current_task)
        if method:
            new_tasks = [task for task in self.planning_world.find_subtasks(method)]
            return new_tasks

    def search(self, final_plan, tasks_to_process):
        if not tasks_to_process:
            return final_plan
        else:
            current_task = tasks_to_process.pop(0)
            if self.planning_world.find_type(current_task) == "CompoundTask":
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
                    self.planning_world.apply_effects(current_task)
                    self.search(final_plan, tasks_to_process)
                    final_plan.insert(0, current_task)
                else:
                    tasks_to_process.append(current_task)
                    self.search(final_plan, tasks_to_process)
            return final_plan


    def create_plan(self):
        try:
            final_plan = []
            self.planning_world = DigitalWorld(self.current_world)
            tasks_to_process = self.planning_world.root_task
            final_plan = self.search(final_plan, tasks_to_process)
            print("PLAN: {}".format(final_plan))
            return final_plan
        except Exception as e:
            print(e)

    def execute(self, primitive):
        self.current_world.apply_effects(primitive)
        print("RUN:{}".format(primitive))
        time.sleep(2)
        #self.sem_controller.interpret(primitive)

    def run(self, plan):
        try:
            print("EXECUTE: {}".format(plan))
            while plan:
                primitive = plan.pop(0)
                if self.current_world.are_preconditions_met(primitive):
                    self.execute(primitive)
                else:
                    raise DispatchingError(primitive)
        except DispatchingError as e:
            new_plan = self.create_plan()
            self.run(new_plan)

def main(args=None):
    rclpy.init(args=args)
    world = DigitalWorld()
    planner = Planner(world)
    world.send_command()
    while True:
        plan = planner.create_plan()
        planner.run(plan)
        time.sleep(1)

    rclpy.spin(planner)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
