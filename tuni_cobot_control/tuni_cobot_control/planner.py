import rclpy
from rclpy.node import Node
import time
import copy
from tuni_cobot_control.world import DigitalWorld
from cmd import Cmd

class MyPrompt(Cmd):
    def __init__(self):
        super(MyPrompt, self).__init__()
        self.world = DigitalWorld()
        self.planner = Planner()
        self.prompt = 'Panda> '
        self.intro = """
Collaborative Robotics Planning
-------------------------------

Type ? to list commands
     plan - to reason about the current status of the world
     handover - to send the corresponding command and plan
     take - to simulate a human taking something from the robot
     assembly - to plan for the Cranfield Assembly benchmark
     packaging - to try cleaning the workspace putting all the objects in a box
     status - to show the robot properties
"""
    def execute(self, primitive):
        self.world.apply_effects(primitive)
        print("RUN:{}".format(primitive))
        time.sleep(2)
        #self.sem_controller.interpret(primitive)

    def run(self, plan):
        try:
            while plan:
                primitive = plan.pop(0)
                if self.world.are_preconditions_met(primitive):
                    self.execute(primitive)
                else:
                    raise DispatchingError(primitive)
        except DispatchingError as e:
            new_plan = self.planner.create_plan(self.world)
            self.run(new_plan)

    def do_exit(self, inp):
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        self.planner.destroy_node()
        return True

    def do_plan(self, inp):
        plan = self.planner.create_plan(self.world)
        self.run(plan)

    def do_take(self, inp):
        self.world.send_command('release')
        plan = self.planner.create_plan(self.world)
        self.run(plan)

    def do_handover(self, inp):
        self.world.send_command('give')
        plan = self.planner.create_plan(self.world)
        self.run(plan)

    def do_packaging(self, inp):
        print("Coming soon")

    def do_assembly(self, inp):
        print("Coming soon")

    def do_status(self, inp):
        self.world.robot.print_status()

class DispatchingError(Exception):
   def __init__(self, primitive):
      self.primitive = primitive

class Planner(Node):

    def __init__(self):
        super().__init__('planner')

    def explore_primitive_task(self, current_task):
        print("explore")
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
            print(current_task)
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


    def create_plan(self, current_world):
        try:
            final_plan = []
            self.planning_world = DigitalWorld(current_world)
            self.planning_world.robot.print_status()
            tasks_to_process = self.planning_world.root_task
            final_plan = self.search(final_plan, tasks_to_process)
            print("PLAN: {}".format(final_plan))
            return final_plan
        except Exception as e:
            print(e)


def main(args=None):
    rclpy.init(args=args)
    MyPrompt().cmdloop()
    #rclpy.spin(planner)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
