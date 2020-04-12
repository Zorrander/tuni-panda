from tuni_cobot_control.state import *
from tuni_cobot_control import condition, operator, robot
import time

class DigitalWorld():

    def __init__(self, original_world=None):
        if original_world:
            self.world = original_world.world
        else:
            self.world = World()
            self.world.get_ontology("file:///home/alex/handover.owl").load()
            self.world['http://onto-server-tuni.herokuapp.com/Panda#Robot']()
            #self.world.get_ontology("http://onto-server-tuni.herokuapp.com/Panda/data").load()
        self.root_task = [self.world['http://onto-server-tuni.herokuapp.com/Panda#Be']]

        with self.world.ontologies['http://onto-server-tuni.herokuapp.com/Panda#']:
            self.robot = robot.Robot()

    def send_command(self):
        cmd = self.world['http://onto-server-tuni.herokuapp.com/Panda#HandoverCommand']()
        cmd.has_action = "give"

    def find_type(self, task):
        compound_tasks = self.world.search(subclass_of = self.world['http://onto-server-tuni.herokuapp.com/Panda#CompoundTask'])
        print(compound_tasks)
        result = "CompoundTask" if task in compound_tasks else "PrimitiveTask"
        return result

    def find_satisfied_method(self, current_task):
        satisfied_methods = []
        for method in current_task.hasMethod:
            if self.are_preconditions_met(method):
                satisfied_methods.append((method, method.hasPriority))
        return satisfied_methods

    def has_highest_priority(self, methods):
        max_prio = 100
        result = "cogrob:temp"
        for method in methods:
            priority = method.hasPriority
            if priority < max_prio:
                result = method
                max_prio = priority
        return result

    def are_preconditions_met(self, primitive):
        with self.world.ontologies['http://onto-server-tuni.herokuapp.com/Panda#']:
            result = True
            for conditions in primitive.INDIRECT_hasCondition:
                print("Explore {} of {}".format(conditions, primitive))
                c = getattr(condition, conditions.name)
                if not c().evaluate(self.world, self.robot):
                        result = False
                print(result)
            return result

    def find_subtasks(self, method):
        return method.hasSubtask

    def apply_effects(self, primitive):
        with self.world.ontologies['http://onto-server-tuni.herokuapp.com/Panda#']:
            primitive.isCompleted = True
            op = getattr(operator, primitive.INDIRECT_useOperator[0].name)
            op().run(self.world, primitive, self.robot)
