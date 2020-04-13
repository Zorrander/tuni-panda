from tuni_cobot_control.state import *
from tuni_cobot_control import condition, operator, robot
import copy

class DigitalWorld():

    def __init__(self, original_world=None):
        if original_world:
            self.world = copy.copy(original_world.world)
            self.robot = copy.copy(original_world.robot)
        else:
            self.world = World()
            self.world.get_ontology("file:///home/alex/handover.owl").load()
            self.world['http://onto-server-tuni.herokuapp.com/Panda#Robot']()
            with self.world.ontologies['http://onto-server-tuni.herokuapp.com/Panda#']:
                self.robot = robot.Robot()

        self.root_task = [self.world['http://onto-server-tuni.herokuapp.com/Panda#Be']]

    def send_command(self, command):
        cmd = self.world['http://onto-server-tuni.herokuapp.com/Panda#Command']()
        cmd.has_action = command

    def find_type(self, task):
        result = "CompoundTask"
        for type in task.is_a:
            if "_name" in type.__dict__ and "PrimitiveTask" == type._name:
                result = "PrimitiveTask"
                break
        return result

    def find_satisfied_method(self, current_task):
        satisfied_methods = []
        for method in current_task.hasMethod:
            if self.are_preconditions_met(method):
                satisfied_methods.append(method)
        if satisfied_methods:
            return self.has_highest_priority(satisfied_methods)

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
                c = getattr(condition, conditions.name)
                if not c().evaluate(self.world, self.robot):
                        result = False
            return result

    def are_effects_satisfied(self, task):
        with self.world.ontologies['http://onto-server-tuni.herokuapp.com/Panda#']:
            result = True
            for effects in task.INDIRECT_hasEffect:
                e = getattr(condition, effects.name)
                if not e().evaluate(self.world, self.robot):
                    print("{} already satisfied".format(effects))
                    result = False
            return result

    def find_subtasks(self, method):
        return method.hasSubtask

    def apply_effects(self, primitive):
        with self.world.ontologies['http://onto-server-tuni.herokuapp.com/Panda#']:
            primitive.isCompleted = True
            op = getattr(operator, primitive.INDIRECT_useOperator[0].name)
            op().run(self.world, primitive, self.robot)
