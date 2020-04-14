from tuni_cobot_control.state import *
from tuni_cobot_control import condition, operator, robot, workspace

import copy


class DispatchingError(Exception):
   def __init__(self, primitive):
      self.primitive = primitive


class DigitalWorld():

    def __init__(self, original_world=None, root_task=None):
        if original_world:
            self.world = copy.copy(original_world.world)
            self.robot = copy.copy(original_world.robot)
            self.workspace = copy.copy(original_world.workspace)
        else:
            self.world = World()
            self.world.get_ontology("file:///home/alex/handover.owl").load()
            self.world['http://onto-server-tuni.herokuapp.com/Panda#Robot']()
            with self.world.ontologies['http://onto-server-tuni.herokuapp.com/Panda#']:
                self.robot = robot.Robot()
                self.workspace = workspace.Workspace()
        self.root_task = list(root_task) if root_task else [self.world['http://onto-server-tuni.herokuapp.com/Panda#Be']]


    def add_object(self, name):
        with self.world.ontologies['http://onto-server-tuni.herokuapp.com/Panda#']:
            objects = self.world.search(is_a = self.world['http://onto-server-tuni.herokuapp.com/Panda#Object'])
            for object in objects:
                if object.is_called == name:
                    self.workspace.contains.append(object())

    def is_workspace_empty(self):
        return self.workspace.is_empty()

    def send_command(self, command, target=None):
        cmd = self.world['http://onto-server-tuni.herokuapp.com/Panda#Command']()
        cmd.has_action = command
        if target:
            cmd.has_target = target

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

    def check_state(self, state):
        result = False
        print("state")
        print(state)
        if not state == False:
            with self.world.ontologies['http://onto-server-tuni.herokuapp.com/Panda#']:
                c = getattr(condition, state.name)
                if c().evaluate(self.world, self.robot, self.workspace):
                    result = True
        return result

    def are_preconditions_met(self, primitive):
        with self.world.ontologies['http://onto-server-tuni.herokuapp.com/Panda#']:
            if len(primitive.INDIRECT_hasCondition) == 0:
                return True
            else:
                for conditions in primitive.INDIRECT_hasCondition:
                    c = getattr(condition, conditions.name)
                    if c().evaluate(self.world, self.robot, self.workspace):
                            return True
            return False

    def are_effects_satisfied(self, task):
        with self.world.ontologies['http://onto-server-tuni.herokuapp.com/Panda#']:
            result = False
            for effects in task.INDIRECT_hasEffect:
                e = getattr(condition, effects.name)
                if e().evaluate(self.world, self.robot, self.workspace):
                    print("{} already satisfied".format(effects))
                    result = True
            return result

    def resolve_conflicts(self, diff_vector):
        with self.world.ontologies['http://onto-server-tuni.herokuapp.com/Panda#']:
            if diff_vector[0] > 0:
                task0 = self.world['http://onto-server-tuni.herokuapp.com/Panda#PickTask']
                task1 = self.world['http://onto-server-tuni.herokuapp.com/Panda#PlaceTask']
                tasks = [task0, task1]
                for task in tasks:
                    task.actsOn = diff_vector[1]
                return tasks

    def find_subtasks(self, method):
        return method.hasSubtask

    def apply_effects(self, primitive):
        try:
            with self.world.ontologies['http://onto-server-tuni.herokuapp.com/Panda#']:
                primitive.isCompleted = True
                op = getattr(operator, primitive.INDIRECT_useOperator[0].name)
                op().run(self.world, primitive, self.robot)
        except:
            raise DispatchingError(primitive)
