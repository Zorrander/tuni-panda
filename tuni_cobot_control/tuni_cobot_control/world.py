from tuni_cobot_control.state import *
from tuni_cobot_control import condition, operator

class DigitalWorld():

    def __init__(self, original_world=None):
        if original_world:
            self.world = original_world.world
        else:
            self.world = World()
            self.world.get_ontology("file:///home/alex/handover.owl").load()
        self.root_task = [self.world['http://onto-server-tuni.herokuapp.com/Panda#be']]

    def send_command(self):
        cmd = self.world['http://onto-server-tuni.herokuapp.com/Panda#HandoverCommand']()
        cmd.has_action = "give"
        print(cmd.has_action)
        print(list(self.world.inconsistent_classes()))
        print(cmd.is_a)

    def find_type(self, task):
        compound_tasks = self.world.search(type = self.world['http://onto-server-tuni.herokuapp.com/Panda#CompoundTask'])
        primitive_tasks = self.world.search(type = self.world['http://onto-server-tuni.herokuapp.com/Panda#PrimitiveTask'])
        print(compound_tasks)
        print(task)
        result = "CompoundTask" if task in compound_tasks or task in primitive_tasks else "PrimitiveTask"
        print(result)
        return result

    def find_satisfied_method(self, current_task):
        satisfied_methods = set()
        for method in current_task.hasMethod:
            if self.are_preconditions_met(method):
                priority = method.hasPriority
                satisfied_methods.add((method, priority))
        return self.has_highest_priority(satisfied_methods)

    def has_highest_priority(self, tuples):
        max_prio = 100
        result = "cogrob:temp"
        for method, priority in tuples:
            if priority < max_prio:
                result = method
                max_prio = priority
        print("max_prio:{}".format(result))
        return result

    def are_preconditions_met(self, primitive):
        with primitive.namespace:
            print("primitive:{}".format(primitive))
            result = True
            for conditions in primitive.INDIRECT_hasCondition:
                print("try")
                print(conditions.is_a[0].name)
                c = getattr(condition, conditions.is_a[0].name)
                print(c().evaluate(self.world))
                if not c().evaluate(self.world):
                        result = False
            print("{} are_preconditions_met : {}".format(primitive, result) )
            return result

    def find_subtasks(self, method):
        print("method.hasSubtask".format(method.hasSubtask))
        return method.hasSubtask

    def apply_effects(self, primitive):
        with primitive.namespace:
            primitive.isCompleted = True
            op = getattr(operator, primitive.INDIRECT_useOperator[0].name)
            op().run(self.world)
            print("efffects applied")
