import rclpy
from rclpy.node import Node
from owlready2 import *
import time
from tuni_cobot_control import state, operator, reasoning

class Planner(Node):

    def __init__(self, planning_world, state_handler):
        super().__init__('planner')
        print("Hello world!")
        #self.sem_controller = sem_controller
        self.final_plan = []
        self.decomp_history = []
        self.planning_world = planning_world
        self.state_handler = state_handler
        self.states = {
            'IsHoldingSomething':  state_handler.is_holding_something,
            #'IsNotHoldingSomething':  state_handler.IsNotHoldingSomething,
            #'IsCapableOfReaching': state_handler.IsCapableOfReaching,
            #'IsNotCapableOfReaching':  state_handler.IsNotCapableOfReaching,
            #'IsReadyToBeTaken': state_handler.IsReadyToBeTaken,
            #'IsNotReadyToBeTaken':  state_handler.IsNotReadyToBeTaken,
            'ReceivedHandoverCommand':  state_handler.received_handover_command,
        }

        self.operators = {
            'IdleOperator':  operator.IdleOperator(),
            'MoveOperator':  operator.MoveOperator(),
            'CloseOperator': operator.CloseOperator(),
            'OpenOperator':  operator.OpenOperator()
        }

    def init_working_world_state(self):
        pass

    def find_type(self, task):
        compound_tasks = self.planning_world.search(type = self.planning_world['http://onto-server-tuni.herokuapp.com/Panda#CompoundTask'])
        primitive_tasks = self.planning_world.search(type = self.planning_world['http://onto-server-tuni.herokuapp.com/Panda#PrimitiveTask'])
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

        print("primitive:{}".format(primitive))
        result = True
        for condition in primitive.INDIRECT_hasCondition:
            print("condition:{}".format(condition.__dict__))
            print(condition.is_a[0].name)
            if not states[condition.is_a[0].name]:
                result = False
        print("{} are_preconditions_met : {}".format(primitive, result) )
        return result

    def find_subtasks(self, method):
        print("method.hasSubtask".format(method.hasSubtask))
        return method.hasSubtask

    def record_decomposition_of_task(self, current_task, method):
        self.decomp_history.insert(0, (current_task, self.final_plan, method))

    def restore_to_last_decomposed_task(self):
        last_record = self.decomp_history.pop(0)
        self.tasks_to_process.append(last_record[0])
        self.final_plan = last_record[1]

    def apply_effects(self, primitive):
        primitive.isCompleted = True
        operators[primitive.INDIRECT_useOperator[0].name].run(self.state_handler)
        sync_reasoner_pellet(self.planning_world, infer_property_values = True, infer_data_property_values = True)

    def send_command(self):
        cmd = self.planning_world['http://onto-server-tuni.herokuapp.com/Panda#HandoverCommand']()
        cmd.has_action = "give"
        print(cmd.has_action)
        sync_reasoner_pellet(self.planning_world, infer_property_values = True, infer_data_property_values = True)
        print(list(self.planning_world.inconsistent_classes()))
        print(cmd.is_a)

    def create_plan(self, root):
        try:
            #self.init_working_world_state()
            self.final_plan = []
            self.decomp_history = []
            self.tasks_to_process = root
            print("TASKS TO PROCESS: {}".format(self.tasks_to_process))
            while self.tasks_to_process:
                current_task = self.tasks_to_process.pop(0)
                print("CURRENT TASK: {}".format(current_task))
                if self.find_type(current_task) == "CompoundTask":
                    method = self.find_satisfied_method(current_task)
                    test = True if method else False
                    print("if method {}".format(test))
                    if method:
                        self.record_decomposition_of_task(current_task, method)
                        new_tasks = [task for task in self.find_subtasks(method) if task not in self.final_plan]
                        print("NEW TASKS TO ADD {}".format(new_tasks))
                        self.tasks_to_process.extend(new_tasks)
                    else:
                        self.restore_to_last_decomposed_task()
                else:  # Primitive task
                    if self.are_preconditions_met(current_task):
                        print("Preconditions of {} are met".format(current_task))
                        self.apply_effects(current_task)
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
    real_world = World()
    planning_world = World()
    onto = planning_world.get_ontology("file:///home/alex/handover.owl").load()
    state_handler = state.State(onto)

    planner = Planner(planning_world, state_handler)
    while True:
        planner.send_command()
        time.sleep(2)
        root_task = [planning_world['http://onto-server-tuni.herokuapp.com/Panda#be']]
        planner.create_plan(root_task)
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
