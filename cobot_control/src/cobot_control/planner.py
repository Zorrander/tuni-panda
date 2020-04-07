from simple_net.planner import Planner
from sem_server_ros.server_com import FusekiEndpoint
from sem_server_ros.queries import QueryTemplateEngine
from cobot_controllers.semantic_controller import SemanticController
from cobot_tuni_msgs.msg import Command

class DispatchingError(Exception):
   def __init__(self, primitive):
      self.primitive = primitive

class RosPlanner:

    def __init__(self, host, dataset, sem_controller):
        self.final_plan = []
        self.decomp_history = []
        self.working_world_state = []
        self.sem_controller = sem_controller
        self.sem_server = FusekiEndpoint(host, dataset)
        self.query_engine = QueryTemplateEngine(__file__)
        self.action_publisher = action_publisher

    def find_type(self, task):
        pass

    def find_preconditions(self, task):
        pass

    def find_effects(self, task):
        pass

    def are_preconditions_met(self, primitive):
        pass

    def find_satisfied_method(self):
        template = self.query_engine.load_template('select_semantic_plan.rq')
        query = self.query_engine.generate(template, sem_command)
        result = self.sem_server.construct_data(query)
        return result

    def find_subtasks(self, method):
        pass

    def record_decomposition_of_task(pass, current_task, final_plan):
        pass

    def restore_to_last_decomposed_task(self):
        pass

    def apply_effects(self, primitive):
        pass

    def create_plan(self, sem_command):
        try:
            self.final_plan = []
            tasks_to_process = list(sem_command)
            while tasks_to_process:
                current_task = tasks_to_process.pop(0)
                if self.find_type(current_task) == "CompoundTask":
                    method = self.find_satisfied_method()
                    if method:
                        self.record_decomposition_of_task(current_task, self.final_plan)
                        subtasks = self.find_subtasks(method)
                        tasks_to_process.insert(subtasks)
                    else:
                        self.restore_to_last_decomposed_task()
                else:  # Primitive task
                    if self.are_preconditions_met(current_task):
                        self.apply_effects(current_task)
                        self.final_plan.append(current_task)
                    else:
                        self.restore_to_last_decomposed_task()
        except Exception as e:
            print(e)

    def execute(self, primitive):
        sem_controller.interpret(primitive)

    def run(self, plan):
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
