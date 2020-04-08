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
        self.sem_controller = sem_controller
        self.sem_server = FusekiEndpoint(host, dataset)
        self.query_engine = QueryTemplateEngine(__file__)
        self.final_plan = []
        self.decomp_history = []

    def init_working_world_state(self):
        template = self.query_engine.load_template('create_plan_graph.rq')
        self.sem_server.add_data(template)

    def find_type(self, task):
        template = self.query_engine.load_template('select_type.rq')
        query = self.query_engine.generate(template, task)
        result = self.sem_server.select_data(query)
        print("F I N D  T Y P E : {}".format(result))
        return str(result[0]['type']['value']).split('#')[1]

    def are_preconditions_met(self, primitive, plan=True):
        return True

    def has_highest_priority(self, tuples):
        max_prio = 100
        result = "cogrob:temp"
        for method, priority in tuples:
            if priority < max_prio:
                result = method
                max_prio = priority
        return result

    def find_satisfied_method(self, current_task):
        template = self.query_engine.load_template('select_satisfied_methods.rq')
        query = self.query_engine.generate(template, current_task)
        result = self.sem_server.select_data(query)
        for state in self.decomp_history:
            if state[4] in result:
                result.remove(state[4])
        result = [('cogrob:'+str(x['method']['value'].split('#')[1]), int(x['priority']['value'])) for x in result]
        print("are_preconditions_met: {}".format(result))
        return self.has_highest_priority(result)

    def find_subtasks(self, method):
        template = self.query_engine.load_template('select_subtasks_methods.rq')
        query = self.query_engine.generate(template, method)
        result = self.sem_server.select_data(query)
        result = [('cogrob:'+str(x['subtask']['value'].split('#')[1])) for x in result]
        print("find_subtasks: {}".format(result))
        return result

    def record_decomposition_of_task(self, current_task, method):
        tasks_to_process = self.tasks_to_process
        tasks_to_process.insert(0, current_task)
        final_plan = self.final_plan
        self.decomp_history.insert(0, (tasks_to_process, final_plan, current_task, method))
        print("record_decomposition_of_task: {}".format(self.decomp_history))

    def restore_to_last_decomposed_task(self):
        last_record = self.decomp_history.pop(0)
        self.tasks_to_process = last_record[0]
        self.tasks_to_process.insert(0, last_record[2])
        self.final_plan = last_record[1]
        print("restore_to_last_decomposed_task: {}".format(self.decomp_history))
        print("restore_to_last_decomposed_task: {}".format(self.tasks_to_process))
        print("restore_to_last_decomposed_task: {}".format(self.final_plan))

    def apply_effects(self, primitive):
        template = self.query_engine.load_template('apply_effects.rq')
        query = self.query_engine.generate(template, primitive)
        result = self.sem_server.add_data(query)
        print("record_decomposition_of_task: {}".format(self.decomp_history))
        return result

    def create_plan(self, sem_command):
        try:
            self.init_working_world_state()
            self.final_plan = []
            self.decomp_history = []
            self.tasks_to_process = [sem_command] if not isinstance(sem_command, list) else sem_command
            while self.tasks_to_process:
                current_task = self.tasks_to_process.pop(0)
                if self.find_type(current_task) == "CompoundTask":
                    method = self.find_satisfied_method(current_task)
                    if method:
                        self.record_decomposition_of_task(current_task, method)
                        subtasks = self.find_subtasks(method)
                        for task in subtasks:
                            self.tasks_to_process.insert(0, task)
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
        self.sem_controller.interpret(primitive)

    def run(self):
        try:
            while self.final_plan:
                primitive = self.final_plan.pop(0)
                if self.are_preconditions_met(primitive, plan=False):
                    self.execute(primitive)
                else:
                    raise DispatchingError(primitive)
        except DispatchingError as e:
            self.final_plan.insert(0, e.primitive)
            self.create_plan(self.final_plan)
            self.run()
