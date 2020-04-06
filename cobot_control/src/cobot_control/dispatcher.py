from sem_server_ros.server_com import FusekiEndpoint
from sem_server_ros.queries import QueryTemplateEngine

'''
Dispatch a plan
'''

class Dispatcher():
    def __init__(self, plan, host, dataset):
        self.time = 0
        self.plan = plan
        self.counter = 3
        self.sem_server = FusekiEndpoint(host, dataset)
        self.query_engine = QueryTemplateEngine(__file__)

    def choose_step(self, available_steps):
        ''' Implement different collaboration policies '''
        return available_steps[0]

    def attribute_to_robot(self, event):
        print("attribute {}".format(event))
        template = self.query_engine.load_template('attribute_event.rq')
        query = self.query_engine.generate(template, 'cogrob:'+str(event).split('#')[1][:-1])
        print(query)
        result = self.sem_server.add_data(query)

    def apply_timestamp(self, action):
        ''' Set TP's execution time to current_timeand add TP to S '''
        ''' Propagate the time of executionto its IMMEDIATE NEIGHBORS in the distancegraph '''
        ''' Put in A all events TPx such that allnegative edges starting from TPx have adestination that is already in S; '''
        print("{} {}".format(action, self.counter))

    def dispatch(self):
        available_steps = self.plan.find_available_steps(self.time)
        while available_steps:
            print("Available steps: {}".format(available_steps))
            # Pick an event to be performed
            task, object = self.choose_step(available_steps)
            # Attribute it to the robot in the KB
            self.attribute_to_robot(task)
            yield (task, object)
            # Apply a timestamp to it
            self.plan.update_after_completion(task, self.time)
            # Update available steps
            available_steps = self.plan.find_available_steps(self.time)
