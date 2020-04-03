'''
Dispatch a plan
'''

class Dispatcher():
    def __init__(self, plan):
        self.time = 0
        self.plan = plan
        self.counter = 3

    def choose_step(self, available_steps):
        ''' Implement different collaboration policies '''
        return available_steps[0]

    def apply_timestamp(self, action):
        ''' Set TP's execution time to current_timeand add TP to S '''
        ''' Propagate the time of executionto its IMMEDIATE NEIGHBORS in the distancegraph '''
        ''' Put in A all events TPx such that allnegative edges starting from TPx have adestination that is already in S; '''
        print("{} {}".format(action, self.counter))

    def dispatch(self):
        available_steps = self.plan.find_available_steps(self.time)
        while available_steps:
            # Pick an event to be performed
            task, object = self.choose_step(available_steps)
            yield (task, object)
            # Apply a timestamp to it
            self.plan.update_after_completion(task, self.time)
            # Update available steps
            available_steps = self.plan.find_available_steps(self.time)
