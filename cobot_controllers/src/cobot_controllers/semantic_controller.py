from sem_server_ros.server_com import FusekiEndpoint
from sem_server_ros.queries import QueryTemplateEngine


class SemanticController():

    def __init__(self, host, dataset, pose_estimator):
        self.sem_server = FusekiEndpoint(host, dataset)
        self.query_engine = QueryTemplateEngine(__file__)
        self.action_map = {
            'ArmEvent': self.move_arm,
            'GripperEvent': self.move_gripper
        }
        self.pose_estimator = pose_estimator

    def move_arm(self, next_target):
        template = self.query_engine.load_template('select_id_target.rq')
        query = self.query_engine.generate(template, next_target)
        print(query)
        result = self.sem_server.select_data(query)
        resp = self.pose_estimator(result['id']['value'])
        self.arm.go_to_cartesian_goal(resp.obj_pose)

    def move_gripper(self, next_target):
        template = self.query_engine.load_template('select_width_target.rq')
        query = self.query_engine.generate(template, next_target)
        print(query)
        result = self.sem_server.select_data(query)
        self.gripper.grasp(20, result['width']['value'])  # force, width

    def interpret(self, action, target):
        ''' decide what to do '''
        while True:
            template = self.query_engine.load_template('select_next_move.rq')
            query = self.query_engine.generate(template, 'cogrob:'+str(action).split('#')[1][:-1])
            print(query)
            result = self.sem_server.select_data(query)
            if result:
                next_move = result['action']['value']
                next_target = result['target']['value']
                print("next move: {}".format(next_move))
                print("next_target: {}".format(next_target))
                self.action_map[next_move]('cogrob:'+str(next_target).split('#')[1][:-1])
            else:
                break
