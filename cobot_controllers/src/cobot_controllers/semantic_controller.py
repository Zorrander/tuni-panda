from sem_server_ros.server_com import FusekiEndpoint
from sem_server_ros.queries import QueryTemplateEngine
from cobot_tuni_msgs.msg import  Command

class SemanticController():

    def __init__(self, host, dataset, pose_estimator, action_publisher):
        self.sem_server = FusekiEndpoint(host, dataset)
        self.query_engine = QueryTemplateEngine(__file__)
        self.action_map = {
            'ArmEvent': self.move_arm,
            'GripperEvent': self.move_gripper
        }
        self.pose_estimator = pose_estimator
        self.action_publisher = action_publisher

    def notify_listeners(self, cmd, obj):
        print("Sending {}".format(Command(cmd, obj)))
        self.action_publisher.publish(Command(cmd, obj))

    def move_arm(self, next_target):
        template = self.query_engine.load_template('select_id_target.rq')
        query = self.query_engine.generate(template, next_target)
        print(query)
        result = self.sem_server.select_data(query)
        print("Estimate pose and go to object {}".format(result[0]['id']['value']))
        #resp = self.pose_estimator(result[0]['id']['value'])
        #print(resp)
        # self.arm.go_to_cartesian_goal(resp.obj_pose)

    def move_gripper(self, next_target):
        template = self.query_engine.load_template('select_width_target.rq')
        query = self.query_engine.generate(template, next_target)
        print(query)
        result = self.sem_server.select_data(query)
        print(result)
        # self.gripper.grasp(20, result['width']['value'])  # force, width

    def interpret(self, action, target):
        ''' decide what to do '''

        action= 'cogrob:'+str(action).split('#')[1][:-1]
        target= 'cogrob:'+str(target).split('#')[1][:-1]
        print("Got {} {}".format(action, target))
        while True:
            template = self.query_engine.load_template('select_next_move.rq')
            query = self.query_engine.generate(template, action)
            result = self.sem_server.select_data(query)
            if result:
                next_action = result[0]['action']['value']
                next_action_type = result[0]['actionClass']['value']
                next_target = result[0]['target']['value']
                print("next move: {}".format(str(next_action_type).split('#')[1]))
                print("next_target: {}".format(target))
                self.action_map[str(next_action_type).split('#')[1]](target)
                self.sem_server.add_data('cogrob:'+str(next_action).split('#')[1], 'cogrob:isCompleted', '"true"^^xsd:boolean')
            else:
                self.notify_listeners(action, target)
                break
