#!/usr/bin/env python3

import os
import time
import rospy
# from rclpy.node import Node
import speech_recognition as sr
import tf
from std_msgs.msg import Empty
from std_srvs.srv import Trigger, TriggerRequest
from geometry_msgs.msg import Point

from semrob.robot import robot
from semrob.world import world

from cobot_msgs.msg import Command, Detection
from cobot_msgs.srv import ReachCartesianPose, Grasp, MoveGripper, NamedTarget, ReachJointPose

# from cobot_sem.ros_world import DigitalWorldInterface


class DigitalWorldInterface(world.DigitalWorld):
    def __init__(self, world_file):
        world.DigitalWorld.__init__(self, world_file)
        #self.robot_name = self.create_client(RobotName, '/robot_name')
        #self.subscription = self.create_subscription(
        #    Point,
        #    'target_pose',
        #    self.camera_callback,
        #    10)
        # self.sub = self.create_subscription(Command, '/plan_request', self.process_command, 10)

        # self.target_reached_sub = self.create_subscription(Empty, '/object_released', self.object_released, 10)

    def convert_detection_pose(self, x ,y):
        listener_tf = tf.TransformListener()
        camera_focal = 550
        (trans1, rot1) = listener_tf.lookupTransform('/panda_link0', '/camera_color_optical_frame', rospy.Time(0))
        z_to_surface = trans1[2]
        to_world_scale = z_to_surface / camera_focal

        x_dist = x * to_world_scale
        y_dist = y * to_world_scale

        my_point = PoseStamped()
        my_point.header.frame_id = "camera_color_optical_frame"
        my_point.header.stamp = rospy.Time(0)
        my_point.pose.position.x = 0
        my_point.pose.position.y = -x_dist
        my_point.pose.position.z = y_dist
        theta = 0
        quat = tf.transformations.quaternion_from_euler(0, 0, theta)
        my_point.pose.orientation.x = quat[0]
        my_point.pose.orientation.y = quat[1]
        my_point.pose.orientation.z = quat[2]
        my_point.pose.orientation.w = quat[3]
        ps = listener_tf.transformPose("/panda_link0", my_point)

        (trans, rot) = listener_tf.lookupTransform('/panda_link0', '/camera_color_optical_frame', rospy.Time(0))
        data = (ps.pose.position.x - trans[0], ps.pose.position.y - trans[1])


    def detection_callback(self, detection_msg):
        bounding_box = detection_msg.bounding_box
        angle = detection_msg.angle
        kps_center = [detection_msg.kps_x, detection_msg.kps_y]
        obj_class = detection_msg.obj_class
        print(self.onto.search(subclass_of = self.core_onto.Entity))
        if obj_class == 0:
            obj = self.object_interface.create("rocker_arm")
        elif obj_class == 1:
            obj = self.object_interface.create("pushrod")

        print("""

            """)

        print("You just detected a ", obj_class)
        print(obj)
        print("""

            """)
        ctr_X = int((bounding_box[0]+bounding_box[2])/2)
        ctr_Y = int((bounding_box[1]+bounding_box[3])/2)
        ref_x = 640/2
        ref_y = 480/2
        dist = [ctr_X - ref_x, ref_y - ctr_Y]
        x_robot, y_robot = self.convert_detection_pose(dist[0], dist[1])
        print(x_robot, y_robot)
        print("""

            """)

    def sphinx_callback(self, recognizer, audio):
        # recognize speech using Sphinx
        try:
            print("Think")
            # print("Sphinx thinks you said " + recognizer.recognize_sphinx(audio, grammar=path.join(path.dirname(path.realpath(__file__)), 'counting.gram')))
            cmd = recognizer.recognize_sphinx(audio, keyword_entries=[("give", 1.0), ("reach", 1.0), ("grasp", 1.0), ("release", 1.0), ("peg", 1.0)])
            bow = cmd.strip().split(' ')
            print(bow)
            if len(bow) == 3:
                print("Sphinx thinks you said [{} - {}]".format(bow[2], bow[0]))
                self.run(command=(bow[2], bow[0]))

        except sr.UnknownValueError:
            print("Sphinx could not understand audio")
        except sr.RequestError as e:
            print("Sphinx error; {0}".format(e))
        print("Say something")

    def start_listening(self):
        r = sr.Recognizer()
        m = sr.Microphone(device_index=8)
        with m as source:
            r.adjust_for_ambient_noise(source, duration=2)
            print("say something!")
            audio = r.listen(m)
            self.sphinx_callback(r, audio)

    def object_released(self, empty_msg):
        self.dismiss_command()

    def get_robot_name(self):
        req = RobotName.Request()
        print("get_robot_name...")
        res = self.robot_name.call(req)
        print(res.name)

    def camera_callback(self, msg):
        self.onto.box1.update_pose(msg.x, msg.y, msg.z)


class RealCollaborativeRobot(robot.CollaborativeRobotInterface):

    def __init__(self):
        # Node.__init__(self, 'real_robot')

        #self.declare_parameter('world_file')
        #param_str = self.get_parameter('world_file')
        #world_interface = DigitalWorldInterface(str(param_str.value))
        param_str = rospy.get_param("/semantic_planning/world_file")
        world_interface = DigitalWorldInterface(str(param_str))

        robot.CollaborativeRobotInterface.__init__(self, world_interface)

        self.target_reached_pub = rospy.Publisher('/target_reached', Empty, queue_size=10)
        self.object_released_pub = rospy.Publisher('/object_released', Empty, queue_size=10)

        self.human_ready_sub = rospy.Subscriber('/human_ready', Empty, self.human_ready)
        self.sub = rospy.Subscriber('/plan_request', Command, self.process_command)

        self.joint_move_to = rospy.ServiceProxy('/go_to_joint_space_goal', ReachJointPose)
        self.cartesian_move_to = rospy.ServiceProxy('/go_to_cartesian_goal', ReachCartesianPose)
        self.grasp = rospy.ServiceProxy('/grasp', Grasp)
        self.reach_named_target = rospy.ServiceProxy('/move_to', NamedTarget)
        self.release = rospy.ServiceProxy('/move_gripper', MoveGripper)
        self.reset = rospy.ServiceProxy('/reset', Trigger)
        self.idle = rospy.ServiceProxy('/idle', Trigger)
        self.communicate = rospy.ServiceProxy('/communicate', Trigger)

    def human_ready(self, empty_msg):
        self.world.onto.agent.isReady = True

    def process_command(self, command_msg):
        print("Received command: ", command_msg)
        action = command_msg.action.lower()
        target = [x.lower() for x in command_msg.targets] if command_msg.targets else []
        self.world.send_command(action, target)

    def move_operator(self, target):
        self.is_waiting = False
        print("_use_move_operator {}...".format(target))
        self.world.onto.agent.isReady = False
        # req.position.layout.dim[0] = 7
        req = NamedTarget()
        req.name = target
        res = self.reach_named_target(req)
        while not res.done():
            time.sleep(0.1)
        self.release_planner()

    def close_operator(self, target):
        self.is_waiting = False
        req = Grasp()
        print(target)
        print(target.has_width)
        req.width = float(target.has_width) if target.has_width else 0.002  # [cm]
        req.force = 100.0  # [N]
        print("Grasping {}...".format(target))
        res = self.grasp(req)
        while not res.done():
            time.sleep(0.1)
        self.release_planner()

    def open_operator(self, target):
        self.is_waiting = False
        # msg = Empty()
        # self.object_released_pub.publish(msg)
        req = MoveGripper()
        req.width = 2.5
        res = self.release(req)
        while not res.done():
            time.sleep(0.1)
        # return release
        self.release_planner()

    def communication_operator(self):
        print("Real robot is communication_operator...")
        if not self.is_waiting:
            msg = Empty()
            self.target_reached_pub.publish(msg)
            self.is_waiting = True
        req = TriggerRequest()
        res = self.communicate(req)
        while not res.done():
            time.sleep(0.1)
        self.release_planner()

    def idle_operator(self):
        print("Real robot is waiting...")
        req = TriggerRequest()
        res = self.idle(req)
        #while not res.done():
        #    time.sleep(0.1)
        #self.release_planner()
        # return wait

    def stop_operator(self):
        def stop():
            print("Stopping...")
        return stop

    def reset_operator(self):
        self.is_waiting = False
        print("_use_reset_operator..")
        self.world.onto.agent.isReady = False
        # req.position.layout.dim[0] = 7
        req = NamedTarget()
        req.name = "init_pose"
        res = self.reach_named_target(req)
        while not res.done():
            time.sleep(0.1)
        self.world.dismiss_command()
        self.release_planner()

    def handle_anchoring_error(self, object):
        print("REACH FINAL STAGE OF ERROR")
        print("COULD NOT ANCHOR", object)

    def handle_grounding_error(self, object):
        print("COULD NOT GROUND", object)

def main(args=None):
    rospy.init_node('semantic_planning', anonymous=True)

    robot = RealCollaborativeRobot()

    detection_sub = rospy.Subscriber("objects_detected", Detection, robot.world.detection_callback)

    rospy.spin()

    # robot.destroy_node()
    # rclpy.shutdown()

if __name__ == '__main__':
    main()
