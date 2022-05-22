#!/usr/bin/env python
from __future__ import division


import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import rospy
from std_msgs.msg import String,Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import threading
from std_msgs.msg import Int16
import tf
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
import random
from control_msgs.msg import GripperCommandActionGoal, GripperCommandGoal, GripperCommand
import franka_gripper.msg
import actionlib

def all_close(goal, actual, tolerance):

  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveIt_Commander(object):
  """MoveIt_Commander"""
  def __init__(self):
    super(MoveIt_Commander, self).__init__()


    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "panda_arm"
    group = moveit_commander.MoveGroupCommander(group_name)
    group_tool_pose=moveit_commander.MoveGroupCommander(group_name)
    hand_grp=moveit_commander.MoveGroupCommander("hand")
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
    XY_command_sub = rospy.Subscriber("/commands",Float32MultiArray, self.callback_cmd)
    request_subscriber = rospy.Subscriber('/request_detection', Int16, self.callback_width)
    request_publisher = rospy.Publisher('/request_detection', Int16, queue_size=10)
    planning_frame = group.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame

    eef_link = group.get_end_effector_link()
    print "ee link is:"
    print "============ End effector: %s" % eef_link


    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()


    print "============ Printing robot state"
    print robot.get_current_state()
    print ""

    self.grasp_client  = actionlib.SimpleActionClient('/franka_gripper/grasp', franka_gripper.msg.GraspAction)
    self.generic_grasp_client =  rospy.Publisher('/franka_gripper/gripper_action/goal', GripperCommandActionGoal, queue_size=10)
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.hand_grp=hand_grp
    self.tool_grp=group_tool_pose
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names
    self.hoverflag=0
    self.orient_flag=0
    self.request_publisher = request_publisher
    self.last_msg_idx = 99999999
    self.last_msg_used = 99999999
    self.detections = [9999,9999,9999,9999,9999]
    self.camera_focal = 609.96
    self.width = 0.000
    self.angle = 0
    self.lift = 0

  def callback_cmd(self,data):
    #global xy_cmd
    if data.data[3] != 9999:

        rotcmd = data.data[3]
        if (rotcmd>180):
            rotcmd = -360 + rotcmd
        if (rotcmd > 0 and rotcmd > 90):
            rotcmd = -180 + rotcmd
        if (rotcmd < 0 and rotcmd <- 90):
            rotcmd = 180 + rotcmd

        self.last_msg_idx = data.data[0]
        self.detections = np.asarray([data.data[1], data.data[2], rotcmd , data.data[4], data.data[5]])

        #self.reach_hover()
        #self.fix_angle(self.detections[2]+self.angle)
        self.reach_grasp_hover_kps()
        self.fix_angle(self.detections[2]+self.angle)
        cartesian_plan, fraction = self.plan_linear_z(-0.061)
        self.execute_plan(cartesian_plan)
        self.grasp(100,self.width)
        #epsilon_inner=0.002, epsilon_outer=0.002
        #print("enter to lift")
        #raw_input()
        if self.lift:
            cartesian_plan, fraction = self.plan_linear_z(0.1)
            self.execute_plan(cartesian_plan)
            cartesian_plan, fraction = self.plan_linear_z(-0.1)
            self.execute_plan(cartesian_plan)
            self.open_hand()
            cartesian_plan, fraction = self.plan_linear_z(0.1)
            self.execute_plan(cartesian_plan)
        if not self.lift:
            self.open_hand()
        self.go_to_joint_state((0.00018762090386758654, -0.7771156373465307, 0.0005407026658047056, -2.365469873129632, -0.00020200796294576732, 1.5704722326730955, 0.7845368039521746))
    else:
        self.last_msg_idx = data.data[0]
        self.detections = np.asarray([9999,9999,9999,9999,9999])

  def callback_width(self, msg):
      if msg.data==1:
          self.width = 0.039
          self.angle = 0
          self.lift = True
      elif msg.data==2:
          self.width = 0.020
          self.angle = 0
          self.lift = True
      elif msg.data==3:
          self.width = 0.034
          self.angle = 0
          self.lift = True
      elif msg.data==4:
          self.width = 0.034
          self.angle = 0
          self.lift = True

  def gripper_move(self,cmd):

    group = self.hand_grp
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = cmd[0]
    joint_goal[1] = cmd[1]
    group.go(joint_goal, wait=True)
    group.stop()
    current_joints = self.hand_grp.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def go_to_joint_state(self,cmd):

    group = self.group
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = cmd[0]
    joint_goal[1] = cmd[1]
    joint_goal[2] = cmd[2]
    joint_goal[3] = cmd[3]
    joint_goal[4] = cmd[4]
    joint_goal[5] = cmd[5]
    joint_goal[6] = cmd[6]
    group.go(joint_goal, wait=True)
    group.stop()
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def go_to_pose_goal(self, target):
    # continue here target added to args
    group = self.group
    current=group.get_current_pose().pose
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = target.pose.orientation.x
    pose_goal.orientation.y = target.pose.orientation.y
    pose_goal.orientation.z = target.pose.orientation.z
    pose_goal.orientation.w = target.pose.orientation.w
    pose_goal.position.x = target.pose.position.x
    pose_goal.position.y = target.pose.position.y
    pose_goal.position.z = target.pose.position.z
    print pose_goal
    group.set_pose_target(pose_goal)
    plan = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

  def get_current_pose(self):

    group = self.group
    wpose = group.get_current_pose().pose

    return wpose

  def plan_linear_x(self, dist):


    group = self.group
    waypoints = []
    wpose = group.get_current_pose().pose
    wpose.position.x += dist
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,
                                       0.01,
                                       0.0)
    return plan, fraction

  def plan_linear_y(self, dist):

    group = self.group
    waypoints = []
    wpose = group.get_current_pose().pose
    wpose.position.y += dist
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,
                                       0.01,
                                       0.0)
    return plan, fraction

  def plan_linear_z(self, dist):

    group = self.group
    waypoints = []
    wpose = group.get_current_pose().pose
    wpose.position.z += dist
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,
                                       0.01,
                                       0.0)
    return plan, fraction

  def execute_plan(self, plan):

    group = self.group
    group.execute(plan, wait=True)

  def fix_angle(self,rotcmd):
      '''
      if (abs(rotcmd) < -90 and abs(rotcmd) > -180):
          rotcmd += 180
      if (abs(rotcmd) < 180 and abs(rotcmd) > 90):
          rotcmd -= 180
      '''
      ps = listener_tf.lookupTransform('/camera_link', '/panda_link8', rospy.Time(0))

      my_point=PoseStamped()
      my_point.header.frame_id = "camera_link"
      my_point.header.stamp = rospy.Time(0)
      my_point.pose.position.x = ps[0][0]
      my_point.pose.position.y = ps[0][1]
      my_point.pose.position.z = ps[0][2]

      theta = rotcmd / 180 * pi
      quat_rotcmd = tf.transformations.quaternion_from_euler(theta, 0, 0)
      quat = tf.transformations.quaternion_multiply(quat_rotcmd, ps[1])

      my_point.pose.orientation.x = quat[0]
      my_point.pose.orientation.y = quat[1]
      my_point.pose.orientation.z = quat[2]
      my_point.pose.orientation.w = quat[3]

      ps=listener_tf.transformPose("panda_link0",my_point)
      self.go_to_pose_goal(ps)

  def reach_grasp_hover_kps(self):


      (trans1,rot1) = listener_tf.lookupTransform('/panda_link0', '/camera_link', rospy.Time(0))
      z_to_surface = trans1[2]
      to_world_scale = z_to_surface / self.camera_focal

      x_dist = self.detections[3] * to_world_scale
      y_dist = self.detections[4] * to_world_scale

      my_point=PoseStamped()
      my_point.header.frame_id = "camera_link"
      my_point.header.stamp =rospy.Time(0)
      my_point.pose.position.x = 0
      my_point.pose.position.y = -x_dist
      my_point.pose.position.z = y_dist #0.1
      theta=0
      quat = tf.transformations.quaternion_from_euler(0, 0, theta)
      my_point.pose.orientation.x = quat[0]
      my_point.pose.orientation.y = quat[1]
      my_point.pose.orientation.z = quat[2]
      my_point.pose.orientation.w = quat[3]
      ps=listener_tf.transformPose("panda_link0",my_point)


      print ("camera link pose")
      print ps

      my_point1=PoseStamped()
      my_point1.header.frame_id = "panda_link8"
      my_point1.header.stamp =rospy.Time(0)
      my_point1.pose.position.x = 0
      my_point1.pose.position.y = 0
      my_point1.pose.position.z = 0
      theta1=0
      quat1 = tf.transformations.quaternion_from_euler(0, 0, theta1)
      my_point1.pose.orientation.x = quat1[0]
      my_point1.pose.orientation.y = quat1[1]
      my_point1.pose.orientation.z = quat1[2]
      my_point1.pose.orientation.w = quat1[3]
      ps1=listener_tf.transformPose("panda_link0",my_point1)


      ps1.pose.position.x = ps.pose.position.x
      ps1.pose.position.y = ps.pose.position.y
      ps1.pose.position.z = ps.pose.position.z

      self.go_to_pose_goal(ps1)
      print "reached pose"
      print self.get_current_pose()
      self.approach_grasp()
      # self.gripper_move((-8.584933242598848e-05,0.00019279284))

      # (trans,rot) = listener_tf.lookupTransform('/world', '/camera_link', rospy.Time(0))
      # data=(ps.pose.position.x-trans[0], ps.pose.position.y-trans[1])

      # cartesian_plan, fraction = self.plan_linear_x(data[0])
      # self.execute_plan(cartesian_plan)
      # cartesian_plan, fraction = self.plan_linear_y(data[1])
      # self.execute_plan(cartesian_plan)





  def reach_hover(self):

      detections_wrt_image = self.detections[0:2]
      # detections_wrt_camera
      print ("self.detections")
      print self.detections

      (trans1,rot1) = listener_tf.lookupTransform('/panda_link0', '/camera_link', rospy.Time(0))
      z_to_surface = trans1[2]
      to_world_scale = z_to_surface / self.camera_focal

      x_dist = self.detections[0] * to_world_scale
      y_dist = self.detections[1] * to_world_scale

      my_point=PoseStamped()
      my_point.header.frame_id = "camera_link"
      my_point.header.stamp =rospy.Time(0)
      my_point.pose.position.x = 0
      my_point.pose.position.y = -x_dist
      my_point.pose.position.z = y_dist
      theta=0
      quat = tf.transformations.quaternion_from_euler(0, 0, theta)
      my_point.pose.orientation.x = quat[0]
      my_point.pose.orientation.y = quat[1]
      my_point.pose.orientation.z = quat[2]
      my_point.pose.orientation.w = quat[3]
      ps=listener_tf.transformPose("panda_link0",my_point)


      (trans,rot) = listener_tf.lookupTransform('/panda_link0', '/camera_link', rospy.Time(0))
      data=(ps.pose.position.x-trans[0], ps.pose.position.y-trans[1])

      cartesian_plan, fraction = self.plan_linear_x(data[0])
      self.execute_plan(cartesian_plan)
      cartesian_plan, fraction = self.plan_linear_y(data[1])
      self.execute_plan(cartesian_plan)


  def approach_grasp(self):
      my_point=PoseStamped()
      my_point.header.frame_id = "camera_link"
      my_point.header.stamp =rospy.Time(0)
      my_point.pose.position.x = 0.40 #0.706 #0.685
      my_point.pose.position.y = 0.015
      my_point.pose.position.z = -0.0185
      theta=0
      quat = tf.transformations.quaternion_from_euler(0, 0, theta)
      my_point.pose.orientation.x = quat[0]
      my_point.pose.orientation.y = quat[1]
      my_point.pose.orientation.z = quat[2]
      my_point.pose.orientation.w = quat[3]
      ps=listener_tf.transformPose("panda_link0",my_point)

      (trans,rot) = listener_tf.lookupTransform('/panda_link0', '/camera_link', rospy.Time(0))
      data=(ps.pose.position.x-trans[0], ps.pose.position.y-trans[1], ps.pose.position.z-trans[2])
      cartesian_plan, fraction = self.plan_linear_x(data[0])
      self.execute_plan(cartesian_plan)
      cartesian_plan, fraction = self.plan_linear_y(data[1])
      self.execute_plan(cartesian_plan)
      cartesian_plan, fraction = self.plan_linear_z(data[2])
      self.execute_plan(cartesian_plan)

  def request_detection(self):
      '''
      while (True):
          msg_identifier = random.randint(0,1000)
          print msg_identifier
          if msg_identifier==self.last_msg_used:
              print "new id not generated"
              continue
          else:
              print "new id generated"
              break
      print msg_identifier
      self.request_publisher.publish(msg_identifier)
      '''

      # self.request_publisher.publish(msg_identifier)
      print "waiting for detection to be received:"
      start_time = time.time()
      while (time.time()-start_time < 10):
          print "wait for the message"
          time.sleep(1)

          print self.last_msg_idx
          print self.last_msg_used

          if self.last_msg_idx != self.last_msg_used:
              self.last_msg_used = self.last_msg_idx
              print ("detections received:")
              print self.detections
              break
          else:
              print "waiting..."


  def open_hand(self, wait=True):

        joint_goal = self.hand_grp.get_current_joint_values()
        joint_goal[0] = 0.0399

        self.hand_grp.set_goal_joint_tolerance(0.001)
        self.hand_grp.go(joint_goal, wait=wait)

        self.hand_grp.stop()

        current_joints = self.hand_grp.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)


  def close_hand(self, wait=True):
        joint_goal = self.hand_grp.get_current_joint_values()
        joint_goal[0] = 0.0001

        self.hand_grp.set_goal_joint_tolerance(0.001)
        self.hand_grp.go(joint_goal, wait=wait)

        if (not wait ):
          return

        self.hand_grp.stop()
        current_joints = self.hand_grp.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

  def grasp2(self, width, force):
        ''' width, epsilon_inner, epsilon_outer, speed, force '''
        grasp_goal = GripperCommandGoal()
        grasp_goal.command.position = float(width)
        grasp_goal.command.max_effort = float(force)
        grasp_msg = GripperCommandActionGoal(goal=grasp_goal)
        self.generic_grasp_client.publish(grasp_msg)

  def grasp(self, force, width, speed=20.0, epsilon_inner=0.002, epsilon_outer=0.002):
        ''' width, epsilon_inner, epsilon_outer, speed, force '''
        print("""
            Grasping with
            {}
            {}
        """.format(force, width))
        self.grasp_client.send_goal(
            franka_gripper.msg.GraspGoal(
                width,
                franka_gripper.msg.GraspEpsilon(
                    epsilon_inner,
                    epsilon_outer
                ),
                speed,
                force
            )
        )
        self.grasp_client.wait_for_result()
        return self.grasp_client.get_result()

def main():

  try:
    print "============ Press `Enter` to begin the Commander by setting up the moveit_commander (press ctrl-d to exit) ..."
    # raw_input()
    Commander = MoveIt_Commander()

    print "============ Press `Enter` to test tf"
    # raw_input()
    Commander.get_current_pose()




    print "============ Press `Enter` to execute home movement ..."
    raw_input()
    Commander.go_to_joint_state((0.00018762090386758654, -0.7771156373465307, 0.0005407026658047056, -2.365469873129632, -0.00020200796294576732, 1.5704722326730955, 0.7845368039521746))
    cartesian_plan, fraction = Commander.plan_linear_z(0.2)
    Commander.execute_plan(cartesian_plan)
    cartesian_plan, fraction = Commander.plan_linear_z(-0.2)
    Commander.execute_plan(cartesian_plan)
    Commander.close_hand()
    Commander.open_hand()

    rospy.spin()
    # print "============ Press `Enter` to find the hover pose"
    # raw_input()
    # Commander.request_detection()
    # print "============ Press `Enter` to reach the hover pose"
    # raw_input()
    # if Commander.detections[0]!= -1 or Commander.detections[1]!= -1:
    #     Commander.reach_hover()
    #

    '''
    print "============ Press `Enter` to find grasp"
    raw_input()
    # #
    Commander.request_detection()
    if Commander.detections[2]!= 9999:
        Commander.reach_hover()
        Commander.fix_angle(Commander.detections[2])



    Commander.request_detection()
    if Commander.detections[0]!= 9999 or Commander.detections[1]!= 9999:
        Commander.reach_grasp_hover_kps()
        Commander.fix_angle(Commander.detections[2])
        cartesian_plan, fraction = Commander.plan_linear_z(-0.061)
        Commander.execute_plan(cartesian_plan)
        Commander.grasp(50,0.039)
    #epsilon_inner=0.002, epsilon_outer=0.002
    print("enter to lift")
    raw_input()
    cartesian_plan, fraction = Commander.plan_linear_z(0.1)
    Commander.execute_plan(cartesian_plan)
    cartesian_plan, fraction = Commander.plan_linear_z(-0.1)
    Commander.execute_plan(cartesian_plan)
    Commander.open_hand()
    cartesian_plan, fraction = Commander.plan_linear_z(0.1)
    Commander.execute_plan(cartesian_plan)
    '''
# #
    print "============ Python Commander demo complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
    rospy.init_node('move_group_python_commander',
                    anonymous=True)

    listener_tf = tf.TransformListener()


    main()
