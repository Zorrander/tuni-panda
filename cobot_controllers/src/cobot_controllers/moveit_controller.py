from tf.transformations import *
import tf
import time
import math 
from cobot_msgs.srv import *
import rospy
import copy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import RobotTrajectory 


class MoveitArm(object):

    def __init__(self, group):
        self.group = group
        self.gripper_release_srv = rospy.ServiceProxy('/move_gripper', MoveGripper)
        self.gripper_grasp_srv = rospy.ServiceProxy('/grasp', Grasp)

        self.arm_joints_SOCKET = [0.17604044847321085, -0.004486328642619283, -0.6682706695587933, -2.449662917020028, -0.01548445735954576, 2.6170337975819904, 1.8612127985672815]
        self.arm_joints_GRAB = [0.09552836457114863,   0.4582943016897168,   -0.49671672280629475,   -2.3144507059799997 ,   0.2909820540878507,   2.7186551869710285,   1.6897436378393556]
        self.arm_joints_PANEL = [0.40683915349204014, 0.05011774442954318, -0.5303004122417656, -2.5375107996756565, -0.014617423742530854, 2.6364270890547146, 2.229896400718366]
        self.arm_joints_INIT =  [0.3853664 , 0.49662741, -0.44943702, -2.27425249  , 0.06632516, 2.99395411,2.39860492]
        
        #self.arm_joints_INIT =  [ 0.50523354 , 0.46390782, -0.44010135 , -2.3131034  , 0.10804452  ,3.02839653 , 2.44997405]

    def read_bot_info(self):
        joints = self.group.get_current_joint_values()
        ee_pose = self.group.get_current_pose()
        ee_position = ee_pose.pose.position
        ee_orientation = ee_pose.pose.orientation
        return ([ee_position.x, ee_position.y, ee_position.z], [ee_orientation.x, ee_orientation.y, ee_orientation.z, ee_orientation.w], 
                [joints[0],joints[1],joints[2],joints[3],joints[4],joints[5],joints[6]])


    def move_to_reset_pose(self, reset_time):
        print(self.group.get_current_joint_values())
        #time.sleep(10000)
        # Release 
        print('release')
        self.gripper_release_srv(MoveGripperRequest(20.0, 0.08))

        # Move to socket's above loc 
        print('move to the top of socket')
        self.move_to_target(self.arm_joints_SOCKET)

        print('wait for manual reset')
        time.sleep(2)


        print('move to grab')
        # Go get the object
        #arm_joints = self.group.get_current_joint_values()
        self.move_to_target(self.arm_joints_GRAB)

        print('grab')
        req = GraspRequest()
        req.width = 0.045
        self.gripper_grasp_srv(req)

        # Move to socket's above loc again to avoid collision
        print('move to socket up')        
        self.move_to_target(self.arm_joints_SOCKET)

        print('move to the top of panel')
        self.move_to_target(self.arm_joints_PANEL)

        print("move_to_reset_pose")
        #arm_joints = self.group.get_current_joint_values()
        return self.move_to_target(self.arm_joints_INIT)


    def move_to_target_cartesian_controller(self, joint_values):
        return

    def rotate_ee(self, angle):
        try:
            self.group.set_max_velocity_scaling_factor(0.2)

            wpose = self.group.get_current_pose().pose

            roll, pitch, yaw = tf.transformations.euler_from_quaternion([wpose.orientation.x, wpose.orientation.y, wpose.orientation.z, wpose.orientation.w])
           
            print("Original yaw - {}".format(yaw))
            #if angle < 0: 
            #    angle = angle + math.pi/2
            
            yaw = angle + math.pi/4 
            print("Intermediate yaw - {}".format(yaw))
            print("comparison {}".format(-math.pi + math.pi/4))
            if yaw > math.pi/4:
                yaw = yaw - math.pi

            print("New yaw - {}".format(yaw))
            
            
            
            quat_rotcmd = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

            #q_tf = [quaternion[0], quaternion[1], quaternion[2], quaternion[3]]
            #q_rot = quaternion_from_euler(0, -math.pi, 2*math.pi)
            #quat = quaternion_multiply(q_rot, q_tf)

            wpose.orientation.x = quat_rotcmd[0]
            wpose.orientation.y = quat_rotcmd[1]
            wpose.orientation.z = quat_rotcmd[2]
            wpose.orientation.w = quat_rotcmd[3]

            self.group.set_pose_target(wpose)
            self.group.go(wait=True)
            self.group.stop()
            self.group.clear_pose_targets()
        except Exception as e:
            print("Error in rotate_ee")
            print(e)
        finally:
            ee_pose = self.group.get_current_pose()
            ee_position = ee_pose.pose.position
            ee_orientation = ee_pose.pose.orientation
            return ([ee_position.x, ee_position.y, ee_position.z], [ee_orientation.x, ee_orientation.y, ee_orientation.z, ee_orientation.w])

            
    # linear movemonet planning along z axis of the reference frame
    def plan_linear_z(self, dist, slow=False):
        try:
            self.group.set_max_velocity_scaling_factor(0.1)
            group = self.group
            waypoints = []
            wpose = group.get_current_pose().pose
            wpose.position.z = dist
            print("move z to ", wpose.position.z)
            waypoints.append(copy.deepcopy(wpose))
            (plan, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0.0)
            if slow:
                plan = self.modify_plan(plan)
            self.group.execute(plan)
            self.group.stop()
            self.group.clear_pose_targets()
        except Exception as e:
            print("Error in move_to_1D_cartesian_target")
            print(e)
        finally:
            ee_pose = self.group.get_current_pose()
            ee_position = ee_pose.pose.position
            ee_orientation = ee_pose.pose.orientation
            return ([ee_position.x, ee_position.y, ee_position.z], [ee_orientation.x, ee_orientation.y, ee_orientation.z, ee_orientation.w])
        
    def modify_plan(self, plan):
        new_plan = RobotTrajectory()
        new_plan.joint_trajectory.joint_names = plan.joint_trajectory.joint_names
        new_plan.joint_trajectory.header = plan.joint_trajectory.header
        speed_factor = 0.1
        for p in plan.joint_trajectory.points:
            new_p = JointTrajectoryPoint()
            new_p.time_from_start = p.time_from_start/speed_factor

            new_p.positions = p.positions

            for i in range(len(p.velocities)):
                new_p.velocities.append(p.velocities[i]*speed_factor)

            for i in range(len(p.accelerations)):
                new_p.accelerations.append(p.accelerations[i]*speed_factor)

            new_plan.joint_trajectory.points.append(new_p)
        return new_plan


    def move_to_2D_cartesian_target(self, pose, slow=False):
        try:
            self.group.set_max_velocity_scaling_factor(0.1) 
            waypoints = []

            next_point = self.group.get_current_pose().pose

            next_point.position.x = pose[0]
            next_point.position.y = pose[1]
            waypoints.append(copy.deepcopy(next_point))
            (plan, fraction) = self.group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold
            if slow:
                plan = self.modify_plan(plan)
            self.group.execute(plan, wait=True) 
            self.group.stop()
            self.group.clear_pose_targets()
        except Exception as e:
            print("Error in move_to_2D_cartesian_target")
            print(e)
        finally: 
            ee_pose = self.group.get_current_pose()
            ee_position = ee_pose.pose.position
            ee_orientation = ee_pose.pose.orientation
            return ([ee_position.x, ee_position.y, ee_position.z], [ee_orientation.x, ee_orientation.y, ee_orientation.z, ee_orientation.w])
            
    def move_to_target(self, joint_values):
        print("GOAL")
        print(list(joint_values))
        self.group.set_max_velocity_scaling_factor(0.8)
        # self.group.allow_replanning(True)
        self.group.go(list(joint_values), wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
        ee_pose = self.group.get_current_pose()
        # print("EE_POSE", ee_pose)
        ee_position = ee_pose.pose.position
        ee_orientation = ee_pose.pose.orientation
        return ([ee_position.x, ee_position.y, ee_position.z], [ee_orientation.x, ee_orientation.y, ee_orientation.z, ee_orientation.w])

    def stop(self):
        #try:
        self.group.stop()
        ee_pose = self.group.get_current_pose()
        # print("EE_POSE", ee_pose)
        ee_position = ee_pose.pose.position
        ee_orientation = ee_pose.pose.orientation
        return ([ee_position.x, ee_position.y, ee_position.z], [ee_orientation.x, ee_orientation.y, ee_orientation.z, ee_orientation.w])

    def move_to_cartesian_target(self, cartesian_values):
        """
        cartesian_values: 7 dimensional vecotr, [cartesian_position, cartesian_Quaternion]
                          [x, y, z, quat_x, quat_y, quat_z, quat_w]
        """
        print("CARTESIAN GOAL")
        print(cartesian_values)
        self.group.set_max_velocity_scaling_factor(0.2)
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = cartesian_values[-4]
        pose_goal.orientation.y = cartesian_values[-3]
        pose_goal.orientation.z = cartesian_values[-2]
        pose_goal.orientation.w = cartesian_values[-1]
        pose_goal.position.x = cartesian_values[0]
        pose_goal.position.y = cartesian_values[1]
        pose_goal.position.z = cartesian_values[2]
        self.group.set_pose_target(pose_goal)

        self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
        
        ee_pose = self.group.get_current_pose()
        # print("EE_POSE", ee_pose)
        ee_position = ee_pose.pose.position
        ee_orientation = ee_pose.pose.orientation
        return ([ee_position.x, ee_position.y, ee_position.z], [ee_orientation.x, ee_orientation.y, ee_orientation.z, ee_orientation.w])


    def handle_reset(self, req):
        print("handle_reset")
        ee_pose, ee_orientation = self.move_to_reset_pose(req.reset_time)
        return ResetEnvResponse(ee_pose, ee_orientation)

    def handle_move_to_target(self, req):
        ee_pose, ee_orientation = self.move_to_target(req.q)
        return TakeActionResponse(ee_pose, ee_orientation)

    def handle_move_to_cartesian_target(self, req):
        ee_pose, ee_orientation = self.move_to_cartesian_target(req.pose)
        return TakeCartesianActionResponse(ee_pose, ee_orientation)

    def handle_read_bot_info(self, req):
        ee_pose, ee_orientation, joints = self.read_bot_info()
        return ReadValuesResponse(ee_pose, ee_orientation, joints)


    def handle_stop(self, req):
        ee_pose, ee_orientation = self.stop()
        return StopActionResponse(ee_pose, ee_orientation)

    def handle_move_to_2D_cartesian_target(self, req):
        ee_pose, ee_orientation = self.move_to_2D_cartesian_target(req.pose, req.slow)
        return Take2DCartesianActionResponse(ee_pose, ee_orientation)

    def handle_move_to_1D_cartesian_target(self, req):
        ee_pose, ee_orientation = self.plan_linear_z(req.z_pose, req.slow)
        return Take1DCartesianActionResponse(ee_pose, ee_orientation)

    def handle_rotate_ee(self, req):
        ee_pose, ee_orientation = self.rotate_ee(req.angle)
        return RotateEEResponse(ee_pose, ee_orientation)