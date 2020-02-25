      moveit_commander.roscpp_initialize(sys.argv)

      group_name = "panda_arm"
      self.robot = moveit_commander.RobotCommander()
      self.scene = moveit_commander.PlanningSceneInterface()
      self.group = moveit_commander.MoveGroupCommander(group_name)

      ## Add the table
      p = PoseStamped()
      p.header.frame_id = self.robot.get_planning_frame()
      p.pose.position.x = 0.
      p.pose.position.y = 0.
      p.pose.position.z = -.05
      self.scene.add_box("table", p, (3.0, 1.0, 0.1))

      ## Add the windows
      # RPY to convert: 90deg, 0, -90deg
      #q = quaternion_from_euler(1.5707, 0, -1.5707)
      p = PoseStamped()
      p.header.frame_id = self.robot.get_planning_frame()
      p.pose.position.x = -0.30
      p.pose.position.y = 0.
      p.pose.position.z = -.05
      p.pose.orientation.x = 0.0
      p.pose.orientation.y = 1.0
      p.pose.orientation.z = 0.0
      p.pose.orientation.w = 1.0
      self.scene.add_box("windows", p, (3.0, 1.0, 0.1))
