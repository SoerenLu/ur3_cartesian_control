#!/usr/bin/python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()

group = moveit_commander.MoveGroupCommander("arm")

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)


print "============ Reference frame: %s" % group.get_planning_frame()

print "============ End effector: %s" % group.get_end_effector_link()

print "============ Robot Groups:"
print robot.get_group_names()

print "============ Printing robot state"
print robot.get_current_state()
print "============"

curr_pose = group.get_current_pose().pose

print "============ Generating plan 1"
pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.w = 1.0
pose_target.position.x = 0
pose_target.position.y = 0
pose_target.position.z = curr_pose.position.z - 0.01
group.set_pose_target(pose_target)

plan1 = group.plan()

group.go(wait=True)

#waypoints = []

#waypoints.append(group.get_current_pose().pose)

#newpose = geometry_msgs.msg.Pose()
#newpose.position.x = waypoints[0].position.z - 0.1 
#waypoints.append(copy.deepcopy(newpose))

#(plan2, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0.0)
#rospy.sleep(1)
#group.execute(plan1)

