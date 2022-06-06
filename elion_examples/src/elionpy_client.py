from elion_examples.srv import *
import rospy
import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg
import numpy as np


def get_quaternion_from_euler(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return qx, qy, qz, qw


def create_collision(id, type, dimension, rpy, position):
    obj = moveit_msgs.msg.CollisionObject()
    obj.id = id
    obj.operation = obj.ADD
    obj1 = shape_msgs.msg.SolidPrimitive()
    obj1.type = type # based on http://docs.ros.org/en/lunar/api/shape_msgs/html/msg/SolidPrimitive.html
    obj1.dimensions = dimension
    pose = geometry_msgs.msg.Pose()
    qx, qy, qz, qw = get_quaternion_from_euler(rpy[0], rpy[1], rpy[2])
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    pose.position.x = position[0]
    pose.position.y = position[1]
    pose.position.z = position[2]
    obj.primitives.append(obj1)
    obj.primitive_poses.append(pose)
    return obj


rospy.wait_for_service('elionpy')
try:
    elion_planner = rospy.ServiceProxy('elionpy', elionpy)
except rospy.ServiceException as e:
    print("Service call failed: %s"%e)

request = moveit_msgs.msg.MotionPlanRequest()
goal = moveit_msgs.msg.RobotState()
collisions = []

request.planner_id = 'PRMstar'
request.allowed_planning_time = 60
request.start_state.joint_state.position = [
                                            1.7301680303369467,
                                            -0.7342165592762893,
                                            -0.5358506493073328,
                                            -2.214051132383283,
                                            -1.9148221683474542,
                                            1.8324940020482856,
                                            -1.588014538557859]
orientation_constraint = moveit_msgs.msg.OrientationConstraint()
orientation_constraint.header.frame_id = "panda_link0"
orientation_constraint.link_name = "panda_link8"
qx, qy, qz, qw = get_quaternion_from_euler(0.0, 1.57079632679, 0.0)
orientation_constraint.orientation.x = qx
orientation_constraint.orientation.y = qy
orientation_constraint.orientation.z = qz
orientation_constraint.orientation.w = qw
orientation_constraint.absolute_x_axis_tolerance = -1.0
orientation_constraint.absolute_y_axis_tolerance = 0.1
orientation_constraint.absolute_z_axis_tolerance = 0.1
request.path_constraints.orientation_constraints = [orientation_constraint]
request.path_constraints.name = "AngleAxis"
goal.joint_state.position = [
                            0.92502450356,
                            -1.7627825445,
                            -1.0122909662,
                            -2.5307274154,
                            -1.8849555922,
                            2.5656340004,
                            -2.5307274154]

collisions.append(create_collision('box', 1, [0.3, 0.0, 0.4], [0.0, 0.0, 0.0], [0.1, 0.1, 0.4]))
collisions.append(create_collision('floor_1', 1, [1.0, 0.0, -0.05], [0.0, 0.0, 0.0], [1.8, 2.0, 0.1]))
collisions.append(create_collision('floor_2', 1, [-1.0, 0.0, -0.05], [0.0, 0.0, 0.0], [1.8, 2.0, 0.1]))
collisions.append(create_collision('floor_3', 1, [0.0, 1.0, -0.05], [0.0, 0.0, 0.0], [2.0, 1.8, 0.1]))
collisions.append(create_collision('floor_4', 1, [0.0, -1.0, -0.05], [0.0, 0.0, 0.0], [2.0, 1.8, 0.1]))

response = elion_planner(request, goal, collisions)