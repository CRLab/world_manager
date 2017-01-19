#!/usr/bin/env python
import os
import time
import sys
import ipdb

import roslib
import rospy
import moveit_commander
import tf
import actionlib
import tf.transformations

import geometry_msgs.msg
from std_srvs.srv import Empty
import moveit_msgs
from moveit_msgs.msg import PlanningSceneComponents
import moveit_msgs.srv
from graspit_msgs.srv import *
import graspit_msgs.msg


from moveit_commander import PlanningSceneInterface
from model_rec_manager import ModelManager, ModelRecManager
from object_filename_dict import file_name_dict




#these imports are specifically for interactive markers
import rospy
import copy
from math import sin, cos

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point
from tf.broadcaster import TransformBroadcaster

from random import random
from math import sin


from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal
from grasping_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import IPython
server = None
menu_handler = MenuHandler()
br = None
counter = 0


#Move the base
def goto(client, x, y, theta, frame="map"):
    move_goal = MoveBaseGoal()
    move_goal.target_pose.pose.position.x = x
    move_goal.target_pose.pose.position.y = y
    move_goal.target_pose.pose.orientation.z = sin(theta/2.0)
    move_goal.target_pose.pose.orientation.w = cos(theta/2.0)
    move_goal.target_pose.header.frame_id = frame
    move_goal.target_pose.header.stamp = rospy.Time.now()
    # TODO wait for things to work
    client.send_goal(move_goal)
    client.wait_for_result()
#Follow the trajectory
def move_to(client, joint_names, positions, duration=5.0):
    if len(joint_names) != len(positions):
        print("Invalid trajectory position")
        return False
    trajectory = JointTrajectory()
    trajectory.joint_names = joint_names
    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[0].positions = positions
    trajectory.points[0].velocities = [0.0 for _ in positions]
    trajectory.points[0].accelerations = [0.0 for _ in positions]
    trajectory.points[0].time_from_start = rospy.Duration(duration)
    follow_goal = FollowJointTrajectoryGoal()
    follow_goal.trajectory = trajectory

    client.send_goal(follow_goal)
    client.wait_for_result()
#Point head on detected object
def look_at(client, x, y, z, frame, duration=1.0):
    goal = PointHeadGoal()
    goal.target.header.stamp = rospy.Time.now()
    goal.target.header.frame_id = frame
    goal.target.point.x = x
    goal.target.point.y = y
    goal.target.point.z = z
    goal.min_duration = rospy.Duration(duration)
    client.send_goal(goal)
    client.wait_for_result()
#Tuck the arm
def tuck(self):
    joints = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
              "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
    pose = [1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]
    while not rospy.is_shutdown():
        result = self.move_group.moveToJointPosition(joints, pose, 0.02)
def processFeedback( feedback ):
    s = "Feedback from marker '" + feedback.marker_name
    s += "' / control '" + feedback.control_name + "'"

    mp = ""
    if feedback.mouse_point_valid:
        mp = " at " + str(feedback.mouse_point.x)
        mp += ", " + str(feedback.mouse_point.y)
        mp += ", " + str(feedback.mouse_point.z)
        mp += " in frame " + feedback.header.frame_id

    if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
        rospy.loginfo( s + ": button click" + mp + "." )
    elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
        rospy.loginfo( s + ": menu item " + str(feedback.menu_entry_id) + " clicked" + mp + "." )
        # GO There!
        if feedback.menu_entry_id == 1:
            client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
            rospy.loginfo("Waiting for move_base...")
            client.wait_for_server()
            listener = tf.TransformListener()
            # IPython.embed()
            wake_up = False
            while !wake_up:
                rospy.loginfo("Retrying here:")
                try:
                    (trans, rot) = listener.lookupTransform('/map', '/world', rospy.Time(0))
                    wake_up = True
                    x = feedback.pose.position.x + trans[0]
                    y = feedback.pose.position.y + trans[1]
                    server.applyChanges()
                    goto(client,x, y, 1.57)
                except Exception:
                    rospy.loginfo("Some error happened, retrying...")
        #Up!
        if feedback.menu_entry_id == 3:
            name = "torso_controller"
            joint_names = ["torso_lift_joint"]
            client = actionlib.SimpleActionClient("%s/follow_joint_trajectory" % name,
                                                   FollowJointTrajectoryAction)
            rospy.loginfo("Waiting for %s..." % name)
            client.wait_for_server()
            rospy.loginfo("Raising torso...")
            move_to(client, joint_names, [0.1, ])
        #Down!
        if feedback.menu_entry_id == 4:
            name = "torso_controller"
            joint_names = ["torso_lift_joint"]
            client = actionlib.SimpleActionClient("%s/follow_joint_trajectory" % name,
                                                   FollowJointTrajectoryAction)
            rospy.loginfo("Waiting for %s..." % name)
            client.wait_for_server()
            rospy.loginfo("Lowering torso...")
            move_to(client, joint_names, [0.4, ])
        #Focus on Object!
        if feedback.menu_entry_id == 5:
            client = actionlib.SimpleActionClient("head_controller/point_head", PointHeadAction)
            rospy.loginfo("Waiting for head_controller...")
            client.wait_for_server()
            rospy.loginfo("Looking at the object...")
            look_at(client, feedback.pose.position.x, feedback.pose.position.y, feedback.pose.position.z, "world")
        #Tuck Arm!
        #TIPS: Raise the torso before you tuck the arm of the robot
        if feedback.menu_entry_id == 6:
            tuck()
        if feedback.menu_entry_id == 7:
            rospy.info("This marker's position is... \t", feedback.pose.position.x, feedback.pose.position.y, feedback.pose.position.z)
            rospy.info("This robot's position is... \t", feedback.pose.position.x, feedback.pose.position.y, feedback.pose.position.z)
    elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        rospy.loginfo( s + ": pose changed")
# TODO
#          << "\nposition = "
#          << feedback.pose.position.x
#          << ", " << feedback.pose.position.y
#          << ", " << feedback.pose.position.z
#          << "\norientation = "
#          << feedback.pose.orientation.w
#          << ", " << feedback.pose.orientation.x
#          << ", " << feedback.pose.orientation.y
#          << ", " << feedback.pose.orientation.z
#          << "\nframe: " << feedback.header.frame_id
#          << " time: " << feedback.header.stamp.sec << "sec, "
#          << feedback.header.stamp.nsec << " nsec" )
    elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
        rospy.loginfo( s + ": mouse down" + mp + "." )
    elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
        rospy.loginfo( s + ": mouse up" + mp + "." )
    server.applyChanges()

def alignMarker( feedback ):
    pose = feedback.pose

    pose.position.x = round(pose.position.x-0.5)+0.5
    pose.position.y = round(pose.position.y-0.5)+0.5

    rospy.loginfo( feedback.marker_name + ": aligning position = " + str(feedback.pose.position.x) + "," + str(feedback.pose.position.y) + "," + str(feedback.pose.position.z) + " to " +
                                                                     str(pose.position.x) + "," + str(pose.position.y) + "," + str(pose.position.z) )

    server.setPose( feedback.marker_name, pose )
    server.applyChanges()

def rand( min_, max_ ):
    return min_ + random()*(max_-min_)

def makeBox( msg ):
    marker = Marker()

    marker.type = Marker.CUBE
    marker.scale.x = msg.scale * 0.45
    marker.scale.y = msg.scale * 0.45
    marker.scale.z = msg.scale * 0.45
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0

    return marker

def makeBoxControl( msg ):
    control =  InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append( makeBox(msg) )
    msg.controls.append( control )
    return control

def saveMarker( int_marker ):
  server.insert(int_marker, processFeedback)


#####################################################################
# Marker Creation

def make6DofMarker( fixed, frame, interaction_mode, position, show_6dof = False):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = frame
    int_marker.pose.position = position
    int_marker.scale = 1

    int_marker.name = "simple_6dof"
    int_marker.description = "Simple 6-DOF Control"

    # insert a box
    makeBoxControl(int_marker)
    int_marker.controls[0].interaction_mode = interaction_mode

    if fixed:
        int_marker.name += "_fixed"
        int_marker.description += "\n(fixed orientation)"

    if interaction_mode != InteractiveMarkerControl.NONE:
        control_modes_dict = { 
                          InteractiveMarkerControl.MOVE_3D : "MOVE_3D",
                          InteractiveMarkerControl.ROTATE_3D : "ROTATE_3D",
                          InteractiveMarkerControl.MOVE_ROTATE_3D : "MOVE_ROTATE_3D" }
        int_marker.name += "_" + control_modes_dict[interaction_mode]
        int_marker.description = "3D Control"
        if show_6dof: 
          int_marker.description += " + 6-DOF controls"
        int_marker.description += "\n" + control_modes_dict[interaction_mode]
    
    if show_6dof: 
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

    server.insert(int_marker, processFeedback)
    menu_handler.apply( server, int_marker.name )




class WorldManager:
    def __init__(self, server):

        self.server = server
        moveit_commander.roscpp_initialize(sys.argv)

        self.planning_scene_topic = rospy.get_param("planning_scene_topic")
        self.run_recognition_topic = rospy.get_param("run_recognition_topic")
        self.detected_model_frame_id=rospy.get_param("detected_model_frame_id")

        self.scene = PlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()

        self.model_manager = ModelRecManager()

        self.planning_scene_service_proxy = rospy.ServiceProxy(self.planning_scene_topic, moveit_msgs.srv.GetPlanningScene)

        self._run_recognition_as = actionlib.SimpleActionServer(self.run_recognition_topic,
                                                                graspit_msgs.msg.RunObjectRecognitionAction,
                                                                execute_cb=self._run_recognition_as_cb,
                                                                auto_start=False)
        self._run_recognition_as.start()
        rospy.loginfo("World Manager Node is Up and Running")

    def _run_recognition_as_cb(self, goal):
        print("_run_recognition_as_cb")

        print("about to remove_all_objects_from_planner()")
        self.remove_all_objects_from_planner()
        print("finished remove_all_objects_from_planner()")

        self.model_manager.refresh()

        print("about to add_all_objects_to_planner()")
        self.add_all_objects_to_planner()
        print("finished add_all_objects_to_planner()")

        _result = graspit_msgs.msg.RunObjectRecognitionResult()
        print("graspit_msgs.msg.RunObjectRecognitionResult()")

        for model in self.model_manager.model_list:
            object_info = graspit_msgs.msg.ObjectInfo(model.object_name, model.model_name, model.get_world_pose())
            _result.object_info.append(object_info)
            position = model.get_world_pose().position
            position = Point(position.x, position.y, position.z)
            make6DofMarker(fixed=False, 
                frame=model.model_name,
                interaction_mode=InteractiveMarkerControl.MOVE_ROTATE_3D,
                position=position, 
                show_6dof = True)
        
            
        print("finished for loop")
        server.applyChanges()
        

        self._run_recognition_as.set_succeeded(_result)
        return []

    def get_body_names_from_planner(self):
        rospy.wait_for_service(self.planning_scene_topic, 5)
        components = PlanningSceneComponents(
            PlanningSceneComponents.WORLD_OBJECT_NAMES + PlanningSceneComponents.TRANSFORMS)

        ps_request = moveit_msgs.srv.GetPlanningSceneRequest(components=components)
        ps_response = self.planning_scene_service_proxy.call(ps_request)

        body_names = [co.id for co in ps_response.scene.world.collision_objects]

        return body_names

    def remove_all_objects_from_planner(self):

        body_names = self.get_body_names_from_planner()

        while len(body_names) > 0:
            print("removing bodies from the planner, this can potentially take several tries")
            for body_name in body_names:
                self.scene.remove_world_object(body_name)

            body_names = self.get_body_names_from_planner()

    def add_all_objects_to_planner(self):
        self.add_obstacles()
        for model in self.model_manager.model_list:
            model_name = model.model_name.strip('/')
            print "Adding " + str(model_name) + "To Moveit"
            filename = file_name_dict[model_name]
            if os.path.isfile(filename):

                stamped_model_pose = geometry_msgs.msg.PoseStamped()
                stamped_model_pose.header.frame_id = self.detected_model_frame_id
                stamped_model_pose.pose = model.get_world_pose()

                self.scene.add_mesh(model.object_name, stamped_model_pose, filename)

            else:
                rospy.logwarn('File doesn\'t exist - object %s, filename %s' % (model.object_name, filename))

    def add_walls(self):
        walls = rospy.get_param('/walls')
        for wall_params in walls:
            rospy.loginfo("Adding wall " + str(wall_params))
            self.add_wall(wall_params)
        return

    def add_wall(self, wall_params):
        name = wall_params["name"]
        x_thickness = wall_params["x_thickness"]
        y_thickness = wall_params["y_thickness"] 
        z_thickness = wall_params["z_thickness"]
        x = wall_params["x"]
        y = wall_params["y"]
        z = wall_params["z"]
        qx = wall_params["qx"]
        qy = wall_params["qy"]
        qz = wall_params["qz"]
        qw = wall_params["qw"]
        frame_id = wall_params["frame_id"]

        back_wall_pose = geometry_msgs.msg.PoseStamped()
        back_wall_pose.header.frame_id = '/' + frame_id
        wall_dimensions = [
            x_thickness, 
            y_thickness,  
            z_thickness
        ]

        back_wall_pose.pose.position = geometry_msgs.msg.Point(**{'x': x, 'y': y, 'z': z})
        back_wall_pose.pose.orientation = geometry_msgs.msg.Quaternion(**{'x': qx,
                                                                          'y': qy,
                                                                          'z': qz,
                                                                          'w': qw})
        self.scene.add_box(name, back_wall_pose, wall_dimensions)

        return

    def add_obstacles(self):
        self.add_walls()

if __name__ == '__main__':

    try:
        rospy.init_node('basic_controls')

        server = InteractiveMarkerServer("basic_controls")
        world_manager = WorldManager(server)
        world_manager.add_obstacles()
        loop = rospy.Rate(30)

        client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base...")
        client.wait_for_server()
        rospy.loginfo("The initial postion of the robot is [3, 2,  0]")
        goto(client, 3, 2, 0)

        menu_handler.insert( "Go There!", callback=processFeedback )
        menu_handler.insert( "Grasp It!", callback=processFeedback )
        menu_handler.insert( "Up!", callback=processFeedback )
        menu_handler.insert( "Down!", callback=processFeedback )
        menu_handler.insert( "Focus on Object!", callback=processFeedback )
        menu_handler.insert( "Tuck Arm!", callback=processFeedback )
        menu_handler.insert( "Marker Position!", callback=processFeedback )
        menu_handler.insert( "Cancel", callback=processFeedback )

        position = Point(-3, 3, 0)
        make6DofMarker( False, "base_link", InteractiveMarkerControl.NONE, position, True)

        server.applyChanges()
        

        

        while not rospy.is_shutdown():
            world_manager.model_manager.rebroadcast_object_tfs()
            loop.sleep()

    except rospy.ROSInterruptException:
        pass
