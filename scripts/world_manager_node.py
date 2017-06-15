#!/usr/bin/env python
import os
import time
import sys
import ipdb
import importlib

import roslib
import rospy
import moveit_commander
import graspit_commander
import tf
import actionlib
import tf.transformations
import sys

import geometry_msgs.msg
from std_srvs.srv import Empty
import moveit_msgs
from moveit_msgs.msg import PlanningSceneComponents
import moveit_msgs.srv
from graspit_msgs.srv import *
import graspit_msgs.msg
import moveit_msgs.msg
import control_msgs.msg
import scene_completion.msg


from moveit_commander import PlanningSceneInterface
from model_rec_manager import ModelManager, ModelRecManager
from object_filename_dict import file_name_dict

from reachability_analyzer.grasp_reachability_analyzer import GraspReachabilityAnalyzer


#these imports are specifically for interactive markers
import rospy
import copy
from math import sin, cos

from visualization_msgs.msg import *
from geometry_msgs.msg import Point
from tf.broadcaster import TransformBroadcaster

from random import random
from math import sin
import numpy
import math


from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal
from grasping_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String

import IPython
import mcubes
import numpy as np

from grasp_execution.robot_interface import RobotInterface
from grasp_execution import execution_stages
from grasp_execution import execution_pipeline

from grasp_execution.hand_managers import hand_manager
from grasp_execution.grasp_execution_node import GraspExecutionNode
from reachability_analyzer.grasp_analyzer_node import GraspAnalyzerNode
# from moveit_python import (MoveGroupInterface,
#                            PickPlaceInterface)
from moveit_python.geometry import rotate_pose_msg_by_euler_angles
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
import trajectory_msgs.msg

br = None
counter = 0





class WorldManager:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)

        self.planning_scene_topic = rospy.get_param("planning_scene_topic")
        self.run_recognition_topic = rospy.get_param("run_recognition_topic")
        self.detected_model_frame_id=rospy.get_param("detected_model_frame_id")

        self.scene = PlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()
        self.model_manager = ModelRecManager()

        self.planning_scene_service_proxy = rospy.ServiceProxy(self.planning_scene_topic, moveit_msgs.srv.GetPlanningScene)

        self.run_scene_completion_topic = rospy.get_param("run_recognition_topic")
        self._run_scene_completion_as = actionlib.SimpleActionServer(self.run_scene_completion_topic,
                                                        graspit_msgs.msg.RunObjectRecognitionAction,
                                                        execute_cb=self._run_scene_completion_as_cb,
                                                        auto_start=False)
        
        self.scene_completion_client = actionlib.SimpleActionClient("/scene_completion/SceneCompletion", scene_completion.msg.CompleteSceneAction)
        
        self.scene_completion_client.wait_for_server()

        self._run_scene_completion_as.start()
        

        rospy.loginfo("World Manager Node is Up and Running")

    def _run_scene_completion_as_cb(self, goal):
        rospy.loginfo("_run_scene_completion_as_cb")

        rospy.loginfo("about to remove_all_objects_from_planner()")
        self.remove_all_objects_from_planner()
        rospy.loginfo("finished remove_all_objects_from_planner()")
        # self.add_walls()

        rospy.loginfo("about to send goal")
        goal = scene_completion.msg.CompleteSceneGoal()
        self.scene_completion_client.send_goal(goal)
        rospy.loginfo("waiting for result")
        self.scene_completion_client.wait_for_result()

        rospy.loginfo("received result")
        result = self.scene_completion_client.get_result()

        rospy.loginfo("adding meshes to planning scene")
        mesh_count = 0
        for mesh, pose, in zip(result.meshes, result.poses):
            mesh_name = "mesh_" + str(mesh_count)
            m = ModelManager(mesh_name, pose.pose)
            m.pose_in_table_frame_msg = pose.pose
            

            vs = []

            for vert in mesh.vertices:
                vs.append((vert.x,vert.y, vert.z)) 

            vertices = np.array(vs)

            ts = []
            for tri in mesh.triangles:
                ts.append(tri.vertex_indices)
            triangles = np.array(ts)
            dae_export_dir = "/tmp/"
            filename = "tmp_mesh" + str(mesh_count) # TODO: random name generator
            dae_filepath = dae_export_dir + filename + ".dae"
            ply_filepath = "/home/bo/ros/grasp_ws/basestation_ws/src/interactive_marker_server/meshes/" + filename + ".ply"
            mcubes.export_mesh(vertices, triangles, dae_filepath, "model")
            import subprocess
        
            cmd_str = "meshlabserver " + "-i " + dae_filepath + " -o " + ply_filepath
            subprocess.call(cmd_str, shell=True)
            # TODO:
            # Find out ply file path and send it back
            m.mesh_path_dae = dae_filepath
            m.mesh_path_ply = ply_filepath
            self.model_manager.model_list.append(m)
            self.scene.add_mesh(mesh_name, pose, dae_filepath)
            mesh_count += 1


        _result = graspit_msgs.msg.RunObjectRecognitionResult()
        rospy.loginfo("graspit_msgs.msg.RunObjectRecognitionResult()")

        for model in self.model_manager.model_list:
            object_info = graspit_msgs.msg.ObjectInfo(model.object_name, model.model_name, model.get_world_pose(), model.mesh_path_dae, model.mesh_path_ply)
            # model.get_base_pose()
            _result.object_info.append(object_info)

        rospy.loginfo("finished for loop")
        # server.applyChanges()        

        self._run_scene_completion_as.set_succeeded(_result)
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
            rospy.loginfo("removing bodies from the planner, this can potentially take several tries")
            for body_name in body_names:
                self.scene.remove_world_object(body_name)

            body_names = self.get_body_names_from_planner()

    def add_all_objects_to_planner(self):
        self.add_obstacles()
        for model in self.model_manager.model_list:
            model_name = model.model_name.strip('/')
            rospy.loginfo( "Adding " + str(model_name) + "To Moveit")
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
        rospy.init_node("WorldManagerNode")
        world_manager = WorldManager()
        # world_manager.add_obstacles()
        loop = rospy.Rate(30)

        while not rospy.is_shutdown():
            world_manager.model_manager.rebroadcast_object_tfs()
            loop.sleep()

    except rospy.ROSInterruptException:
        pass
