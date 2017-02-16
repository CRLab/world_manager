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


from geometry_msgs.msg import Point
from tf.broadcaster import TransformBroadcaster

from random import random
from math import sin


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
        rospy.init_node('world_manager_node')

        world_manager = WorldManager(server)
        world_manager.add_obstacles()
        loop = rospy.Rate(30)

        while not rospy.is_shutdown():
            world_manager.model_manager.rebroadcast_object_tfs()
            loop.sleep()

    except rospy.ROSInterruptException:
        pass
