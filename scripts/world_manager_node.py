#!/usr/bin/env python
import sys

import rospy
import moveit_commander

import geometry_msgs.msg
import world_manager.srv

from moveit_commander import PlanningSceneInterface
from model_manager import ModelPoseBroadcaster


class WorldManager:
    def __init__(self):

        moveit_commander.roscpp_initialize(sys.argv)

        self.planning_scene_topic = rospy.get_param("planning_scene_topic")

        self.scene = PlanningSceneInterface()

        self.model_pose_broadcaster = ModelPoseBroadcaster()

        self.clear_planning_scene_service = rospy.Service(
            "/world_manager/clear_planning_scene",
            world_manager.srv.ClearObjects,
            self.remove_all_objects_from_planner)
        self.add_mesh_to_planning_scene_service = rospy.Service(
            "/world_manager/add_object", world_manager.srv.AddObject,
            self.add_object_to_planning_scene)
        self.get_objects_from_planning_scene_service = rospy.Service(
            "/world_manager/get_objects", world_manager.srv.GetObjects,
            self.get_objects_from_planning_scene)

        rospy.sleep(1.0)
        self.add_walls()
        rospy.loginfo("World Manager Node is Up and Running")

    def add_object_to_planning_scene(self, request):
        # self.remove_all_objects_from_planner(None)
        # this makes sure tf is continually broadcast
        scene_object = world_manager.msg.SceneObject(request.objectname,
                                                            request.mesh_filepath,
                                                            request.pose_stamped)
        self.model_pose_broadcaster.add_model(scene_object)

        # remove the old completion if it is there
        self.scene.remove_world_object(request.objectname)

        # add the new object to the planning scene
        self.scene.add_mesh(request.objectname, request.pose_stamped,
                            request.mesh_filepath)
        return []

    def get_objects_from_planning_scene(self, request):

        response = world_manager.srv.GetObjectsResponse(self.model_pose_broadcaster._model_list)

        return response

    def get_body_names_from_planner(self):
        rospy.wait_for_service(self.planning_scene_topic, 5)

        body_names = self.scene.get_known_object_names()

        return body_names

    def remove_all_objects_from_planner(self, request):

        body_names = self.get_body_names_from_planner()

        while len(body_names) > 0:
            print(
                "removing bodies from the planner, this can potentially take several tries"
            )
            for body_name in body_names:
                self.scene.remove_world_object(body_name)

            body_names = self.get_body_names_from_planner()

        self.model_pose_broadcaster.clear_models()

        self.add_walls()
        return []

    def add_walls(self):
        walls = rospy.get_param('/walls')
        for wall_params in walls:
            rospy.loginfo("Adding wall " + str(wall_params))
            self.add_wall(wall_params)

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
        wall_dimensions = [x_thickness, y_thickness, z_thickness]

        back_wall_pose.pose.position = geometry_msgs.msg.Point(
            **{'x': x,
               'y': y,
               'z': z})
        back_wall_pose.pose.orientation = geometry_msgs.msg.Quaternion(
            **{'x': qx,
               'y': qy,
               'z': qz,
               'w': qw})
        self.scene.add_box(name, back_wall_pose, wall_dimensions)


if __name__ == '__main__':

    try:

        rospy.sleep(3.0)
        rospy.init_node('world_manager_node')

        rospy.loginfo("Starting Up World Manager")
        world_manager_ = WorldManager()

        rospy.loginfo("World Manager Has Started")

        loop = rospy.Rate(30)
        while not rospy.is_shutdown():
            world_manager_.model_pose_broadcaster.broadcast_object_tfs()
            loop.sleep()

    except rospy.ROSInterruptException:
        pass
