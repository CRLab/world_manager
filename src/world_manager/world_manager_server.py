#!/usr/bin/env python
import sys

import rospy
import moveit_commander
import geometry_msgs.msg
import world_manager.srv
import std_srvs.srv

from moveit_commander import PlanningSceneInterface
from tf_manager import TFManager


class WorldManagerServer:

    def __init__(self):

        moveit_commander.roscpp_initialize(sys.argv)

        self.planning_scene_topic = rospy.get_param("planning_scene_topic")
        self.scene = PlanningSceneInterface()

        self.tf_manager = TFManager()

        self.clear_planning_scene_service = rospy.Service(
            "/world_manager/clear_objects",
            std_srvs.srv.Empty,
            self.clear_objects_cb)

        self.add_mesh = rospy.Service(
            "/world_manager/add_mesh", world_manager.srv.AddMesh,
            self.add_mesh_cb)

        self.add_tf_service = rospy.Service(
            "/world_manager/add_tf", world_manager.srv.AddTF,
            self.add_tf_cb)

        self.add_walls_service = rospy.Service(
            "/world_manager/add_walls", std_srvs.srv.Empty,
            self.add_walls_cb)

        rospy.sleep(1.0)
        self.clear_objects_cb(request=None)
        self.add_walls_cb(request=None)
        rospy.loginfo("World Manager Node is Up and Running")

    def add_mesh_cb(self, request):
        so = request.scene_object

        # add the tf
        self.tf_manager.add_tf(so)

        # remove the old completion if it is there
        self.scene.remove_world_object(so.objectname)

        # add the new object to the planning scene
        self.scene.add_mesh(so.object_name, so.pose_stamped,
                            so.mesh_filepath)
        return []

    def add_tf_cb(self, request):
        self.tf_manager.add_tf(request.frame_name, request.pose_stamped)
        return []

    def clear_objects_cb(self, request):

        rospy.wait_for_service(self.planning_scene_topic, 5)
        body_names = self.scene.get_known_object_names()

        for body_name in body_names:
            self.scene.remove_world_object(body_name)

        self.tf_manager.clear_tfs()

        return []

    def add_walls_cb(self, request):
        walls = rospy.get_param('/walls')
        for wall_params in walls:
            rospy.loginfo("Adding wall " + str(wall_params))
            self._add_wall(wall_params)

        return []

    def _add_wall(self, wall_params):
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
        world_manager_ = WorldManagerServer()

        rospy.loginfo("World Manager Has Started")

        loop = rospy.Rate(30)
        while not rospy.is_shutdown():
            world_manager_.tf_manager.broadcast_tfs()
            loop.sleep()

    except rospy.ROSInterruptException:
        pass
