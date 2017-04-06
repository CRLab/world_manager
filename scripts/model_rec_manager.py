"""
@package - model_rec_manager
@brief - calls model_rec and then manages the resulting models. Broadcast the pointclouds and TF
"""

import roslib;
import rospy
from numpy import dot,  linalg
from numpy import mat
import numpy as np
import sensor_msgs, sensor_msgs.msg
import graspit_msgs.srv
import tf
import tf.transformations
import tf_conversions.posemath as pm




'''
import objrec_ros_integration, objrec_ros_integration.srv
import sensor_msgs, sensor_msgs.msg
import graspit_msgs.srv

import StringIO
import yaml

import rospy

import tf
import tf_conversions.posemath as pm
from collections import namedtuple
DetectedModel = namedtuple('DetectedModel', ['name', 'poseStamped'], verbose=False)

class ModelPoseBroadcaster(object):

    def __init__(self):

        self._model_list = list()
        self._tf_broadcaster = tf.TransformBroadcaster()

    def add_model(self, model_name, pose):
        self._model_list.append(DetectedModel(model_name, pose))

    def broadcast_object_tfs(self):
        for model in self._model_list:
            tf_pose = pm.toTf(pm.fromMsg(model.poseStamped.pose))
            self._tf_broadcaster.sendTransform(tf_pose[0], tf_pose[1], rospy.Time.now(), model.name, model.poseStamped.header.frame_id)

    def clear_models(self):
        self._model_list = []

'''

class ModelManager(object):
    def __init__(self, model_name, pose_in_camera_frame):
        self.model_name = model_name
        self.object_name = model_name
        #TODO 1
        #this is current camera to object
        #we need to use tf listener to get camera to world. then use it to get this pose in world frame 
        #rather than camera. 
        #1. camera
        self.listener = ModelRecManager.tf_listener
        self.table_frame = rospy.get_param("table_id")
        self.camera_frame = rospy.get_param("frame_id")
        self.listener.waitForTransform(self.camera_frame, self.table_frame, rospy.Time(0),rospy.Duration(10))
        (camera_to_table_tran, camera_to_table_rot) = self.listener.lookupTransform(self.camera_frame, self.table_frame,rospy.Time(0))
        tr = tf.TransformerROS()
        camera_to_table_matrix = tr.fromTranslationRotation(camera_to_table_tran, camera_to_table_rot)


        # self.listener.waitForTransform(self.camera_frame, self.object_name, rospy.Time(0), rospy.Duration(10))
        # (trans, rot) = self.listener.lookupTransform(self.camera_frame, self.object_name, rospy.Time(0))
        # pose_in_camera_matrix = tr.fromTranslationRotation(trans, rot)
        pose_in_camera_pykdl = pm.fromMsg(pose_in_camera_frame)
        pose_in_camera_matrix = pm.toMatrix(pose_in_camera_pykdl)
        pose_in_table_matrix = np.dot(np.linalg.inv(camera_to_table_matrix), pose_in_camera_matrix)
        pose_in_table_msg = pm.toMsg(pm.fromMatrix(pose_in_table_matrix))

        self.pose_in_table_frame_msg = pose_in_table_msg
        self.bc = ModelRecManager.tf_broadcaster
        

    

    def broadcast_tf(self):
        tf_pose = pm.toTf(pm.fromMsg(self.pose_in_table_frame_msg))
        self.bc.sendTransform(tf_pose[0], tf_pose[1], rospy.Time.now(), self.object_name, self.table_frame)

    def get_dist(self):
        self.broadcast_tf()
        self.listener.waitForTransform(self.camera_frame, self.object_name, rospy.Time(0), rospy.Duration(10))
        (trans, rot) = self.listener.lookupTransform(self.camera_frame, self.object_name, rospy.Time(0))
        return linalg.norm(trans)

    def __len__(self):
        return self.get_dist()

    def get_world_pose(self):
        self.broadcast_tf()
        self.listener.waitForTransform("/world", self.object_name, rospy.Time(0),rospy.Duration(10))
        return pm.toMsg(pm.fromTf(self.listener.lookupTransform("/world", self.object_name, rospy.Time(0))))
        
    # def get_base_pose(self):
    #     self.broadcast_tf()
    #     self.listener.waitForTransform("/base_link", self.object_name, rospy.Time(0),rospy.Duration(10))
    #     base_to_object = pm.toMsg(pm.fromTf(self.listener.lookupTransform("/base_link", self.object_name, rospy.Time(0))))
    #     object_base_data = (base_to_object.position.x, base_to_object.position.y, base_to_object.position.z)
    #     object_base_quart = (base_to_object.orientation.x, base_to_object.orientation.y, base_to_object.orientation.z, base_to_object.orientation.w)
    #     object_base_euler = tf.transformations.euler_from_quaternion(object_base_quart)
    #     object_base_data += (object_base_euler)
    #     np.savetxt("/home/bo/Desktop/reachability_exp/data/obj_base_data.csv", np.array(object_base_data).reshape((1,6)),  '%g', ' ')

    def get_base_pose(self):
        # import IPython
        # IPython.embed()
        self.broadcast_tf()
        self.listener.waitForTransform("/base_link", self.object_name, rospy.Time(0),rospy.Duration(10))
        (base_to_object_tran, base_to_object_rot) = self.listener.lookupTransform("/base_link", self.object_name, rospy.Time(0))
        tr = tf.TransformerROS()
        base_to_object_matrix = tr.fromTranslationRotation(base_to_object_tran, base_to_object_rot)

        np.savetxt("/home/bo/Desktop/reachability_exp/data/obj_base_dataTRANS.csv", np.array(base_to_object_matrix),  '%g', ' ')

        # import IPython
        # IPython.embed()s



        base_to_object = pm.toMsg(pm.fromTf(self.listener.lookupTransform("/base_link", self.object_name, rospy.Time(0))))
        # object_base_data = (base_to_object.position.x, base_to_object.position.y, base_to_object.position.z)
        # object_base_quart = (base_to_object.orientation.x, base_to_object.orientation.y, base_to_object.orientation.z, base_to_object.orientation.w)
        # object_base_euler = tf.transformations.euler_from_quaternion(object_base_quart)
        # object_base_data += (object_base_euler)
        import IPython
        IPython.embed()
        np.savetxt("/home/bo/Desktop/reachability_exp/data/obj_base_data_pose.csv", np.array(base_to_object).reshape((1,6)),  '%g', ' ')
        base_to_object_pose_stamped = geometry_msgs.msg.PoseStamped()
        base_to_object_pose_stamped.pose = base_to_object
        base_to_object_pose_stamped.header.frame_id = "base_link"
        with open('base_to_object.yaml', 'w') as outfile:
            yaml.dump(base_to_object_pose_stamped, outfile, default_flow_style=False)


class ModelRecManager(object):

    def __init__(self):
        self.__publish_target = True
        self.model_list = list()

        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()

        ModelRecManager.tf_listener = self.tf_listener
        ModelRecManager.tf_broadcaster = self.tf_broadcaster
        self.model_name_server = rospy.Service('/get_object_info', graspit_msgs.srv.GetObjectInfo, self.get_object_info)

    def refresh(self):
        #clear out old models
        self.model_list = list()

        find_objects_srv = rospy.ServiceProxy('/objrec_node/find_objects', objrec_ros_integration.srv.FindObjects)


        resp = find_objects_srv()

        for i in range(len(resp.object_name)):
            rospy.loginfo("Adding ModelManager for object" + str(resp.object_name[i]) )
            rospy.loginfo("Pose in Camera Frame: " + str(resp.object_pose[i]))
            self.model_list.append(ModelManager(resp.object_name[i],
                                                resp.object_pose[i]))
        self.uniquify_object_names()

        for model in self.model_list:
            model.model_name = model.model_name

    def rebroadcast_object_tfs(self):
        for model in self.model_list:
            model.broadcast_tf()

    def get_model_names(self):
        return [model.model_name for model in self.model_list]

    def get_object_info(self, req):
        resp = graspit_msgs.srv.GetObjectInfoResponse()
        for model in self.model_list:
            model_name = model.model_name
            object_name = model.object_name
            object_pose = model.get_world_pose()
            object_innn = model.get_base_pose()
            object_info = graspit_msgs.msg.ObjectInfo(object_name, model_name, object_pose)
            resp.object_info.append(object_info)
        return resp

    def uniquify_object_names(self):
        object_name_dict = {}
        for model in self.model_list:
            if model.object_name in object_name_dict:
                object_name_dict[model.object_name].append(model)
            else:
                object_name_dict[model.object_name] = [model]

        model_names = dict(object_name_dict)

        for model_list in object_name_dict.values():
            if len(model_list) > 1:
                for model_num, model in enumerate(model_list):
                    test_name = model.object_name
                    while test_name in model_names:
                        test_name = "%s_%i" % (model.object_name, model_num)
                        model_num += 1
                    model.object_name = test_name
                    model_names[test_name] = model

