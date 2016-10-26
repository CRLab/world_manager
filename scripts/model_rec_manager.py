"""
@package - model_rec_manager
@brief - calls model_rec and then manages the resulting models. Broadcast the pointclouds and TF
"""

import roslib;
import rospy
from numpy import dot,  linalg
from numpy import mat

import tf
import tf.transformations
import tf_conversions.posemath as pm

import objrec_ros_integration, objrec_ros_integration.srv
# import model_rec2, model_rec2.srv
import sensor_msgs, sensor_msgs.msg
import graspit_msgs.srv

import StringIO


class ModelManager(object):
    def __init__(self, model_name, pose, NEW_MODEL_REC):
        self.NEW_MODEL_REC = NEW_MODEL_REC
        self.model_name = model_name
        self.object_name = model_name
        self.pose = pose
        self.bc = ModelRecManager.tf_broadcaster
        self.listener = ModelRecManager.tf_listener
        self.detected_frame = rospy.get_param("frame_id")

    def broadcast_tf(self):
        tf_pose = pm.toTf(pm.fromMsg(self.pose))
        if self.NEW_MODEL_REC:
            self.bc.sendTransform(tf_pose[0], tf_pose[1], rospy.Time.now(), self.object_name, self.detected_frame)
        else:
            self.bc.sendTransform(tf_pose[0], tf_pose[1], rospy.Time.now(), self.object_name, "/world")

    def get_dist(self):
        self.broadcast_tf()
        self.listener.waitForTransform(self.detected_frame, self.object_name, rospy.Time(0), rospy.Duration(10))
        (trans, rot) = self.listener.lookupTransform(self.detected_frame, self.object_name, rospy.Time(0))
        return linalg.norm(trans)

    def __len__(self):
        return self.get_dist()

    def get_world_pose(self):
        self.broadcast_tf()
        self.listener.waitForTransform("/world", self.object_name, rospy.Time(0),rospy.Duration(10))
        return pm.toMsg(pm.fromTf(self.listener.lookupTransform("/world", self.object_name, rospy.Time(0))))

class ModelRecManager(object):

    def __init__(self, NEW_MODEL_REC):
        self.NEW_MODEL_REC = NEW_MODEL_REC
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

        if self.NEW_MODEL_REC:
            find_objects_srv = rospy.ServiceProxy('/objrec_node/find_objects', objrec_ros_integration.srv.FindObjects)
        else:
            find_objects_srv = rospy.ServiceProxy('/recognize_objects',  model_rec2.srv.FindObjects)

        resp = find_objects_srv()

        for i in range(len(resp.object_name)):
            rospy.loginfo("Adding ModelManager for object" + str(resp.object_name[i]) )
            rospy.loginfo("Pose: " + str(resp.object_pose[i]))
            self.model_list.append(ModelManager(resp.object_name[i],
                                                resp.object_pose[i], self.NEW_MODEL_REC))
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

