import rospy

import tf
import tf_conversions.posemath as pm

from collections import namedtuple

DetectedModel = namedtuple(
    'DetectedModel', ['name', 'poseStamped'], verbose=False)


class ModelPoseBroadcaster(object):
    def __init__(self):

        self._model_list = list()
        self._tf_broadcaster = tf.TransformBroadcaster()

    def add_model(self, model_name, pose):
        self._model_list.append(DetectedModel(model_name, pose))

    def broadcast_object_tfs(self):
        for model in self._model_list:
            tf_pose = pm.toTf(pm.fromMsg(model.poseStamped.pose))
            self._tf_broadcaster.sendTransform(
                tf_pose[0], tf_pose[1],
                rospy.Time.now(), model.name,
                model.poseStamped.header.frame_id)

    def clear_models(self):
        self._model_list = []
