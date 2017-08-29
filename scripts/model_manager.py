import rospy

import tf
import tf_conversions.posemath as pm


class ModelPoseBroadcaster(object):
    def __init__(self):

        self._model_dict = dict()
        self._tf_broadcaster = tf.TransformBroadcaster()

    def add_model(self, scene_object):
        self._model_dict[scene_object.objectname] = scene_object

    def broadcast_object_tfs(self):
        for objectname in self._model_dict.keys():
            tf_pose = pm.toTf(pm.fromMsg(self._model_dict[objectname].pose_stamped.pose))
            self._tf_broadcaster.sendTransform(
                tf_pose[0], tf_pose[1],
                rospy.Time.now(), objectname,
                self._model_dict[objectname].pose_stamped.header.frame_id)

    def clear_models(self):
        self._model_list.clear()
