import world_manager.srv
import world_manager.msg
import std_srvs.srv
import geometry_msgs.msg
import rospy


class WorldManagerClient:
    def __init__(self):
        pass

    def add_mesh(self, mesh_name, mesh_filepath, pose_stamped):

        service_proxy = rospy.ServiceProxy("/world_manager/add_mesh",
                                           world_manager.srv.AddMesh)
        service_proxy.wait_for_service(timeout=5)
        scene_object = world_manager.msg.SceneObject(mesh_name, mesh_filepath, pose_stamped)
        service_proxy(scene_object)

    def add_box(self, object_name, pose_stamped, edge_length_x, edge_length_y, edge_length_z):
        # type: (str, geometry_msgs.msg.PoseStamped, float, float, float) -> ()
        service_proxy = rospy.ServiceProxy("/world_manager/add_box", world_manager.srv.AddBox)
        service_proxy.wait_for_service(timeout=5)

        scene_box = world_manager.msg.SceneBox(object_name, pose_stamped, edge_length_x, edge_length_y, edge_length_z)
        service_proxy(scene_box)

    def clear_objects(self):
        service_proxy = rospy.ServiceProxy("/world_manager/clear_objects",
                                           std_srvs.srv.Empty)
        service_proxy.wait_for_service(timeout=5)
        service_proxy()

    def add_walls(self):
        service_proxy = rospy.ServiceProxy("/world_manager/add_walls",
                                           std_srvs.srv.Empty)
        service_proxy.wait_for_service(timeout=5)
        service_proxy()

    def add_tf(self, frame_name, pose_stamped):
        service_proxy = rospy.ServiceProxy("/world_manager/add_tf",
                                           world_manager.srv.AddTF)
        service_proxy.wait_for_service(timeout=5)
        service_proxy(frame_name, pose_stamped)
