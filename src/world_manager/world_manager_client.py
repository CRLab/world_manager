import world_manager.srv
import std_srvs.srv
import rospy


class WorldManagerClient():
    def add_mesh(self, mesh_name, mesh_filepath, pose_stamped):

        service_proxy = rospy.ServiceProxy("/world_manager/add_mesh",
                                           world_manager.srv.AddMesh)
        service_proxy.wait_for_service(timeout=5)
        service_proxy(mesh_name, mesh_filepath, pose_stamped)

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
