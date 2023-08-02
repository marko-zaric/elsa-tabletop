#!/usr/bin/env python3
import numpy as np
import colorsys
import rospy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion, Vector3
from elsa_perception_msgs.msg import FullSceneStamped

class RvizMarkers:
    def __init__(self):
        self.fullstate_sub = rospy.Subscriber("fullscene", FullSceneStamped, self.callback)
        self.obj_markers_pub = rospy.Publisher("elsa_perception/scene_viz_markers", MarkerArray, queue_size=1)
    
    def callback(self, msg):
        # update current with new
        seen_objects = set()
        seen_obj_markers = MarkerArray()
        o = 0

        for obj in msg.physical_scene:
            seen_objects.add(obj.obj_identity)


            m = self.create_obj_marker(msg, o)
            o += 1
            seen_obj_markers.markers.append(m)

        self.obj_markers_pub.publish(seen_obj_markers)


    def create_obj_marker(self, msg, o_id):
        m = Marker()
        m.type = 1
        m.id = o_id
        m.header.frame_id = msg.header.frame_id
        m.lifetime = rospy.Duration(1)

        m.scale = Vector3(x = msg.physical_scene[o_id].spatial_features.dx,
            y = msg.physical_scene[o_id].spatial_features.dy,
            z = msg.physical_scene[o_id].spatial_features.dz)

        m.pose.position = Point(x = msg.physical_scene[o_id].spatial_features.x,
            y = msg.physical_scene[o_id].spatial_features.y,
            z = msg.physical_scene[o_id].spatial_features.z)
            
        # quaternion for rotation around z of angle theta
        theta = msg.physical_scene[o_id].spatial_features.phi
        quat1 = [np.sin(theta/2.0) * 0.0, np.sin(theta/2.0) * 0.0, np.sin(theta/2.0) * 1.0, np.cos(theta/2.0)] 
        quat1 = quat1 / np.linalg.norm(quat1)
        m.pose.orientation = Quaternion(x=quat1[0], y=quat1[1], z=quat1[2], w=quat1[3])
        rgb_color = colorsys.hsv_to_rgb(*msg.physical_scene[o_id].mean_color)

        m.color = ColorRGBA(r=rgb_color[0], g=rgb_color[1], b=rgb_color[2], a=1.0)
        return m

if __name__ == "__main__":
    rospy.init_node("elsa_perception_rviz_markers")
    RvizMarkers()
    rospy.spin()
    