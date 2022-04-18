#!/usr/bin/env python3

# Python includes
import numpy as np
import subprocess
import open3d as o3d

# ROS includes
import roslib
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3
from tf import transformations
from rviz_tools_py import rviz_tools
from rovi.msg import Floats
from rospy.numpy_msg import numpy_msg


def cleanup_node():
  print("Shutting down node")
  markers.deleteAllMarkers()

def cb_redraw(msg):
  f=Floats()
  f.data=np.ravel(np.array(pcd.points))
  pub_wp.publish(f)

# Initialize the ROS Node
rospy.init_node('vscene', anonymous=False, log_level=rospy.INFO, disable_signals=False)
pub_wp=rospy.Publisher("/vscene/floats",numpy_msg(Floats),queue_size=1)
rospy.Subscriber("/request/redraw",Bool,cb_redraw)
rospy.on_shutdown(cleanup_node)

Tf = transformations.translation_matrix((0,0,200))
thispath=subprocess.getoutput("rospack find rsim_kmt")
pcd=o3d.io.read_point_cloud(thispath+'/model/ring.ply')
pcd=pcd.transform(Tf)
mesh_file = "package://rsim_kmt/model/ring.stl"
rospy.Timer(rospy.Duration(3),cb_redraw,oneshot=True)
markers = rviz_tools.RvizMarkers('pallet', 'vscene_marker')

while not rospy.is_shutdown():
  scale = Vector3(1,1,1)
  markers.publishMesh(Tf, mesh_file, 'brown', scale, 0.5)
  rospy.Rate(5).sleep() #1 Hz
