#!/usr/bin/env python3

# Python includes
import numpy as np
import subprocess
import open3d as o3d

# ROS includes
import roslib
import rospy
import tf
import tf2_ros
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3
from tf import transformations
from rviz_tools_py import rviz_tools
from rovi.msg import Floats
from rospy.numpy_msg import numpy_msg
from rovi_utils import tflib

def getRT(base,ref):
  try:
    ts=tfBuffer.lookup_transform(base,ref,rospy.Time())
    rospy.loginfo("getRT::TF lookup success "+base+"->"+ref)
    RT=tflib.toRT(ts.transform)
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    print('frame not found',ref)
    RT=np.eye(4)
  return RT

def pTr(RT,pc):
  return np.dot(RT[:3],np.vstack((pc.T,np.ones((1,len(pc)))))).T

def cleanup_node():
  print("Shutting down node")
  markers.deleteAllMarkers()

def cb_redraw(msg):
  f=Floats()
  f.data=np.ravel(Points)
  pub_wp.publish(f)

def loadply(name,frame):
  global Points
  wTo = getRT('world',frame)
  thispath=subprocess.getoutput("rospack find rsim_kmt")
  pcd=o3d.io.read_point_cloud(thispath+'/model/'+name+'.ply')
  pcd=pcd.transform(wTo)
  Points=np.vstack((Points,np.array(pcd.points)))

def load3d(name,frame):
  global Meshes,Points,Tf
  mesh_file = 'package://rsim_kmt/model/'+name+'.stl'
  Meshes.append(mesh_file)
  wTo = getRT('world',frame)
  Tf.append(wTo)
  thispath=subprocess.getoutput("rospack find rsim_kmt")
  pcd=o3d.io.read_point_cloud(thispath+'/model/'+name+'.ply')
  pcd=pcd.transform(wTo)
  Points=np.vstack((Points,np.array(pcd.points)))


# Initialize the ROS Node
rospy.init_node('vscene', anonymous=False, log_level=rospy.INFO, disable_signals=False)
pub_wp=rospy.Publisher("/vscene/floats",numpy_msg(Floats),queue_size=1)
rospy.Subscriber("/request/redraw",Bool,cb_redraw)
rospy.on_shutdown(cleanup_node)

markers = rviz_tools.RvizMarkers('world', 'vscene_marker')
Points=np.array([]).reshape((-1,3))
Meshes=[]
Tf=[]

###TF
tfBuffer=tf2_ros.Buffer()
listener=tf2_ros.TransformListener(tfBuffer)
broadcaster=tf2_ros.StaticTransformBroadcaster()

rospy.sleep(2)

load3d('marud','marud')
load3d('tatep','tatep')

scale=Vector3(1,1,1)

while not rospy.is_shutdown():
  markers.publishMesh(Tf[0],Meshes[0],(0.8,0.8,0.8),scale, 0.5)
  markers.publishMesh(Tf[1],Meshes[1],(0.7,0.7,0.7),scale, 0.5)
  Tf2=np.array(Tf[1])
  Tf2[0,3]=Tf2[0,3]-400
  markers.publishMesh(Tf2,Meshes[1],(0.7,0.7,0.7),scale, 0.5)
  rospy.Rate(3).sleep()
