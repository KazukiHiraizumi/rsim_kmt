#!/usr/bin/env python3

import numpy as np
import roslib
import rospy
import tf
import tf2_ros
import open3d as o3d
import time
from scipy.spatial.transform import Rotation as R
from rovi.msg import Floats
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Bool
from std_msgs.msg import String
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from rovi_utils import tflib

Param={
  "enable":0,
  "margin_top":0,
  "margin_left":0,
  "margin_bottom":0,
  "margin_right":0,
}
Config={
  "capture_frame_id":"camera/capture0"
}

def np2F(d):  #numpy to Floats
  f=Floats()
  f.data=np.ravel(d)
  return f

def Pnul():
  return np.reshape([],(-1,3))

def getRT(base,ref):
  try:
    ts=tfBuffer.lookup_transform(base,ref,rospy.Time())
    rospy.loginfo("getRT::TF lookup success "+base+"->"+ref)
    RT=tflib.toRT(ts.transform)
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    RT=None
  return RT

def pTr(RT,pc):
  return np.dot(RT[:3],np.vstack((pc.T,np.ones((1,len(pc)))))).T


def cb_img(msg):
  cvimg=bridge.imgmsg_to_cv2(msg,"mono8")

def cb_do1(msg):
  global Param
  try:
    Param.update(rospy.get_param("/searcher2d"))
  except Exception as e:
    print("get_param exception:",e.args)
  if not Param["enable"]:
    pub_thru1.publish(mTrue)
    return

def cb_do2(msg):
  global Param
  try:
    Param.update(rospy.get_param("/searcher2d"))
  except Exception as e:
    print("get_param exception:",e.args)
  if not Param["enable"]:
    pub_thru2.publish(mTrue)
    return

########################################################
rospy.init_node("searcher2d",anonymous=True)
###Load params
try:
  Config.update(rospy.get_param("/config/searcher2d"))
except Exception as e:
  print("get_param exception:",e.args)
###Topics
rospy.Subscriber("~image_in",Image,cb_img)
rospy.Subscriber("~do1",Bool,cb_do1)  #would be "request capture"
rospy.Subscriber("~do2",Bool,cb_do2)  #would be "request solve"

pub_ps=rospy.Publisher("/searcher2d/image",Image,queue_size=1)
pub_str=rospy.Publisher("/report",String,queue_size=1)
pub_done1=rospy.Publisher("~done1",Bool,queue_size=1)
pub_done2=rospy.Publisher("~done2",Bool,queue_size=1)
pub_thru1=rospy.Publisher("~thru1",Bool,queue_size=1)
pub_thru2=rospy.Publisher("~thru2",Bool,queue_size=1)

###Bool message
mTrue=Bool();mTrue.data=True
mFalse=Bool();mFalse.data=False
###TF
tfBuffer=tf2_ros.Buffer()
listener=tf2_ros.TransformListener(tfBuffer)
broadcaster=tf2_ros.StaticTransformBroadcaster()

bridge=CvBridge()

#if __name__=="__main__":
#

try:
  rospy.spin()
except KeyboardInterrupt:
  print("Shutting down")
