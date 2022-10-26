#!/usr/bin/env python3

import numpy as np
import roslib
import rospy
import tf
import tf2_ros
import os
import sys
import subprocess
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Bool
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped
from rovi_utils import tflib

Param={
  "xyz":[0,0,0],
  "rpy":[0,0,0],
  "path1":{
    "ip":"mov_z",
    "xyz":[-100,0,1400],
    "rpy":[0,120,0],
    "var":[0,-400,-800],
    "pause":3
  },
  "path2":{
    "ip":"rot_z",
    "xyz":[-100,0,600],
    "rpy":[0,120,0],
    "var":[-45,45],
    "pause":3
  },
  "path3o":{
    "ip":"mov_z",
    "xyz":[0,0,600],
    "rpy":[0,180,90],
    "var":[90],
    "pause":1,
    "capture":True
  },
  "path3":{
    "org":"path3o",
    "ip":"rot_z",
    "xyz":[-100,0,600],
    "rpy":[0,120,0],
    "var":[0,360],
    "pitch":36,
    "pause":1,
    "capture":True
  },
  "path4":{
    "ip":"rot_z",
    "xyz":[-200,0,600],
    "rpy":[0,120,0],
    "var":[90],
    "pause":1
  },
  "path5":{
    "ip":"rot_z",
    "xyz":[-500,0,1000],
    "rpy":[0,180,90],
    "var":[90],
    "pause":1
  },
  "uf":"uf0"
}
Config={
  "target_frame_id":"tool0_controller",
}

def getRT(base,ref):
  try:
    ts=tfBuffer.lookup_transform(base,ref,rospy.Time())
    rospy.loginfo("cropper::getRT::TF lookup success "+base+"->"+ref)
    RT=tflib.toRT(ts.transform)
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    RT=None
  return RT

def mov(pos):
  print("vrobo move",pos,Param["uf"])
  rot=R.from_euler('xyz',pos[3:6],degrees=True)
  Tm=np.eye(4)
  Tm[:3,:3]=rot.as_matrix()
  Tm[:3,3]=np.array(pos[:3]).T
  bTu=getRT("base",Param["uf"])
  tf=TransformStamped()
  tf.header.stamp=rospy.Time.now()
  tf.child_frame_id=Config["target_frame_id"]
  tf.transform=tflib.fromRT(bTu.dot(Tm))
  pub_tf.publish(tf);

def setbase(pos):
  rot=R.from_euler('xyz',pos[3:6],degrees=True)
  Rt=np.eye(4)
  Rt[:3,:3]=rot.as_matrix()
  Rt[:3,3]=np.array(pos[:3]).T
  tf=TransformStamped()
  tf.header.stamp=rospy.Time.now()
  tf.header.frame_id="world"
  tf.child_frame_id="base"
  tf.transform=tflib.fromRT(Rt)
  pub_tf.publish(tf);


def cb_mov(msg):
  global Param
  try:
    Param.update(rospy.get_param("~param"))
  except Exception as e:
    print("get_param exception:",e.args)
  mov(Param["xyz"]+Param["rpy"])

def cb_base(msg):
  global Param
  try:
    Param.update(rospy.get_param("~param"))
  except Exception as e:
    print("get_param exception:",e.args)
  setbase(Param["xyz"]+Param["rpy"])

def pause(prm):
  rospy.sleep(prm['pause'])
  if 'capture' in prm:
    rospy.Timer(rospy.Duration(0.1),lambda ev:pub_capt.publish(mTrue),oneshot=True)
    rospy.wait_for_message('/response/capture',Bool)

def mov_z(prm):
  mov(prm["xyz"]+prm["rpy"])
  rospy.sleep(0.01)
  wTc=getRT(Param["uf"],Config["target_frame_id"])
  if "pitch" in prm: vars=np.arange(prm["var"][0],prm["var"][1],prm["pitch"])
  else: vars=prm["var"]
  print("vars",vars)
  for z in vars:
    rt=np.eye(4)
    rt[2,3]=z
    wTcc=rt.dot(wTc)
    euler=R.from_matrix(wTcc[:3,:3]).as_euler('xyz',degrees=True)
    mov([wTcc[0,3],wTcc[1,3],wTcc[2,3],euler[0],euler[1],euler[2]])
    pause(prm)
  pub_inpos.publish(mTrue)

def rot_z(prm):
  mov(prm["xyz"]+prm["rpy"])
  rospy.sleep(0.01)
  wTc=getRT(Param["uf"],Config["target_frame_id"])
  if "pitch" in prm: vars=np.arange(prm["var"][0],prm["var"][1],prm["pitch"])
  else: vars=prm["var"]
  for rz in vars:
    rt=np.eye(4)
    rt[:3,:3]=R.from_euler('z',rz,degrees=True).as_matrix()
    wTcc=rt.dot(wTc)
    euler=R.from_matrix(wTcc[:3,:3]).as_euler('xyz',degrees=True)
    mov([wTcc[0,3],wTcc[1,3],wTcc[2,3],euler[0],euler[1],euler[2]])
    pause(prm)
  pub_inpos.publish(mTrue)

def path_exec(path):
  global Param
  exec("sub_"+path+".unregister()")
  try:
    Param.update(rospy.get_param("~param"))
  except Exception as e:
    print("get_param exception:",e.args)
  if 'org' in Param[path]:
    org=Param[path]['org']
    prm=Param[org]
    exec(prm["ip"]+"(prm)")    
  prm=Param[path]
  exec(prm["ip"]+"(prm)")
  exec("global sub_"+path+"; sub_"+path+"=rospy.Subscriber('/vrobo/"+path+"',Bool,cb_"+path+")")

def cb_path1(msg):
  path_exec("path1")
def cb_path2(msg):
  path_exec("path2")
def cb_path3(msg):
  path_exec("path3")
def cb_path4(msg):
  path_exec("path4")
def cb_path5(msg):
  path_exec("path5")


########################################################
rospy.init_node("vrobo",anonymous=True)
thispath=subprocess.getoutput("rospack find rovi_sim")
###Load params
try:
  Config.update(rospy.get_param("/config/vrobo"))
except Exception as e:
  print("get_param exception:",e.args)
###Topics
rospy.Subscriber("/vrobo/mov",Bool,cb_mov)
sub_path1=rospy.Subscriber("/vrobo/path1",Bool,cb_path1)
sub_path2=rospy.Subscriber("/vrobo/path2",Bool,cb_path2)
sub_path3=rospy.Subscriber("/vrobo/path3",Bool,cb_path3)
sub_path4=rospy.Subscriber("/vrobo/path4",Bool,cb_path4)
sub_path5=rospy.Subscriber("/vrobo/path5",Bool,cb_path5)
rospy.Subscriber("/vrobo/setbase",Bool,cb_base)
pub_inpos=rospy.Publisher("/vrobo/inpos",Bool,queue_size=1)
pub_tf=rospy.Publisher("/update/config_tf",TransformStamped,queue_size=1)
pub_capt=rospy.Publisher("/request/capture",Bool,queue_size=1)
###TF
tfBuffer=tf2_ros.Buffer()
listener=tf2_ros.TransformListener(tfBuffer)
###Bool
mTrue=Bool();mTrue.data=True
mFalse=Bool()
#if __name__=="__main__":

try:
  rospy.spin()
except KeyboardInterrupt:
  print("Shutting down")
