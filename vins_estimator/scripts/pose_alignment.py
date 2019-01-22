#! /usr/bin/python

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_msgs.msg import String
import numpy as np
from pyquaternion import Quaternion

class PoseAlignment(object):
  def __init__(self):
    rospy.init_node("pose_alignment")
    pose_topic_sub = rospy.get_param("~pose_topic_sub","/mavros/local_position/pose")
    vins_init_topic_sub = rospy.get_param("~vins_init_topic_sub", "/vins_estimator/start")
    vins_odom_topic_sub = rospy.get_param("~vins_odom_topic_sub", "/vins_estimator/odometry")
    pose_topic_pub = rospy.get_param("~pose_topic_pub", "/alignment/aligned_pose")

    self.gps_sub = rospy.Subscriber(pose_topic_sub, PoseStamped, self.gps_cb)
    self.vins_init_sub = rospy.Subscriber(vins_init_topic_sub, Bool, self.vins_init_cb)
    self.vins_odom_sub = rospy.Subscriber(vins_odom_topic_sub, Odometry, self.vins_odom_cb)
    self.pose_pub = rospy.Publisher(pose_topic_pub, PoseStamped, queue_size=1)
    self.vins_init = False
    self.vins_pose = PoseStamped()
    self.vins_pose_home = PoseStamped()
    self.gps_pose_home = PoseStamped()
    self.set_home = False
    self.q_vins_home = Quaternion()
    self.t_vins_home = []
    self.q_gps_home = Quaternion()
    self.t_gps_home = []
    self.q_gps_world = Quaternion()
    self.t_gps_world = []
    self.T_gps_world = Quaternion.transformation_matrix
    self.pose = PoseStamped()

  def vins_init_cb(self, msg):
    self.vins_init = msg.data

  def vins_odom_cb(self, msg):
    self.vins_pose = msg.pose

  def gps_cb(self, msg):
    if self.vins_init == True and self.set_home == False:
      self.set_home = True
      self.vins_pose_home = self.vins_pose
      self.gps_pose_home.pose = msg.pose
      w = self.vins_pose_home.pose.orientation.w
      x = self.vins_pose_home.pose.orientation.x
      y = self.vins_pose_home.pose.orientation.y
      z = self.vins_pose_home.pose.orientation.z
      self.q_vins_home = Quaternion(w, x, y, z)
      x = self.vins_pose_home.pose.position.x
      y = self.vins_pose_home.pose.position.y
      z = self.vins_pose_home.pose.position.z
      self.t_vins_home = np.array([x, y, z])
      w = self.gps_pose_home.pose.orientation.w
      x = self.gps_pose_home.pose.orientation.x
      y = self.gps_pose_home.pose.orientation.y
      z = self.gps_pose_home.pose.orientation.z
      self.q_gps_home = Quaternion(w, x, y, z)
      x = self.gps_pose_home.pose.position.x
      y = self.gps_pose_home.pose.position.y
      z = self.gps_pose_home.pose.position.z
      self.t_gps_home = np.array([x, y, z])

    if self.set_home == True:
      q_inv = self.q_vins_home.inverse
      t_inv = -q_inv.rotation_matrix.dot(np.reshape(self.t_vins_home,[3,1]))
      T_inv = q_inv.transformation_matrix
      T_inv[0,3] = t_inv[0]
      T_inv[1,3] = t_inv[1]
      T_inv[2,3] = t_inv[2]
      x = self.vins_pose.pose.orientation.x
      y = self.vins_pose.pose.orientation.y
      z = self.vins_pose.pose.orientation.z
      w = self.vins_pose.pose.orientation.w
      q_vins = Quaternion(w, x, y, z)
      T_vins = q_vins.transformation_matrix
      T_vins[0,3] = self.vins_pose.pose.position.x
      T_vins[1,3] = self.vins_pose.pose.position.y
      T_vins[2,3] = self.vins_pose.pose.position.z
      dT = T_inv.dot(T_vins)
      T_gps_home = self.q_gps_home.transformation_matrix
      T_gps_home[0,3] = self.t_gps_home[0]
      T_gps_home[1,3] = self.t_gps_home[1]
      T_gps_home[2,3] = self.t_gps_home[2]
      self.T_gps_world = T_gps_home.dot(dT)
      self.t_gps_world = self.T_gps_world[:3, 3]
      self.q_gps_world = Quaternion(matrix = self.T_gps_world[:3, :3])

      self.pose.header = msg.header
      self.pose.header.frame_id = "map"
      self.pose.pose.position.x = self.t_gps_world[0]
      self.pose.pose.position.y = self.t_gps_world[1]
      self.pose.pose.position.z = self.t_gps_world[2]
      elem = self.q_gps_world.elements
      self.pose.pose.orientation.w = elem[0]
      self.pose.pose.orientation.x = elem[1]
      self.pose.pose.orientation.y = elem[2]
      self.pose.pose.orientation.z = elem[3]
      self.pose_pub.publish(self.pose)

if __name__ == '__main__':

  pose_alignment = PoseAlignment()

  rate = rospy.Rate(10)
  while not rospy.is_shutdown():
    rate.sleep()
  rospy.spin
