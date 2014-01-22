#!/usr/bin/env python  
import roslib
roslib.load_manifest('kitti')
import rospy
import os, sys
import numpy as np
from datetime import datetime
import cv, yaml
from std_msgs.msg import String,Header
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs import point_cloud2
import glob
import ctypes,struct

dataset_dir = '/home/atexpa/data/kitti_raw'

def read_timestamps(fname):
  times = []
  with open(fname,'r') as f:
    for l in f:
      times.append(np.datetime64(l))
  return times

def read_velodyn_data(fname):
  A = np.fromfile(fname,dtype=np.single)
  A = A.reshape([A.shape[0]/4, 4])
  A = A.astype(np.float32)
  X = A[:,0]
  Y = A[:,1]
  Z = A[:,2]
  I = A[:,3]
  return [[x,y,z,i] for x,y,z,i in zip(X,Y,Z,I)]
  #return [[x,y,z] for x,y,z in zip(X,Y,Z)]

def read_velodyn(path,idx):
  t_start = read_timestamps(path+'/velodyne_points/timestamps_start.txt')
  t_stop = read_timestamps(path+'/velodyne_points/timestamps_end.txt')
  t = read_timestamps(path+'/velodyne_points/timestamps.txt')
  data = read_velodyn_data(path+'/velodyne_points/data/%010d.bin'%idx)
  return data, t_start[idx], t_stop[idx], t[idx]

def read_calibration(fname, n_cam = 4):
  res = [CameraInfo() for i in range(n_cam)]
  with open(fname,'r') as fid:
    db = yaml.load(fid)
    for i in range(n_cam):
      cam = '%02d'%i
      res[i].width = int(np.array(db['S_rect_'+cam].split(' ')).astype(np.double).astype(np.uint32)[0])
      res[i].height = int(np.array(db['S_rect_'+cam].split(' ')).astype(np.double).astype(np.uint32)[1])
# data are rectified ...
#      K = np.array(db['K_'+cam].split(' ')).astype(np.double).reshape([3,3])
#      res.K = K
#      D = np.array(db['D_'+cam].split(' ')).astype(np.double).reshape([5,1])
      res[i].D = list(np.zeros([5,1]).flatten())
      res[i].distortion_model = 'plumb_bob'
      res[i].binning_x = 0
      res[i].binning_y = 0
      res[i].roi.x_offset = 0
      res[i].roi.y_offset = 0
      res[i].roi.height = res[i].height
      res[i].roi.width = res[i].width
      res[i].roi.do_rectify = False
####     
      res[i].R = list(np.eye(3).flatten())
      P = np.array(db['P_rect_'+cam].split(' ')).astype(np.double).reshape([3,4])
      res[i].P = list(P.flatten())
      res[i].K = list(P[:,0:3].flatten())
  return res

#def read_oxts(fname):

class Kitti(object):
  def __init__(self, seq_name = '2011_09_26_drive_0001'):
    self.seq_name = seq_name
    #init_tf()
    self.init_images()
    self.init_oxts()
    self.finish = False
    self.init_velodyne()
    #self.init_tracklets()
    self.bridge = CvBridge()
  def next(self):
    self.emit_im(self.current)
    self.emit_velodyne(self.current)
    self.current += 1
    if self.current >= self.im_num:
      self.finish = True
  def init_tf(self):
    ''' load calibrations '''
  def init_images(self):
    self.current = 0
    seq = dataset_dir+'/'+self.seq_name
    self.calib = read_calibration(seq+'/calib_cam_to_cam.txt')
    if os.path.exists(seq+'/image_00'):
      self.has_gray = True
      self.im_num = len(glob.glob(seq+'/image_00/data/*.png')) 
      self.t_00 = read_timestamps(seq+'/image_00/timestamps.txt') 
      self.t_01 = read_timestamps(seq+'/image_01/timestamps.txt')
      self.pub_00 = rospy.Publisher("image_00/image_raw",Image)
      self.pub_01 = rospy.Publisher("image_01/image_raw",Image)
      self.pub_00_cal = rospy.Publisher("image_00/camera_info",CameraInfo)
      self.pub_01_cal = rospy.Publisher("image_01/camera_info",CameraInfo)
    else:
      self.has_gray = False
    if os.path.exists(seq+'/image_02'):
      self.has_color = True
      self.t_02 = read_timestamps(seq+'/image_02/timestamps.txt') 
      self.t_03 = read_timestamps(seq+'/image_03/timestamps.txt') 
      self.pub_02 = rospy.Publisher("image_02/image_raw",Image)
      self.pub_03 = rospy.Publisher("image_03/image_raw",Image)
      self.pub_02_cal = rospy.Publisher("image_02/camera_info",CameraInfo)
      self.pub_03_cal = rospy.Publisher("image_03/camera_info",CameraInfo)
    else:
      self.has_color = False
  def init_velodyne(self):
    seq = dataset_dir+'/'+self.seq_name
    if os.path.exists(seq+'/velodyne_points'):
      self.has_lidar = True
      self.t_vstart = read_timestamps(seq+'/velodyne_points/timestamps_start.txt');
      self.t_vend = read_timestamps(seq+'/velodyne_points/timestamps_end.txt');
      self.pub_pts = rospy.Publisher("velodyne/pts",PointCloud2)
      self.pts_fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1),
          PointField('rgb', 12, PointField.FLOAT32, 1)]
    else:
      self.has_lidar = False
  def init_oxts(self):
    seq = dataset_dir+'/'+self.seq_name
    if os.path.exists(seq+'/oxts'):
      self.has_gps = True
      self.t_gps = read_timestamps(seq+'/oxts/timestamps.txt')
    else:
      self.has_gps = False
  def emit_im(self,idx):
    seq = dataset_dir+'/'+self.seq_name
    if self.has_gray:
      I0 = cv.LoadImage(seq+'/image_00/data/%010d.png'%idx,False)
      I1 = cv.LoadImage(seq+'/image_01/data/%010d.png'%idx,False)
    if self.has_color:
      I2 = cv.LoadImage(seq+'/image_02/data/%010d.png'%idx)
      I3 = cv.LoadImage(seq+'/image_03/data/%010d.png'%idx)
    if self.has_gray:
      self.pub_00.publish(self.bridge.cv_to_imgmsg(I0,'mono8'))
      self.pub_00_cal.publish(self.calib[0])
      self.pub_01.publish(self.bridge.cv_to_imgmsg(I1,'mono8'))
      self.pub_01_cal.publish(self.calib[1])
    if self.has_color:
      self.pub_02.publish(self.bridge.cv_to_imgmsg(I2,'bgr8'))
      self.pub_02_cal.publish(self.calib[2])
      self.pub_03.publish(self.bridge.cv_to_imgmsg(I3,'bgr8'))
      self.pub_03_cal.publish(self.calib[3])
  def emit_velodyne(self,idx):
    seq = dataset_dir+'/'+self.seq_name
    if self.has_lidar:
      pts,t1,t2,t = read_velodyn(seq,idx)
      header = Header()
      header.frame_id = '/stereorig'
      msg = point_cloud2.create_cloud(header,self.pts_fields,pts)
      self.pub_pts.publish(msg)





if __name__ == '__main__':
  rospy.init_node('stereo')
#  br = tf.TransformBroadcaster()
  rate = rospy.Rate(0.05)
#  data = Kitti()
  data = Kitti('2011_09_29_drive_0071')
#  data = Kitti('2011_09_30_drive_0028')
#  data = Kitti('2011_10_03_drive_0034')
  data.has_color = False
  data.has_lidar = True
#  data = Kitti('2011_09_26_drive_0011')
  while not rospy.is_shutdown() and not data.finish:
    data.next()
    rospy.sleep(0.1)

