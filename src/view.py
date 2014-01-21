#!/usr/bin/env python
import roslib
roslib.load_manifest('kitti')
import cv
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import message_filters


def makeMagic(left, right, out):
  chans=[]
  for i in range(6):
    chans.append(cv.CreateImage(cv.GetSize(left),8,1))
  cv.Split(left, chans[0], chans[1], chans[2], None);
  cv.Split(right, chans[3], chans[4], chans[5], None);
  cv.Merge(chans[3],chans[4],chans[2], None, out);


class Viewer(object):
  def __init__(self):
    self.bridge = CvBridge()
    self.sub_00 = message_filters.Subscriber("image_00/image_raw",Image)
    self.sub_01 = message_filters.Subscriber("image_01/image_raw",Image)
    self.sub_02 = message_filters.Subscriber("image_02/image_raw",Image)
    self.sub_03 = message_filters.Subscriber("image_03/image_raw",Image)
    tmp = [self.sub_00, self.sub_01, self.sub_02, self.sub_03]
    self.ts = message_filters.TimeSynchronizer(tmp,10)
    self.ts.registerCallback(self.callback)
    cv.NamedWindow("Camera 00", cv.CV_WINDOW_NORMAL)
    cv.NamedWindow("Camera 01", cv.CV_WINDOW_NORMAL)
    cv.NamedWindow("Camera 02", cv.CV_WINDOW_NORMAL)
    cv.NamedWindow("Camera 03", cv.CV_WINDOW_NORMAL)
    cv.NamedWindow("Anaglyf col", cv.CV_WINDOW_NORMAL)
    
  def callback(self,I0,I1,I2,I3):
    try:
      cv_i0 = self.bridge.imgmsg_to_cv(I0, "mono8")
      cv_i1 = self.bridge.imgmsg_to_cv(I1, "mono8")
      cv_i2 = self.bridge.imgmsg_to_cv(I2, "bgr8")
      cv_i3 = self.bridge.imgmsg_to_cv(I3, "bgr8")
    except CvBridgeError, e:
      print e
    cv.ShowImage("Camera 00", cv_i0)
    cv.ShowImage("Camera 01", cv_i1)
    cv.ShowImage("Camera 02", cv_i2)
    cv.ShowImage("Camera 03", cv_i3)
    merge=cv.CreateImage(cv.GetSize(cv_i2),8,3)
    makeMagic(cv_i3,cv_i2,merge)
    cv.ShowImage("Anaglyf col", merge)
    cv.WaitKey(3)

if __name__ == '__main__':
  rospy.init_node('viewer')
  viewer = Viewer()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv.DestroyAllWindows()



