#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(object):
    def __init__(self):
        # Params
        self.image = None
        self.br = CvBridge()
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(30)

        # Subscribers
        rospy.Subscriber("/carla/ego_vehicle/rgb_front/image", Image, self.callback)

    def callback(self, msg):
        rospy.loginfo('Image received...')
        self.image = self.br.imgmsg_to_cv2(msg)

    def start(self):
        rospy.loginfo("Timing images")
        #rospy.spin()
        while not rospy.is_shutdown():
            rospy.loginfo('publishing image')
            #br = CvBridge()
            if self.image is not None:
                cv2.imshow('image', self.image)
                cv2.waitKey(1)
            self.loop_rate.sleep()
        cv2.destroyAllWindows()

    

    

if __name__ == '__main__':
    rospy.init_node("ImageTransport", anonymous=True)
    image_subcriber = ImageSubscriber()
    image_subcriber.start()