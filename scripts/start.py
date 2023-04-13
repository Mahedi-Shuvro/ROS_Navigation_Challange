import numpy as np
import cv2
import rospy as rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist

def start(img):
    start_image= cv2.imread('../ROS_Navigation_Challange/markers/markers/')
    bridge=CvBridge()
    cv_image=bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')

    if __name__ =='main_':
        rospy.init_node('realtime_image_stream',annonymous=True)
        rospy.Subscriber('/camera/rgb/image_raw', cv_image, start)
        rospy.spin()

def move():
    pub=rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('move_rosbot', annonymous=True)
    rate=rospy.Rate(10)

    move_cmd=Twist()
    move_cmd.linear.x=0.2
    move_cmd.angular.z=0.0

    end_time=rospy.Time.now()+rospy.Duration(5)
    pub.publish(move_cmd)
    rate.sleep()

    stop_cmd=Twist()
    pub.publish(stop_cmd)

    if __name__=='main_':
        try:
            move()
        except rospy.ROSInterruptException:
            pass

res=cv2.matchTemplate(cv2.camera_gray, cv2.start_gray, cv2.TM_CCOEFF_NORMED)

threshold=0.8
if np.max(res) > threshold:
    print('Start image found')
    move()
else:
    print('Start image not found')