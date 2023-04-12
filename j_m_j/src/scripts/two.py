import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

class HazardMarkerPlacementNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('hazard_marker_placement')

        # Subscribe to the laser scan topic
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # Create a publisher for the hazard marker pose
        self.marker_pub = rospy.Publisher('/hazard_marker_pose', PoseStamped, queue_size=10)

        # Wait for the transform between the laser and base_link frames to become available
        self.listener = tf.TransformListener()
        self.listener.waitForTransform('/base_link', '/laser', rospy.Time(), rospy.Duration(5.0))

    def scan_callback(self, scan):
        # Find the index of the minimum range in the laser scan data
        min_range_index = scan.ranges.index(min(scan.ranges))

        # Compute the angle of the laser that corresponds to the minimum range
        angle_min = scan.angle_min + min_range_index * scan.angle_increment

        # Compute the distance to the hazard marker using trigonometry
        distance = scan.ranges[min_range_index] * math.cos(angle_min)

        # Get the current pose of the robot in the map frame
        try:
            (trans, rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr('Failed to get transform from map to base_link')
            return

        # Compute the position of the hazard marker in the map frame
        marker_pose = PoseStamped()
        marker_pose.header.stamp = rospy.Time.now()
        marker_pose.header.frame_id = 'map'
        marker_pose.pose.position.x = trans[0] + distance * math.cos(rot[2])
        marker_pose.pose.position.y = trans[1] + distance * math.sin(rot[2])
        marker_pose.pose.orientation.w = 1.0

        # Publish the hazard marker pose
        self.marker_pub.publish(marker_pose)

if __name__ == '__main__':
    node = HazardMarkerPlacementNode()
    rospy.spin()
