#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32

class PathLengthCalculator:
    def __init__(self):
        rospy.init_node('path_length_calculator')
        self.path_length_pub = rospy.Publisher('/path_length', Float32, queue_size=10)
        rospy.Subscriber('/move_base/SamplePlanner/plan', Path, self.path_callback)
        self.first_path_processed = False

    def path_callback(self, path_msg):
        if not self.first_path_processed:
            self.first_path_processed = True
            path_length = 0.0
            prev_pose = None

            for pose_stamped in path_msg.poses:
                if prev_pose is not None:
                    # Calculate Euclidean distance between consecutive poses
                    dx = pose_stamped.pose.position.x - prev_pose.pose.position.x
                    dy = pose_stamped.pose.position.y - prev_pose.pose.position.y
                    dz = pose_stamped.pose.position.z - prev_pose.pose.position.z
                    segment_length = (dx ** 2 + dy ** 2 + dz ** 2) ** 0.5
                    path_length += segment_length

                prev_pose = pose_stamped

            rospy.loginfo("Generated path length: {:.2f} meters".format(path_length))
            self.path_length_pub.publish(path_length)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    path_calculator = PathLengthCalculator()
    path_calculator.run()
    