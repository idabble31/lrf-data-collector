#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt

class OdometryPlotter:
    def __init__(self):
        rospy.init_node('odometry_plotter_node', anonymous=True)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.timestamps = []
        self.x_positions = []
        self.y_positions = []
        self.yaw_angles = []

    def odom_callback(self, msg):
        timestamp = msg.header.stamp.to_sec()
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
        
        self.timestamps.append(timestamp)
        self.x_positions.append(x)
        self.y_positions.append(y)
        self.yaw_angles.append(yaw)

    def plot_odometry(self):
        plt.figure(figsize=(10, 6))
        plt.plot(self.timestamps, self.x_positions, label='X Position (m)')
        plt.plot(self.timestamps, self.y_positions, label='Y Position (m)')
        plt.plot(self.timestamps, self.yaw_angles, label='Yaw Angle (rad)')
        plt.xlabel('Time (s)')
        plt.ylabel('Value')
        plt.title('Odometry Data Plot')
        plt.legend()
        plt.grid(True)
        plt.show()

if __name__ == '__main__':
    try:
        odometry_plotter = OdometryPlotter()
        rospy.spin()
        odometry_plotter.plot_odometry()
    except rospy.ROSInterruptException:
        pass
