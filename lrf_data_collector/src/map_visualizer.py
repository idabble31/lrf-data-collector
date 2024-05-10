#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Use the Agg backend to run matplotlib in the main thread
import matplotlib.pyplot as plt
import os

class MapVisualizer:
    def __init__(self):
        rospy.init_node('map_visualizer')
        self.map_data = None
        self.map_resolution = None
        self.map_origin = None
        self.map_width = None
        self.map_height = None
        self.map_save_path = '/home/robotom/ros_motion_planning/src/lrf_data_collector/data/map_visualization.png'
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

    def map_callback(self, map_msg):
        self.map_data = map_msg.data
        self.map_resolution = map_msg.info.resolution
        self.map_origin = map_msg.info.origin.position
        self.map_width = map_msg.info.width
        self.map_height = map_msg.info.height
        rospy.loginfo("Received map data")

        self.display_map()

    def display_map(self):
        if self.map_data and self.map_resolution and self.map_origin and self.map_width and self.map_height:
            # Convert occupancy grid data to numpy array
            map_array = np.array(self.map_data).reshape((self.map_height, self.map_width))

            # Flip the map array vertically to match matplotlib's coordinate system
            map_array = np.flipud(map_array)

            # Create grid of map coordinates
            x = np.linspace(self.map_origin.x, self.map_origin.x + self.map_width * self.map_resolution, self.map_width)
            y = np.linspace(self.map_origin.y, self.map_origin.y + self.map_height * self.map_resolution, self.map_height)

            # Plot the map using matplotlib with the "viridis" colormap
            plt.figure(figsize=(10, 10))
            plt.imshow(map_array, cmap='viridis', extent=[x[0], x[-1], y[0], y[-1]])
            plt.title('Map for Navigation')
            plt.xlabel('X (meters)')
            plt.ylabel('Y (meters)')
            plt.grid(True)
            plt.colorbar(label='Occupancy Probability')

            # Save the plot to the specified location
            plt.savefig(self.map_save_path)
            rospy.loginfo(f"Map visualization saved to: {self.map_save_path}")
        else:
            rospy.logwarn("Map data not received yet!")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    map_visualizer = MapVisualizer()
    map_visualizer.run()
