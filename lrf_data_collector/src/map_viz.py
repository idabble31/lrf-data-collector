#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid
import matplotlib.pyplot as plt

# Custom function to convert OccupancyGrid to a format suitable for plotting
def occupancy_grid_to_plot_data(occupancy_grid):
    """
    Converts an OccupancyGrid message to a format suitable for plotting.

    Args:
        occupancy_grid (OccupancyGrid): The OccupancyGrid message.

    Returns:
        list: A list of lists, where each inner list represents a row of the plot data.
    """
    plot_data = []
    for y in range(occupancy_grid.info.height):
        row_data = []
        for x in range(occupancy_grid.info.width):
            # Get the index into the data array
            index = y * occupancy_grid.info.width + x
            value = occupancy_grid.data[index]

            # Convert value to a plotting representation (e.g., 0 for free space, 1 for obstacle)
            if value == 0:
                row_data.append(0)  # Free space
            else:
                row_data.append(1)  # Obstacle
        plot_data.append(row_data)
    return plot_data

class MapPlotterNode:

    def __init__(self):
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.map_data = None
        self.map_msg = None  # Store the received map message

    def map_callback(self, map_msg):
        self.map_msg = map_msg  # Store the map message
        self.map_data = occupancy_grid_to_plot_data(map_msg)
        self.plot_map()

    def plot_map(self):
        if self.map_data is None or self.map_msg is None:
            return

        # Convert map data to numpy array for plotting
        map_array = [[0 if val == 0 else 1 for val in row] for row in self.map_data]
        map_array = list(reversed(map_array))  # Flip array vertically for correct orientation

        # Plot the map using Matplotlib
        plt.figure(figsize=(10, 10))
        plt.imshow(map_array, cmap='binary', origin='lower', extent=[0, self.map_msg.info.resolution * self.map_msg.info.width, 0, self.map_msg.info.resolution * self.map_msg.info.height])
        plt.title('Occupancy Grid Map')
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.colorbar(label='Occupancy Probability')
        plt.savefig('/home/robotom/ros_motion_planning/src/lrf_data_collector/data/map_plot.png')  # Save the plot as an image
        rospy.loginfo("Map plot saved as 'map_plot.png'")

if __name__ == '__main__':
    rospy.init_node('map_plotter_node')
    map_plotter = MapPlotterNode()
    rospy.spin()
