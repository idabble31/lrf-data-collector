#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid
import plotly.graph_objects as go
from plotly.subplots import make_subplots

class MapPlotterNode:
    def __init__(self):
        rospy.init_node('map_plotter_node', anonymous=True)
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.fig = make_subplots(rows=1, cols=1)
        self.map_data = None

    def map_callback(self, msg):
        rospy.loginfo("Received map data")
        self.map_data = msg
        self.plot_map()

    def plot_map(self):
        if self.map_data is None:
            rospy.logwarn("Map data is not available.")
            return

        # Extract map metadata
        width = self.map_data.info.width
        height = self.map_data.info.height
        resolution = self.map_data.info.resolution
        origin_x = self.map_data.info.origin.position.x
        origin_y = self.map_data.info.origin.position.y

        # Extract grid data
        grid = self.map_data.data

        # Convert map data to a 2D grid
        grid = [grid[i:i+width] for i in range(0, width * height, width)]

        # Plot the map using Plotly
        self.fig.data = []  # Clear previous data
        self.fig.add_trace(go.Heatmap(z=grid, colorscale='Viridis'))

        # Update layout and display the plot
        self.fig.update_layout(
            title="Occupancy Grid Map",
            xaxis=dict(
                title='X (meters)',
                tickvals=list(range(width)),
                ticktext=[round(origin_x + i * resolution, 2) for i in range(width)]
            ),
            yaxis=dict(
                title='Y (meters)',
                tickvals=list(range(height)),
                ticktext=[round(origin_y + i * resolution, 2) for i in range(height)]
            )
        )
        self.fig.show()

if __name__ == '__main__':
    try:
        map_plotter = MapPlotterNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
