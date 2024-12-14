import time
import threading
import matplotlib
matplotlib.use('Agg')  # Switch to Agg backend (for non-interactive use)

from lib.BreezySLAM.python.breezyslam.algorithms import RMHC_SLAM
from lib.BreezySLAM.python.breezyslam.sensors import RPLidarA1 as LaserModel

from roboviz import MapVisualizer
from function import *
from module import *


lidar_LM1 = Lidar_LM1()

MAP_SIZE_PIXELS         = 500
MAP_SIZE_METERS         = 10
LIDAR_DEVICE            = '/dev/ttyUSB0'


# Ideally we could use all 250 or so samples that the RPLidar delivers in one 
# scan, but on slower computers you'll get an empty map and unchanging position
# at that rate.

MIN_SAMPLES   = 200

if __name__ == '__main__':

    # Connect to Lidar unit
    #lidar_thread = threading.Thread(target=lidar_reading(lidar_LM1))  
    lidar_thread = threading.Thread(target=lidar_reading, args=(lidar_LM1,))
    lidar_thread.start()
    #lidar_thread.join()

    # Create an RMHC SLAM object with a laser model and optional robot model
    slam = RMHC_SLAM(LaserModel(), MAP_SIZE_PIXELS, MAP_SIZE_METERS)

    # Set up a SLAM display
    viz = MapVisualizer(MAP_SIZE_PIXELS, MAP_SIZE_METERS, 'SLAM')

    # Initialize an empty trajectory
    trajectory = []

    # Initialize empty map
    mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)

    # Create an iterator to collect scan data from the RPLidar
    #iterator = lidar.iter_scans()

    # We will use these to store previous scan in case current scan is inadequate
    previous_distances = None
    previous_angles    = None

    # First scan is crap, so ignore it
    #next(iterator)

    while True:

        # Get the SLAM cloud (replaces lidar scan data)
        cloud_data = lidar_LM1.get_sorted_slam_cloud()

        if len(cloud_data) == 0:
            continue  # Skip if there's no data

        # Assuming each item in cloud_data is a (quality, angle, distance) triple
        # cloud_data is a list of (quality, angle, distance) triples

        items = [item for item in cloud_data]
        # Extract distances and angles from triples
        distances = [item[1] for item in items]
        angles    = [item[0] for item in items]

        # Update SLAM with current Lidar scan and scan angles if adequate
        if len(distances) > MIN_SAMPLES:
            slam.update(distances, scan_angles_degrees=angles)
            previous_distances = distances.copy()
            previous_angles    = angles.copy()

        # If not adequate, use previous
        elif previous_distances is not None:
            slam.update(previous_distances, scan_angles_degrees=previous_angles)

        # Get current robot position
        x, y, theta = slam.getpos()

        # Get current map bytes as grayscale
        slam.getmap(mapbytes)

        # Display map and robot pose, exiting gracefully if user closes it
        if not viz.display(x/1000., y/1000., theta, mapbytes):
            exit(0)
 
    # Shut down the lidar connection
    lidar_LM1.stop()
    lidar_LM1.disconnect()

   