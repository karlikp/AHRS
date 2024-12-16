#!/usr/bin/env python3
from lib.BreezySLAM.python.breezyslam.algorithms import Deterministic_SLAM, RMHC_SLAM

from lib.BreezySLAM.examples.mines import MinesLaser, Rover, load_data
from lib.BreezySLAM.examples.progressbar import ProgressBar
from lib.BreezySLAM.examples.pgm_utils import pgm_save

from sys import argv, exit, stdout
from module.Lidar_LM1 import Lidar_LM1
from function import *

#import time as time_extension
import cv2
import threading
import time
import numpy as np

# Map size, scale
MAP_SIZE_PIXELS          = 800
MAP_SIZE_METERS          =  32

lidar_LM1 = Lidar_LM1()

def main():

    lidar_thread = threading.Thread(target=lidar_reading, args=(lidar_LM1,))
    lidar_thread.start()

    use_odometry  =  False #if int(argv[2]) else False

    # const ensure repeatability of results
    seed = 1234 #int(argv[3]) if len(argv) > 3 else 0

    odometries = []
    odometries.append([212387282, 29221, 25178])

    # Build a robot model if we want odometry
    robot = Rover() if use_odometry else None
        
    # Create a CoreSLAM object with laser params and optional robot object
    slam = RMHC_SLAM(MinesLaser(), MAP_SIZE_PIXELS, MAP_SIZE_METERS, random_seed=seed) \
           if seed \
           else Deterministic_SLAM(MinesLaser(), MAP_SIZE_PIXELS, MAP_SIZE_METERS)
    
    # Start with an empty trajectory of positions
    trajectory = []

    # Start timing
    start_sec = time.time()

    # Loop over scans
    while True:  # Keep scanning until you decide to stop
        # Get lidar data (simulated by lidar_LM1)
        time.sleep(2)
        missing_cloud = lidar_LM1.get_slam_cloud()

        # Handle empty data
        while not missing_cloud:
            time.sleep(2)
            missing_cloud = lidar_LM1.get_slam_cloud()
        
        lidar_LM1.clear_slam_cloud()

        sorted_missing_cloud = sorted(missing_cloud, key=lambda x: x[1])

        if not sorted_missing_cloud:
            print("empty sorted list")
        
        lidars = filter_distances(interpolate_missing_angles(sorted_missing_cloud))


        #for scanno, scan in enumerate(lidars):
            # Use odometry if specified
        if use_odometry:
            velocities = robot.computePoseChange([212387282, 29221, 25178])
            slam.update(lidars, velocities)
        else:
            slam.update(lidars)

            # Get new position
            x_mm, y_mm, theta_degrees = slam.getpos()    
            
            # Add new position to trajectory
            trajectory.append((x_mm, y_mm))
                  
        # Create a byte array to receive the computed maps
        mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)
        
        # Get final map    
        slam.getmap(mapbytes)
        
        # Put trajectory into map as black pixels
        for coords in trajectory:
                    
            x_mm, y_mm = coords
                                
            x_pix = mm2pix(x_mm)
            y_pix = mm2pix(y_mm)
                                                                                                
            mapbytes[y_pix * MAP_SIZE_PIXELS + x_pix] = 0

        # Convert the byte array into a NumPy array
        map_array = np.array(mapbytes, dtype=np.uint8).reshape(MAP_SIZE_PIXELS, MAP_SIZE_PIXELS)

        # Display the map using OpenCV
        cv2.imshow("SLAM Map", map_array)
        cv2.waitKey(1)  # Wait a short time to update the display

        # # Optional: Save the map every few seconds (for debugging purposes)
        # if scanno % 10 == 0:
        #     pgm_save(f'output_{scanno}.pgm', mapbytes, (MAP_SIZE_PIXELS, MAP_SIZE_PIXELS))

        # # Break if needed (you can add a stopping condition)
        # if scanno > 100:  # Just an example condition for stopping
        #     break

    # After loop ends, save the final map
    pgm_save('final_output.pgm', mapbytes, (MAP_SIZE_PIXELS, MAP_SIZE_PIXELS))

                        
        # # Save map and trajectory as PGM file    
        # pgm_save('%s.pgm' % "output1", mapbytes, (MAP_SIZE_PIXELS, MAP_SIZE_PIXELS))
            
# Helpers ---------------------------------------------------------        

def mm2pix(mm):
        
    return int(mm / (MAP_SIZE_METERS * 1000. / MAP_SIZE_PIXELS))  
    
                    
# Start the main function
if __name__ == "__main__":
    main()
