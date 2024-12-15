#!/usr/bin/env python3
from breezyslam.algorithms import Deterministic_SLAM, RMHC_SLAM

from lib.BreezySLAM.examples.mines import MinesLaser, Rover, load_data
from lib.BreezySLAM.examples.progressbar import ProgressBar
from lib.BreezySLAM.examples.pgm_utils import pgm_save

from sys import argv, exit, stdout
from module.Lidar_LM1 import Lidar_LM1
from function import *

#import time as time_extension
import threading
import time

# Map size, scale
MAP_SIZE_PIXELS          = 800
MAP_SIZE_METERS          =  32

# class MinesRPLidarA1 : Laser
# {
#     public:
        
#         MinesRPLidarA1(int detection_margin = 0, float offset_mm = 0) :
#         Laser(SCAN_SIZE, 5.0f, 360, 1200, detection_margin, offset_mm)
#         {
#         }
        
#         MinesRPLidarA1(void) : Laser() {}
#     };

lidar_LM1 = Lidar_LM1()

def main():

    lidar_thread = threading.Thread(target=lidar_reading, args=(lidar_LM1,))
    lidar_thread.start()

    #Bozo filter for input args
    # if len(argv) < 3:
    #     print('Usage:   %s <dataset> <use_odometry> [random_seed]' % argv[0])
    #     print('Example: %s exp2 1 9999' % argv[0])
    #     exit(1)
    
    # Grab input args
    # dataset = argv[1]
    use_odometry  =  False #if int(argv[2]) else False

    # const ensure repeatability of results
    seed = 1234 #int(argv[3]) if len(argv) > 3 else 0
    
	# Load the data from the file, ignoring timestamps
    #_  lidars, odometries = load_data('.', dataset)
    
    missing_cloud = lidar_LM1.get_slam_cloud()

    while not missing_cloud:
        time.sleep(2)
        missing_cloud = lidar_LM1.get_slam_cloud()

    sorted_missing_cloud = sorted(missing_cloud, key=lambda x: x[1])

    # for point in sorted_missing_cloud:
    #    print(f"Distance of miss: {point[0]} mm, Angle: {point[1]} degrees")

    lidar_LM1.clear_slam_cloud()
    odometries = []
    odometries.append([212387282, 29221, 25178])

    #interpolation
    complete_cloud = interpolate_missing_angles(sorted_missing_cloud)

    # for distance, angle in complete_cloud:
    #     print(f"Angle: {angle}°, Distance: {distance} mm")

    lidars = filter_distances(complete_cloud)

    for n in lidars:
        print(f"{n}\n")
    # Build a robot model if we want odometry
    robot = Rover() if use_odometry else None
        
    # Create a CoreSLAM object with laser params and optional robot object
    slam = RMHC_SLAM(MinesLaser(), MAP_SIZE_PIXELS, MAP_SIZE_METERS, random_seed=seed) \
           if seed \
           else Deterministic_SLAM(MinesLaser(), MAP_SIZE_PIXELS, MAP_SIZE_METERS)
           
    # Report what we're doing
    nscans = len(lidars)
    print('Processing %d scans with%s odometry / with%s particle filter...' % \
        (nscans, \
         '' if use_odometry else 'out', '' if seed else 'out'))
    progbar = ProgressBar(0, nscans, 80)
    
    # Start with an empty trajectory of positions
    trajectory = []

    # Start timing
    start_sec = time.time()
    
    # Loop over scans    
    for scanno in range(nscans):
    
        if use_odometry:
                  
            # Convert odometry to pose change (dxy_mm, dtheta_degrees, dt_seconds)
            velocities = robot.computePoseChange(odometries[scanno])
                                 
            # Update SLAM with lidar and velocities
            slam.update(lidars[scanno], velocities)
            
        else:
            
            print(type(lidars))
            print("\n\n")


            # Update SLAM with lidar alone
            slam.update(lidars)
            #slam.update(lidars[scanno])

        # Get new position
        x_mm, y_mm, theta_degrees = slam.getpos()    
        
        # Add new position to trajectory
        trajectory.append((x_mm, y_mm))
        
        # Tame impatience
        progbar.updateAmount(scanno)
        stdout.write('\r%s' % str(progbar))
        stdout.flush()

    # Report elapsed time
    elapsed_sec = time.time() - start_sec
    print('\n%d scans in %f sec = %f scans / sec' % (nscans, elapsed_sec, nscans/elapsed_sec))
                    
                                
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
                    
    # Save map and trajectory as PGM file    
    pgm_save('%s.pgm' % "output1", mapbytes, (MAP_SIZE_PIXELS, MAP_SIZE_PIXELS))
            
# Helpers ---------------------------------------------------------        

def mm2pix(mm):
        
    return int(mm / (MAP_SIZE_METERS * 1000. / MAP_SIZE_PIXELS))  
    
                    
main()
