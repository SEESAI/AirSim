# Python client example to get Lidar data from a drone
#

import setup_path 
import airsim

import sys
import math
import time
import argparse
import pprint
import numpy

# Makes the drone fly and get Lidar data
class LidarTest:

    def __init__(self):

        # connect to the AirSim simulator
        self.client = airsim.MultirotorClient(ip="192.168.12.14")
        self.client.confirmConnection()
        self.client.enableApiControl(True)

    def execute(self):
        '''
        COMMENTED THIS OUT TO MAKE TESTING FASTER

        print("arming the drone...")
        self.client.armDisarm(True)

        state = self.client.getMultirotorState()
        s = pprint.pformat(state)
        #print("state: %s" % s)

        airsim.wait_key('Press any key to takeoff')
        self.client.takeoffAsync().join()

        state = self.client.getMultirotorState()
        #print("state: %s" % pprint.pformat(state))

        airsim.wait_key('Press any key to move vehicle to (-10, 10, -10) at 5 m/s')
        self.client.moveToPositionAsync(-10, 10, -10, 5).join()

        self.client.hoverAsync().join()

        airsim.wait_key('Press any key to get Lidar readings')
        '''


        lidarInfo = self.client.getLidarInfo()
        '''
        Lidar info 
            pose = Pose()
            vertical_fov_lower = 0.0
            vertical_fov_upper = 0.0
            horizontal_fov_lower = 0.0
            horizontal_fov_upper = 0.0
            channels_per_scan = 0.0
            scans_per_revolution = 0.0
            revolutions_per_second = 0.0
        '''


        for i in range(1,5):
            lidarData = self.client.getLidarDataBuffer()

            '''
            LidarDataBuffer
                sensor_pose_in_world_frame = Pose()
                timestamps_ns = np.uint64(0)
                azimuth_angles = Vector3r()
                ranges = Vector3r()
                
            ranges are provided as channels_per_scan measurements from vertical_fov_upper to vertical_fov_lower per each azimuth angle.                
            '''
            if (len(lidarData.timestamps_ns) < 3):
                print("\tNo points received from Lidar data")
            else:
                print("\tReading %d: number_of_scans: %d" % (i, len(lidarData.timestamps_ns)))
                print("\t\tlidar position: %s" % (pprint.pformat(lidarData.sensor_pose_in_world_frame.position)))
                print("\t\tlidar orientation: %s" % (pprint.pformat(lidarData.sensor_pose_in_world_frame.orientation)))
            time.sleep(0.1)


    def stop(self):

        airsim.wait_key('Press any key to reset to original state')

        self.client.armDisarm(False)
        self.client.reset()

        self.client.enableApiControl(False)
        print("Done!\n")

# main
if __name__ == "__main__":
    args = sys.argv
    args.pop(0)

    arg_parser = argparse.ArgumentParser("Lidar.py makes drone fly and gets Lidar data")

    arg_parser.add_argument('-save-to-disk', type=bool, help="save Lidar data to disk", default=False)
  
    args = arg_parser.parse_args(args)    
    lidarTest = LidarTest()
    try:
        lidarTest.execute()
    finally:
        lidarTest.stop()
