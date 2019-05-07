// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_LidarBase_hpp
#define msr_airlib_LidarBase_hpp

#include "sensors/SensorBase.hpp"

namespace msr { namespace airlib {

class LidarBase : public SensorBase {
public:
    LidarBase(const std::string& sensor_name = "")
        : SensorBase(sensor_name)
    {}

public: //types
    struct Output { //fields to enable creation of ROS message PointCloud2 and LaserScan

        // header
        TTimePoint time_stamp;
        Pose relative_pose;

        // data
        // - array of floats that represent [x,y,z] coordinate for each point hit within the range
        //       x0, y0, z0, x1, y1, z1, ..., xn, yn, zn
        //       TODO: Do we need an intensity place-holder [x,y,z, intensity]?
        // - in lidar local NED coordinates
        // - in meters
        vector<real_T> point_cloud;
    };

public:
    virtual void reportState(StateReporter& reporter) override
    {
        //call base
        UpdatableObject::reportState(reporter);

        reporter.writeValue("Lidar-Timestamp", output_.time_stamp);
        reporter.writeValue("Lidar-NumPoints", static_cast<int>(output_.point_cloud.size() / 3));
    }

    const LidarData& getOutput() const
    {
		// this call is used for pretty green dots on the screen
        return output_;
    }

	const LidarAPIData& getAPIOutput()
	{
		// this call is used for the API

		// this differs from getOutput in that it returns all points since it was last called
		// or a full scan, whichever is smaller
		// whereas output_ is cleared every LIDAR update even if the API hasn't called

		// copy the buffer into the output
		APIoutput_.pose = scan_buffer_.pose;
		APIoutput_.time_stamps = scan_buffer_.time_stamps;
		APIoutput_.azimuth_angles = scan_buffer_.azimuth_angles;
		APIoutput_.ranges = scan_buffer_.ranges;
		
		// and clear the buffer
		scan_buffer_.time_stamps.clear();
		scan_buffer_.azimuth_angles.clear();
		scan_buffer_.ranges.clear();

		return APIoutput_;
	}

protected:
    void setOutput(TTimePoint updateTime, Pose & lidar_pose, vector<real_T> & point_cloud) // was previously LidarData& output
    {
		output_.pose = lidar_pose;
		output_.time_stamp = updateTime;
		output_.point_cloud = point_cloud;
    }

protected:
	LidarAPIData scan_buffer_;

private:
	LidarAPIData APIoutput_;
	LidarData output_;
};

}} //namespace
#endif
