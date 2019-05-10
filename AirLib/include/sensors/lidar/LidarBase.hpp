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

	const LidarAPIData & getAPIOutput()
	{
		// this call is used for the API

		// this differs from getOutput in that it returns all points since it was last called
		// or a full scan, whichever is smaller
		// whereas output_ is cleared every LIDAR update even if the API hasn't called

		// ToDo - mutex with writing the buffer

		// copy the buffer into the output
		std::lock_guard<std::mutex> APIoutput_mutex_(APIoutput_mutex_);

		APIoutput_.pose = APIoutput_buffer_.pose;
		APIoutput_.time_stamps = APIoutput_buffer_.time_stamps;
		APIoutput_.azimuth_angles = APIoutput_buffer_.azimuth_angles;
		APIoutput_.ranges = APIoutput_buffer_.ranges;
		
		// and clear the buffer
		APIoutput_buffer_.time_stamps.clear();
		APIoutput_buffer_.azimuth_angles.clear();
		APIoutput_buffer_.ranges.clear();

		return APIoutput_;
	}

	const LidarData & getOutput()
	{
		// this call is used for pretty green dots on the screen

		// copy the buffer to the output
		std::lock_guard<std::mutex> output_lock(output_mutex_);
		output_.pose = output_buffer_.pose;
		output_.time_stamp = output_buffer_.time_stamp;
		output_.point_cloud = output_buffer_.point_cloud;

		return output_;
	}


protected:
	void setAPIOutput(Pose & pose, std::vector<uint64_t> & ts, std::vector<msr::airlib::real_T> & az, 
		std::vector<msr::airlib::real_T> & r, int scans_per_revolution, int channels_per_scan)
	{
		// Append latest data to the buffer
		ts.insert(ts.begin(), APIoutput_buffer_.time_stamps.begin(), APIoutput_buffer_.time_stamps.end());
		az.insert(az.begin(), APIoutput_buffer_.azimuth_angles.begin(), APIoutput_buffer_.azimuth_angles.end());
		r.insert(r.begin(), APIoutput_buffer_.ranges.begin(), APIoutput_buffer_.ranges.end());

		// Trim to be one revolution
		int max_buffer_length = scans_per_revolution;
		if (ts.size() > max_buffer_length) {
			int excess_length = ts.size() - max_buffer_length;
			ts.erase(ts.begin(), ts.begin() + excess_length);
			az.erase(az.begin(), az.begin() + excess_length);
			r.erase(r.begin(), r.begin() + (excess_length * channels_per_scan));
		}

		// Update the buffer
		std::lock_guard<std::mutex> APIoutput_mutex_(APIoutput_mutex_);
		APIoutput_buffer_.pose = pose;
		APIoutput_buffer_.time_stamps = ts;
		APIoutput_buffer_.azimuth_angles = az;
		APIoutput_buffer_.ranges = r;

	}
    void setOutput(TTimePoint updateTime, Pose & lidar_pose, vector<real_T> & point_cloud) // was previously LidarData& output
    {
		// Update the buffer
		std::lock_guard<std::mutex> output_lock(output_mutex_);
		output_buffer_.pose = lidar_pose;
		output_buffer_.time_stamp = updateTime;
		output_buffer_.point_cloud = point_cloud;
    }

protected:

private:
	LidarAPIData APIoutput_buffer_;
	LidarAPIData APIoutput_;
	std::mutex APIoutput_mutex_;

	LidarData output_buffer_;
	LidarData output_;
	std::mutex output_mutex_;
};

}} //namespace
#endif
