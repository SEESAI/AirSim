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

    const LidarData& getOutput()
    {
		// this call is used for pretty green dots on the screen

		// copy the buffer to the output
		std::lock_guard<std::mutex> output_lock(output_mutex_);
		output_.pose = output_internal_.pose;
		output_.time_stamp = output_internal_.time_stamp;
		output_.point_cloud = output_internal_.point_cloud;

		return output_;
    }

	const LidarDataBuffer & getOutputBuffer()
	{
		// this call is used for the API

		// this differs from getOutput in that it returns all points since it was last called
		// or a full scan, whichever is smaller
		// whereas output_ is cleared every LIDAR update even if the API hasn't called

		// copy the buffer into the output
		std::lock_guard<std::mutex> APIoutput_lock(APIoutput_mutex_);

		output_buffer_.sensor_pose_in_world_frame = output_buffer_internal_.sensor_pose_in_world_frame;
		output_buffer_.timestamps_ns = output_buffer_internal_.timestamps_ns;
		output_buffer_.azimuth_angles = output_buffer_internal_.azimuth_angles;
		output_buffer_.ranges = output_buffer_internal_.ranges;
		
		// and clear the buffer
		output_buffer_internal_.timestamps_ns.clear();
		output_buffer_internal_.azimuth_angles.clear();
		output_buffer_internal_.ranges.clear();

		return output_buffer_;
	}

    const LidarInfo & getInfo()
	{
		// return the lidar general info
		
		return lidar_info_;
	}

    const vector<int>& getSegmentationOutput() const
    {
        return segmentation_output_;
    }

protected:

    void setOutput(TTimePoint updateTime, Pose & lidar_pose, vector<real_T> & point_cloud) // was previously LidarData& output
    {
		// Update the buffer
		std::lock_guard<std::mutex> output_lock(output_mutex_);
		output_internal_.pose = lidar_pose;
		output_internal_.time_stamp = updateTime;
		output_internal_.point_cloud = point_cloud;
    }

	void setOutputBuffer(Pose & pose, std::vector<uint64_t> & ts, std::vector<msr::airlib::real_T> & az, 
		std::vector<msr::airlib::real_T> & r, int scans_per_revolution, int channels_per_scan)
	{
		// Append latest data to the buffer
		std::lock_guard<std::mutex> APIoutput_lock(APIoutput_mutex_);
		ts.insert(ts.begin(), output_buffer_internal_.timestamps_ns.begin(), output_buffer_internal_.timestamps_ns.end());
		az.insert(az.begin(), output_buffer_internal_.azimuth_angles.begin(), output_buffer_internal_.azimuth_angles.end());
		r.insert(r.begin(), output_buffer_internal_.ranges.begin(), output_buffer_internal_.ranges.end());

		// Trim to be two revolutions max
		unsigned max_buffer_length = 2 * scans_per_revolution;
		if (ts.size() > max_buffer_length) {
			size_t excess_length = ts.size() - max_buffer_length;
			ts.erase(ts.begin(), ts.begin() + excess_length);
			az.erase(az.begin(), az.begin() + excess_length);
			r.erase(r.begin(), r.begin() + (excess_length * channels_per_scan));
		}

		// Update the buffer
		output_buffer_internal_.sensor_pose_in_world_frame = pose;
		output_buffer_internal_.timestamps_ns = ts;
		output_buffer_internal_.azimuth_angles = az;
		output_buffer_internal_.ranges = r;
	}

    void setSegmentationOutput(vector<int>& segmentation_output)
    {
        segmentation_output_ = segmentation_output;
    }

protected:
	LidarInfo lidar_info_;

private:
	LidarDataBuffer output_buffer_internal_;
	LidarDataBuffer output_buffer_;
	std::mutex APIoutput_mutex_;

	LidarData output_internal_;
	LidarData output_;
	std::mutex output_mutex_;

    //Todo: add buffer for segmentation
    vector<int> segmentation_output_;
};

}} //namespace
#endif
