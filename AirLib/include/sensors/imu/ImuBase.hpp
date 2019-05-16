// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_ImuBase_hpp
#define msr_airlib_ImuBase_hpp


#include "sensors/SensorBase.hpp"


namespace msr { namespace airlib {

class ImuBase  : public SensorBase {
public:
    ImuBase(const std::string& sensor_name = "")
        : SensorBase(sensor_name)
    {}

public: //types
    struct Output {	//structure is same as ROS IMU message
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        TTimePoint time_stamp; 
        Quaternionr orientation;
		Vector3r angular_velocity;
        Vector3r linear_acceleration;
	};


public:
    virtual void reportState(StateReporter& reporter) override
    {
        //call base
        UpdatableObject::reportState(reporter);

        reporter.writeValue("IMU-Ang", output_.angular_velocity);
        reporter.writeValue("IMU-Lin", output_.linear_acceleration);
    }

    const Output& getOutput() const
    {
        return output_;
    }

	const IMUDataBuffer& getOutputBuffer() 
	{
		// Lock the mutex to prevent buffer update during call
		std::lock_guard<std::mutex> output_lock(APIoutput_mutex_);

		// Copy the buffer into the output (which is passed by reference - hence we need a copy)
		output_buffer_.time_stamps = output_buffer_internal_.time_stamps;
		output_buffer_.orientation = output_buffer_internal_.orientation;
		output_buffer_.angular_velocity = output_buffer_internal_.angular_velocity;
		output_buffer_.linear_acceleration = output_buffer_internal_.linear_acceleration;

		// Clear the buffer
		output_buffer_internal_.time_stamps.clear();
		output_buffer_internal_.orientation.clear();
		output_buffer_internal_.angular_velocity.clear();
		output_buffer_internal_.linear_acceleration.clear();

		return output_buffer_;
	}

protected:
    void setOutput(const Output& output)
    {
		output_ = output;

		// Lock the mutex to prevent buffer update during call
		std::lock_guard<std::mutex> output_lock(APIoutput_mutex_);

		output_buffer_internal_.time_stamps.push_back(output.time_stamp);
		output_buffer_internal_.orientation.push_back(output.orientation.w());
		output_buffer_internal_.orientation.push_back(output.orientation.x());
		output_buffer_internal_.orientation.push_back(output.orientation.y());
		output_buffer_internal_.orientation.push_back(output.orientation.z());
		output_buffer_internal_.angular_velocity.push_back(output.angular_velocity.x());
		output_buffer_internal_.angular_velocity.push_back(output.angular_velocity.y());
		output_buffer_internal_.angular_velocity.push_back(output.angular_velocity.z());
		output_buffer_internal_.linear_acceleration.push_back(output.linear_acceleration.x());
		output_buffer_internal_.linear_acceleration.push_back(output.linear_acceleration.y());
		output_buffer_internal_.linear_acceleration.push_back(output.linear_acceleration.z());

		// Trim to be a sensible size
		unsigned max_buffer_length = 1000; // Hard coded for now - may wish to make this 1s of data
		if (output_buffer_internal_.time_stamps.size() > max_buffer_length) {
			output_buffer_internal_.time_stamps.erase(output_buffer_internal_.time_stamps.begin(), 
				output_buffer_internal_.time_stamps.end() - max_buffer_length);
			output_buffer_internal_.orientation.erase(output_buffer_internal_.orientation.begin(),
				output_buffer_internal_.orientation.end() - max_buffer_length *4);
			output_buffer_internal_.angular_velocity.erase(output_buffer_internal_.angular_velocity.begin(),
				output_buffer_internal_.angular_velocity.end() - max_buffer_length * 3);
			output_buffer_internal_.linear_acceleration.erase(output_buffer_internal_.linear_acceleration.begin(),
				output_buffer_internal_.linear_acceleration.end() - max_buffer_length * 3);

		}
	}


private: 
    Output output_;
	IMUDataBuffer output_buffer_;
	IMUDataBuffer output_buffer_internal_;
	std::mutex APIoutput_mutex_;

};


}} //namespace
#endif 
