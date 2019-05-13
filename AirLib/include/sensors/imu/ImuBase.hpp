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
        Quaternionr orientation;
		Vector3r angular_velocity;
        Vector3r linear_acceleration;
		uint64_t time_stamp;
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

	const IMUAPIData& getAPIOutput() 
	{
		// Lock the mutex to prevent buffer update during call
		std::lock_guard<std::mutex> output_lock(APIoutput_mutex_);

		// Copy the buffer into the output (which is passed by reference - hence we need a copy)
		APIoutput_.time_stamps = APIoutput_buffer_.time_stamps;
		APIoutput_.orientation = APIoutput_buffer_.orientation;
		APIoutput_.angular_velocity = APIoutput_buffer_.angular_velocity;
		APIoutput_.linear_acceleration = APIoutput_buffer_.linear_acceleration;

		// Clear the buffer
		APIoutput_buffer_.time_stamps.clear();
		APIoutput_buffer_.orientation.clear();
		APIoutput_buffer_.angular_velocity.clear();
		APIoutput_buffer_.linear_acceleration.clear();

		return APIoutput_;
	}

protected:
    void setOutput(const Output& output)
    {
		output_ = output;

		// Lock the mutex to prevent buffer update during call
		std::lock_guard<std::mutex> output_lock(APIoutput_mutex_);

		APIoutput_buffer_.time_stamps.push_back(output.time_stamp);
		APIoutput_buffer_.orientation.push_back(output.orientation.w());
		APIoutput_buffer_.orientation.push_back(output.orientation.x());
		APIoutput_buffer_.orientation.push_back(output.orientation.y());
		APIoutput_buffer_.orientation.push_back(output.orientation.z());
		APIoutput_buffer_.angular_velocity.push_back(output.angular_velocity.x());
		APIoutput_buffer_.angular_velocity.push_back(output.angular_velocity.y());
		APIoutput_buffer_.angular_velocity.push_back(output.angular_velocity.z());
		APIoutput_buffer_.linear_acceleration.push_back(output.linear_acceleration.x());
		APIoutput_buffer_.linear_acceleration.push_back(output.linear_acceleration.y());
		APIoutput_buffer_.linear_acceleration.push_back(output.linear_acceleration.z());

	}


private: 
    Output output_;
	IMUAPIData APIoutput_;
	IMUAPIData APIoutput_buffer_;
	std::mutex APIoutput_mutex_;

};


}} //namespace
#endif 
