// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_Lidar_hpp
#define msr_airlib_Lidar_hpp

#include <random>
#include "common/Common.hpp"
#include "LidarSimpleParams.hpp"
#include "LidarBase.hpp"
#include "common/DelayLine.hpp"
#include "common/FrequencyLimiter.hpp"

namespace msr { namespace airlib {

class LidarSimple : public LidarBase {
public:
    LidarSimple(const AirSimSettings::LidarSetting& setting = AirSimSettings::LidarSetting())
        : LidarBase(setting.sensor_name)
    {
        // initialize params
        params_.initializeFromSettings(setting);

		//initialize frequency limiter
		freq_limiter_.initialize(params_.update_frequency, params_.startup_delay);
		start_time_ = clock()->nowNanos();

		// Get channel params
		channels_per_scan_ = params_.number_of_channels;
		rotation_rate_ = params_.horizontal_rotation_frequency;
		int32 points_per_second = params_.points_per_second;

		// Work out the scans per revolution
		channels_per_scan_ = (channels_per_scan_ > 0) ? channels_per_scan_ : 1;
		points_per_second = (points_per_second > 0) ? points_per_second : 1;
		rotation_rate_ = (rotation_rate_ > 0) ? rotation_rate_ : 1.0f;
		scans_per_revolution_ = static_cast<int32>(FMath::RoundHalfFromZero(static_cast<double>(points_per_second) /
			(rotation_rate_ * static_cast<double>(channels_per_scan_))));
		scans_per_revolution_ = (scans_per_revolution_ > 0) ? scans_per_revolution_ : 1;
    }

    //*** Start: UpdatableState implementation ***//
    virtual void reset() override
    {
        LidarBase::reset();

        freq_limiter_.reset();
		start_time_ = clock()->nowNanos();

        updateOutput();
    }

    virtual void update() override
    {
        LidarBase::update();

        freq_limiter_.update();

        if (freq_limiter_.isWaitComplete()) {
            updateOutput();
        }
    }

    virtual void reportState(StateReporter& reporter) override
    {
        //call base
        LidarBase::reportState(reporter);

        reporter.writeValue("Lidar-NumChannels", params_.number_of_channels);
        reporter.writeValue("Lidar-Range", params_.range);
        reporter.writeValue("Lidar-FOV-Upper", params_.vertical_FOV_upper);
        reporter.writeValue("Lidar-FOV-Lower", params_.vertical_FOV_lower);
    }
    //*** End: UpdatableState implementation ***//

    virtual ~LidarSimple() = default;

    const LidarSimpleParams& getParams() const
    {
        return params_;
    }

protected:
    virtual void getPointCloud(const Pose& lidar_pose, const Pose& vehicle_pose, 
        TTimeDelta delta_time, vector<real_T>& point_cloud) = 0;

    
private: //methods
	void updateOutput()
	{
		TTimePoint updateTime = clock()->nowNanos();
        const GroundTruth& ground_truth = getGroundTruth();
		vector<real_T> point_cloud;
        // calculate the pose before obtaining the point-cloud. Before/after is a bit arbitrary
        // decision here. If the pose can change while obtaining the point-cloud (could happen for drones)
        // then the pose won't be very accurate either way.
        //
        // TODO: Seems like pose is in vehicle inertial-frame (NOT in Global NED frame).
        //    That could be a bit unintuitive but seems consistent with the position/orientation returned as part of 
        //    ImageResponse for cameras and pose returned by getCameraInfo API.
        //    Do we need to convert pose to Global NED frame before returning to clients?
        getPointCloud(params_.relative_pose, // relative lidar pose
            ground_truth.kinematics->pose,   // relative vehicle pose
			updateTime,
			point_cloud);

		Pose lidar_pose = params_.relative_pose + ground_truth.kinematics->pose;
		
        setOutput(updateTime, lidar_pose, point_cloud);
		last_time_ = updateTime;
    }

protected:
	LidarSimpleParams params_;
	TTimePoint last_time_;
	TTimePoint start_time_;
	int32 channels_per_scan_ = 0;
	int32 scans_per_revolution_ = 0;
	double rotation_rate_ = 0.0f;

private:
	FrequencyLimiter freq_limiter_;
};

}} //namespace
#endif
