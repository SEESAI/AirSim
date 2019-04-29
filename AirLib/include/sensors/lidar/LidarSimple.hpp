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

		//reset the LIDAR rotation calculations
		start_time_ = clock()->nowNanos();
		previous_sectors_since_start_ = 0;

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

		// Save the info in case asked for it
		lidar_info_.vertical_fov_lower = params_.vertical_FOV_lower;
		lidar_info_.vertical_fov_upper = params_.vertical_FOV_upper;
		lidar_info_.horizontal_fov_lower = params_.horizontal_FOV_start;
		lidar_info_.horizontal_fov_upper = params_.horizontal_FOV_end;
		lidar_info_.channels_per_scan = channels_per_scan_;
		lidar_info_.revolutions_per_second = rotation_rate_;
		lidar_info_.scans_per_revolution = scans_per_revolution_;
		lidar_info_.pose = params_.relative_pose;
    }

    //*** Start: UpdatableState implementation ***//
    virtual void reset() override
    {
        LidarBase::reset();

        freq_limiter_.reset();
		start_time_ = clock()->nowNanos();
		previous_sectors_since_start_ = 0;
		gauss_dist_.reset();
		uniform_dist_.reset();

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
        TTimeDelta delta_time, vector<real_T>& point_cloud, std::vector<uint64_t> & ts, 
		std::vector<msr::airlib::real_T> & az, std::vector<msr::airlib::real_T> & r) = 0;

    
private: //methods
	void updateOutput()
	{
        // calculate the pose before obtaining the point-cloud. Before/after is a bit arbitrary
        // decision here. If the pose can change while obtaining the point-cloud (could happen for drones)
        // then the pose won't be very accurate either way.
        //
        // TODO: Seems like pose is in vehicle inertial-frame (NOT in Global NED frame).
        //    That could be a bit unintuitive but seems consistent with the position/orientation returned as part of 
        //    ImageResponse for cameras and pose returned by getCameraInfo API.
        //    Do we need to convert pose to Global NED frame before returning to clients?
		TTimePoint updateTime = clock()->nowNanos();
		const GroundTruth& ground_truth = getGroundTruth();
		std::vector<uint64_t> ts;
		std::vector<msr::airlib::real_T> az;
		std::vector<msr::airlib::real_T> r;
		msr::airlib::vector<msr::airlib::real_T> pc;

        getPointCloud(params_.relative_pose, // relative lidar pose
            ground_truth.kinematics->pose,   // relative vehicle pose
			updateTime, pc, ts, az, r);

		Pose lidar_pose = params_.relative_pose + ground_truth.kinematics->pose;
		
        setOutput(updateTime, lidar_pose, pc);
		setOutputBuffer(lidar_pose, ts, az, r, scans_per_revolution_, channels_per_scan_);
    }

protected:
	LidarSimpleParams params_;
	TTimePoint start_time_;
	int32 channels_per_scan_ = 0;
	int32 scans_per_revolution_ = 0;
	int32 previous_sectors_since_start_ = 0;
	double rotation_rate_ = 0.0f;
	RandomGeneratorGausianR gauss_dist_ = RandomGeneratorGausianR(0, 1);
	RandomGeneratorR uniform_dist_ = RandomGeneratorR(0.0f, 1.0f);

private:
	FrequencyLimiter freq_limiter_;
};

}} //namespace
#endif
