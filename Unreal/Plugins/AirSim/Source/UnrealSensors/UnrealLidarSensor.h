// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include "common/Common.hpp"
#include "GameFramework/Actor.h"
#include "sensors/lidar/LidarSimple.hpp"
#include "NedTransform.h"

// UnrealLidarSensor implementation that uses Ray Tracing in Unreal.
// The implementation uses a model similar to CARLA Lidar implementation.
// Thanks to CARLA folks for this.
class UnrealLidarSensor : public msr::airlib::LidarSimple {
public:
    typedef msr::airlib::AirSimSettings AirSimSettings;

public:
    UnrealLidarSensor(const AirSimSettings::LidarSetting& setting,
        AActor* actor, const NedTransform* ned_transform);

protected:
    virtual void getPointCloud(const msr::airlib::Pose& lidar_pose, const msr::airlib::Pose& vehicle_pose,
        msr::airlib::TTimeDelta delta_time, msr::airlib::vector<msr::airlib::real_T>& point_cloud,
		std::vector<uint64_t> & ts, std::vector<msr::airlib::real_T> & az, std::vector<msr::airlib::real_T> & r) override;

private:
    using Vector3r = msr::airlib::Vector3r;
    using VectorMath = msr::airlib::VectorMath;

    void createLasers();
    bool shootLaser(const msr::airlib::Pose& lidar_pose, const msr::airlib::Pose& vehicle_pose,
        const float horizontal_angle, const float vertical_angle, 
        const msr::airlib::LidarSimpleParams params, Vector3r &point);

private:
    AActor* actor_;
    const NedTransform* ned_transform_;

	msr::airlib::vector<msr::airlib::real_T> laser_polar_angles_;
	msr::airlib::vector<msr::airlib::real_T> laser_azimuth_angles_;
};