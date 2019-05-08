// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "UnrealLidarSensor.h"
#include "AirBlueprintLib.h"
#include "common/Common.hpp"
#include "NedTransform.h"
#include "DrawDebugHelpers.h"

// ctor
UnrealLidarSensor::UnrealLidarSensor(const AirSimSettings::LidarSetting& setting,
    AActor* actor, const NedTransform* ned_transform)
    : LidarSimple(setting), actor_(actor), ned_transform_(ned_transform)
{
    createLasers();
}

// initializes information based on lidar configuration
void UnrealLidarSensor::createLasers()
{
	// Make sure angle data sensible
	float azimuth_start = params_.horizontal_FOV_start;
	float azimuth_end = params_.horizontal_FOV_end;
	float altitude_lower = params_.vertical_FOV_lower;
	float altitude_upper = params_.vertical_FOV_upper;
	azimuth_start = std::fmod(360.0f + azimuth_start, 360.0f);
	azimuth_end = std::fmod(360.0f + azimuth_end, 360.0f);
	azimuth_end = (azimuth_end <= azimuth_start) ? azimuth_end + 360.0f : azimuth_end;
	if (azimuth_start > 180.0f) {
		azimuth_start -= 360.0f;
		azimuth_end -= 360.0f;
	}

	altitude_upper = (altitude_upper <= altitude_lower) ? altitude_lower : altitude_upper;

    // calculate verticle angle distance between each laser
    float delta_angle = 0;
    if (channels_per_scan_ > 1)
        delta_angle = (altitude_upper - altitude_lower) /
            static_cast<float>(channels_per_scan_ - 1);

    // store vertical angles for each laser
    laser_altitude_angles_.clear();
    for (int32 i = 0; i < channels_per_scan_; ++i)
    {
        const float angle = altitude_lower + static_cast<float>(i) * delta_angle;
		laser_altitude_angles_.emplace_back(angle);
    }

	// calculate horizontal angle distance between each laser
	delta_angle = 0;
	if (scans_per_revolution_ > 1)
		delta_angle = (azimuth_end - azimuth_start) /
		static_cast<float>(scans_per_revolution_);

	// store horizontal angles for each laser
	laser_azimuth_angles_.clear();
	for (int32 i = 0; i < scans_per_revolution_; ++i)
	{
		const float angle = azimuth_start + static_cast<float>(i) * delta_angle;
		laser_azimuth_angles_.emplace_back(angle);
	}

}

// returns a point-cloud for the tick
void UnrealLidarSensor::getPointCloud(const msr::airlib::Pose& lidar_pose, const msr::airlib::Pose& vehicle_pose,
    const msr::airlib::TTimeDelta update_time, msr::airlib::vector<msr::airlib::real_T>& point_cloud)
{

	// work out the current sector based on time since start
	double revsSinceStart = static_cast<double>(update_time - start_time_) * rotation_rate_ / 1000000000;
	int32 sectorsSinceStart = static_cast<int32>(FMath::RoundHalfFromZero((revsSinceStart * static_cast<double>(scans_per_revolution_))));

	// if we're not at a new sector then give up
	if (sectorsSinceStart <= previous_sectors_since_start_)
		return;

	// Work out the number of sectors in this scan, capped to 1 revolution
	int32 numSectors = (sectorsSinceStart - previous_sectors_since_start_);
	numSectors = (numSectors < scans_per_revolution_) ? numSectors : scans_per_revolution_;

	// cap the points to scan via ray-tracing; this is currently needed for car/Unreal tick scenarios
    // since SensorBase mechanism uses the elapsed clock time instead of the tick delta-time.
    constexpr int32 MAX_POINTS_IN_SCAN = 1e+5f;
	int32 maxSectors = MAX_POINTS_IN_SCAN / channels_per_scan_;
	if (numSectors > maxSectors){
		numSectors = maxSectors;
        UAirBlueprintLib::LogMessageString("Lidar: ", "Capping number of points to scan", LogDebugLevel::Failure);
    }

	// Work out the actual end sector, and remember the total for next time
	int32 endSector = sectorsSinceStart % scans_per_revolution_;
	previous_sectors_since_start_ = sectorsSinceStart;

	// Save the pose
	scan_buffer_.pose = vehicle_pose + lidar_pose;

    // shoot lasers
    for (int32 i = 0; i < numSectors; ++i)
    {
		int32 sector = endSector - numSectors + i;
		sector = (sector < 0) ? (sector + scans_per_revolution_) : sector;
		const float azimuth_angle = laser_azimuth_angles_[sector];

		scan_buffer_.time_stamps.emplace_back(update_time);
		scan_buffer_.azimuth_angles.emplace_back(azimuth_angle);

        for (int32 j = 0; j < channels_per_scan_; ++j)
        {
			const float altitude_angle = laser_altitude_angles_[j];

            Vector3r point;
            // shoot laser and get the impact point, if any
			if (shootLaser(lidar_pose, vehicle_pose, azimuth_angle, altitude_angle, params_, point)) {
				point_cloud.emplace_back(point.x());
				point_cloud.emplace_back(point.y());
				point_cloud.emplace_back(point.z());
				scan_buffer_.ranges.emplace_back(point.norm());
			}
			else
				scan_buffer_.ranges.emplace_back(0);
        }
    }

	// Trim scan_buffer to be one revolution
	int32 max_buffer_length = scans_per_revolution_;
	if (scan_buffer_.time_stamps.size() > max_buffer_length) {
		int32 excess_length = scan_buffer_.time_stamps.size() - max_buffer_length;
		scan_buffer_.time_stamps.erase(scan_buffer_.time_stamps.begin(), scan_buffer_.time_stamps.begin() + excess_length);
		scan_buffer_.azimuth_angles.erase(scan_buffer_.azimuth_angles.begin(), scan_buffer_.azimuth_angles.begin() + excess_length);
		scan_buffer_.ranges.erase(scan_buffer_.ranges.begin(), scan_buffer_.ranges.begin() + (excess_length * channels_per_scan_));
	}

    return;
}

// simulate shooting a laser via Unreal ray-tracing.
bool UnrealLidarSensor::shootLaser(const msr::airlib::Pose& lidar_pose, const msr::airlib::Pose& vehicle_pose,
    const float horizontal_angle, const float vertical_angle, 
    const msr::airlib::LidarSimpleParams params, Vector3r &point)
{
    // start position
    Vector3r start = lidar_pose.position + vehicle_pose.position;

    // We need to compose rotations here rather than rotate a vector by a quaternion
    // Hence using coordOrientationAdd(..) rather than rotateQuaternion(..)

    // get ray quaternion in lidar frame (angles must be in radians)
    msr::airlib::Quaternionr ray_q_l = msr::airlib::VectorMath::toQuaternion(
        msr::airlib::Utils::degreesToRadians(vertical_angle),   //pitch - rotation around Y axis
        0,                                                      //roll  - rotation around X axis
        msr::airlib::Utils::degreesToRadians(horizontal_angle));//yaw   - rotation around Z axis

    // get ray quaternion in body frame
    msr::airlib::Quaternionr ray_q_b = VectorMath::coordOrientationAdd(ray_q_l, lidar_pose.orientation);

    // get ray quaternion in world frame
    msr::airlib::Quaternionr ray_q_w = VectorMath::coordOrientationAdd(ray_q_b, vehicle_pose.orientation);

    // get ray vector (end position)
    Vector3r end = VectorMath::rotateVector(VectorMath::front(), ray_q_w, true) * params.range + start;
   
    FHitResult hit_result = FHitResult(ForceInit);
    bool is_hit = UAirBlueprintLib::GetObstacle(actor_, ned_transform_->fromLocalNed(start), ned_transform_->fromLocalNed(end), hit_result, actor_, ECC_Visibility);

    if (is_hit)
    {
        if (false && UAirBlueprintLib::IsInGameThread())
        {
            // Debug code for very specific cases.
            // Mostly shouldn't be needed. Use SimModeBase::drawLidarDebugPoints()
            DrawDebugPoint(
                actor_->GetWorld(),
                hit_result.ImpactPoint,
                5,                       //size
                FColor::Red,
                true,                    //persistent (never goes away)
                0.1                      //point leaves a trail on moving object
            );
        }

        // decide the frame for the point-cloud
        if (params.data_frame == AirSimSettings::kVehicleInertialFrame) {
            // current detault behavior; though it is probably not very useful.
            // not changing the default for now to maintain backwards-compat.
            point = ned_transform_->toLocalNed(hit_result.ImpactPoint);
        }
        else if (params.data_frame == AirSimSettings::kSensorLocalFrame) {
            // point in vehicle intertial frame
            Vector3r point_v_i = ned_transform_->toLocalNed(hit_result.ImpactPoint);

            // tranform to lidar frame
            point = VectorMath::transformToBodyFrame(point_v_i, lidar_pose + vehicle_pose, true);

            // The above should be same as first transforming to vehicle-body frame and then to lidar frame
            //    Vector3r point_v_b = VectorMath::transformToBodyFrame(point_v_i, vehicle_pose, true);
            //    point = VectorMath::transformToBodyFrame(point_v_b, lidar_pose, true);

            // On the client side, if it is needed to transform this data back to the world frame,
            // then do the equivalent of following,
            //     Vector3r point_w = VectorMath::transformToWorldFrame(point, lidar_pose + vehicle_pose, true);
            // See SimModeBase::drawLidarDebugPoints()

            // TODO: Optimization -- instead of doing this for every point, it should be possible to do this
            // for the point-cloud together? Need to look into matrix operations to do this together for all points.
        }
        else 
            throw std::runtime_error("Unknown requested data frame");

        return true;
    }
    else 
    {
        return false;
    }
}
