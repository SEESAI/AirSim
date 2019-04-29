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
	// Extract params
	float azimuth_start = params_.horizontal_FOV_start;
	float azimuth_end = params_.horizontal_FOV_end;
	// Conversion of "FOV upper" & "lower" to polar angles (angle away from the positive z axis)
	float polar_start = 90.0f+params_.vertical_FOV_lower;
	float polar_end = 90.0f+params_.vertical_FOV_upper; 

	// Make sure angle data sensible
	azimuth_start = std::fmod(360.0f + azimuth_start, 360.0f);
	azimuth_end = std::fmod(360.0f + azimuth_end, 360.0f);
	azimuth_end = (azimuth_end <= azimuth_start) ? azimuth_end + 360.0f : azimuth_end;
	if (azimuth_start > 180.0f) {
		azimuth_start -= 360.0f;
		azimuth_end -= 360.0f;
	}
	polar_end = (polar_end <= polar_start) ? polar_start : polar_end;

    // calculate verticle angle distance between each laser
    float delta_angle = 0;
    if (channels_per_scan_ > 1)
        delta_angle = (polar_end - polar_start) /
            static_cast<float>(channels_per_scan_ - 1);

    // store vertical angles for each laser
    laser_polar_angles_.clear();
    for (int32 i = 0; i < channels_per_scan_; ++i)
    {
        const float angle = polar_start + static_cast<float>(i) * delta_angle;
		laser_polar_angles_.emplace_back(angle);
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
    const msr::airlib::TTimeDelta update_time, msr::airlib::vector<msr::airlib::real_T>& point_cloud, 	
	std::vector<uint64_t> & ts, std::vector<msr::airlib::real_T> & az, std::vector<msr::airlib::real_T> & r)
{

	// work out the current sector based on time since start
	double revsSinceStart = static_cast<double>(update_time - start_time_) * rotation_rate_ / 1e9;
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

    // shoot lasers and append
    for (int32 i = 0; i < numSectors; ++i)
    {
		int32 sector = endSector - numSectors + i;
		sector = (sector < 0) ? (sector + scans_per_revolution_) : sector;

		ts.emplace_back(update_time);
		az.emplace_back(laser_azimuth_angles_[sector] * M_PIf / 180.0);

		for (int32 j = 0; j < channels_per_scan_; ++j)
		{
			const float azimuth_angle = laser_azimuth_angles_[sector] + gauss_dist_.next() * params_.azimuth_stddev;
			const float polar_angle = laser_polar_angles_[j] + gauss_dist_.next() * params_.polar_stddev;

			if (uniform_dist_.next() < params_.point_loss_likelihood)
				// Delete random points
				r.emplace_back(0);
			else {
				Vector3r point;
				if (shootLaser(lidar_pose, vehicle_pose, azimuth_angle, polar_angle, params_, point)) {
					float range = point.norm();
					float range_new = range;

					// Add random early returns
					if (uniform_dist_.next() < (params_.random_return_likelihood * range / params_.range))
						range_new = uniform_dist_.next() * range;

					// Add random noise
					else
						range_new = range + gauss_dist_.next() * params_.range_stddev;

					// Scale & save the result (if it is valid)
					if (range_new > 0) {
						float scale = range_new / range;
						point.x() = point.x() * scale;
						point.y() = point.y() * scale;
						point.z() = point.z() * scale;

						point_cloud.emplace_back(point.x());
						point_cloud.emplace_back(point.y());
						point_cloud.emplace_back(point.z());
						r.emplace_back(range_new);
					}
					else
						r.emplace_back(0);
				}
				else
					r.emplace_back(0);
			}
		}
    }

    return;
}

// simulate shooting a laser via Unreal ray-tracing.
bool UnrealLidarSensor::shootLaser(const msr::airlib::Pose& lidar_pose, const msr::airlib::Pose& vehicle_pose,
    const float azimuth_angle, const float polar_angle,
    const msr::airlib::LidarSimpleParams params, Vector3r &point)
{
    // start position of Lidar in World Frame
    Vector3r start = (lidar_pose + vehicle_pose).position;

	// Convert Polar and Azimuth angles to Ray Pitch and Yaw 
	float ray_pitch_angle = polar_angle-90.0f;
	float ray_yaw_angle = azimuth_angle;

    // We need to compose rotations here rather than rotate a vector by a quaternion
    // Hence using coordOrientationAdd(..) rather than rotateQuaternion(..)

    // get ray quaternion in lidar frame (angles must be in radians)
    msr::airlib::Quaternionr ray_q_l = msr::airlib::VectorMath::toQuaternion(
        msr::airlib::Utils::degreesToRadians(ray_pitch_angle),  //pitch - rotation around Y axis
        0,                                                      //roll  - rotation around X axis
        msr::airlib::Utils::degreesToRadians(ray_yaw_angle));	//yaw   - rotation around Z axis

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
