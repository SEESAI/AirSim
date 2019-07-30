// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_GpsBase_hpp
#define msr_airlib_GpsBase_hpp


#include "sensors/SensorBase.hpp"
#include "common/CommonStructs.hpp"


namespace msr { namespace airlib {

class GpsBase  : public SensorBase {
public:
    GpsBase(const std::string& sensor_name = "")
        : SensorBase(sensor_name)
    {}

public: //types
    //TODO: cleanup GPS structures that are not needed
    struct GpsPoint {
    public:    
        double latitude, longitude;
        float height, altitude;
        int health;

        GpsPoint()
        {}

        GpsPoint(double latitude_val, double longitude_val, float altitude_val, int health_val = -1, float height_val = std::numeric_limits<float>::quiet_NaN())
        {
            latitude = latitude_val; longitude = longitude_val;
            height = height_val, altitude = altitude_val;
            health = health_val;
        }

        string to_string()
        {
            return Utils::stringf("latitude=%f, longitude=%f, altitude=%f, height=%f, health=%d", latitude, longitude, altitude, height, health);
        }
    };

    enum NavSatStatusType : char {
        STATUS_NO_FIX =  80,       //unable to fix position
        STATUS_FIX =      0,        //unaugmented fix
        STATUS_SBAS_FIX = 1,        //with satellite-based augmentation
        STATUS_GBAS_FIX = 2         //with ground-based augmentation
    };

    enum NavSatStatusServiceType : unsigned short int {
        SERVICE_GPS =     1,
        SERVICE_GLONASS = 2,
        SERVICE_COMPASS = 4,      //includes BeiDou.
        SERVICE_GALILEO = 8
    };


    struct NavSatStatus {
        NavSatStatusType status;
        NavSatStatusServiceType service;
    };

    enum PositionCovarianceType : unsigned char {
        COVARIANCE_TYPE_UNKNOWN = 0,
        COVARIANCE_TYPE_APPROXIMATED = 1,
        COVARIANCE_TYPE_DIAGONAL_KNOWN = 2,
        COVARIANCE_TYPE_KNOWN = 3
    };

    enum GnssFixType : unsigned char {
        GNSS_FIX_NO_FIX = 0,
        GNSS_FIX_TIME_ONLY = 1,
        GNSS_FIX_2D_FIX = 2,
        GNSS_FIX_3D_FIX = 3
    };

    struct GnssReport {
        GeoPoint geo_point;
        real_T eph, epv;    //GPS HDOP/VDOP horizontal/vertical dilution of position (unitless), 0-100%
        Vector3r velocity;
        GnssFixType fix_type;
        uint64_t time_utc = 0;
    };

    struct NavSatFix {
        GeoPoint geo_point;
        double position_covariance[9] = {};
        NavSatStatus status;
        PositionCovarianceType position_covariance_type;
    };

    struct Output {	//same as ROS message
        TTimePoint time_stamp;
        GnssReport gnss;
        bool is_valid = false;
    };


public:
    virtual void reportState(StateReporter& reporter) override
    {
        //call base
        UpdatableObject::reportState(reporter);

        reporter.writeValue("GPS-Loc", output_.gnss.geo_point);
        reporter.writeValue("GPS-Vel", output_.gnss.velocity);
        reporter.writeValue("GPS-Eph", output_.gnss.eph);
        reporter.writeValue("GPS-Epv", output_.gnss.epv);
    }

    const Output& getOutput() const
    {
        return output_;
    }

	const GPSDataBuffer& getOutputBuffer()
	{
		// Lock the mutex to prevent buffer update during call
		std::lock_guard<std::mutex> output_lock(APIoutput_mutex_);

		// Copy the buffer into the output (which is passed by reference - hence we need a copy)
		output_buffer_.timestamps_ns = output_buffer_internal_.timestamps_ns;
		output_buffer_.latitude = output_buffer_internal_.latitude;
		output_buffer_.longitude = output_buffer_internal_.longitude;
		output_buffer_.altitude = output_buffer_internal_.altitude;

		// Clear the buffer
		output_buffer_internal_.timestamps_ns.clear();
		output_buffer_internal_.latitude.clear();
		output_buffer_internal_.longitude.clear();
		output_buffer_internal_.altitude.clear();

		return output_buffer_;
	}

protected:
    void setOutput(const Output& output)
    {
        output_ = output;

		// Lock the mutex to prevent buffer update during call
		std::lock_guard<std::mutex> output_lock(APIoutput_mutex_);

		output_buffer_internal_.timestamps_ns.push_back(output.time_stamp);
		output_buffer_internal_.latitude.push_back(output.gnss.geo_point.latitude);
		output_buffer_internal_.longitude.push_back(output.gnss.geo_point.longitude);
		output_buffer_internal_.altitude.push_back(output.gnss.geo_point.altitude);

		// Trim to be a sensible size
		unsigned max_buffer_length = 100; // Hard coded for now - may wish to make this 1s of data
		if (output_buffer_internal_.timestamps_ns.size() > max_buffer_length) {
			output_buffer_internal_.timestamps_ns.erase(output_buffer_internal_.timestamps_ns.begin(),
				output_buffer_internal_.timestamps_ns.end() - max_buffer_length);
			output_buffer_internal_.latitude.erase(output_buffer_internal_.latitude.begin(),
				output_buffer_internal_.latitude.end() - max_buffer_length);
			output_buffer_internal_.longitude.erase(output_buffer_internal_.longitude.begin(),
				output_buffer_internal_.longitude.end() - max_buffer_length);
			output_buffer_internal_.altitude.erase(output_buffer_internal_.altitude.begin(),
				output_buffer_internal_.altitude.end() - max_buffer_length);

		}
    }


private: 
    Output output_;
	GPSDataBuffer output_buffer_;
	GPSDataBuffer output_buffer_internal_;
	std::mutex APIoutput_mutex_;
};

}} //namespace
#endif 
