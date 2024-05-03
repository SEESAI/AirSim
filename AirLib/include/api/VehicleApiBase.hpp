// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_VehicleApiBase_hpp
#define air_VehicleApiBase_hpp

#include "common/CommonStructs.hpp"
#include "common/UpdatableObject.hpp"
#include "common/Common.hpp"
#include "common/Waiter.hpp"
#include "safety/SafetyEval.hpp"
#include "common/CommonStructs.hpp"
#include "common/ImageCaptureBase.hpp"
#include "sensors/SensorCollection.hpp"
#include "sensors/lidar/LidarBase.hpp"
#include "sensors/imu/ImuBase.hpp"
#include "sensors/barometer/BarometerBase.hpp"
#include "sensors/magnetometer/MagnetometerBase.hpp"
#include "sensors/distance/DistanceBase.hpp"
#include "sensors/gps/GpsBase.hpp"
#include <exception>
#include <string>

namespace msr
{
namespace airlib
{

    /*
Vehicle controller allows to obtain state from vehicle and send control commands to the vehicle.
State can include many things including sensor data, logs, estimated state from onboard computer etc.
Control commands can be low level actuation commands or high level movement commands.
The base class defines usually available methods that all vehicle controllers may implement.
Some methods may not be applicable to specific vehicle in which case an exception may be raised or call may be ignored.
*/
    class VehicleApiBase : public UpdatableObject
    {
    public:
        virtual void enableApiControl(bool is_enabled) = 0;
        virtual bool isApiControlEnabled() const = 0;
        virtual bool armDisarm(bool arm) = 0;
        virtual GeoPoint getHomeGeoPoint() const = 0;

        virtual void update() override
        {
            UpdatableObject::update();
        }

        virtual void cancelLastTask()
        {
            //if derived class supports async task then override this method
        }

        virtual bool isReady(std::string& message) const
        {
            unused(message);
            return true;
        }

        virtual bool canArm() const
        {
            return true;
        }

        //if vehicle supports it, call this method to send
        //kinematics and other info to somewhere (ex. log viewer, file, cloud etc)
        virtual void sendTelemetry(float last_interval = -1)
        {
            //no default action
            unused(last_interval);
        }

        //below APIs are used by FastPhysicsEngine
        virtual real_T getActuation(unsigned int actuator_index) const
        {
            unused(actuator_index);
            throw VehicleCommandNotImplementedException("getActuation API is not supported for this vehicle");
        }
        virtual size_t getActuatorCount() const
        {
            throw VehicleCommandNotImplementedException("getActuatorCount API is not supported for this vehicle");
        }

        virtual void getStatusMessages(std::vector<std::string>& messages)
        {
            unused(messages);
            //default implementation
        }

        /*
        For RCs, there are two cases: (1) vehicle may be configured to use
        RC bound to its hardware (2) vehicle may be configured to get RC data
        supplied via API calls. Below two APIs are not symmetrical, i.e.,
        getRCData() may or may not return same thing as setRCData().
        */
        //get reading from RC bound to vehicle (if unsupported then RCData::is_valid = false)
        virtual RCData getRCData() const
        {
            static const RCData invalid_rc_data{};
            return invalid_rc_data;
        }

        //set external RC data to vehicle (if unsupported then returns false)
        virtual bool setRCData(const RCData& rc_data)
        {
            unused(rc_data);
            return false;
        }

        // Sensors APIs
        virtual const SensorCollection& getSensors() const
        {
            throw VehicleCommandNotImplementedException("getSensors API is not supported for this vehicle");
        }

        // Lidar APIs
        virtual const LidarInfo& getLidarInfo(const std::string& lidar_name) const
        {
            auto* lidar = static_cast<const LidarBase*>(findSensorByName(lidar_name, SensorBase::SensorType::Lidar));
            if (lidar == nullptr)
                throw VehicleControllerException(Utils::stringf("No lidar with name %s exist on vehicle", lidar_name.c_str()));
            
            return lidar->getInfo();
        }

        virtual const LidarData& getLidarData(const std::string& lidar_name) const
        {
            LidarBase* lidar = nullptr;

            // Find lidar with the given name (for empty input name, return the first one found)
            // Not efficient but should suffice given small number of lidars
            uint count_lidars = getSensors().size(SensorBase::SensorType::Lidar);
            for (uint i = 0; i < count_lidars; i++) {
                LidarBase* current_lidar = static_cast<LidarBase*>(getSensors().getByType(SensorBase::SensorType::Lidar, i));
                if (current_lidar != nullptr && (current_lidar->getName() == lidar_name || lidar_name == "")) {
                    lidar = current_lidar;
                    break;
                }
            }

            if (lidar == nullptr)
                throw VehicleControllerException(Utils::stringf("No lidar with name %s exist on vehicle", lidar_name.c_str()));

            return lidar->getOutput();
        }

        virtual const LidarDataBuffer& getLidarDataBuffer(const std::string& lidar_name) const
        {
            LidarBase* lidar = nullptr;

            // Find lidar with the given name (for empty input name, return the first one found)
            // Not efficient but should suffice given small number of lidars
            uint count_lidars = getSensors().size(SensorBase::SensorType::Lidar);
            for (uint i = 0; i < count_lidars; i++) {
                LidarBase* current_lidar = static_cast<LidarBase*>(getSensors().getByType(SensorBase::SensorType::Lidar, i));
                if (current_lidar != nullptr && (current_lidar->getName() == lidar_name || lidar_name == "")) {
                    lidar = current_lidar;
                    break;
                }
            }

            if (lidar == nullptr)
                throw VehicleControllerException(Utils::stringf("No lidar with name %s exist on vehicle", lidar_name.c_str()));
            
            return lidar->getOutputBuffer();
        }

        // IMU API
        virtual const ImuInfo getImuInfo() const
        {
            // just take the first imu - assuming there is one
            uint count_imu_sensors = getSensors().size(SensorBase::SensorType::Imu);
            if (count_imu_sensors == 0) {
                ImuInfo i;
                return i;
            }

            auto* imu = static_cast<const ImuBase*>(getSensors().getByType(SensorBase::SensorType::Imu, 0));
            return imu->getInfo();
        }

        virtual const ImuBase::Output& getImuData(const std::string& imu_name) const
        {
            auto* imu = static_cast<const ImuBase*>(findSensorByName(imu_name, SensorBase::SensorType::Imu));
            if (imu == nullptr)
                throw VehicleControllerException(Utils::stringf("No IMU with name %s exist on vehicle", imu_name.c_str()));

            return imu->getOutput();
        }

        virtual ImuDataBuffer getImuDataBuffer() const
        {
            // just take the first imu - assuming there is one
            uint count_imu_sensors = getSensors().size(SensorBase::SensorType::Imu);
            if (count_imu_sensors == 0) {
                ImuDataBuffer i;
                return i;
            }

            ImuBase* imu = static_cast<ImuBase*>(getSensors().getByType(SensorBase::SensorType::Imu, 0));
            return imu->getOutputBuffer();
        }

        // Barometer API
        virtual const BarometerBase::Output& getBarometerData(const std::string& barometer_name) const
        {
            auto* barometer = static_cast<const BarometerBase*>(findSensorByName(barometer_name, SensorBase::SensorType::Barometer));
            if (barometer == nullptr)
                throw VehicleControllerException(Utils::stringf("No barometer with name %s exist on vehicle", barometer_name.c_str()));

            return barometer->getOutput();
        }

        // Magnetometer API
        virtual const MagnetometerBase::Output& getMagnetometerData(const std::string& magnetometer_name) const
        {
            auto* magnetometer = static_cast<const MagnetometerBase*>(findSensorByName(magnetometer_name, SensorBase::SensorType::Magnetometer));
            if (magnetometer == nullptr)
                throw VehicleControllerException(Utils::stringf("No magnetometer with name %s exist on vehicle", magnetometer_name.c_str()));

            return magnetometer->getOutput();
        }
        virtual const Vector3r &getMagnetometerGroundTruthMagneticField(const std::string &magnetometer_name) const
        {
            auto* magnetometer = static_cast<const MagnetometerBase*>(findSensorByName(magnetometer_name, SensorBase::SensorType::Magnetometer));
            if (magnetometer == nullptr)
                throw VehicleControllerException(Utils::stringf("No magnetometer with name %s exist on vehicle", magnetometer_name.c_str()));

            return magnetometer->getGroundTruthMagneticField();
        }

        // Gps API
        virtual const GpsBase::Output& getGpsData(const std::string& gps_name) const
        {
            auto* gps = static_cast<const GpsBase*>(findSensorByName(gps_name, SensorBase::SensorType::Gps));
            if (gps == nullptr)
                throw VehicleControllerException(Utils::stringf("No gps with name %s exist on vehicle", gps_name.c_str()));

            return gps->getOutput();
        }

        virtual GpsDataBuffer getGpsDataBuffer() const
        {
            uint count_gps_sensors = getSensors().size(SensorBase::SensorType::Gps);
            if (count_gps_sensors == 0) {
                GpsDataBuffer g;
                return g;
            }

            GpsBase* gps = static_cast<GpsBase*>(getSensors().getByType(SensorBase::SensorType::Gps, 0));
            return gps->getOutputBuffer();
        }

        // Distance Sensor API
        virtual const DistanceSensorData& getDistanceSensorData(const std::string& distance_sensor_name) const
        {
            auto* distance_sensor = static_cast<const DistanceBase*>(findSensorByName(distance_sensor_name, SensorBase::SensorType::Distance));
            if (distance_sensor == nullptr)
                throw VehicleControllerException(Utils::stringf("No distance sensor with name %s exist on vehicle", distance_sensor_name.c_str()));

            return distance_sensor->getOutput();
        }

        virtual ~VehicleApiBase() = default;

        //exceptions
        class VehicleControllerException : public std::runtime_error
        {
        public:
            VehicleControllerException(const std::string& message)
                : runtime_error(message)
            {
            }
        };

        class VehicleCommandNotImplementedException : public VehicleControllerException
        {
        public:
            VehicleCommandNotImplementedException(const std::string& message)
                : VehicleControllerException(message)
            {
            }
        };

        class VehicleMoveException : public VehicleControllerException
        {
        public:
            VehicleMoveException(const std::string& message)
                : VehicleControllerException(message)
            {
            }
        };

    private:
        const SensorBase* findSensorByName(const std::string& sensor_name, const SensorBase::SensorType type) const
        {
            const SensorBase* sensor = nullptr;

            // Find sensor with the given name (for empty input name, return the first one found)
            // Not efficient but should suffice given small number of sensors
            uint count_sensors = getSensors().size(type);
            for (uint i = 0; i < count_sensors; i++) {
                const SensorBase* current_sensor = getSensors().getByType(type, i);
                if (current_sensor != nullptr && (current_sensor->getName() == sensor_name || sensor_name == "")) {
                    sensor = current_sensor;
                    break;
                }
            }

            return sensor;
        }
    };
}
} //namespace
#endif
