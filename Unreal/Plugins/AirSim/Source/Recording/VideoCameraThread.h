#pragma once

#include "CoreMinimal.h"
#include "HAL/Runnable.h"

#include "AirBlueprintLib.h"
#include "api/VehicleSimApiBase.hpp"
#include "physics/Kinematics.hpp"
#include <memory>
#include "common/ClockFactory.hpp"
#include "common/AirSimSettings.hpp"

class FVideoCameraThread : public FRunnable
{
public:
    typedef msr::airlib::AirSimSettings::VideoCameraSetting VideoCameraSetting;
    typedef msr::airlib::VehicleSimApiBase VehicleSimApiBase;
    typedef msr::airlib::ImageCaptureBase ImageCaptureBase;

public:
    FVideoCameraThread();
    virtual ~FVideoCameraThread();
    static void startRecording(const VideoCameraSetting& settings,
                               const common_utils::UniqueValueMap<std::string, VehicleSimApiBase*>& vehicle_sim_apis);
    static void stopRecording();
    static bool isRecording();

protected:
    virtual bool Init() override;
    virtual uint32 Run() override;
    virtual void Stop() override;
    virtual void Exit() override;

private:
    void EnsureCompletion();

private:
    FThreadSafeCounter stop_task_counter_;

    static std::unique_ptr<FVideoCameraThread> instance_;

    std::unique_ptr<FRunnableThread> thread_;

    VideoCameraSetting settings_;
    common_utils::UniqueValueMap<std::string, VehicleSimApiBase*> vehicle_sim_apis_;
    std::unordered_map<std::string, const ImageCaptureBase*> image_captures_;

    bool is_ready_ = false;
    bool is_complete_ = false;
};
