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

public:
	FVideoCameraThread();
    virtual ~FVideoCameraThread();
    static void startRecording(const msr::airlib::ImageCaptureBase* camera, const msr::airlib::Kinematics::State* kinematics, 
        const VideoCameraSetting& settings, msr::airlib::VehicleSimApiBase* vehicle_sim_api);
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
    FRenderCommandFence read_pixel_fence_;
    
    static std::unique_ptr<FVideoCameraThread> instance_;

    std::unique_ptr<FRunnableThread> thread_;

	VideoCameraSetting settings_;
    const msr::airlib::ImageCaptureBase* image_capture_;
    const msr::airlib::Kinematics::State* kinematics_;
    msr::airlib::VehicleSimApiBase* vehicle_sim_api_;

    msr::airlib::TTimePoint next_screenshot_due_;

    bool is_ready_;
};