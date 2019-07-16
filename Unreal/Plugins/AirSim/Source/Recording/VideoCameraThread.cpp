#include "VideoCameraThread.h"
#include "TaskGraphInterfaces.h"
#include "HAL/RunnableThread.h"

#include <thread>
#include <mutex>
#include "RenderRequest.h"
#include "PIPCamera.h"


std::unique_ptr<FVideoCameraThread> FVideoCameraThread::instance_;


FVideoCameraThread::FVideoCameraThread()
    : stop_task_counter_(0), kinematics_(nullptr), is_ready_(false)
{
    thread_.reset(FRunnableThread::Create(this, TEXT("FVideoCameraThread"), 0, TPri_BelowNormal)); // Windows default, possible to specify more priority
}


void FVideoCameraThread::startRecording(const msr::airlib::ImageCaptureBase* image_capture, 
	const msr::airlib::Kinematics::State* kinematics,
    const VideoCameraSetting& settings,
	msr::airlib::VehicleSimApiBase* vehicle_sim_api)
{
    stopRecording();

    instance_.reset(new FVideoCameraThread());
    instance_->image_capture_ = image_capture;
    instance_->kinematics_ = kinematics;
    instance_->settings_ = settings;
    instance_->vehicle_sim_api_ = vehicle_sim_api;

    instance_->last_screenshot_on_ = 0;
    instance_->last_pose_ = msr::airlib::Pose();

    instance_->is_ready_ = true;

}

FVideoCameraThread::~FVideoCameraThread()
{
    stopRecording();
}

bool FVideoCameraThread::isRecording()
{
    return instance_ != nullptr;
}

void FVideoCameraThread::stopRecording()
{
    if (instance_)
    {
        instance_->EnsureCompletion();
        instance_.reset();
    }
}

/*********************** methods for instance **************************************/

bool FVideoCameraThread::Init()
{
    if (image_capture_)
    {
        UAirBlueprintLib::LogMessage(TEXT("Initiated video camera thread"), TEXT(""), LogDebugLevel::Success);
    }
    return true;
}

uint32 FVideoCameraThread::Run()
{
    while (stop_task_counter_.GetValue() == 0)
    {
        //make sire all vars are set up
        if (is_ready_) {
            bool interval_elapsed = msr::airlib::ClockFactory::get()->elapsedSince(last_screenshot_on_) >= settings_.record_interval;
            if (interval_elapsed)
            {
                last_screenshot_on_ = msr::airlib::ClockFactory::get()->nowNanos();
                last_pose_ = kinematics_->pose;
				std::vector<msr::airlib::ImageCaptureBase::ImageResponse> responses;
                image_capture_->getImages(settings_.requests, responses);
				vehicle_sim_api_->saveVideoCameraImages(responses);
            }
        }
    }

    return 0;
}


void FVideoCameraThread::Stop()
{
    stop_task_counter_.Increment();
}


void FVideoCameraThread::Exit()
{

}

void FVideoCameraThread::EnsureCompletion()
{
    Stop();
    thread_->WaitForCompletion();
    //UAirBlueprintLib::LogMessage(TEXT("Stopped recording thread"), TEXT(""), LogDebugLevel::Success);
}
