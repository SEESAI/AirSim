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

	if (settings.enabled) {
		instance_.reset(new FVideoCameraThread());
		instance_->image_capture_ = image_capture;
		instance_->kinematics_ = kinematics;
		instance_->settings_ = settings;
		instance_->vehicle_sim_api_ = vehicle_sim_api;

		instance_->next_screenshot_due_ = 0;

		instance_->is_ready_ = true;
	}

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
	// Recording interval in nanoseconds
	msr::airlib::TTimePoint record_interval_nanos = settings_.record_interval * 1e9;

	// Set the first screenshot time to keep images regular
	msr::airlib::TTimePoint current_airsim_time = msr::airlib::ClockFactory::get()->nowNanos();
	next_screenshot_due_ = settings_.record_interval * (current_airsim_time / +1);

	while (stop_task_counter_.GetValue() == 0)
    {
        //make sire all vars are set up
        if (is_ready_) {
			// Wait until next screenshot due
            bool picture_due = msr::airlib::ClockFactory::get()->nowNanos() >= next_screenshot_due_;
            if (picture_due)
            {
				//Get the images
				std::vector<msr::airlib::ImageCaptureBase::ImageResponse> responses;
                image_capture_->getImages(settings_.requests, responses);

				//Store them in the main vehicle API class (and get any updated image requests)
				std::vector<msr::airlib::ImageCaptureBase::ImageRequest>  newRequests = vehicle_sim_api_->saveVideoCameraImages(responses);
				if (!newRequests.empty())
					settings_.requests = newRequests;

				// Set the next screenshot time to keep images regular
				current_airsim_time = msr::airlib::ClockFactory::get()->nowNanos();
				next_screenshot_due_ = settings_.record_interval * (current_airsim_time / +1);
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
