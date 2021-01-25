#include "VideoCameraThread.h"
#include "HAL/RunnableThread.h"

#include <thread>
#include <mutex>
#include "RenderRequest.h"
#include "PIPCamera.h"
#include <string>


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
		instance_->Stop();
		instance_->EnsureCompletion();
	}
}

/*********************** methods for instance **************************************/

bool FVideoCameraThread::Init()
{
	if (image_capture_)
	{
		UAirBlueprintLib::LogMessage(TEXT("Video camera thread"), TEXT("Started"), LogDebugLevel::Success);
	}
	return true;
}

uint32 FVideoCameraThread::Run()
{
	while (!is_ready_) {
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}

	// Recording interval in nanoseconds
	msr::airlib::TTimePoint record_interval_nanos = settings_.record_interval * 1e9;

	// Set the first screenshot time to keep images regular
	msr::airlib::TTimePoint current_airsim_time = msr::airlib::ClockFactory::get()->nowNanos();
	msr::airlib::TTimePoint next_screenshot_due = record_interval_nanos * (current_airsim_time / record_interval_nanos + 1);
	msr::airlib::TTimePoint last_picture_taken = 0;
	std::vector<msr::airlib::ImageCaptureBase::ImageRequest> requests = settings_.requests;

	while (stop_task_counter_.GetValue() == 0)
	{
		// Wait until next screenshot due
		current_airsim_time = msr::airlib::ClockFactory::get()->nowNanos();
		bool picture_due = current_airsim_time >= next_screenshot_due;
		if (picture_due)
		{
			//Get the images (converting to pointers) according to the requests
			std::vector<std::shared_ptr<msr::airlib::ImageCaptureBase::ImageResponse>> response_ptrs(requests.size());
			{
				std::vector<msr::airlib::ImageCaptureBase::ImageResponse> responses;
				image_capture_->getImages(requests, responses);
				for (int i = 0; i < responses.size(); i++) {
					response_ptrs[i] = std::make_shared<msr::airlib::ImageCaptureBase::ImageResponse >();
					*response_ptrs[i] = std::move(responses[i]);
				}
			}
			
			// Set the next screenshot time to keep images regular
			next_screenshot_due = record_interval_nanos * (current_airsim_time / record_interval_nanos + 1);
			if (!response_ptrs.empty()) {
				msr::airlib::TTimePoint current_picture_taken = response_ptrs[0]->time_stamp;
				uint64_t dtMS = (current_picture_taken - last_picture_taken) / 1e6;
				UAirBlueprintLib::LogMessage("Video Camera dt", std::to_string(dtMS).c_str(), LogDebugLevel::Success);
				last_picture_taken = current_picture_taken;
			}

			//Store them in the main vehicle API class (and get any updated image requests)
			std::vector<msr::airlib::ImageCaptureBase::ImageRequest> new_requests;
			vehicle_sim_api_->saveVideoCameraImages(response_ptrs, new_requests);
			if (!new_requests.empty())
				requests = std::move(new_requests);
		}
		else {
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
	}

	UAirBlueprintLib::LogMessage(TEXT("Video camera thread"), TEXT("Stopped"), LogDebugLevel::Success);

	return 0;
}


void FVideoCameraThread::Stop()
{
	stop_task_counter_.Increment();
}


void FVideoCameraThread::Exit()
{
	stop_task_counter_.Increment();
	EnsureCompletion();
}

void FVideoCameraThread::EnsureCompletion()
{
	thread_->WaitForCompletion();
}
