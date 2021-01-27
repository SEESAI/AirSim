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
		UAirBlueprintLib::LogMessage(TEXT("Video Camera: "), TEXT("Started"), LogDebugLevel::Success);
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
	msr::airlib::TTimePoint last_picture_taken_time = current_airsim_time;
	msr::airlib::TTimePoint last_picture_saved_time = current_airsim_time;
	std::vector<msr::airlib::ImageCaptureBase::ImageRequest> requests = settings_.requests;
	float dt_frame_lpf = record_interval_nanos / 1e9f;
	float dt_videos_lpf = record_interval_nanos / 1e9f;

	while (stop_task_counter_.GetValue() == 0)
	{
		//Check for new requests from the API
		vehicle_sim_api_->getVideoCameraRequests(requests);

		// Don't go any further if no requests
		if (requests.empty()) {
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			last_picture_taken_time = msr::airlib::ClockFactory::get()->nowNanos();
			continue;
		}

		//Get the images according to the requests
		//Note this blocks until an image is available from the renderer (defined by Unreal frame rate)
		std::vector<msr::airlib::ImageCaptureBase::ImageResponse> responses;
		vehicle_sim_api_->getImageCapture()->getImages(requests, responses);

		msr::airlib::TTimePoint this_picture_taken_time = msr::airlib::ClockFactory::get()->nowNanos();
		if (!responses.empty()) {
			// If we have an image then set "this picture time" from the image
			this_picture_taken_time = responses[0].time_stamp;
		}

		// Work out the (smoothed) frame rate
		float dtFrame = (this_picture_taken_time - last_picture_taken_time) / 1e9f;
		dt_frame_lpf += (dtFrame - dt_frame_lpf) / 10;
		last_picture_taken_time = this_picture_taken_time;

		// Check if we need to save it
		if (this_picture_taken_time >= next_screenshot_due) {
			//Store them in the main vehicle API class
			std::vector<msr::airlib::ImageCaptureBase::ImageRequest> new_requests;
			std::vector<std::shared_ptr<msr::airlib::ImageCaptureBase::ImageResponse>> response_ptrs(requests.size());
			for (int i = 0; i < responses.size(); i++) {
				response_ptrs[i] = std::make_shared<msr::airlib::ImageCaptureBase::ImageResponse >();
				*response_ptrs[i] = std::move(responses[i]);
			}
			vehicle_sim_api_->saveVideoCameraImages(response_ptrs);

			// Report the frame rate (both rendering and saving)
			float dtVideo = (this_picture_taken_time - last_picture_saved_time) / 1e9f;
			dt_videos_lpf += (dtVideo - dt_videos_lpf) / 10;
			last_picture_saved_time = this_picture_taken_time;
			std::stringstream stream;
			stream << "Receiving " << responses.size() << ((responses.size() == 1) ? " image at " : " images at ");
			stream << std::fixed << std::setprecision(1) << (1.f / dt_frame_lpf) << "Hz, saving at ";
			stream << std::fixed << std::setprecision(1) << (1.f / dt_videos_lpf) << "Hz (" << (dtVideo * 1000) << "ms)";
			UAirBlueprintLib::LogMessage("Video Camera: ", stream.str().c_str(), LogDebugLevel::Success);

			// Set the next picture prior to its due time (to ensure it is queued correctly)
			next_screenshot_due = this_picture_taken_time + record_interval_nanos - uint64_t(dtFrame * 1e9f / 2);
		}
	}

	UAirBlueprintLib::LogMessage(TEXT("Video Camera: "), TEXT("Stopped"), LogDebugLevel::Success);

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
