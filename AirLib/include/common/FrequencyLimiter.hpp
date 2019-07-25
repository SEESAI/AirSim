// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_FrequencyLimiter_hpp
#define msr_airlib_FrequencyLimiter_hpp

#include "common/Common.hpp"
#include "UpdatableObject.hpp"
#include "common/Common.hpp"

namespace msr { namespace airlib {

class FrequencyLimiter : UpdatableObject {
public:
    FrequencyLimiter(real_T frequency = Utils::max<float>(), real_T startup_delay = 0)
    {
        initialize(frequency, startup_delay);
    }

    void initialize(real_T frequency = Utils::max<float>(), real_T startup_delay = 0)
    {
        frequency_ = frequency;
        startup_delay_ = startup_delay;
    }

    //*** Start: UpdatableState implementation ***//
    virtual void reset() override
    {
        //disable checks for reset/update sequence because
        //this object may get created but not used
        clearResetUpdateAsserts();  
        UpdatableObject::reset();

		first_time_ns_ = clock()->nowNanos();
		latest_time_ns_ = 0; // latest is time since reset
		last_time_ns_ = latest_time_ns_;
		previous_time_ns_ = last_time_ns_;

        if (Utils::isApproximatelyZero(frequency_))
            interval_size_ns_ = std::numeric_limits<uint64_t>::max();  //some high number
        else
			interval_size_ns_ = 1e9 * (1.0f / frequency_);

		if (Utils::isDefinitelyGreaterThan(startup_delay_, 0.0f)) {
			startup_complete_ = false;
			next_time_ns_ = latest_time_ns_ + startup_delay_ * 1e9;
		}
		else {
			startup_complete_ = true;
			next_time_ns_ = ((latest_time_ns_ / interval_size_ns_) + 1) * interval_size_ns_;
		}

        update_count_ = 0;
        interval_complete_ = false;
    }

    virtual void update() override
    {
        UpdatableObject::update();

		latest_time_ns_ = clock()->nowNanos() - first_time_ns_;
        ++update_count_;

		interval_complete_ = latest_time_ns_ >= next_time_ns_;

        //when any interval is done, reset the state and repeat
        if (interval_complete_) {
			startup_complete_ = true;
			previous_time_ns_ = last_time_ns_;
			last_time_ns_ = latest_time_ns_;
			next_time_ns_ = ((latest_time_ns_ / interval_size_ns_) + 1) * interval_size_ns_;
            startup_complete_ = true;
        }
    }
    //*** End: UpdatableState implementation ***//


    TTimeDelta getElapsedTotalSec() const
    {
        return clock()->elapsedBetween(latest_time_ns_, first_time_ns_);
    }

    TTimeDelta getElapsedIntervalSec() const
    {
        return clock()->elapsedBetween(latest_time_ns_, last_time_ns_);
    }

    TTimeDelta getLastElapsedIntervalSec() const
    {
		return clock()->elapsedBetween(last_time_ns_, previous_time_ns_);
	}

    bool isWaitComplete() const
    {
        return interval_complete_;
    }

    bool isStartupComplete() const
    {
        return startup_complete_;
    }

    uint getUpdateCount() const
    {
        return update_count_;
    }

private:
    uint update_count_;
    real_T frequency_;
    real_T startup_delay_;
    bool interval_complete_;
    bool startup_complete_;
	TTimePoint interval_size_ns_;
	TTimePoint latest_time_ns_, last_time_ns_, previous_time_ns_;
	TTimePoint first_time_ns_, next_time_ns_;

};

}} //namespace
#endif 
