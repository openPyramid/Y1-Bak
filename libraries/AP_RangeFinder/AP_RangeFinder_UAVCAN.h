#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"

// Data timeout
#define AP_RANGEFINDER_UAVCAN_TIMEOUT_MS 500

class AP_RangeFinder_UAVCAN : public AP_RangeFinder_Backend
{

public:
    // constructor
    AP_RangeFinder_UAVCAN(RangeFinder::RangeFinder_State &_state);

    // static detection function
    static bool detect();

    // update state
    void update(void);

protected:
		virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
			return MAV_DISTANCE_SENSOR_RADAR;
		}

private:

    uint16_t distance_cm;
    uint32_t last_update_ms;

	bool handle_range_finder_msg(uint16_t &range_cm, uint8_t readingType) override;

    // MAV_DISTANCE_SENSOR sensor_type = MAV_DISTANCE_SENSOR_UNKNOWN;
};
