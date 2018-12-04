#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"
#include "AP_RangeFinder_OATF_TF.h"


class AP_RangeFinder_OATF_OA : public AP_RangeFinder_Backend
{

public:
    // constructor
    AP_RangeFinder_OATF_OA(RangeFinder::RangeFinder_State &_state);

    // static detection function
    static bool detect(AP_SerialManager &serial_manager, uint8_t serial_instance);

    // update state
    void update(void);
    bool get_base_instance(AP_RangeFinder_OATF_TF*& inst){inst = base_obj_ptr; return true;}
    void set_base_instance(AP_RangeFinder_OATF_TF* base_point){base_obj_ptr = base_point;}
    void set_tf_enable_flag(bool status){enabled_tf = status;}
protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_RADAR;
    }

private:
    AP_RangeFinder_OATF_TF* base_obj_ptr=nullptr;
    // get a reading
    bool get_reading(uint8_t direction, int16_t att100_singleorien, uint16_t &oadist_cm, uint16_t &tfdist_cm);
    void prompt_reading(uint8_t direction, int16_t att100_singleorien);
    AP_HAL::UARTDriver *uart = nullptr;
    uint32_t last_reading_ms = 0;
    uint8_t linebuf[PACK_SIZE_OATF];
    uint8_t check = 0;
    bool enabled_tf;                     //indicates if OATF's tf detector is enabled

    struct OAstc _oastate;

};

