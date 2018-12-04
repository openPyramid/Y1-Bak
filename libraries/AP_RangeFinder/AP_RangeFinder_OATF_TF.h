#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"

#define PACK_SIZE_OATF 11

// OATF avoid radar detect mode set according to the way of the gimbal used.
enum OATF_AvoidDetect_Mode {
    OATF_NoDirRev_FwBw = 0,           //no gimbal or gimbal is not uesed to change detect direction 2 avoid radar are connect,one for forward detection another for backward
    OATF_NoDirRev_Fw,                       //no gimbal or gimbal is not uesed to change detect direction 1 avoid radar are connect for forward detection
    OATF_HasDirRev_Single_FwBw,               //gimbal is uesed to change detect direction 1 avoid radar are connect for forward and backward detection
    OATF_HasDirRev_Doulbe_FwBw,               //gimbal is uesed to change detect direction 1 avoid radar are connect for forward  and backward detection
};


// OATF gimbal direction enum
enum OATF_Gimbal_Dir {
    OATF_FORWARD = 0,
    OATF_BACKWARD = 1
};

struct OAstc {
    uint16_t oa_dist_cm;
    OATF_Gimbal_Dir oa_dir;
    uint8_t oa_state;
};

class AP_RangeFinder_OATF_TF : public AP_RangeFinder_Backend
{

public:

    // constructor
    AP_RangeFinder_OATF_TF(RangeFinder::RangeFinder_State &_state,
                                   AP_SerialManager &serial_manager,
                                   uint8_t serial_instance);

    // static detection function
    static bool detect(AP_SerialManager &serial_manager, uint8_t serial_instance);

    // update state
    void update(void);

    bool get_oa_state(struct OAstc& oastate){oastate = _oastate; return true;}

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_RADAR;
    }

private:
    // get a reading
    bool get_reading(uint8_t direction, int16_t att100_singleorien, uint16_t &oadist_cm, uint16_t &tfdist_cm);
    void prompt_reading(uint8_t direction, int16_t att100_singleorien);
    AP_HAL::UARTDriver *uart = nullptr;
    uint32_t last_reading_ms = 0;
    uint8_t linebuf[PACK_SIZE_OATF];
    uint8_t check = 0;

    struct OAstc _oastate;


};

