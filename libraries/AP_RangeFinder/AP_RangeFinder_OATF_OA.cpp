/*
 * Copyright (C) 2016  Intel Corporation. All rights reserved.
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>
#include "AP_RangeFinder_OATF_OA.h"

#define OATF_SERIAL_LV_BAUD_RATE 115200
#define OATF_AVDDETECT_MODE OATF_NoDirRev_FwBw

extern const AP_HAL::HAL& hal;

/*
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_OATF_OA::AP_RangeFinder_OATF_OA(RangeFinder::RangeFinder_State &_state):enabled_tf(false),
    AP_RangeFinder_Backend(_state)
{
}

/*
   detect if a OATF rangefinder is connected. We'll detect by
   trying to take a reading on Serial. If we get a result the sensor is
   there.
*/
bool AP_RangeFinder_OATF_OA::detect(AP_SerialManager &serial_manager, uint8_t serial_instance)
{
    return serial_manager.find_serial(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance) != nullptr;
}


/*
   update the state of the sensor
*/
void AP_RangeFinder_OATF_OA::update(void)
{
    if(base_obj_ptr==nullptr) return ;

    if(!enabled_tf){
        base_obj_ptr->update();
    }

    if (base_obj_ptr->get_oa_state(_oastate)) {
        // update range_valid state based on distance measured
        state.distance_cm = _oastate.oa_dist_cm;
        last_reading_ms = AP_HAL::millis();
        update_status();
    }else if (AP_HAL::millis() - last_reading_ms > 500) {
        set_status(RangeFinder::RangeFinder_NoData);
    }

}

