#include <AP_HAL/AP_HAL.h>
#include <AP_Notify/AP_Notify.h>
#include <GCS_MAVLink/GCS.h>

#include "AP_Compass.h"

#if HAL_COMPASS_CALIB_SIDE236 == 1
#include <AP_AHRS/AP_AHRS.h>

extern AP_HAL::HAL& hal;

//only be call by vehicle compass_cal_udate in one thread so we use static variables
void
Compass::compass_cal_update()
{
    bool running = false;
    uint8_t mag_active_mask=0;
    int8_t first_mag_step=-1;
    uint8_t same_status_mask=0;
    bool enable_next = false;
    bool detect_failure = false;

    uint32_t now = AP_HAL::millis();

    //the following is orientation and enable sample detect, we consider all active mags
    //goto the same calib step and wait for sampling enable command before we triggle sampling
    //==============================================
    for(uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++){
        //we only consider active mags
        if(_calibrator[i].running() || _calibrator[i].get_status() == COMPASS_CAL_WAITING_TO_START){
            //mask for active
            mag_active_mask |= 0x01<<i;
            //we refer to the step that first found in the loop
            if(first_mag_step==-1){
                first_mag_step = _calibrator[i].get_status();
                same_status_mask |= 0x01<<i;
            }else if(_calibrator[i].get_status()==first_mag_step){
                //mark for staying the same step
                same_status_mask |= 0x01<<i;
            }
        }
    }

    if(mag_active_mask==0){
        //no mag active for calibration we reset feed calib watchdog 
        _calib_watchdog = now;

    //me must have active mags and the active mags must in the same calib step
    }else if(same_status_mask==mag_active_mask){

        if(first_mag_step==COMPASS_CAL_WAITING_TO_START){
           _completed_sides_mask = (~_sample_side_mask)&0x3F;

            //directly enable to next step
            enable_next = true;
            //feed dog
            _calib_watchdog = now;
#if COMPASS_CAL_DEBUG
            printf("Magcalib: waiting to start. mag act: %u",mag_active_mask);
#endif
        }else{
            uint8_t mag_sample_en_mask=0;
            uint8_t mag_sample_finish_mask=0;
            for(uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++){
                //mark active mag who has enabled sample
                if(((mag_active_mask>>i) & 0x01) && _calibrator[i].get_sample_enabled()) 
                    mag_sample_en_mask |=(0x01<<i) ;

                //mark active mag who has completed sample
                if(((mag_active_mask>>i) & 0x01) && _calibrator[i].get_side_completion((compass_cal_status_t)first_mag_step))
                    mag_sample_finish_mask |= (0x01<<i) ;
            }

            if(mag_sample_finish_mask==mag_active_mask){
                update_side_completed(_current_orient, _sample_side_mask, _completed_sides_mask);
                //tell every mag now we can go to next step
                enable_next = true;
                //feed dog
                _calib_watchdog = now;
#if COMPASS_CAL_DEBUG
                printf("Magcalib: side sample completed.\n\r");
#endif
            }else if(mag_sample_en_mask==0 && _detect_sta==CAL_STA_STANBY){
                 //none mag enable sample we start orientation detector
                 _detect_sta=CAL_STA_ORIENT_DETECT;

                 //feed dog
                 _calib_watchdog = now;
#if COMPASS_CAL_DEBUG
                printf("Magcalib: start orient detect.\n\r");
#endif
            }

            }
        }

    //STEP1: if we need side detect means we must in calib running step
    if(_detect_sta==CAL_STA_ORIENT_DETECT){
        if(orientation_detect(_current_orient)){
            bool need_rotate_detect = false;
            switch(_current_orient){
                case ORIENTATION_RIGHTSIDE_UP:{
                    if(first_mag_step==COMPASS_CAL_RUNNING_STEP_ONE && !(_completed_sides_mask&0x01)){
                        need_rotate_detect = true;
#if COMPASS_CAL_DEBUG
                        printf("Magcalib: orient RSP.\n\r");
#endif
                    }
                }break;

                case ORIENTATION_TAIL_DOWN:{
                    if(first_mag_step==COMPASS_CAL_RUNNING_STEP_ONE) break;
                    else if( !(_completed_sides_mask&0x02)){
                        need_rotate_detect = true;
#if COMPASS_CAL_DEBUG
                     printf("Magcalib: orient TLD.\n\r");
#endif
                    }
                }break;

                case ORIENTATION_NOSE_DOWN:{
                    if(first_mag_step==COMPASS_CAL_RUNNING_STEP_ONE) break;
                    else if((_sample_side_mask==3 && !(_completed_sides_mask&0x02))||
                        ( _sample_side_mask==11 && !(_completed_sides_mask&0x02))||
                       !(_completed_sides_mask&0x04)){
                        need_rotate_detect = true;
#if COMPASS_CAL_DEBUG
                        printf("Magcalib: orient NOD.\n\r");
#endif
                    }
                }break;

                case ORIENTATION_LEFT:{
                    if(first_mag_step==COMPASS_CAL_RUNNING_STEP_ONE) break;
                    else if( !(_completed_sides_mask&0x08)){
                        need_rotate_detect = true;
#if COMPASS_CAL_DEBUG
                        printf("Magcalib: orient LFT.\n\r");
#endif
                    }
                }break;
        
                case ORIENTATION_RIGHT:{
                    if(first_mag_step==COMPASS_CAL_RUNNING_STEP_ONE) break;
                    else if(( _sample_side_mask==11 && !(_completed_sides_mask&0x08))||
                       !(_completed_sides_mask&0x10)){
                            need_rotate_detect = true;
#if COMPASS_CAL_DEBUG
                            printf("Magcalib: orient RGT.\n\r");
#endif
                        }
                }break;

                case ORIENTATION_UPSIDE_DOWN:{
                    if(first_mag_step==COMPASS_CAL_RUNNING_STEP_ONE) break;
                    else if(!(_completed_sides_mask&0x20)){
                        need_rotate_detect = true;
#if COMPASS_CAL_DEBUG
                        printf("Magcalib: orient USD.\n\r");
#endif
                    }
                }break;

                case ORIENTATION_ERROR:{
                    detect_failure = true;
#if COMPASS_CAL_DEBUG
                    printf("Magcalib: orient ERR.\n\r");
#endif
                }break;
                default :break;
            }

            if(need_rotate_detect){
                _detect_sta = CAL_STA_ROTATE_DETECT;
                //before start the rotation detector we must reset it.
                rotation_detect_reset();

                //feed dog
                _calib_watchdog = now;
            }
        }
    }

    //STEP2: running rotation detector
    if(_detect_sta == CAL_STA_ROTATE_DETECT){
        if(rotation_detect(_current_orient)){
            //in a right orient rotation, we enable sampling
            for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
                _calibrator[i].set_sample_enable(true);
            }
            _detect_sta = CAL_STA_STANBY;
            //feed dog
            _calib_watchdog = now;
#if COMPASS_CAL_DEBUG
            printf("Magcalib: enable sample.\n\r");
#endif
        }
        //if return false wait timeout
    }

    //STEP3: check detect result
    if (mag_active_mask!=0 && (detect_failure || now -_calib_watchdog>=MAG_CAL_STEP_TIMEOUT)) {
        AP_Notify::events.compass_cal_failed = 1;
        cancel_calibration_all();
        _detect_sta = CAL_STA_STANBY;
        if(detect_failure)
            gcs().send_text(MAV_SEVERITY_ERROR, "Magcalib: orien/rotat detect failed.");
        else gcs().send_text(MAV_SEVERITY_ERROR, "Magcalib: calib timeout.");
    }

    //STEP4 update every mag
    for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
        bool failure;
        _calibrator[i].update(failure,enable_next);

        if (failure || _calibrator[i].check_for_timeout()) {
            AP_Notify::events.compass_cal_failed = 1;
            cancel_calibration_all();
            gcs().send_text(MAV_SEVERITY_ERROR, "Magcalib: mag %u calib failed.",i);
        }

        if (_calibrator[i].running()) {
            running = true;
        } else if (_cal_autosave && !_cal_saved[i] && _calibrator[i].get_status() == COMPASS_CAL_SUCCESS) {
            _accept_calibration(i);
        }
    }

    AP_Notify::flags.compass_cal_running = running;

    if (is_calibrating()) {
        _cal_has_run = true;
        return;
    } else if (_cal_has_run && _auto_reboot()) {
        hal.scheduler->delay(1000);
        hal.scheduler->reboot(false);
    }
}

bool
Compass::_start_calibration(uint8_t i, bool retry, float delay)
{
    if (!healthy(i)) {
        return false;
    }
    if (!use_for_yaw(i)) {
        return false;
    }
    if (!is_calibrating()) {
        AP_Notify::events.initiated_compass_cal = 1;
    }
    if (i == get_primary() && _state[i].external != 0) {
        _calibrator[i].set_tolerance(_calibration_threshold);
    } else {
        // internal compasses or secondary compasses get twice the
        // threshold. This is because internal compasses tend to be a
        // lot noisier
        _calibrator[i].set_tolerance(_calibration_threshold*2);
    }
    
    //we add _sample_side_mask condition to disable auto rotate check and fixed in 2 sides calib type
    if (_rotate_auto && _sample_side_mask!=0) {
        enum Rotation r = _state[i].external?(enum Rotation)_state[i].orientation.get():ROTATION_NONE;
        if (r != ROTATION_CUSTOM) { 
            _calibrator[i].set_orientation(r, _state[i].external, _rotate_auto>=2);
        }
    }
    _cal_saved[i] = false;
    _detect_sta = CAL_STA_STANBY;
    uint8_t side_cnt = 0;
    for(uint8_t i=0; i<COMPASS_CAL_ORIENTATION_SIDE_COUNT; i++){
        if((_sample_side_mask>>i) & 0x01) side_cnt++;
    }
    _calibrator[i].start(retry, delay, get_offsets_max(), i,side_cnt);

    // disable compass learning both for calibration and after completion
    _learn.set_and_save(0);

    return true;
}

bool
Compass::_start_calibration_mask(uint8_t mask, bool retry, bool autosave, float delay, bool autoreboot)
{
    _cal_autosave = autosave;
    _compass_cal_autoreboot = autoreboot;

    for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
        if ((1<<i) & mask) {
            if (!_start_calibration(i,retry,delay)) {
                _cancel_calibration_mask(mask);
                return false;
            }
        }
    }
    return true;
}

void
Compass::start_calibration_all(bool retry, bool autosave, float delay, bool autoreboot)
{
    _cal_autosave = autosave;
    _compass_cal_autoreboot = autoreboot;

    for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
        // ignore any compasses that fail to start calibrating
        // start all should only calibrate compasses that are being used
        _start_calibration(i,retry,delay);
    }
}

void
Compass::_cancel_calibration(uint8_t i)
{
    AP_Notify::events.initiated_compass_cal = 0;

    if (_calibrator[i].running() || _calibrator[i].get_status() == COMPASS_CAL_WAITING_TO_START) {
        AP_Notify::events.compass_cal_canceled = 1;
    }
    _cal_saved[i] = false;
    _calibrator[i].clear();
}

void
Compass::_cancel_calibration_mask(uint8_t mask)
{
    for(uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
        if((1<<i) & mask) {
            _cancel_calibration(i);
        }
    }
}

void
Compass::cancel_calibration_all()
{
    _cancel_calibration_mask(0xFF);
}

bool
Compass::_accept_calibration(uint8_t i)
{
    CompassCalibrator& cal = _calibrator[i];
    uint8_t cal_status = cal.get_status();

    if (_cal_saved[i] || cal_status == COMPASS_CAL_NOT_STARTED) {
        return true;
    } else if (cal_status == COMPASS_CAL_SUCCESS) {
        _cal_complete_requires_reboot = true;
        _cal_saved[i] = true;

        Vector3f ofs, diag, offdiag;
        cal.get_calibration(ofs, diag, offdiag);

        set_and_save_offsets(i, ofs);
        set_and_save_diagonals(i,diag);
        set_and_save_offdiagonals(i,offdiag);

        //we add _sample_side_mask condition to disable auto rotate check and fixed in 2 sides calib type
        if (_state[i].external && _rotate_auto >= 2 && _sample_side_mask!=0) {
            _state[i].orientation.set_and_save_ifchanged(cal.get_orientation());
        }

        if (!is_calibrating()) {
            AP_Notify::events.compass_cal_saved = 1;
        }
        return true;
    } else {
        return false;
    }
}

bool
Compass::_accept_calibration_mask(uint8_t mask)
{
    bool success = true;
    for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
        if ((1<<i) & mask) {
            if (!_accept_calibration(i)) {
                success = false;
            }
            _calibrator[i].clear();
        }
    }

    return success;
}

void
Compass::send_mag_cal_progress(mavlink_channel_t chan)
{
    uint8_t cal_mask = 0;

    for (uint8_t compass_id=0; compass_id<COMPASS_MAX_INSTANCES; compass_id++) {
        // ensure we don't try to send with no space available
        if (!HAVE_PAYLOAD_SPACE(chan, MAG_CAL_PROGRESS)) {
            return;
        }

        auto& calibrator = _calibrator[compass_id];
        uint8_t cal_status = calibrator.get_status();

        if (cal_status == COMPASS_CAL_WAITING_TO_START  ||
            cal_status == COMPASS_CAL_RUNNING_STEP_ONE ||
            cal_status == COMPASS_CAL_RUNNING_STEP_TWO ||
            cal_status == COMPASS_CAL_RUNNING_STEP_THR ||
            cal_status == COMPASS_CAL_RUNNING_STEP_FOUR ||
            cal_status == COMPASS_CAL_RUNNING_STEP_FIVE ||
            cal_status == COMPASS_CAL_RUNNING_STEP_SIX) {
            uint8_t completion_pct = calibrator.get_completion_percent();
            auto& completion_mask = calibrator.get_completion_mask();
            Vector3f direction(0.0f,0.0f,0.0f);
            uint8_t attempt = _calibrator[compass_id].get_attempt();

            mavlink_msg_mag_cal_progress_send(
                chan,
                compass_id, cal_mask,
                cal_status, attempt, completion_pct, completion_mask,
                direction.x, direction.y, direction.z
            );
        }
    }
}

void Compass::send_mag_cal_report(mavlink_channel_t chan)
{
    uint8_t cal_mask = 0;

    for (uint8_t compass_id=0; compass_id<COMPASS_MAX_INSTANCES; compass_id++) {
        // ensure we don't try to send with no space available
        if (!HAVE_PAYLOAD_SPACE(chan, MAG_CAL_REPORT)) {
            return;
        }

        uint8_t cal_status = _calibrator[compass_id].get_status();
        if (cal_status == COMPASS_CAL_SUCCESS ||
            cal_status == COMPASS_CAL_FAILED ||
            cal_status == COMPASS_CAL_BAD_ORIENTATION) {
            float fitness = _calibrator[compass_id].get_fitness();
            Vector3f ofs, diag, offdiag;
            _calibrator[compass_id].get_calibration(ofs, diag, offdiag);
            uint8_t autosaved = _cal_saved[compass_id];

            mavlink_msg_mag_cal_report_send(
                chan,
                compass_id, cal_mask,
                cal_status, autosaved,
                fitness,
                ofs.x, ofs.y, ofs.z,
                diag.x, diag.y, diag.z,
                offdiag.x, offdiag.y, offdiag.z,
                _calibrator[compass_id].get_orientation_confidence(),
                _calibrator[compass_id].get_original_orientation(),
                _calibrator[compass_id].get_orientation()
            );
        }
    }
}

bool
Compass::is_calibrating() const
{
    for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
        switch(_calibrator[i].get_status()) {
            case COMPASS_CAL_NOT_STARTED:
            case COMPASS_CAL_SUCCESS:
            case COMPASS_CAL_FAILED:
            case COMPASS_CAL_BAD_ORIENTATION:
                break;
            default:
                return true;
        }
    }
    return false;
}

/*
  handle an incoming MAG_CAL command
 */
MAV_RESULT Compass::handle_mag_cal_command(const mavlink_command_long_t &packet)
{
    MAV_RESULT result = MAV_RESULT_FAILED;

    switch (packet.command) {
    case MAV_CMD_DO_START_MAG_CAL: {
        result = MAV_RESULT_ACCEPTED;
        if (hal.util->get_soft_armed()) {
            hal.console->printf("Disarm for compass calibration\n");
            result = MAV_RESULT_FAILED;
            break;
        }
        if (packet.param1 < 0 || packet.param1 > 255) {
            result = MAV_RESULT_FAILED;
            break;
        }

        uint8_t mag_mask = packet.param1;
        bool retry = !is_zero(packet.param2);
        bool autosave = !is_zero(packet.param3);
        float delay = packet.param4;
        bool autoreboot = !is_zero(packet.param5);

        if (mag_mask == 0) { // 0 means all
            start_calibration_all(retry, autosave, delay, autoreboot);
        } else {
            if (!_start_calibration_mask(mag_mask, retry, autosave, delay, autoreboot)) {
                result = MAV_RESULT_FAILED;
            }
        }
        
        break;
    }

    case MAV_CMD_DO_ACCEPT_MAG_CAL: {
        result = MAV_RESULT_ACCEPTED;
        if(packet.param1 < 0 || packet.param1 > 255) {
            result = MAV_RESULT_FAILED;
            break;
        }
        
        uint8_t mag_mask = packet.param1;
        
        if (mag_mask == 0) { // 0 means all
            mag_mask = 0xFF;
        }
        
        if(!_accept_calibration_mask(mag_mask)) {
            result = MAV_RESULT_FAILED;
        }
        break;
    }
        
    case MAV_CMD_DO_CANCEL_MAG_CAL: {
        result = MAV_RESULT_ACCEPTED;
        if(packet.param1 < 0 || packet.param1 > 255) {
            result = MAV_RESULT_FAILED;
            break;
        }
        
        uint8_t mag_mask = packet.param1;
        
        if (mag_mask == 0) { // 0 means all
            cancel_calibration_all();
            break;
        }
        
        _cancel_calibration_mask(mag_mask);
        break;
    }
    }
    
    return result;
}

void Compass::update_side_completed(calib_orientation_t& orient, const uint8_t side_mask_cmd, uint8_t& side_mask){
    switch(orient){
        case ORIENTATION_RIGHTSIDE_UP:{
            side_mask |= 0x01;
        }break;

        case ORIENTATION_TAIL_DOWN:{
            if(side_mask_cmd==3 || side_mask_cmd==11){
                side_mask |= 0x06;
            }else  side_mask |= 0x02;
        }break;

        case ORIENTATION_NOSE_DOWN:{
            if(side_mask_cmd==3 || side_mask_cmd==11){
                side_mask |= 0x06;
            }else  side_mask |= 0x04;
        }break;

        case ORIENTATION_LEFT:{
            if(side_mask_cmd==11){
                side_mask |= 0x18;
            }else  side_mask |= 0x08;
        }break;

        case ORIENTATION_RIGHT:{
            if(side_mask_cmd==11){
                side_mask |= 0x18;
            }else  side_mask |= 0x10;
        }break;

        case ORIENTATION_UPSIDE_DOWN:{
            side_mask |= 0x20;
        }break;
        default :break;
    }
}

calib_orientation_t Compass::orientation_recognize_with_att(bool strict)
{
    const AP_AHRS &ahrs = AP::ahrs();
    float lean_thr;             //lean angle threshold
    int16_t roll;
    int16_t pitch;
    calib_orientation_t orient = ORIENTATION_ERROR;
    
    if(strict) lean_thr = 12.0f;
    else lean_thr = 45.0f;

    roll = (int16_t)(ahrs.roll * RAD_TO_DEG);
    pitch = (int16_t)(ahrs.pitch * RAD_TO_DEG);
    if (abs(roll) < lean_thr && abs(pitch) < lean_thr) orient = ORIENTATION_RIGHTSIDE_UP;
    else if(90.0f -pitch < lean_thr) orient = ORIENTATION_TAIL_DOWN;
    else if(90.0f +pitch < lean_thr) orient = ORIENTATION_NOSE_DOWN;
    else if(abs(90.0f +roll) < lean_thr && abs(pitch) < lean_thr) orient = ORIENTATION_LEFT;
    else if(abs(90.0f -roll) < lean_thr && abs(pitch) < lean_thr) orient = ORIENTATION_RIGHT;
    else if((abs(180.0f -roll) < lean_thr || abs(180.0f + roll) < lean_thr) && abs(pitch) < lean_thr) orient = ORIENTATION_UPSIDE_DOWN;

    return orient;
}

//return false means detect not finish, when we are doing detect,
//call interval no more than time_callinterval_ms = 1000 untill it return true reprecent 'finish'
//
bool Compass::orientation_detect(calib_orientation_t& orient_final){

    const uint16_t time_valid_ms = 500;
    const uint16_t time_callinterval_ms = 1000;
    const uint16_t detect_timeout_ms = 10000;
    calib_orientation_t orient = ORIENTATION_ERROR;

    uint32_t now = AP_HAL::millis();

    //first call or call interval over time_callinterval_ms will cuase detetor resart
    if(_time_lastcall_ms == 0 ||now -_time_lastcall_ms >=time_callinterval_ms){
        _time_start_ms = now;
        _time_inthr_ms = 0;
        _detect_finish = false;
        goto RETURN;
    }

    //arrive here means we already start detection
    if(!_detect_finish){
        //detect timeout
        if(now -_time_start_ms >= detect_timeout_ms){
            orient = ORIENTATION_ERROR;
            _last_orient = orient;
            _detect_finish = true;
#if COMPASS_CAL_DEBUG
            printf("Orientdet: orient detect timeout.\n\r");
#endif
            goto RETURN;
        }
        
        orient = orientation_recognize_with_att(true);
        if(orient!=ORIENTATION_ERROR){
#if COMPASS_CAL_DEBUG
            printf("Orientdet: orient!=err.\n\r");
#endif
            if(_last_orient==orient){
                _time_inthr_ms += (now -_time_lastcall_ms);
                
                if(_time_inthr_ms >= time_valid_ms){
                    printf("Orientdet: detect finish.\n\r");
                    _detect_finish = true;
                }
            //if in another valid orientation we reset the in threshold timer and detect this orientation
            //but will not reset start time
            }else{
                _time_inthr_ms = 0;
#if COMPASS_CAL_DEBUG
                printf("Orientdet: turn to detect another orient.\n\r");
#endif
            }
            //in this step we only store valid orientation to prevent _time_inthr_ms reset to 0
            _last_orient = orient;
        }else printf("Orientdet: orient=ERR\n\r");
        
    //we have finish detection
    }else{
#if COMPASS_CAL_DEBUG
        print("Orientdet: detect has finished.\n\r");
#endif
        //update start time to prevent timeout
        _time_start_ms = now;

        if(_last_orient == ORIENTATION_ERROR){
           goto RETURN;
        }

        //continue detect orient with relax threshold, this may not be reach
         orient = orientation_recognize_with_att(false);
         if(orient==ORIENTATION_ERROR || orient!=_last_orient){
            _time_inthr_ms = 0;
            _detect_finish = false;
         }
    }

RETURN:
    _time_lastcall_ms = now;
    orient_final = _last_orient;
    return _detect_finish;
    }

//calibrate rotation detetor call rotation_detect_reset once before continously run detector
void Compass::rotation_detect_reset(){
    _rotate_detect_finish = false;
    _gyro_x_integral = 0.0f;
    _gyro_y_integral = 0.0f;
    _gyro_z_integral = 0.0f;
    _time_last_gyro = 0;
}

//calibrate rotation detetor call rotation_detect_reset once before continously run detector
//reutrn true if right axis is rotated according to orient 
bool Compass::rotation_detect(calib_orientation_t& orient){
    const AP_InertialSensor &ins = AP::ins();
    const Vector3f &gyro = ins.get_gyro(0);
    uint32_t now = AP_HAL::millis();

    /* ensure we have a valid first gyro timestamp and a valid target orient*/
    if (_time_last_gyro > 0 && !_rotate_detect_finish && orient!=ORIENTATION_ERROR) {
        
        /* integrate */
        float delta_t = (now - _time_last_gyro) / 1e3f;
        //accept time interval no more than 100ms
        if(delta_t < 0.1f){     
            _gyro_x_integral += gyro.x * delta_t;
            _gyro_y_integral += gyro.y * delta_t;
            _gyro_z_integral += gyro.z * delta_t;
        }

        switch(orient){
            case ORIENTATION_RIGHTSIDE_UP:
            case ORIENTATION_UPSIDE_DOWN:{
                if(fabs(_gyro_z_integral) > 90.0f*DEG_TO_RAD)
                    _rotate_detect_finish = true;
            }break;

            case ORIENTATION_LEFT:
            case ORIENTATION_RIGHT:{
                if(fabs(_gyro_y_integral) > 90.0f*DEG_TO_RAD)
                    _rotate_detect_finish = true;
            }break;

            case ORIENTATION_TAIL_DOWN:
            case ORIENTATION_NOSE_DOWN:{
                if(fabs(_gyro_x_integral)> 90.0f*DEG_TO_RAD)
                    _rotate_detect_finish = true;
            }break;
            default :break;
        }
    
    }

    //no matter if the intergral happen, we update the time stamp everytime we read gyro
    _time_last_gyro = now;

    return _rotate_detect_finish;
}

#endif
