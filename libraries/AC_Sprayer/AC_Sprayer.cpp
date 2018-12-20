#include <AP_HAL/AP_HAL.h>
#include "AC_Sprayer.h"

#if HAL_WITH_UAVCAN
#include <AP_UAVCAN/AP_UAVCAN.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>
#endif


extern const AP_HAL::HAL& hal;

// ------------------------------

const AP_Param::GroupInfo AC_Sprayer::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Sprayer enable/disable
    // @Description: Allows you to enable (1) or disable (0) the sprayer
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 0, AC_Sprayer, _enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: PUMP_RATE
    // @DisplayName: Pump speed
    // @Description: Desired pump speed when traveling 1m/s expressed as a percentage
    // @Units: %
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("PUMP_RATE",   1, AC_Sprayer, _pump_pct_1ms, AC_SPRAYER_DEFAULT_PUMP_RATE),

    // @Param: SPINNER
    // @DisplayName: Spinner rotation speed
    // @Description: Spinner's rotation speed in PWM (a higher rate will disperse the spray over a wider area horizontally)
    // @Units: ms
    // @Range: 1000 2000
    // @User: Standard
    AP_GROUPINFO("SPINNER",     2, AC_Sprayer, _spinner_pwm, AC_SPRAYER_DEFAULT_SPINNER_PWM),

    // @Param: SPEED_MIN
    // @DisplayName: Speed minimum
    // @Description: Speed minimum at which we will begin spraying
    // @Units: cm/s
    // @Range: 0 1000
    // @User: Standard
    AP_GROUPINFO("SPEED_MIN",   3, AC_Sprayer, _speed_min, AC_SPRAYER_DEFAULT_SPEED_MIN),

    // @Param: PUMP_MIN
    // @DisplayName: Pump speed minimum
    // @Description: Minimum pump speed expressed as a percentage
    // @Units: %
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("PUMP_MIN",   4, AC_Sprayer, _pump_min_pct, AC_SPRAYER_DEFAULT_PUMP_MIN),

    AP_GROUPEND
};

AC_Sprayer::AC_Sprayer()
{
    AP_Param::setup_object_defaults(this, var_info);

    // check for silly parameter values
    if (_pump_pct_1ms < 0.0f || _pump_pct_1ms > 100.0f) {
        _pump_pct_1ms.set_and_save(AC_SPRAYER_DEFAULT_PUMP_RATE);
    }
    if (_spinner_pwm < 0) {
        _spinner_pwm.set_and_save(AC_SPRAYER_DEFAULT_SPINNER_PWM);
    }
    _flags.inited = false;

    // To-Do: ensure that the pump and spinner servo channels are enabled
}

bool AC_Sprayer::running()const
{
    return _state==Running;
}
void AC_Sprayer::init()
{
    _cmd_auto_enable = true;
    _triggle_auto = false;
    _triggle_manual = false;
    _triggle_tankempty = false;
    _flags.levelsensor_hasliq = false;

    _pump_manual_pct = 50;
    _pump_grdspd_rate = (100.0f -AC_SPRAYER_DEFAULT_PUMP_MIN)/(500.0f - 100.0f);

    SRV_Channels::set_output_limit(SRV_Channel::k_sprayer_pump, SRV_Channel::SRV_CHANNEL_LIMIT_MIN);
    SRV_Channels::set_output_limit(SRV_Channel::k_sprayer_spinner, SRV_Channel::SRV_CHANNEL_LIMIT_MIN);

    _flags.inited = true;
}

//set max flight ground speed for auto spraying.
void AC_Sprayer::set_max_ground_speed(int16_t speed_cms)
{
    if(speed_cms<=_speed_min){
        _pump_grdspd_rate = 100.0f;
        return ;
    //as for agr vehicles, we default limit max speed no more than 10m/s
    }else if(speed_cms>1000.0f){
        speed_cms = 1000.0f;
    }
    //update ratio from ground speed to pump rate
    _pump_grdspd_rate = (100.0f -AC_SPRAYER_DEFAULT_PUMP_MIN)/(speed_cms - 100.0f);
}


//it will be inited by aux init
void AC_Sprayer::handle_cmd_manual(const bool true_false)
{
    //init output value if not init
    if(!_flags.inited){
        init();
    }else if(true_false){
        _triggle_manual = true;
    }
}


//we devide cmd into manual and auto, so we can handle it differently 
void AC_Sprayer::handle_cmd_auto(const bool enable)
{
    //init output value if not init
    if(!_flags.inited){
        init();
    }else{
        _triggle_auto = true;
        _cmd_auto_enable = enable;
    }
}

#if HAL_WITH_UAVCAN
bool AC_Sprayer::set_agr(float vel, float fluid, uint8_t velBase)
{
    bool success = false;
    if (AP_BoardConfig_CAN::get_can_num_ifaces() != 0) {
        for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_DRIVERS; i++) {
            if (hal.can_mgr[i] != nullptr) {
                AP_UAVCAN *uavcan = hal.can_mgr[i]->get_UAVCAN();
                if (uavcan != nullptr) {
                    success |= uavcan->agr_write(vel, fluid, velBase);
					// hal.console->printf("agr cmd vel %f, fluid %f, velBase %dr\n", vel, fluid, velBase);
                }
            }
        }
    }
    return success;
}
#endif

void AC_Sprayer::check_tankempty()
{
// TODO: get level sensor status from agr module and the flow sensor
    bool agr_level_sens = true;
    if(agr_level_sens==false && _flags.levelsensor_hasliq==true){
        _triggle_tankempty = true;
    }
     _flags.levelsensor_hasliq = agr_level_sens;
}

//set pump mode used when in or out auto agrmode and calibration
void AC_Sprayer::set_pump_mode(Pump_mode_en mode)
{
    if(mode==Auto){
        //we force _state to Suspend once if we at Stop state and begin to inter auto agrmode
        if(_state==Stop) _state = Suspend;
    }
    _pump_mode = mode;
}

void AC_Sprayer::change_pump_speed(const int8_t flag_val)
{
    _ch_pumpspeedchg_flg = flag_val;
}
void AC_Sprayer::stop_spraying()
{
    SRV_Channels::set_output_limit(SRV_Channel::k_sprayer_pump, SRV_Channel::SRV_CHANNEL_LIMIT_MIN);
    SRV_Channels::set_output_limit(SRV_Channel::k_sprayer_spinner, SRV_Channel::SRV_CHANNEL_LIMIT_MIN);
    _flags.spraying = false;
}

/// update - adjust pwm of servo controlling pump speed according to the desired quantity and our horizontal speed
void AC_Sprayer::update()
{
    //we must init by before update sprayer status
    if(!_flags.inited){
        return ;
     }

    //check if tank is empty
    check_tankempty();

    switch(_state){
        case Stop :
            //we need system enable or nothing will happen
            if(_enabled){
                if(_triggle_manual){
                    _state = Running;
                    _triggle_manual = false;
                }
             }
                
            if(_triggle_tankempty || _triggle_auto){
                _triggle_auto = false;
                _triggle_tankempty = false;
             }
            break;

        case Suspend :
            if(_enabled){
                if(_triggle_manual || _triggle_auto){
                    if(_triggle_manual || (_triggle_auto && _cmd_auto_enable)){
                        _state = Running;
                    }
                    _triggle_auto = false;
                    _triggle_manual = false;
                }
            }else{
                if(_triggle_tankempty){
                    _triggle_tankempty = false;
                }
                _state = Stop;
            }
            break;

        case Running :
            if(!_enabled || _triggle_manual || _triggle_tankempty){
                _state = Stop;
                _triggle_manual = false;
                _triggle_tankempty = false;
            }else if(_triggle_auto){
                if(!_cmd_auto_enable) _state = Suspend;
                _triggle_auto = false;
            }
             break;
         default:
            _state = Stop;
            break;
    }

    //
    update_pump_controller();
}

/// update - adjust pwm of servo controlling pump speed according to the desired quantity and our horizontal speed
void AC_Sprayer::update_pump_controller()
{
    // exit immediately if the pump function has not been set-up for any servo
    if (!SRV_Channels::function_assigned(SRV_Channel::k_sprayer_pump)) {
        return;
    }

    // exit immediately if we are disabled or shouldn't be running
    if (_state!=Running) {
        if(_flags.spraying) stop_spraying();
        
        return;
    }
    
    bool should_be_spraying = _flags.spraying;
    float ground_speed = 0;
    if(_pump_mode==Auto){
        // get horizontal velocity
        Vector3f velocity;
        if (!AP::ahrs().get_velocity_NED(velocity)) {
            // treat unknown velocity as zero which should lead to pump stopping
            // velocity will already be zero but this avoids a coverity warning
            velocity.zero();
        }
        ground_speed = norm(velocity.x * 100.0f, velocity.y * 100.0f);

        // get the current time
        const uint32_t now = AP_HAL::millis();

        // check our speed vs the minimum
        if (ground_speed >= _speed_min) {
            // if we are not already spraying
            if (!_flags.spraying) {
                // set the timer if this is the first time we've surpassed the min speed
                if (_speed_over_min_time == 0) {
                    _speed_over_min_time = now;
                }else{
                    // check if we've been over the speed long enough to engage the sprayer
                    if((now - _speed_over_min_time) > AC_SPRAYER_DEFAULT_TURN_ON_DELAY) {
                        should_be_spraying = true;
                        _speed_over_min_time = 0;
                    }
                }
            }
            // reset the speed under timer
            _speed_under_min_time = 0;
        } else {
            // we are under the min speed.
            if (_flags.spraying) {
                // set the timer if this is the first time we've dropped below the min speed
                if (_speed_under_min_time == 0) {
                    _speed_under_min_time = now;
                }else{
                    // check if we've been over the speed long enough to engage the sprayer
                    if((now - _speed_under_min_time) > AC_SPRAYER_DEFAULT_SHUT_OFF_DELAY) {
                        should_be_spraying = false;
                        _speed_under_min_time = 0;
                    }
                }
            }
            // reset the speed over timer
            _speed_over_min_time = 0;
        }
    }else{
    //Manual pump speed control
        should_be_spraying = true;
    }

    // if spraying or testing update the pump servo
    if (should_be_spraying) {
        int8_t percent = _pump_min_pct;
        if(_pump_mode==Auto){
            if(ground_speed > _speed_min){
                percent = _pump_min_pct + (ground_speed -_speed_min)  * _pump_grdspd_rate;
            }else{
                percent = _pump_min_pct;
            }
            
        }else if(_pump_mode==Calibr){
            percent = 100;
        }else{
            if(_ch_pumpspeedchg_flg==1){
                _pump_manual_pct += 9;
            }else if(_ch_pumpspeedchg_flg==-1){
                _pump_manual_pct -= 9;
            }
            _pump_manual_pct = MAX(_pump_manual_pct, 0);
            _pump_manual_pct = MIN(_pump_manual_pct,100);
            percent = _pump_min_pct + _pump_manual_pct;
        }
        percent = MAX(percent, _pump_min_pct); // ensure min pump speed
        percent = MIN(percent,100);     // clamp to range

        //now we use _pump_pct_1ms as coeffecient to transfer pump percent(0-100) to delta PWM of pump ESC range
        int16_t pump_pwm = AC_SPRAYER_DEFAULT_PUMP_STOP_PWM + percent * _pump_pct_1ms;
        constrain_int16(pump_pwm,AC_SPRAYER_DEFAULT_PUMP_STOP_PWM, AC_SPRAYER_DEFAULT_PUMP_MAX_PWM);

        SRV_Channels::set_output_pwm(SRV_Channel::k_sprayer_pump, pump_pwm);
//        SRV_Channels::set_output_pwm(SRV_Channel::k_sprayer_spinner, _spinner_pwm);
        _flags.spraying = true;
    } else {
        stop_spraying();
    }
}
