/// @file	AC_Sprayer.h
/// @brief	Crop sprayer library

/**
    The crop spraying functionality can be enabled in ArduCopter by doing the following:
        - set CH7_OPT or CH8_OPT parameter to 15 to allow turning the sprayer on/off from one of these channels
        - set RC10_FUNCTION to 22 to enable the servo output controlling the pump speed on servo-out 10
        - set RC11_FUNCTION to 23 to enable the servo output controlling the spinner on servo-out 11
        - ensure the RC10_MIN, RC10_MAX, RC11_MIN, RC11_MAX accurately hold the min and maximum servo values you could possibly output to the pump and spinner
        - set the SPRAY_SPINNER to the pwm value the spinner should spin at when on
        - set the SPRAY_PUMP_RATE to the value the pump servo should move to when the vehicle is travelling 1m/s expressed as a percentage (i.e. 0 ~ 100) of the full servo range.  I.e. 0 = the pump will not operate, 100 = maximum speed at 1m/s.  50 = 1/2 speed at 1m/s, full speed at 2m/s
        - set the SPRAY_PUMP_MIN to the minimum value that the pump servo should move to while engaged expressed as a percentage (i.e. 0 ~ 100) of the full servo range
        - set the SPRAY_SPEED_MIN to the minimum speed (in cm/s) the vehicle should be moving at before the pump and sprayer are turned on.  0 will mean the pump and spinner will always be on when the system is enabled with ch7/ch8 switch
**/
#pragma once

#include <inttypes.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_AHRS/AP_AHRS.h>

#define AC_SPRAYER_DEFAULT_PUMP_MIN_PWM    1100
#define AC_SPRAYER_DEFAULT_PUMP_STOP_PWM    1200
#define AC_SPRAYER_DEFAULT_PUMP_MAX_PWM    1900
#define AC_SPRAYER_DEFAULT_PUMP_RATE        7.0f   /// (MAX - STOP) /100 default quantity of spray per meter travelled
#define AC_SPRAYER_DEFAULT_PUMP_MIN         15.0f    ///  default minimum pump speed expressed as a percentage from 0 to 100
#define AC_SPRAYER_DEFAULT_SPINNER_PWM      1300    ///< default speed of spinner (higher means spray is throw further horizontally
#define AC_SPRAYER_DEFAULT_SPEED_MIN        100     ///< we must be travelling at least 1m/s to begin spraying
#define AC_SPRAYER_DEFAULT_TURN_ON_DELAY    100     ///< delay between when we reach the minimum speed and we begin spraying.  This reduces the likelihood of constantly turning on/off the pump
#define AC_SPRAYER_DEFAULT_SHUT_OFF_DELAY   1000    ///< shut-off delay in milli seconds.  This reduces the likelihood of constantly turning on/off the pump

//#define AC_SPRAYER_DEFAULT_PUMP_RATE        10.0f   ///< default quantity of spray per meter travelled
//#define AC_SPRAYER_DEFAULT_PUMP_MIN         0       ///< default minimum pump speed expressed as a percentage from 0 to 100
//#define AC_SPRAYER_DEFAULT_SPINNER_PWM      1300    ///< default speed of spinner (higher means spray is throw further horizontally
//#define AC_SPRAYER_DEFAULT_SPEED_MIN        100     ///< we must be travelling at least 1m/s to begin spraying
//#define AC_SPRAYER_DEFAULT_TURN_ON_DELAY    100     ///< delay between when we reach the minimum speed and we begin spraying.  This reduces the likelihood of constantly turning on/off the pump
//#define AC_SPRAYER_DEFAULT_SHUT_OFF_DELAY   1000    ///< shut-off delay in milli seconds.  This reduces the likelihood of constantly turning on/off the pump

/// @class  AC_Sprayer
/// @brief  Object managing a crop sprayer comprised of a spinner and a pump both controlled by pwm
class AC_Sprayer {
public:
    AC_Sprayer();

    /* Do not allow copies */
    AC_Sprayer(const AC_Sprayer &other) = delete;
    AC_Sprayer &operator=(const AC_Sprayer&) = delete;
    
    typedef enum{
        Manual = 0,           //spraying at latest manual spray percentage values
        Auto,                       //spraying according to ground speed, not means auto stop and start.
        Calibr                      //spraying at calibr setting value(default is max)
    }Pump_mode_en;

    /// run_RC - allow or disallow spraying to occur by RC
    void handle_cmd_manual(const bool true_false);
    void handle_cmd_auto(const bool enable);
    void set_pump_mode(Pump_mode_en mode);
    Pump_mode_en get_pump_mode() const {return _pump_mode;};

#if HAL_WITH_UAVCAN
	bool set_agr(float vel, float fluid, uint8_t velBase);
#endif

    //set max flight ground speed for auto spraying.
    void set_max_ground_speed(int16_t speed_cms);

    void change_pump_speed(const int8_t val);

    /// running - returns true if spraying is currently permitted
    bool running() const;

    /// spraying - returns true if spraying is actually happening
    bool spraying() const { return _flags.spraying; }

    /// test_pump - set to true to turn on pump as if travelling at 1m/s as a test
    void test_pump(bool true_false) { return ;}

    /// To-Do: add function to decode pilot input from channel 6 tuning knob

    /// set_pump_rate - sets desired quantity of spray when travelling at 1m/s as a percentage of the pumps maximum rate
    //now we use _pump_pct_1ms as coeffecient to transfer pump percent(0-100) to delta PWM of pump ESC range
    void set_pump_rate(float pct_at_1ms) { _pump_pct_1ms.set(pct_at_1ms); }

    /// update - adjusts servo positions based on speed and requested quantity
    void update();

    static const struct AP_Param::GroupInfo var_info[];

private:
    void init();
    void stop_spraying();
    void check_tankempty();
    void update_pump_controller();

    // parameters
    AP_Int8         _enabled;               ///< top level enable/disable control
    AP_Float        _pump_pct_1ms;          ///<now we use _pump_pct_1ms as coeffecient to transfer pump percent(0-100) to delta PWM of pump ESC range
    AP_Int8         _pump_min_pct;          ///< minimum pump rate (expressed as a percentage from 0 to 100)
//    AP_Float        _pump_pct_1ms;          ///< desired pump rate (expressed as a percentage of top rate) when travelling at 1m/s
//    AP_Int8         _pump_min_pct;          ///< minimum pump rate (expressed as a percentage from 0 to 100)
    AP_Int16        _spinner_pwm;           ///< pwm rate of spinner
    AP_Float        _speed_min;             ///< minimum speed in cm/s above which the sprayer will be started

    /// flag bitmask
    struct sprayer_flags_type {
        uint8_t inited          :1;
        uint8_t spraying    : 1;            ///< 1 if we are currently spraying
        uint8_t levelsensor_hasliq       :1;         ///true if levelsensor report empty
    } _flags;

    // internal variables
    uint32_t        _speed_over_min_time;   ///< time at which we reached speed minimum
    uint32_t        _speed_under_min_time;  ///< time at which we fell below speed minimum

    typedef enum{
        Stop = 0,
        Suspend,
        Running,
    }Sprayer_sta_en;
    Sprayer_sta_en _state = Stop;
    
    bool _triggle_tankempty;       //tank empty triggle for stop pump
    bool _triggle_manual;
    bool _triggle_auto;
    bool _cmd_auto_enable;

// pump relevant parameters
    Pump_mode_en _pump_mode=Manual;

    float _pump_grdspd_rate;
    int8_t _pump_manual_pct;
    int8_t _ch_pumpspeedchg_flg=0;

};
