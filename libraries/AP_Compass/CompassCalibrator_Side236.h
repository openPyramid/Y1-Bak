#pragma once

#include <AP_Math/AP_Math.h>

#ifndef HAL_COMPASS_CALIB_SIDE236
#define HAL_COMPASS_CALIB_SIDE236 0
#endif

//#define APPLY_PX4_FIT_ALGORITHM 1
#ifndef APPLY_PX4_FIT_ALGORITHM
#define APPLY_PX4_FIT_ALGORITHM 0
#endif

#if HAL_COMPASS_CALIB_SIDE236 == 1
//calibration debug
//#define COMPASS_CAL_DEBUG 1
#ifndef COMPASS_CAL_DEBUG
#define COMPASS_CAL_DEBUG 0
#endif

#define COMPASS_CAL_NUM_SPHERE_PARAMS 4
#define COMPASS_CAL_NUM_ELLIPSOID_PARAMS 9
#define COMPASS_CAL_NUM_SAMPLES 300

//RMS tolerance
#define COMPASS_CAL_DEFAULT_TOLERANCE 5.0f

//side sample constant
#define COMPASS_CAL_INTERVAL_PERSIDE_MS       10000              //the time in ms we allow sample perside
#define COMPASS_CAL_ORIENTATION_SIDE_COUNT 6
#define COMPASS_CAL_NUM_SAMPLES_PERSIDE (COMPASS_CAL_NUM_SAMPLES/COMPASS_CAL_ORIENTATION_SIDE_COUNT)
#define COMPASS_CAL_INTERVAL_SAMPLE_MS     ( (COMPASS_CAL_INTERVAL_PERSIDE_MS/COMPASS_CAL_NUM_SAMPLES_PERSIDE)>>8)

enum compass_cal_status_t {
    COMPASS_CAL_NOT_STARTED=0,
    COMPASS_CAL_WAITING_TO_START=1,
    COMPASS_CAL_RUNNING_STEP_ONE=2,
    COMPASS_CAL_RUNNING_STEP_TWO=3,
    COMPASS_CAL_RUNNING_STEP_THR=4,
    COMPASS_CAL_RUNNING_STEP_FOUR=5,
    COMPASS_CAL_RUNNING_STEP_FIVE=6,
    COMPASS_CAL_RUNNING_STEP_SIX=7,
    COMPASS_CAL_SUCCESS=8,
    COMPASS_CAL_FAILED=9,
    COMPASS_CAL_BAD_ORIENTATION=10,
};

class CompassCalibrator {
public:
    typedef uint8_t completion_mask_t[10];

    CompassCalibrator();

    void start(bool retry, float delay, uint16_t offset_max, uint8_t compass_idx, uint8_t side_cnt);

    void clear();

    void update(bool &failure, bool enable_next);
    void new_sample(const Vector3f& sample);
#if COMPASS_CAL_DEBUG
    void new_sample(const Vector3f& sample, Vector3f& rawfield);
#endif
    void set_sample_enable(bool enable);
    bool get_sample_enabled() const ;

    bool check_for_timeout();

    bool running() const;

    void set_orientation(enum Rotation orientation, bool is_external, bool fix_orientation) {
        _check_orientation = true;
        _orientation = orientation;
        _orig_orientation = orientation;
        _is_external = is_external;
        _fix_orientation = fix_orientation;
    }
    
    void set_tolerance(float tolerance) { _tolerance = tolerance; }

    void get_calibration(Vector3f &offsets, Vector3f &diagonals, Vector3f &offdiagonals);
    enum Rotation get_orientation(void) { return _orientation; }
    enum Rotation get_original_orientation(void) { return _orig_orientation; }

    float get_completion_percent() const;
    completion_mask_t& get_completion_mask();
    enum compass_cal_status_t get_status() const { return _status; }
    float get_fitness() const { return sqrtf(_fitness); }
    float get_orientation_confidence() const { return _orientation_confidence; }
    uint8_t get_attempt() const { return _attempt; }
    bool get_side_completion(compass_cal_status_t side_status) const;

private:
    class param_t {
    public:
        float* get_sphere_params() {
            return &radius;
        }

        float* get_ellipsoid_params() {
            return &offset.x;
        }

        float radius;
        Vector3f offset;
        Vector3f diag;
        Vector3f offdiag;
    };

    // compact class for approximate attitude, to save memory
    class AttitudeSample {
    public:
        Matrix3f get_rotmat();
        void set_from_ahrs();
    private:
        int8_t roll;
        int8_t pitch;
        int8_t yaw;
    };

    class CompassSample {
    public:
        Vector3f get() const;
        void set(const Vector3f &in);
        AttitudeSample att;
    private:
        int16_t x;
        int16_t y;
        int16_t z;
    };

    enum Rotation _orientation;
    enum Rotation _orig_orientation;
    bool _is_external;
    bool _check_orientation;
    bool _fix_orientation;
    uint8_t _compass_idx;

    enum compass_cal_status_t _status;

    // timeout watchdog state
    uint32_t _last_sample_ms;

    // behavioral state
    float _delay_start_sec;
    uint32_t _start_time_ms;
    bool _retry;
    float _tolerance;
    uint8_t _attempt;
    uint16_t _offset_max;

    completion_mask_t _completion_mask;

    //fit state
    class param_t _params;
    uint16_t _fit_step;
    CompassSample *_sample_buffer;
    float _fitness; // mean squared residuals
    float _initial_fitness;
    float _sphere_lambda;
    float _ellipsoid_lambda;
    uint16_t _samples_collected;
    uint16_t _samples_thinned;
    float _orientation_confidence;

    //side236
    bool _samples_enabled;
    uint16_t _total_samples_num;                                          //total num of the samples depended on side_cnt
    uint16_t _samples_perside;                                               //counter for each side sampling
    bool _side_completed;                                                       // side step finish flag true means one side completed will be reset when start one side sample progress
    uint8_t _sample_side_cnt;                                                   //how many sides we should sample according to _sample_side_mask
    float _mag_sphere_radius;
    
    bool set_status(compass_cal_status_t status);

    // returns true if sample should be added to buffer
    bool accept_sample(const Vector3f &sample);
    bool accept_sample(const CompassSample &sample);

    // returns true if fit is acceptable
    bool fit_acceptable();

    void reset_state();
    void initialize_fit();

    bool fitting() const;
    bool sampling() const;

    // thins out samples between step one and step two
    void thin_samples();

    float calc_residual(const Vector3f& sample, const param_t& params) const;
    float calc_mean_squared_residuals(const param_t& params) const;
    float calc_mean_squared_residuals() const;

    void calc_initial_offset();
    void calc_sphere_jacob(const Vector3f& sample, const param_t& params, float* ret) const;
    void run_sphere_fit();

    void calc_ellipsoid_jacob(const Vector3f& sample, const param_t& params, float* ret) const;
    void run_ellipsoid_fit();

    /**
     * Update #_completion_mask for the geodesic section of \p v. Corrections
     * are applied to \p v with #_params.
     *
     * @param v[in] A vector representing one calibration sample.
     */
    void update_completion_mask(const Vector3f& v);
    /**
     * Reset and update #_completion_mask with the current samples.
     */
    void update_completion_mask();

    Vector3f calculate_earth_field(CompassSample &sample, enum Rotation r);
    bool calculate_orientation();

#if APPLY_PX4_FIT_ALGORITHM

    bool run_lm_sphere_fit(float &_fitness, float &_sphere_lambda);
    bool run_lm_ellipsoid_fit(float &_fitness, float &_sphere_lambda);

    // Returns calibrate_return_error if any parameter is not finite
    // Logs if parameters are out of range
    bool  check_calibration_result(float offset_x, float offset_y, float offset_z,
        float sphere_radius,
        float diag_x, float diag_y, float diag_z,
        float offdiag_x, float offdiag_y, float offdiag_z);

#endif

};
#endif
