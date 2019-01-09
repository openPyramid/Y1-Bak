/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
 * The intention of a magnetometer in a compass application is to measure
 * Earth's magnetic field. Measurements other than those of Earth's magnetic
 * field are considered errors. This algorithm computes a set of correction
 * parameters that null out errors from various sources:
 *
 * - Sensor bias error
 * - "Hard iron" error caused by materials fixed to the vehicle body that
 *     produce static magnetic fields.
 * - Sensor scale-factor error
 * - Sensor cross-axis sensitivity
 * - "Soft iron" error caused by materials fixed to the vehicle body that
 *     distort magnetic fields.
 *
 * This is done by taking a set of samples that are assumed to be the product
 * of rotation in earth's magnetic field and fitting an offset ellipsoid to
 * them, determining the correction to be applied to adjust the samples into an
 * origin-centered sphere.
 *
 * The state machine of this library is described entirely by the
 * compass_cal_status_t enum, and all state transitions are managed by the
 * set_status function. Normally, the library is in the NOT_STARTED state. When
 * the start function is called, the state transitions to WAITING_TO_START,
 * until two conditions are met: the delay as elapsed, and the memory for the
 * sample buffer has been successfully allocated.
 * Once these conditions are met, the state transitions to RUNNING_STEP_ONE, and
 * samples are collected via calls to the new_sample function. These samples are
 * accepted or rejected based on distance to the nearest sample. The samples are
 * assumed to cover the surface of a sphere, and the radius of that sphere is
 * initialized to a conservative value. Based on a circle-packing pattern, the
 * minimum distance is set such that some percentage of the surface of that
 * sphere must be covered by samples.
 *
 * Once the sample buffer is full, a sphere fitting algorithm is run, which
 * computes a new sphere radius. The sample buffer is thinned of samples which
 * no longer meet the acceptance criteria, and the state transitions to
 * RUNNING_STEP_TWO. Samples continue to be collected until the buffer is full
 * again, the full ellipsoid fit is run, and the state transitions to either
 * SUCCESS or FAILED.
 *
 * The fitting algorithm used is Levenberg-Marquardt. See also:
 * http://en.wikipedia.org/wiki/Levenberg%E2%80%93Marquardt_algorithm
 */

#include "CompassCalibrator_Side236.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_GeodesicGrid.h>
#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>

#if HAL_COMPASS_CALIB_SIDE236 == 1

extern const AP_HAL::HAL& hal;

////////////////////////////////////////////////////////////
///////////////////// PUBLIC INTERFACE /////////////////////
////////////////////////////////////////////////////////////

CompassCalibrator::CompassCalibrator():
_tolerance(COMPASS_CAL_DEFAULT_TOLERANCE),
_sample_buffer(nullptr)
{
    clear();
}

void CompassCalibrator::clear() {
    set_status(COMPASS_CAL_NOT_STARTED);
}

void CompassCalibrator::start(bool retry, float delay, uint16_t offset_max, uint8_t compass_idx, uint8_t side_cnt)
{
    if(running()) {
        return;
    }
    _offset_max = offset_max;
    _attempt = 0;                                           //we don't retry at internal loop once caliberation failed
    _retry = retry;
    _delay_start_sec = delay;
    _start_time_ms = AP_HAL::millis();
    _compass_idx = compass_idx;

    _sample_side_cnt = side_cnt;

    if(_sample_side_cnt < 2) return ;
    else if(_sample_side_cnt>COMPASS_CAL_ORIENTATION_SIDE_COUNT){
        _sample_side_cnt  = COMPASS_CAL_ORIENTATION_SIDE_COUNT;
    }
    _total_samples_num =  COMPASS_CAL_NUM_SAMPLES/COMPASS_CAL_ORIENTATION_SIDE_COUNT * _sample_side_cnt;
    set_status(COMPASS_CAL_WAITING_TO_START);
}

void CompassCalibrator::set_sample_enable(bool enable){
    if (!running()) {
        return;
    }
    _samples_enabled = enable;
}

bool CompassCalibrator::get_sample_enabled() const {
    return _samples_enabled;
}


void CompassCalibrator::get_calibration(Vector3f &offsets, Vector3f &diagonals, Vector3f &offdiagonals) {
    if (_status != COMPASS_CAL_SUCCESS) {
        return;
    }

    offsets = _params.offset;
    diagonals = _params.diag;
    offdiagonals = _params.offdiag;
}

float CompassCalibrator::get_completion_percent() const {
    // first sampling step is 1/3rd of the progress bar
    // never return more than 99% unless _status is COMPASS_CAL_SUCCESS

    switch(_status) {
        case COMPASS_CAL_NOT_STARTED:
        case COMPASS_CAL_WAITING_TO_START:
            return 0.0f;
        case COMPASS_CAL_RUNNING_STEP_ONE:
        case COMPASS_CAL_RUNNING_STEP_TWO:
        case COMPASS_CAL_RUNNING_STEP_THR:
        case COMPASS_CAL_RUNNING_STEP_FOUR:
        case COMPASS_CAL_RUNNING_STEP_FIVE:
        case COMPASS_CAL_RUNNING_STEP_SIX:{
            //calculate percentage multiply 100.0f to make to value from 0 to 100 in float form
            float pct = (_samples_collected*100.0f/_total_samples_num);
            //if one step sample has completed but fit have been finish, we sub 1percent to indicate gcs not to show next step guide
//            if(pct>99.0f &&  !_side_completed) pct = pct -1.0f;
            if(pct>100.0f) pct = 100.0f;
            return pct;
        }
        case COMPASS_CAL_SUCCESS:
            return 100.0f;
        case COMPASS_CAL_FAILED:
        case COMPASS_CAL_BAD_ORIENTATION:
        default:
            return 0.0f;
    };
}

bool CompassCalibrator::get_side_completion(compass_cal_status_t side_status) const {
    return side_status==_status && _side_completed;
}

void CompassCalibrator::update_completion_mask(const Vector3f& v)
{
    Matrix3f softiron{
        _params.diag.x,    _params.offdiag.x, _params.offdiag.y,
        _params.offdiag.x, _params.diag.y,    _params.offdiag.z,
        _params.offdiag.y, _params.offdiag.z, _params.diag.z
    };
    Vector3f corrected = softiron * (v + _params.offset);
    int section = AP_GeodesicGrid::section(corrected, true);
    if (section < 0) {
        return;
    }
    _completion_mask[section / 8] |= 1 << (section % 8);
}

void CompassCalibrator::update_completion_mask()
{
    memset(_completion_mask, 0, sizeof(_completion_mask));
    for (int i = 0; i < _samples_collected; i++) {
        update_completion_mask(_sample_buffer[i].get());
    }
}

CompassCalibrator::completion_mask_t& CompassCalibrator::get_completion_mask()
{
    return _completion_mask;
}

bool CompassCalibrator::check_for_timeout() {
    uint32_t tnow = AP_HAL::millis();
    if(running() && tnow - _last_sample_ms > 1000) {
        _retry = false;
        set_status(COMPASS_CAL_FAILED);
        return true;
    }
    return false;
}

void CompassCalibrator::new_sample(const Vector3f& sample) {
    _last_sample_ms = AP_HAL::millis();

    if(running() &&
        _samples_perside < COMPASS_CAL_NUM_SAMPLES_PERSIDE &&
        _samples_collected < _total_samples_num &&
        _samples_enabled &&
        accept_sample(sample)) {
        update_completion_mask(sample);
        _sample_buffer[_samples_collected].set(sample);
        _sample_buffer[_samples_collected].att.set_from_ahrs();
        _samples_collected++;
        _samples_perside++;
    }
}
#if COMPASS_CAL_DEBUG
void CompassCalibrator::new_sample(const Vector3f& sample, Vector3f& rawfield) {
    _last_sample_ms = AP_HAL::millis();

    if(running() &&
        _samples_perside < COMPASS_CAL_NUM_SAMPLES_PERSIDE &&
        _samples_collected < _total_samples_num &&
        _samples_enabled &&
        accept_sample(sample)) {
        update_completion_mask(sample);
        _sample_buffer[_samples_collected].set(sample);
        _sample_buffer[_samples_collected].att.set_from_ahrs();
        _samples_collected++;
        _samples_perside++;
        rawfield = sample;
    }
}
#endif

void CompassCalibrator::update(bool &failure, bool enable_next) {
    failure = false;

    if(sampling()) {
        return;
    }

    //if we are here, means we have completed sampling
    if(_status == COMPASS_CAL_WAITING_TO_START){
        if(enable_next) set_status(COMPASS_CAL_RUNNING_STEP_ONE);

    }else if(_status == COMPASS_CAL_RUNNING_STEP_ONE) {
//        if (_fit_step >= 10) {
//            if(is_equal(_fitness,_initial_fitness) || isnan(_fitness)) {           //if true, means that fitness is diverging instead of converging
//                set_status(COMPASS_CAL_FAILED);
//                failure = true;
//            }
//            _side_completed = true;
//            if(enable_next){
//                set_status(COMPASS_CAL_RUNNING_STEP_TWO);
//            }else{
//                return ;
//            }
//        } else {
//            if (_fit_step == 0) {
//                calc_initial_offset();
//            }
//            run_sphere_fit();
//            _fit_step++;
//        }
        _side_completed = true;
        if(enable_next){
            set_status(COMPASS_CAL_RUNNING_STEP_TWO);
        }
    }else if((_status == COMPASS_CAL_RUNNING_STEP_TWO && _sample_side_cnt==2) ||
    (_status == COMPASS_CAL_RUNNING_STEP_THR && _sample_side_cnt==3) ||
    (_status == COMPASS_CAL_RUNNING_STEP_SIX && _sample_side_cnt==6)) {
#if APPLY_PX4_FIT_ALGORITHM ==0
        if (_fit_step >= 200) {
            _side_completed = true;
            if(fit_acceptable() && calculate_orientation()) {
                set_status(COMPASS_CAL_SUCCESS);
            } else {
                set_status(COMPASS_CAL_FAILED);
                failure = true;
            }
        } else if (_fit_step < 100) {
            if(_fit_step==0) calc_initial_offset();
            
            run_sphere_fit();
            _fit_step++;
        } else {
            if(_fit_step==100){
                _fitness = 1.0e30f;
                if(is_equal(_fitness,_initial_fitness) || isnan(_fitness)) {           //if true, means that fitness is diverging instead of converging
                    set_status(COMPASS_CAL_FAILED);
                    failure = true;
                }
            }
            run_ellipsoid_fit();
            _fit_step++;
        }

#else
        if (_fit_step >= 200) {
            bool result = check_calibration_result(_params.offset.x, _params.offset.y, _params.offset.z,
                                                        _params.radius, _params.diag.x, _params.diag.y, _params.diag.z,
                                                        _params.offdiag.x, _params.offdiag.y, _params.offdiag.z);

            _side_completed = true;
            if(result){// && calculate_orientation()) {
                printf("Magcalib: update calib OK.");
                set_status(COMPASS_CAL_SUCCESS);
            } else {
                set_status(COMPASS_CAL_FAILED);
                printf("Magcalib: update calib ERR.");
                failure = true;
            }
        }else if(_fit_step<100){
            if(_sample_buffer == nullptr) return;
            
            run_lm_sphere_fit(_fitness, _sphere_lambda);
            _fit_step++;
        }else{
            if(_sample_buffer == nullptr) return;
            //need to reset _fitness in PX4 
            if(_fit_step==100) _fitness = 1.0e30f;

            run_lm_ellipsoid_fit(_fitness, _ellipsoid_lambda);
            _fit_step++;
        }
#endif

    }else if(running()){
        _side_completed = true;
        if(enable_next){
            set_status((compass_cal_status_t)(_status+1));
        }
    }
}

/////////////////////////////////////////////////////////////
////////////////////// PRIVATE METHODS //////////////////////
/////////////////////////////////////////////////////////////
bool CompassCalibrator::running() const {
    return _status == COMPASS_CAL_RUNNING_STEP_ONE || 
                _status == COMPASS_CAL_RUNNING_STEP_TWO || 
                _status == COMPASS_CAL_RUNNING_STEP_THR ||
                _status == COMPASS_CAL_RUNNING_STEP_FOUR ||
                _status == COMPASS_CAL_RUNNING_STEP_FIVE ||
                _status == COMPASS_CAL_RUNNING_STEP_SIX;
}

//not used in side236
bool CompassCalibrator::fitting() const {
    return running() && _samples_collected == _total_samples_num;
}

bool CompassCalibrator::sampling() const {
    return running() &&
                _samples_perside < COMPASS_CAL_NUM_SAMPLES_PERSIDE &&
                _samples_collected < _total_samples_num;
}

void CompassCalibrator::initialize_fit() {
    //initialize _fitness before starting a fit
#if APPLY_PX4_FIT_ALGORITHM ==0
    if (_samples_collected != 0) {
        _fitness = calc_mean_squared_residuals(_params);
    } else {
        _fitness = 1.0e30f;
    }
#else
     _fitness = 1.0e30f;
#endif
    _ellipsoid_lambda = 1.0f;
    _sphere_lambda = 1.0f;
    _initial_fitness = _fitness;
    _fit_step = 0;
}

void CompassCalibrator::reset_state() {
    _samples_collected = 0;
    _samples_thinned = 0;
    _samples_perside = 0;
    _samples_enabled = false;
    _side_completed = false;              //mark non side complete
    _mag_sphere_radius = 300.0f;
    _params.radius = 300;       //200;
    _params.offset.zero();
    _params.diag = Vector3f(1.0f,1.0f,1.0f);
    _params.offdiag.zero();

    memset(_completion_mask, 0, sizeof(_completion_mask));
    initialize_fit();
}

bool CompassCalibrator::set_status(compass_cal_status_t status) {
    if (status != COMPASS_CAL_NOT_STARTED && _status == status) {
        return true;
    }

    switch(status) {
        case COMPASS_CAL_NOT_STARTED:
            reset_state();
            _status = COMPASS_CAL_NOT_STARTED;

            if(_sample_buffer != nullptr) {
                free(_sample_buffer);
                _sample_buffer = nullptr;
            }
            return true;

        case COMPASS_CAL_WAITING_TO_START:
            reset_state();
            _status = COMPASS_CAL_WAITING_TO_START;

//            set_status(COMPASS_CAL_RUNNING_STEP_ONE);
            return true;

        case COMPASS_CAL_RUNNING_STEP_ONE:
            if(_status != COMPASS_CAL_WAITING_TO_START) {
                return false;
            }

            if(_attempt == 1 && (AP_HAL::millis()-_start_time_ms)*1.0e-3f < _delay_start_sec) {
                return false;
            }

            if (_sample_buffer == nullptr) {
                _sample_buffer =
                    (CompassSample*) calloc(_total_samples_num, sizeof(CompassSample));
            }

            if(_sample_buffer != nullptr) {
                initialize_fit();
                _status = COMPASS_CAL_RUNNING_STEP_ONE;
                //reset counter after get in a running status 
                _samples_perside = 0;
                _side_completed = false;
                _samples_enabled = false;
                return true;
            }

            return false;

        case COMPASS_CAL_RUNNING_STEP_TWO:
            if(_status != COMPASS_CAL_RUNNING_STEP_ONE) {
                return false;
            }
//            thin_samples();
            initialize_fit();
            _status = COMPASS_CAL_RUNNING_STEP_TWO;
            //reset counter after get in a running status 
            _samples_perside = 0;
            _side_completed = false;
            _samples_enabled = false;
            return true;

        case COMPASS_CAL_RUNNING_STEP_THR:
            if(_status != COMPASS_CAL_RUNNING_STEP_TWO) {
                return false;
            }
            _status = COMPASS_CAL_RUNNING_STEP_THR;
            //reset counter after get in a running status 
            _samples_perside = 0;
            _side_completed = false;
            _samples_enabled = false;
            return true;

        case COMPASS_CAL_RUNNING_STEP_FOUR:
            if(_status != COMPASS_CAL_RUNNING_STEP_THR) {
                return false;
            }
            _status = COMPASS_CAL_RUNNING_STEP_FOUR;
            //reset counter after get in a running status 
            _samples_perside = 0;
            _side_completed = false;
            _samples_enabled = false;
            return true;

        case COMPASS_CAL_RUNNING_STEP_FIVE:
            if(_status != COMPASS_CAL_RUNNING_STEP_FOUR) {
                return false;
            }
            _status = COMPASS_CAL_RUNNING_STEP_FIVE;
            //reset counter after get in a running status 
            _samples_perside = 0;
            _side_completed = false;
            _samples_enabled = false;
            return true;

            case COMPASS_CAL_RUNNING_STEP_SIX:
                if(_status != COMPASS_CAL_RUNNING_STEP_FIVE) {
                    return false;
                }
                _status = COMPASS_CAL_RUNNING_STEP_SIX;
                //reset counter after get in a running status 
                _samples_perside = 0;
                _side_completed = false;
                _samples_enabled = false;
                return true;

        case COMPASS_CAL_SUCCESS:
            if((_status == COMPASS_CAL_RUNNING_STEP_TWO && _sample_side_cnt==2)||
                (_status == COMPASS_CAL_RUNNING_STEP_THR && _sample_side_cnt==3)||
                (_status == COMPASS_CAL_RUNNING_STEP_SIX && _sample_side_cnt==6)) {

                if(_sample_buffer != nullptr) {
                    free(_sample_buffer);
                    _sample_buffer = nullptr;
                }

                _status = COMPASS_CAL_SUCCESS;
                
                //reset counter after get in a running status 
                _samples_perside = 0;
                _samples_enabled = false;

                return true;
            }else{
                return false;
            }
            
        case COMPASS_CAL_FAILED:
            if (_status == COMPASS_CAL_BAD_ORIENTATION) {
                // don't overwrite bad orientation status
                return false;
            }
            FALLTHROUGH;
            
        case COMPASS_CAL_BAD_ORIENTATION:
            if(_status == COMPASS_CAL_NOT_STARTED) {
                return false;
            }

//            if(_retry && set_status(COMPASS_CAL_WAITING_TO_START)) {
//                _attempt++;
//                return true;
//            }

            if(_sample_buffer != nullptr) {
                free(_sample_buffer);
                _sample_buffer = nullptr;
            }

            _status = status;
            return true;
            
        default:
            return false;
    };
}

bool CompassCalibrator::fit_acceptable() {
    if( !isnan(_fitness) &&
        _params.radius > 150 && _params.radius < 950 && //Earth's magnetic field strength range: 250-850mG
        fabsf(_params.offset.x) < _offset_max &&
        fabsf(_params.offset.y) < _offset_max &&
        fabsf(_params.offset.z) < _offset_max &&
        _params.diag.x > 0.2f && _params.diag.x < 5.0f &&
        _params.diag.y > 0.2f && _params.diag.y < 5.0f &&
        _params.diag.z > 0.2f && _params.diag.z < 5.0f &&
        fabsf(_params.offdiag.x) <  1.0f &&      //absolute of sine/cosine output cannot be greater than 1
        fabsf(_params.offdiag.y) <  1.0f &&
        fabsf(_params.offdiag.z) <  1.0f ){

            return _fitness <= sq(_tolerance);
        }
    return false;
}

void CompassCalibrator::thin_samples() {
    if(_sample_buffer == nullptr) {
        return;
    }

    _samples_thinned = 0;
    // shuffle the samples http://en.wikipedia.org/wiki/Fisher%E2%80%93Yates_shuffle
    // this is so that adjacent samples don't get sequentially eliminated
    for(uint16_t i=_samples_collected-1; i>=1; i--) {
        uint16_t j = get_random16() % (i+1);
        CompassSample temp = _sample_buffer[i];
        _sample_buffer[i] = _sample_buffer[j];
        _sample_buffer[j] = temp;
    }

    for(uint16_t i=0; i < _samples_collected; i++) {
        if(!accept_sample(_sample_buffer[i])) {
            _sample_buffer[i] = _sample_buffer[_samples_collected-1];
            _samples_collected --;
            _samples_thinned ++;
        }
    }

    update_completion_mask();
}

//new accept_sample for side236
bool CompassCalibrator::accept_sample(const Vector3f& sample)
{
    if(_sample_buffer == nullptr) {
        return false;
    }
    //we update radius every 2 samples at step1
    if(_samples_collected>0 && _samples_collected < COMPASS_CAL_NUM_SAMPLES_PERSIDE && !(_samples_collected%2)){
        _params.radius = sample.length();
    }
    //min_sample_dist is just an appropriate value for disperse the sample in on circumference 
    float min_sample_dist = fabs(5.4f* _params.radius/ sqrtf((float)COMPASS_CAL_NUM_SAMPLES_PERSIDE)) / 7.0f;
//    float min_sample_dist =  2 * M_PI * _params.radius/42.0f;
    //we consider last 5 samples calculating the distance
    float distance=0.0f;
    uint16_t i;
    if(_samples_collected<5){
        i = 0;
    }else{
        i = _samples_collected -5;
    }
    
    for (; i<_samples_collected; i++){
        distance = (sample - _sample_buffer[i].get()).length();
        if(distance < min_sample_dist) {
            return false;
        }
    }
    return true;
}

bool CompassCalibrator::accept_sample(const CompassSample& sample) {
    return accept_sample(sample.get());
}

float CompassCalibrator::calc_residual(const Vector3f& sample, const param_t& params) const {
    Matrix3f softiron(
        params.diag.x    , params.offdiag.x , params.offdiag.y,
        params.offdiag.x , params.diag.y    , params.offdiag.z,
        params.offdiag.y , params.offdiag.z , params.diag.z
    );
    return params.radius - (softiron*(sample+params.offset)).length();
}

float CompassCalibrator::calc_mean_squared_residuals() const
{
    return calc_mean_squared_residuals(_params);
}

float CompassCalibrator::calc_mean_squared_residuals(const param_t& params) const
{
    if(_sample_buffer == nullptr || _samples_collected == 0) {
        return 1.0e30f;
    }
    float sum = 0.0f;
    for(uint16_t i=0; i < _samples_collected; i++){
        Vector3f sample = _sample_buffer[i].get();
        float resid = calc_residual(sample, params);
        sum += sq(resid);
    }
    sum /= _samples_collected;
    return sum;
}

void CompassCalibrator::calc_sphere_jacob(const Vector3f& sample, const param_t& params, float* ret) const{
    const Vector3f &offset = params.offset;
    const Vector3f &diag = params.diag;
    const Vector3f &offdiag = params.offdiag;
    Matrix3f softiron(
        diag.x    , offdiag.x , offdiag.y,
        offdiag.x , diag.y    , offdiag.z,
        offdiag.y , offdiag.z , diag.z
    );

    float A =  (diag.x    * (sample.x + offset.x)) + (offdiag.x * (sample.y + offset.y)) + (offdiag.y * (sample.z + offset.z));
    float B =  (offdiag.x * (sample.x + offset.x)) + (diag.y    * (sample.y + offset.y)) + (offdiag.z * (sample.z + offset.z));
    float C =  (offdiag.y * (sample.x + offset.x)) + (offdiag.z * (sample.y + offset.y)) + (diag.z    * (sample.z + offset.z));
    float length = (softiron*(sample+offset)).length();

    // 0: partial derivative (radius wrt fitness fn) fn operated on sample
    ret[0] = 1.0f;
    // 1-3: partial derivative (offsets wrt fitness fn) fn operated on sample
    ret[1] = -1.0f * (((diag.x    * A) + (offdiag.x * B) + (offdiag.y * C))/length);
    ret[2] = -1.0f * (((offdiag.x * A) + (diag.y    * B) + (offdiag.z * C))/length);
    ret[3] = -1.0f * (((offdiag.y * A) + (offdiag.z * B) + (diag.z    * C))/length);
}

void CompassCalibrator::calc_initial_offset()
{
    // Set initial offset to the average value of the samples
    _params.offset.zero();
    for(uint16_t k = 0; k<_samples_collected; k++) {
        _params.offset -= _sample_buffer[k].get();
    }
    _params.offset /= _samples_collected;
}

void CompassCalibrator::run_sphere_fit()
{
    if(_sample_buffer == nullptr) {
        return;
    }

    const float lma_damping = 10.0f;

    float fitness = _fitness;
    float fit1, fit2;
    param_t fit1_params, fit2_params;
    fit1_params = fit2_params = _params;

    float JTJ[COMPASS_CAL_NUM_SPHERE_PARAMS*COMPASS_CAL_NUM_SPHERE_PARAMS] = { };
    float JTJ2[COMPASS_CAL_NUM_SPHERE_PARAMS*COMPASS_CAL_NUM_SPHERE_PARAMS] = { };
    float JTFI[COMPASS_CAL_NUM_SPHERE_PARAMS] = { };

    // Gauss Newton Part common for all kind of extensions including LM
    for(uint16_t k = 0; k<_samples_collected; k++) {
        Vector3f sample = _sample_buffer[k].get();

        float sphere_jacob[COMPASS_CAL_NUM_SPHERE_PARAMS];

        calc_sphere_jacob(sample, fit1_params, sphere_jacob);

        for(uint8_t i = 0;i < COMPASS_CAL_NUM_SPHERE_PARAMS; i++) {
            // compute JTJ
            for(uint8_t j = 0; j < COMPASS_CAL_NUM_SPHERE_PARAMS; j++) {
                JTJ[i*COMPASS_CAL_NUM_SPHERE_PARAMS+j] += sphere_jacob[i] * sphere_jacob[j];
                JTJ2[i*COMPASS_CAL_NUM_SPHERE_PARAMS+j] += sphere_jacob[i] * sphere_jacob[j];   //a backup JTJ for LM
            }
            // compute JTFI
            JTFI[i] += sphere_jacob[i] * calc_residual(sample, fit1_params);
        }
    }


    //------------------------Levenberg-Marquardt-part-starts-here---------------------------------//
    //refer: http://en.wikipedia.org/wiki/Levenberg%E2%80%93Marquardt_algorithm#Choice_of_damping_parameter
    for(uint8_t i = 0; i < COMPASS_CAL_NUM_SPHERE_PARAMS; i++) {
        JTJ[i*COMPASS_CAL_NUM_SPHERE_PARAMS+i] += _sphere_lambda;
        JTJ2[i*COMPASS_CAL_NUM_SPHERE_PARAMS+i] += _sphere_lambda/lma_damping;
    }

    if(!inverse(JTJ, JTJ, 4)) {
        return;
    }

    if(!inverse(JTJ2, JTJ2, 4)) {
        return;
    }

    for(uint8_t row=0; row < COMPASS_CAL_NUM_SPHERE_PARAMS; row++) {
        for(uint8_t col=0; col < COMPASS_CAL_NUM_SPHERE_PARAMS; col++) {
            fit1_params.get_sphere_params()[row] -= JTFI[col] * JTJ[row*COMPASS_CAL_NUM_SPHERE_PARAMS+col];
            fit2_params.get_sphere_params()[row] -= JTFI[col] * JTJ2[row*COMPASS_CAL_NUM_SPHERE_PARAMS+col];
        }
    }

    fit1 = calc_mean_squared_residuals(fit1_params);
    fit2 = calc_mean_squared_residuals(fit2_params);

    if(fit1 > _fitness && fit2 > _fitness){
        _sphere_lambda *= lma_damping;
    } else if(fit2 < _fitness && fit2 < fit1) {
        _sphere_lambda /= lma_damping;
        fit1_params = fit2_params;
        fitness = fit2;
    } else if(fit1 < _fitness){
        fitness = fit1;
    }
    //--------------------Levenberg-Marquardt-part-ends-here--------------------------------//

    if(!isnan(fitness) && fitness < _fitness) {
        _fitness = fitness;
        _params = fit1_params;
        update_completion_mask();
    }
}



void CompassCalibrator::calc_ellipsoid_jacob(const Vector3f& sample, const param_t& params, float* ret) const{
    const Vector3f &offset = params.offset;
    const Vector3f &diag = params.diag;
    const Vector3f &offdiag = params.offdiag;
    Matrix3f softiron(
        diag.x    , offdiag.x , offdiag.y,
        offdiag.x , diag.y    , offdiag.z,
        offdiag.y , offdiag.z , diag.z
    );

    float A =  (diag.x    * (sample.x + offset.x)) + (offdiag.x * (sample.y + offset.y)) + (offdiag.y * (sample.z + offset.z));
    float B =  (offdiag.x * (sample.x + offset.x)) + (diag.y    * (sample.y + offset.y)) + (offdiag.z * (sample.z + offset.z));
    float C =  (offdiag.y * (sample.x + offset.x)) + (offdiag.z * (sample.y + offset.y)) + (diag.z    * (sample.z + offset.z));
    float length = (softiron*(sample+offset)).length();

    // 0-2: partial derivative (offset wrt fitness fn) fn operated on sample
    ret[0] = -1.0f * (((diag.x    * A) + (offdiag.x * B) + (offdiag.y * C))/length);
    ret[1] = -1.0f * (((offdiag.x * A) + (diag.y    * B) + (offdiag.z * C))/length);
    ret[2] = -1.0f * (((offdiag.y * A) + (offdiag.z * B) + (diag.z    * C))/length);
    // 3-5: partial derivative (diag offset wrt fitness fn) fn operated on sample
    ret[3] = -1.0f * ((sample.x + offset.x) * A)/length;
    ret[4] = -1.0f * ((sample.y + offset.y) * B)/length;
    ret[5] = -1.0f * ((sample.z + offset.z) * C)/length;
    // 6-8: partial derivative (off-diag offset wrt fitness fn) fn operated on sample
    ret[6] = -1.0f * (((sample.y + offset.y) * A) + ((sample.x + offset.x) * B))/length;
    ret[7] = -1.0f * (((sample.z + offset.z) * A) + ((sample.x + offset.x) * C))/length;
    ret[8] = -1.0f * (((sample.z + offset.z) * B) + ((sample.y + offset.y) * C))/length;
}

void CompassCalibrator::run_ellipsoid_fit()
{
    if(_sample_buffer == nullptr) {
        return;
    }

    const float lma_damping = 10.0f;


    float fitness = _fitness;
    float fit1, fit2;
    param_t fit1_params, fit2_params;
    fit1_params = fit2_params = _params;


    float JTJ[COMPASS_CAL_NUM_ELLIPSOID_PARAMS*COMPASS_CAL_NUM_ELLIPSOID_PARAMS] = { };
    float JTJ2[COMPASS_CAL_NUM_ELLIPSOID_PARAMS*COMPASS_CAL_NUM_ELLIPSOID_PARAMS] = { };
    float JTFI[COMPASS_CAL_NUM_ELLIPSOID_PARAMS] = { };

    // Gauss Newton Part common for all kind of extensions including LM
    for(uint16_t k = 0; k<_samples_collected; k++) {
        Vector3f sample = _sample_buffer[k].get();

        float ellipsoid_jacob[COMPASS_CAL_NUM_ELLIPSOID_PARAMS];

        calc_ellipsoid_jacob(sample, fit1_params, ellipsoid_jacob);

        for(uint8_t i = 0;i < COMPASS_CAL_NUM_ELLIPSOID_PARAMS; i++) {
            // compute JTJ
            for(uint8_t j = 0; j < COMPASS_CAL_NUM_ELLIPSOID_PARAMS; j++) {
                JTJ [i*COMPASS_CAL_NUM_ELLIPSOID_PARAMS+j] += ellipsoid_jacob[i] * ellipsoid_jacob[j];
                JTJ2[i*COMPASS_CAL_NUM_ELLIPSOID_PARAMS+j] += ellipsoid_jacob[i] * ellipsoid_jacob[j];
            }
            // compute JTFI
            JTFI[i] += ellipsoid_jacob[i] * calc_residual(sample, fit1_params);
        }
    }



    //------------------------Levenberg-Marquardt-part-starts-here---------------------------------//
    //refer: http://en.wikipedia.org/wiki/Levenberg%E2%80%93Marquardt_algorithm#Choice_of_damping_parameter
    for(uint8_t i = 0; i < COMPASS_CAL_NUM_ELLIPSOID_PARAMS; i++) {
        JTJ[i*COMPASS_CAL_NUM_ELLIPSOID_PARAMS+i] += _ellipsoid_lambda;
        JTJ2[i*COMPASS_CAL_NUM_ELLIPSOID_PARAMS+i] += _ellipsoid_lambda/lma_damping;
    }

    if(!inverse(JTJ, JTJ, 9)) {
        return;
    }

    if(!inverse(JTJ2, JTJ2, 9)) {
        return;
    }

    for(uint8_t row=0; row < COMPASS_CAL_NUM_ELLIPSOID_PARAMS; row++) {
        for(uint8_t col=0; col < COMPASS_CAL_NUM_ELLIPSOID_PARAMS; col++) {
            fit1_params.get_ellipsoid_params()[row] -= JTFI[col] * JTJ[row*COMPASS_CAL_NUM_ELLIPSOID_PARAMS+col];
            fit2_params.get_ellipsoid_params()[row] -= JTFI[col] * JTJ2[row*COMPASS_CAL_NUM_ELLIPSOID_PARAMS+col];
        }
    }

    fit1 = calc_mean_squared_residuals(fit1_params);
    fit2 = calc_mean_squared_residuals(fit2_params);

    if(fit1 > _fitness && fit2 > _fitness){
        _ellipsoid_lambda *= lma_damping;
    } else if(fit2 < _fitness && fit2 < fit1) {
        _ellipsoid_lambda /= lma_damping;
        fit1_params = fit2_params;
        fitness = fit2;
    } else if(fit1 < _fitness){
        fitness = fit1;
    }
    //--------------------Levenberg-part-ends-here--------------------------------//

    if(fitness < _fitness) {
        _fitness = fitness;
        _params = fit1_params;
        update_completion_mask();
    }
}

/////////////////////////////////////////////////////////
//////////PX4 ellipsoid fit algorithm //////////////////////////
/////////////////////////////////////////////////////////
/**
 * Least-squares fit of a sphere to a set of points.
 *
 * Fits a sphere to a set of points on the sphere surface.
 *
 * @param x point coordinates on the X axis
 * @param y point coordinates on the Y axis
 * @param z point coordinates on the Z axis
 * @param size number of points
 * @param max_iterations abort if maximum number of iterations have been reached. If unsure, set to 100.
 * @param delta abort if error is below delta. If unsure, set to 0 to run max_iterations times.
 * @param sphere_x coordinate of the sphere center on the X axis
 * @param sphere_y coordinate of the sphere center on the Y axis
 * @param sphere_z coordinate of the sphere center on the Z axis
 * @param sphere_radius sphere radius
 *
 * @return 0 on success, 1 on failure
 */

#if APPLY_PX4_FIT_ALGORITHM
//@param sphere_fit 1:sphere_fit 0:ellipsoid_fit

//void CompassCalibrator::ellipsoid_fit_least_squares(param_t& params, bool sphere_fit)
//{
//	float _fitness = 1.0e30f, _sphere_lambda = 1.0f, _ellipsoid_lambda = 1.0f;
//
//	if(sphere_fit){
//		run_lm_sphere_fit(_fitness, _sphere_lambda);
//	}else{
//		run_lm_ellipsoid_fit(_fitness, _ellipsoid_lambda);
//	}
//    return ;
//}

bool CompassCalibrator::run_lm_sphere_fit(float &_fitness, float &_sphere_lambda)
{
	//Run Sphere Fit using Levenberg Marquardt LSq Fit
	const float lma_damping = 10.0f;
	float fitness = _fitness;
	float fit1 = 0.0f, fit2 = 0.0f;

	float JTJ[16];
	float JTJ2[16];
	float JTFI[4];
	float residual = 0.0f;
	memset(JTJ, 0, sizeof(JTJ));
	memset(JTJ2, 0, sizeof(JTJ2));
	memset(JTFI, 0, sizeof(JTFI));

	// Gauss Newton Part common for all kind of extensions including LM
	for (uint16_t k = 0; k < _samples_collected; k++) {
                  Vector3f sample = _sample_buffer[k].get();
		float sphere_jacob[4];
		//Calculate Jacobian
		float A = (_params.diag.x  * (sample.x - _params.offset.x)) + (_params.offdiag.x * (sample.y - _params.offset.y)) + (_params.offdiag.y * (sample.z - _params.offset.z));
		float B = (_params.offdiag.x * (sample.x - _params.offset.x)) + (_params.diag.y  * (sample.y - _params.offset.y)) + (_params.offdiag.z * (sample.z - _params.offset.z));
		float C = (_params.offdiag.y * (sample.x - _params.offset.x)) + (_params.offdiag.z * (sample.y - _params.offset.y)) + (_params.diag.z  * (sample.z - _params.offset.z));
		float length = sqrtf(A * A + B * B + C * C);

		// 0: partial derivative (radius wrt fitness fn) fn operated on sample
		sphere_jacob[0] = 1.0f;
		// 1-3: partial derivative (offsets wrt fitness fn) fn operated on sample
		sphere_jacob[1] = 1.0f * (((_params.diag.x    * A) + (_params.offdiag.x * B) + (_params.offdiag.y * C)) / length);
		sphere_jacob[2] = 1.0f * (((_params.offdiag.x * A) + (_params.diag.y    * B) + (_params.offdiag.z * C)) / length);
		sphere_jacob[3] = 1.0f * (((_params.offdiag.y * A) + (_params.offdiag.z * B) + (_params.diag.z    * C)) / length);
		residual = _params.radius - length;

		for (uint8_t i = 0; i < 4; i++) {
			// compute JTJ
			for (uint8_t j = 0; j < 4; j++) {
				JTJ[i * 4 + j] += sphere_jacob[i] * sphere_jacob[j];
				JTJ2[i * 4 + j] += sphere_jacob[i] * sphere_jacob[j]; //a backup JTJ for LM
			}

			JTFI[i] += sphere_jacob[i] * residual;
		}
	}


	//------------------------Levenberg-Marquardt-part-starts-here---------------------------------//
	//refer: http://en.wikipedia.org/wiki/Levenberg%E2%80%93Marquardt_algorithm#Choice_of_damping_parameter
	float fit1_params[4] = {_params.radius, _params.offset.x, _params.offset.y, _params.offset.z};
	float fit2_params[4];
	memcpy(fit2_params, fit1_params, sizeof(fit1_params));

	for (uint8_t i = 0; i < 4; i++) {
		JTJ[i * 4 + i] += _sphere_lambda;
		JTJ2[i * 4 + i] += _sphere_lambda / lma_damping;
	}

	if (!inverse4x4(JTJ, JTJ)) {
		return false;
	}

	if (!inverse4x4(JTJ2, JTJ2)) {
		return false;
	}

	for (uint8_t row = 0; row < 4; row++) {
		for (uint8_t col = 0; col < 4; col++) {
			fit1_params[row] -= JTFI[col] * JTJ[row * 4 + col];
			fit2_params[row] -= JTFI[col] * JTJ2[row * 4 + col];
		}
	}

	//Calculate mean squared residuals
	for (uint16_t k = 0; k < _samples_collected; k++) {
                 Vector3f sample = _sample_buffer[k].get();
		float A = (_params.diag.x    * (sample.x - fit1_params[1])) + (_params.offdiag.x * (sample.y - fit1_params[2])) + (_params.offdiag.y *
				(sample.z + fit1_params[3]));
		float B = (_params.offdiag.x * (sample.x - fit1_params[1])) + (_params.diag.y    * (sample.y - fit1_params[2])) + (_params.offdiag.z *
				(sample.z + fit1_params[3]));
		float C = (_params.offdiag.y * (sample.x - fit1_params[1])) + (_params.offdiag.z * (sample.y - fit1_params[2])) + (_params.diag.z    *
				(sample.z - fit1_params[3]));
		float length = sqrtf(A * A + B * B + C * C);
		residual = fit1_params[0] - length;
		fit1 += residual * residual;

		A = (_params.diag.x    * (sample.x - fit2_params[1])) + (_params.offdiag.x * (sample.y - fit2_params[2])) + (_params.offdiag.y *
				(sample.z - fit2_params[3]));
		B = (_params.offdiag.x * (sample.x - fit2_params[1])) + (_params.diag.y    * (sample.y - fit2_params[2])) + (_params.offdiag.z *
				(sample.z - fit2_params[3]));
		C = (_params.offdiag.y * (sample.x - fit2_params[1])) + (_params.offdiag.z * (sample.y - fit2_params[2])) + (_params.diag.z    *
				(sample.z - fit2_params[3]));
		length = sqrtf(A * A + B * B + C * C);
		residual = fit2_params[0] - length;
		fit2 += residual * residual;
	}

	fit1 = sqrtf(fit1) / _samples_collected;
	fit2 = sqrtf(fit2) / _samples_collected;

	if (fit1 > _fitness && fit2 > _fitness) {
		_sphere_lambda *= lma_damping;

	} else if (fit2 < _fitness && fit2 < fit1) {
		_sphere_lambda /= lma_damping;
		memcpy(fit1_params, fit2_params, sizeof(fit1_params));
		fitness = fit2;

	} else if (fit1 < _fitness) {
		fitness = fit1;
	}

	//--------------------Levenberg-Marquardt-part-ends-here--------------------------------//

	if (isfinite(fitness) && fitness < _fitness) {
		_fitness = fitness;
		_params.radius = fit1_params[0];
		_params.offset.x = fit1_params[1];
		_params.offset.y = fit1_params[2];
		_params.offset.z = fit1_params[3];
		return true;

	} else {
		return false;
	}
}

bool CompassCalibrator::run_lm_ellipsoid_fit(float &_fitness, float &_sphere_lambda)
{
	//Run Sphere Fit using Levenberg Marquardt LSq Fit
	const float lma_damping = 10.0f;
	float fitness = _fitness;
	float fit1 = 0.0f, fit2 = 0.0f;

	float JTJ[81];
	float JTJ2[81];
	float JTFI[9];
	float residual = 0.0f;
	memset(JTJ, 0, sizeof(JTJ));
	memset(JTJ2, 0, sizeof(JTJ2));
	memset(JTFI, 0, sizeof(JTFI));
	float ellipsoid_jacob[9];

	// Gauss Newton Part common for all kind of extensions including LM
	for (uint16_t k = 0; k < _samples_collected; k++) {
                 Vector3f sample = _sample_buffer[k].get();
		//Calculate Jacobian
		float A = (_params.diag.x    * (sample.x - _params.offset.x)) + (_params.offdiag.x * (sample.y - _params.offset.y)) + (_params.offdiag.y * (sample.z - _params.offset.z));
		float B = (_params.offdiag.x * (sample.x - _params.offset.x)) + (_params.diag.y    * (sample.y - _params.offset.y)) + (_params.offdiag.z * (sample.z - _params.offset.z));
		float C = (_params.offdiag.y * (sample.x - _params.offset.x)) + (_params.offdiag.z * (sample.y - _params.offset.y)) + (_params.diag.z    * (sample.z - _params.offset.z));
		float length = safe_sqrt(A * A + B * B + C * C);
		residual = _params.radius - length;
		fit1 += residual * residual;
		// 0-2: partial derivative (offset wrt fitness fn) fn operated on sample
		ellipsoid_jacob[0] = 1.0f * (((_params.diag.x    * A) + (_params.offdiag.x * B) + (_params.offdiag.y * C)) / length);
		ellipsoid_jacob[1] = 1.0f * (((_params.offdiag.x * A) + (_params.diag.y    * B) + (_params.offdiag.z * C)) / length);
		ellipsoid_jacob[2] = 1.0f * (((_params.offdiag.y * A) + (_params.offdiag.z * B) + (_params.diag.z    * C)) / length);
		// 3-5: partial derivative (diag offset wrt fitness fn) fn operated on sample
		ellipsoid_jacob[3] = -1.0f * ((sample.x + _params.offset.x) * A) / length;
		ellipsoid_jacob[4] = -1.0f * ((sample.y + _params.offset.y) * B) / length;
		ellipsoid_jacob[5] = -1.0f * ((sample.z + _params.offset.z) * C) / length;
		// 6-8: partial derivative (off-diag offset wrt fitness fn) fn operated on sample
		ellipsoid_jacob[6] = -1.0f * (((sample.y + _params.offset.y) * A) + ((sample.x + _params.offset.x) * B)) / length;
		ellipsoid_jacob[7] = -1.0f * (((sample.z + _params.offset.z) * A) + ((sample.x + _params.offset.x) * C)) / length;
		ellipsoid_jacob[8] = -1.0f * (((sample.z + _params.offset.z) * B) + ((sample.y + _params.offset.y) * C)) / length;

		for (uint8_t i = 0; i < 9; i++) {
			// compute JTJ
			for (uint8_t j = 0; j < 9; j++) {
				JTJ[i * 9 + j] += ellipsoid_jacob[i] * ellipsoid_jacob[j];
				JTJ2[i * 9 + j] += ellipsoid_jacob[i] * ellipsoid_jacob[j]; //a backup JTJ for LM
			}

			JTFI[i] += ellipsoid_jacob[i] * residual;
		}
	}


	//------------------------Levenberg-Marquardt-part-starts-here---------------------------------//
	//refer: http://en.wikipedia.org/wiki/Levenberg%E2%80%93Marquardt_algorithm#Choice_of_damping_parameter
	float fit1_params[9] = {_params.offset.x, _params.offset.y, _params.offset.z, _params.diag.x, _params.diag.y, _params.diag.z, _params.offdiag.x, _params.offdiag.y, _params.offdiag.z};
	float fit2_params[9];
	memcpy(fit2_params, fit1_params, sizeof(fit1_params));

	for (uint8_t i = 0; i < 9; i++) {
		JTJ[i * 9 + i] += _sphere_lambda;
		JTJ2[i * 9 + i] += _sphere_lambda / lma_damping;
	}


	if (!inverse(JTJ, JTJ, 9)) {
		return false;
	}

	if (!inverse(JTJ2, JTJ2, 9)) {
		return false;
	}



	for (uint8_t row = 0; row < 9; row++) {
		for (uint8_t col = 0; col < 9; col++) {
			fit1_params[row] -= JTFI[col] * JTJ[row * 9 + col];
			fit2_params[row] -= JTFI[col] * JTJ2[row * 9 + col];
		}
	}

	//Calculate mean squared residuals
	for (uint16_t k = 0; k < _samples_collected; k++) {
         Vector3f sample = _sample_buffer[k].get();
		float A = (fit1_params[3]    * (sample.x - fit1_params[0])) + (fit1_params[6] * (sample.y - fit1_params[1])) + (fit1_params[7] *
				(sample.z - fit1_params[2]));
		float B = (fit1_params[6] * (sample.x - fit1_params[0])) + (fit1_params[4]   * (sample.y - fit1_params[1])) + (fit1_params[8] *
				(sample.z - fit1_params[2]));
		float C = (fit1_params[7] * (sample.x - fit1_params[0])) + (fit1_params[8] * (sample.y - fit1_params[1])) + (fit1_params[5]    *
				(sample.z - fit1_params[2]));
		float length = safe_sqrt(A * A + B * B + C * C);
		residual = _params.radius - length;
		fit1 += residual * residual;

		A = (fit2_params[3]    * (sample.x - fit2_params[0])) + (fit2_params[6] * (sample.y - fit2_params[1])) + (fit2_params[7] *
				(sample.z - fit2_params[2]));
		B = (fit2_params[6] * (sample.x - fit2_params[0])) + (fit2_params[4]   * (sample.y - fit2_params[1])) + (fit2_params[8] *
				(sample.z - fit2_params[2]));
		C = (fit2_params[7] * (sample.x - fit2_params[0])) + (fit2_params[8] * (sample.y - fit2_params[1])) + (fit2_params[5]    *
				(sample.z - fit2_params[2]));
		length = safe_sqrt(A * A + B * B + C * C);
		residual = _params.radius - length;
		fit2 += residual * residual;
	}

	fit1 = safe_sqrt(fit1) / _samples_collected;
	fit2 = safe_sqrt(fit2) / _samples_collected;

	if (fit1 > _fitness && fit2 > _fitness) {
		_sphere_lambda *= lma_damping;

	} else if (fit2 < _fitness && fit2 < fit1) {
		_sphere_lambda /= lma_damping;
		memcpy(fit1_params, fit2_params, sizeof(fit1_params));
		fitness = fit2;

	} else if (fit1 < _fitness) {
		fitness = fit1;
	}

	//--------------------Levenberg-Marquardt-part-ends-here--------------------------------//
	if (isfinite(fitness) && fitness < _fitness) {
		_fitness = fitness;
		_params.offset.x = fit1_params[0];
		_params.offset.y = fit1_params[1];
		_params.offset.z = fit1_params[2];
		_params.diag.x= fit1_params[3];
		_params.diag.y = fit1_params[4];
		_params.diag.z = fit1_params[5];
		_params.offdiag.x= fit1_params[6];
		_params.offdiag.y = fit1_params[7];
		_params.offdiag.z = fit1_params[8];
		return true;

	} else {
		return false;
	}
}

// Returns calibrate_return_error if any parameter is not finite
// Logs if parameters are out of range
#define MAG_MAX_OFFSET_LEN 1300
bool  CompassCalibrator::check_calibration_result(float offset_x, float offset_y, float offset_z,
				float sphere_radius,
				float diag_x, float diag_y, float diag_z,
				float offdiag_x, float offdiag_y, float offdiag_z)
{
	float must_be_finite[] = {offset_x, offset_y, offset_z,
							  sphere_radius,
							  diag_x, diag_y, diag_z,
							  offdiag_x, offdiag_y, offdiag_z};

	float should_be_not_huge[] = {offset_x, offset_y, offset_z};
	float should_be_positive[] = {sphere_radius, diag_x, diag_y, diag_z};

	// Make sure every parameter is finite
	const int num_finite = sizeof(must_be_finite) / sizeof(*must_be_finite);
	for (unsigned i = 0; i < num_finite; ++i) {
		if (!isfinite(must_be_finite[i])) {
                        printf("ERROR: calibration (sphere NaN)");
                        return false;
		}
	}

	// Notify if offsets are too large
	const int num_not_huge = sizeof(should_be_not_huge) / sizeof(*should_be_not_huge);
	for (unsigned i = 0; i < num_not_huge; ++i) {
		if (abs(should_be_not_huge[i]) > MAG_MAX_OFFSET_LEN) {
			printf("Warning: mag with large offsets");
			break;
		}
	}

	// Notify if a parameter which should be positive is non-positive
	const int num_positive = sizeof(should_be_positive) / sizeof(*should_be_positive);
	for (unsigned i = 0; i < num_positive; ++i) {
		if (should_be_positive[i] <= 0.0f) {
			printf("Warning: mag with non-positive scale");
			break;
		}
	}

	return true;
}

#endif

//////////////////////////////////////////////////////////
//////////// CompassSample public interface //////////////
//////////////////////////////////////////////////////////

#define COMPASS_CAL_SAMPLE_SCALE_TO_FIXED(__X) ((int16_t)constrain_float(roundf(__X*8.0f), INT16_MIN, INT16_MAX))
#define COMPASS_CAL_SAMPLE_SCALE_TO_FLOAT(__X) (__X/8.0f)

Vector3f CompassCalibrator::CompassSample::get() const {
    return Vector3f(COMPASS_CAL_SAMPLE_SCALE_TO_FLOAT(x),
                    COMPASS_CAL_SAMPLE_SCALE_TO_FLOAT(y),
                    COMPASS_CAL_SAMPLE_SCALE_TO_FLOAT(z));
}

void CompassCalibrator::CompassSample::set(const Vector3f &in) {
    x = COMPASS_CAL_SAMPLE_SCALE_TO_FIXED(in.x);
    y = COMPASS_CAL_SAMPLE_SCALE_TO_FIXED(in.y);
    z = COMPASS_CAL_SAMPLE_SCALE_TO_FIXED(in.z);
}


void CompassCalibrator::AttitudeSample::set_from_ahrs(void) {
    const Matrix3f &dcm = AP::ahrs().get_DCM_rotation_body_to_ned();
    float roll_rad, pitch_rad, yaw_rad;
    dcm.to_euler(&roll_rad, &pitch_rad, &yaw_rad);
    roll = constrain_int16(127 * (roll_rad / M_PI), -127, 127);
    pitch = constrain_int16(127 * (pitch_rad / M_PI_2), -127, 127);
    yaw = constrain_int16(127 * (yaw_rad / M_PI), -127, 127);
}

Matrix3f CompassCalibrator::AttitudeSample::get_rotmat(void) {
    float roll_rad, pitch_rad, yaw_rad;
    roll_rad = roll * (M_PI / 127);
    pitch_rad = pitch * (M_PI_2 / 127);
    yaw_rad = yaw * (M_PI / 127);
    Matrix3f dcm;
    dcm.from_euler(roll_rad, pitch_rad, yaw_rad);
    return dcm;
}

/*
  calculate the implied earth field for a compass sample and compass
  rotation. This is used to check for consistency between
  samples. 

  If the orientation is correct then when rotated the same (or
  similar) earth field should be given for all samples. 

  Note that this earth field uses an arbitrary north reference, so it
  may not match the true earth field.
 */
Vector3f CompassCalibrator::calculate_earth_field(CompassSample &sample, enum Rotation r)
{
    Vector3f v = sample.get();

    // convert the sample back to sensor frame
    v.rotate_inverse(_orientation);

    // rotate to body frame for this rotation
    v.rotate(r);

    // apply offsets, rotating them for the orientation we are testing
    Vector3f rot_offsets = _params.offset;
    rot_offsets.rotate_inverse(_orientation);

    rot_offsets.rotate(r);
    
    v += rot_offsets;

    // rotate the sample from body frame back to earth frame
    Matrix3f rot = sample.att.get_rotmat();

    Vector3f efield = rot * v;

    // earth field is the mag sample in earth frame
    return efield;
}

/*
  calculate compass orientation using the attitude estimate associated
  with each sample, and fix orientation on external compasses if
  the feature is enabled
 */
bool CompassCalibrator::calculate_orientation(void)
{
    if (!_check_orientation) {
        // we are not checking orientation
        return true;
    }

    float variance[ROTATION_MAX] {};

    for (enum Rotation r = ROTATION_NONE; r<ROTATION_MAX; r = (enum Rotation)(r+1)) {
        // calculate the average implied earth field across all samples
        Vector3f total_ef {};
        for (uint32_t i=0; i<_samples_collected; i++) {
            Vector3f efield = calculate_earth_field(_sample_buffer[i], r);
            total_ef += efield;
        }
        Vector3f avg_efield = total_ef / _samples_collected;

        // now calculate the square error for this rotation against the average earth field
        for (uint32_t i=0; i<_samples_collected; i++) {
            Vector3f efield = calculate_earth_field(_sample_buffer[i], r);
            float err = (efield - avg_efield).length_squared();
            // divide by number of samples collected to get the variance
            variance[r] += err / _samples_collected;
        }
    }

    // find the rotation with the lowest variance
    enum Rotation besti = ROTATION_NONE;
    float bestv = variance[0];
    for (enum Rotation r = ROTATION_NONE; r<ROTATION_MAX; r = (enum Rotation)(r+1)) {
        if (variance[r] < bestv) {
            bestv = variance[r];
            besti = r;
        }
    }

    // consider this a pass if the best orientation is 2x better
    // variance than 2nd best
    const float variance_threshold = 2.0;
    
    float second_best = besti==ROTATION_NONE?variance[1]:variance[0];
    enum Rotation besti2 = ROTATION_NONE;
    for (enum Rotation r = ROTATION_NONE; r<ROTATION_MAX; r = (enum Rotation)(r+1)) {
        if (!rotation_equal(besti, r)) {
            if (variance[r] < second_best) {
                second_best = variance[r];
                besti2 = r;
            }
        }
    }

    _orientation_confidence = second_best/bestv;
    
    bool pass;
    if (besti == _orientation) {
        // if the orientation matched then allow for a low threshold
        pass = true;
    } else {
        pass = _orientation_confidence > variance_threshold;
    }
    if (!pass) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Mag(%u) bad orientation: %u/%u %.1f", _compass_idx,
                        besti, besti2, (double)_orientation_confidence);
    } else if (besti == _orientation) {
        // no orientation change
        gcs().send_text(MAV_SEVERITY_INFO, "Mag(%u) good orientation: %u %.1f", _compass_idx, besti, (double)_orientation_confidence);
    } else if (!_is_external || !_fix_orientation) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Mag(%u) internal bad orientation: %u %.1f", _compass_idx, besti, (double)_orientation_confidence);
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "Mag(%u) new orientation: %u was %u %.1f", _compass_idx, besti, _orientation, (double)_orientation_confidence);
    }

    if (!pass) {
        set_status(COMPASS_CAL_BAD_ORIENTATION);
        return false;
    }

    if (_orientation == besti) {
        // no orientation change
        return true;
    }

    if (!_is_external || !_fix_orientation) {
        // we won't change the orientation, but we set _orientation
        // for reporting purposes
        _orientation = besti;
        set_status(COMPASS_CAL_BAD_ORIENTATION);
        return false;
    }
    
    // correct the offsets for the new orientation
    Vector3f rot_offsets = _params.offset;
    rot_offsets.rotate_inverse(_orientation);
    rot_offsets.rotate(besti);
    _params.offset = rot_offsets;

    // rotate the samples for the new orientation
    for (uint32_t i=0; i<_samples_collected; i++) {
        Vector3f s = _sample_buffer[i].get();
        s.rotate_inverse(_orientation);
        s.rotate(besti);
        _sample_buffer[i].set(s);
    }

    _orientation = besti;

    // re-run the fit to get the diagonals and off-diagonals for the
    // new orientation
    initialize_fit();
#if APPLY_PX4_FIT_ALGORITHM ==0
    run_sphere_fit();
    run_ellipsoid_fit();
#else
    run_lm_sphere_fit(_fitness, _sphere_lambda);
    run_lm_ellipsoid_fit(_fitness, _ellipsoid_lambda);
#endif
    
    return fit_acceptable();
}
#endif
