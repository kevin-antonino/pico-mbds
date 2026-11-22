#include "WheelSpeedController.hpp"
#include <math.h>

 WheelSpeedControllerInitStatus WheelSpeedController::initialize(){
    QuadratureEncoderInitStatus encoder_status = encoder_.initialize();
    if (encoder_status != QuadratureEncoderInitStatus::READY){
        return WheelSpeedControllerInitStatus::BAD_ENCODER;
    } else if (((dir_pin_ < 0) || (pwm_pin_ < 0)) || 
        ((dir_pin_ > 28) || (pwm_pin_ > 28))){
        return WheelSpeedControllerInitStatus::BAD_CONFIG;
    }

    // init direction Pin
    gpio_init(dir_pin_);
    gpio_set_dir(dir_pin_, true);

    // init PWM Pin
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv_int(&cfg, 1);
    pwm_config_set_wrap(&cfg, PWM_COUNTS_PER_CYCLE);
    pwm_init(pwm_gpio_to_slice_num(pwm_pin_), &cfg, true);
    gpio_set_function(pwm_pin_, GPIO_FUNC_PWM);

    // Set level and mode
    write_output_pins();
    mode_ = WheelSpeedControllerMode::ACTIVE;

    return WheelSpeedControllerInitStatus::READY;
}

void WheelSpeedController::execute(){
    encoder_.execute();
    if (mode_ == WheelSpeedControllerMode::ACTIVE){
        calculate_error();
        calculate_pwm_and_direction();
        limit_output();
    } else if (mode_ == WheelSpeedControllerMode::LOCKOUT){
        n_frames_lockout_++;
        if (n_frames_lockout_ > RESET_TIMOUT_FRAMES){
            n_frames_lockout_ = 0;
            n_frames_sat_ = 0;
            mode_ = WheelSpeedControllerMode::ACTIVE;
        }
    }
    write_output_pins();
}

void WheelSpeedController::calculate_pwm_and_direction(){
    float motor_cmd_V{0}, duty{0};
    motor_cmd_V = KP*error_ + KI*error_int_ + KFF*speed_cmd_radps_;
    duty = fminf(fabsf(motor_cmd_V) / SUPPLY_VOLTAGE_V, 1.0f);
    direction_ = (motor_cmd_V < 0.0f) ? 0 : 1;
    pwm_cmd_ = static_cast<uint>(duty * PWM_COUNTS_PER_CYCLE);
}

void WheelSpeedController::limit_output(){
    if (pwm_cmd_ < MAX_PWM){
        n_frames_sat_ = 0;
    } else {
        pwm_cmd_ = MAX_PWM;
        n_frames_sat_++;  
        if (n_frames_sat_ > SATURATION_TIMEOUT_FRAMES){
            pwm_cmd_ = 0;
            error_int_ = 0.0f; // Reset integrator
            mode_ = WheelSpeedControllerMode::LOCKOUT; // Kick controller into lockout
        }
    }
}

void WheelSpeedController::set_speed_command(float cmd){
    if (cmd > MAX_SPEED_CMD_radps){
        speed_cmd_radps_ = MAX_SPEED_CMD_radps;
    } else if (cmd < -MAX_SPEED_CMD_radps){
        speed_cmd_radps_ = -MAX_SPEED_CMD_radps;
    } else if (fabsf(cmd) < MIN_SPEED_CMD_radps) {
        speed_cmd_radps_ = 0.0f;
    } else {
        speed_cmd_radps_ = cmd;
    }
    // put a rate limit here?
}

void WheelSpeedController::calculate_error(){
    const float abs_cmd = fabsf(speed_cmd_radps_);
    uint cmd_dir = (speed_cmd_radps_ < 0.0f) ? 0 : 1;
    error_ = speed_cmd_radps_ - encoder_.get_wheel_speed();
    // Flip integrator sign if cmd sign changes
    if (cmd_dir != last_cmd_dir_) {
        error_int_ = -error_int_;  
        last_cmd_dir_ = cmd_dir;
    }
    if (abs_cmd < MIN_SPEED_CMD_radps){
        error_int_ = 0.0f;  // Reset integrator if commanding still wheels
    } else if (n_frames_sat_ < 1){ 
        error_int_ += error_ * DELTA_TIME;  // Only integrate if within limit
    }
}

void WheelSpeedController::write_output_pins(){
    gpio_put(dir_pin_, direction_);
    pwm_set_gpio_level(pwm_pin_, pwm_cmd_);
}

WheelSpeedControllerTelemetry WheelSpeedController::get_telemetry() const {
    WheelSpeedControllerTelemetry telem_struct{}; 
    telem_struct.wheel_speed_radps  = encoder_.get_wheel_speed();
    telem_struct.mode               = mode_;
    telem_struct.speed_cmd_radps    = speed_cmd_radps_; 
    telem_struct.pwm_cmd            = pwm_cmd_; 
    telem_struct.direction          = direction_;
    telem_struct.error              = error_; 
    telem_struct.error_int          = error_int_; 
    telem_struct.n_frames_sat       = n_frames_sat_; 
    telem_struct.n_frames_lockout   = n_frames_lockout_; 

    return telem_struct;
}
