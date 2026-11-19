#include "ElectronicSpeedController.hpp"

ElectronicSpeedController::ElectronicSpeedController(const ElectronicSpeedControllerConfig& config){
    pwm_pin_ = config.pwm_pin;
    dir_pin_ = config.dir_pin;
}

 ControllerInitStatus ElectronicSpeedController::initialize(){
    EncoderStatus encoder_status = encoder_.initialize();
    if (status != EncoderStatus::READY){
        return ControllerInitStatus::BAD_ENCODER;
    } else if (dir_pin_ < 0) || (pwm_pin_ < 0) || 
        (dir_pin_ > 28) || (pwm_pin_ > 28){
        return ControllerInitStatus::BAD_CONFIG;
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
    mode_ = ControllerMode::ACTIVE;

    return ControllerInitStatus::READY;
}

void ElectronicSpeedController::execute(){
    encoder_.execute();
    if (mode_ == ControllerMode::ACTIVE){
        calculate_pwm_and_direction();
        apply_limits();
    } else if (mode_ == ControllerMode::LOCKOUT){
        n_frames_lockout_++;
        if (n_frames_lockout_ > RESET_TIMOUT_FRAMES){
            n_frames_lockout_ = 0;
            n_frames_sat_ = 0;
            mode_ = ControllerMode::ACTIVE;
        }
    }
    write_output_pins();
}

void ElectronicSpeedController::calculate_pwm_and_direction(){
    const float motor_cmd_V, duty;
    error_ = speed_cmd_radps_ - encoder_.get_wheel_speed();
    motor_cmd_V = KP*error_ + KI*error_int_ + KFF*speed_cmd_radps_;
    duty = fminf(fabsf(motor_cmd_V) / SUPPLY_VOLTAGE_V, 1.0f);
    direction_ = (motor_cmd_V < 0.0f) ? 0 : 1;
    pwm_cmd_ = static_cast<uint>(duty * PWM_COUNTS_PER_CYCLE);
}

void ElectronicSpeedController::apply_limits(){
    if (pwm_cmd_ < MAX_PWM){
        error_int_ += error_ * DELTA_TIME; 
        n_frames_sat_ = 0;
    } else {
        pwm_cmd_ = MAX_PWM;
        n_frames_sat_++;  
        if (n_frames_sat_ > SATURATION_TIMEOUT_FRAMES){
            pwm_cmd_ = 0;
            error_int_ = 0.0f;
            mode_ = ControllerMode::LOCKOUT;
        }
    }
}

void ElectronicSpeedController::set_speed_command(float cmd){
    if (fabsf(cmd) > MAX_SPEED_CMD_radps){
        speed_cmd_radps_ = MAX_SPEED_CMD_radps;
    } else {
        speed_cmd_radps_ = cmd;
    }
    // put a rate limit here?
}

void ElectronicSpeedController::write_output_pins(){
    gpio_put(dir_pin_, direction_);
    pwm_set_gpio_level(pwm_pin_, pwm_cmd_);
}
