#include "ElectronicSpeedController.hpp"

ElectronicSpeedController::ElectronicSpeedController(const ElectronicSpeedControllerConfig& config){
    pwm_pin_ = config.pwm_pin;
}

ElectronicSpeedController::initialize(){
    EncoderStatus status = encoder_.initialize();
    if (status == EncoderStatus::READY){
        mode_ = ControllerMode::ACTIVE;
    }

    // init PWM Pin
    gpio_init(pwm_pin_);
    gpio_set_dir(pwm_pin_, true);
    
    // Handle pwm_config
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv_int(&cfg, 1);
    pwm_config_set_wrap(&cfg, PWM_COUNTS_PER_CYCLE);
    pwm_init(pwm_gpio_to_slice_num(pwm_pin_), &cfg, true);
    gpio_set_function(pwm_pin_, GPIO_FUNC_PWM);

    // Set level (0)
    set_pwm_output();
}

void ElectronicSpeedController::execute(){
    encoder_.execute();
    if (mode_ == ControllerMode::ACTIVE){
        generate_pwm_command();
        apply_limits();
    } else if (mode_ == ControllerMode::LOCKOUT){
        n_frames_lockout_++;
        if (n_frames_lockout_ > RESET_TIMOUT_FRAMES){
            n_frames_lockout_ = 0;
            n_frames_sat_ = 0;
            mode_ = ControllerMode::ACTIVE;
        }
    }
    set_pwm_output();
}

void ElectronicSpeedController::generate_pwm_command(){
    error_ = speed_cmd_radps_ - encoder_.get_wheel_speed();
    const float motor_cmd_V = KP*error_ + KI*error_int_ + KFF*speed_cmd_radps_;
    direction_ = (motor_cmd_V < 0.0f) ? -1 : 1;
    pwm_cmd_ = static_cast<uint>(fabsf(motor_cmd_V) / SUPPLY_VOLTAGE_V * PWM_COUNTS_PER_CYCLE);
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
}

void ElectronicSpeedController::set_pwm_output(){
    pwm_set_gpio_level(pwm_pin_, pwm_cmd_);
}
