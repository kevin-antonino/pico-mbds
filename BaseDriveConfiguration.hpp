#include "WheelSpeedController.hpp"

constexpr WheelSpeedControllerConfig front_left_config = {
    .pwm_pin = 6;
    .dir_pin = 7;
    .encoder_config = {
        .encoder_AB_pin = 16;
        .calibration_data = {0, 64, 128, 192}
        .invert = false;
    }
};

constexpr WheelSpeedControllerConfig front_right_config = {
    .pwm_pin = 8;
    .dir_pin = 9;
    .encoder_config = {
        .encoder_AB_pin = 18;
        .calibration_data = {0, 64, 128, 192}
        .invert = false;
    }
};

constexpr WheelSpeedControllerConfig rear_left_config = {
    .pwm_pin = 10;
    .dir_pin = 11;
    .encoder_config = {
        .encoder_AB_pin = 20;
        .calibration_data = {0, 64, 128, 192}
        .invert = false;
    }
};

constexpr WheelSpeedControllerConfig rear_right_config = {
    .pwm_pin = 12;
    .dir_pin = 13;
    .encoder_config = {
        .encoder_AB_pin = 26;
        .calibration_data = {0, 64, 128, 192}
        .invert = false;
    }
};


constexpr std::array<WheelSpeedControllerConfig, 4> BASE_DRIVE_CONFIG{front_left_config, front_right_config, rear_left_config, rear_right_config};
