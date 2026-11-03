#ifndef SPEEDOMETER_HPP
#define SPEEDOMETER_HPP
#include <cstdint>
#include "hardware/gpio.h"

typedef struct {
    bool A{false};
    bool B{false};
    uint64_t timestamp_us{0};
} EncoderReading;

typedef struct {
    uint encoder_A_pin;
    uint encoder_B_pin;
} SpeedometerConfig;

class Speedometer{
    public:
        Speedometer(const SpeedometerConfig& config);
        void execute();
        float get_wheel_speed() const;
        
    private:
        // Harware specific constants
        static constexpr uint32_t ENCODER_PULSE_PER_REV = 12;
        static constexpr float GEAR_RATIO = 119.757575;

        // Offline Calculations
        static constexpr float PI = 3.14159265;
        static constexpr float DELTA_THETA_RAD = 2.0f * PI / ENCODER_COUNTS_PER_REV;

        // Config-specific constants
        const uint encoder_A_pin_;
        const uint encoder_B_pin_;

        // Variables
        uint8_t sign_{0};
        float wheel_speed_radps_{0.0f};
        EncoderReading current_increment_;
        EncoderReading reading_;

        void read_encoder_channels();
        void update_speed();
        void determine_sign();
};

#endif
