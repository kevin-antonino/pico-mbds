#ifndef SPEEDOMETER_HPP
#define SPEEDOMETER_HPP
#include <cstdint>
#include "hardware/gpio.h"

typedef struct {
    bool count{0};
    bool delta_count{0};
    uint64_t timestamp_us{0};
} QuadratureReading;

typedef struct {
    uint encoder_AB_pin;
    uint8_t encoder_number;
    PIO pio_block;
} SpeedometerConfig;

class Speedometer{
    public:
        Speedometer(const SpeedometerConfig& config);
        void initialize() const;
        float calculate_wheel_speed();
        
    private:
        // Harware specific constants
        static constexpr uint32_t ENCODER_COUNTS_PER_REV = 12;
        static constexpr float GEAR_RATIO = 119.757575;

        // Offline Calculations
        static constexpr float PI = 3.14159265;
        static constexpr float COUNTS_TO_RAD = 2.0f * PI / ENCODER_COUNTS_PER_REV;

        // Config-specific constants
        const uint encoder_AB_pin_;

        // Variables
        QuadratureReading reading_;
        QuadratureReading prev_reading_;
        float wheel_speed_radps_{0.0f};
};

#endif
