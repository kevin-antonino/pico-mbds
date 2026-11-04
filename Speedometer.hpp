#ifndef SPEEDOMETER_HPP
#define SPEEDOMETER_HPP
#include <cstdint>
#include "hardware/gpio.h"

typedef struct {
    int count{0};
    uint64_t timestamp_us{0};
} QuadratureSample;

typedef struct {
    uint encoder_AB_pin;
    const uint encoder_number;
    PIO pio_block;
} QuadratureEncoderConfig;

class QuadratureEncoder{
    public:
        QuadratureEncoder(const QuadratureEncoderConfig& config);
        void initialize();
        void execute();
        float get_wheel_speed const();
        
    private:
        // Harware specific constants and offline calculations 
        static constexpr uint32_t ENCODER_COUNTS_PER_REV = 12;
        static constexpr float GEAR_RATIO = 119.757575;
        static constexpr float PI = 3.14159265;
        static constexpr float ENCODER_COUNTS_TO_WHEEL_RAD = 2.0f * PI / ENCODER_COUNTS_PER_REV * GEAR_RATIO;

        // Config-specific constants
        const uint encoder_AB_pin_;
        const uint encoder_number;
        PIO pio_block_;

        // Variables
        QuadratureSample prev_sample_;
        QuadratureSample current_sample_;
        float wheel_speed_radps_{0.0f};
};

#endif
