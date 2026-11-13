#ifndef SPEEDOMETER_HPP
#define SPEEDOMETER_HPP

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "hardware/pwm.h"
#include <pico/divider.h>

#include "quadrature_encoder_substep.pio.h"

typedef struct {
    uint encoder_AB_pin;
    uint calibration_data[4]; // relative phase sizes
} QuadratureEncoderConfig;

// substep_state to handle RP examples code
typedef struct substep_state_t { 
    // output of the encoder execute function:
    int speed_substepps;     // estimated speed in substeps per second
    uint pos_substep;        // estimated position in substeps
    uint substep_count;      // raw step count

    // internal fields to keep track of the previous state:
    uint prev_trans_pos, prev_trans_us;
    uint prev_step_us;
    uint prev_low, prev_high;
    uint idle_stop_sample_count;
    int speed_2_20;
    int stopped;
} substep_state_t;

class QuadratureEncoder{
    public:
        QuadratureEncoder(const QuadratureEncoderConfig& config);
        void initialize();
        void execute();
        float get_wheel_speed const();
        
    private:
        static uint n_encoders_{}; // Increments for each constructor called (decrements when destroyed)
        substep_state_t state_{}; // Make sure these are all zeros
        uint encoder_number_{}; // Unique to each instance
        PIO pio_block_{};
        uint state_machine_{};
        uint clock_cycles_per_us_{};
        float wheel_speed_radps_{0.0f};

        // Harware specific constants and offline calculations 
        static constexpr uint32_t ENCODER_COUNTS_PER_REV = 12;
        static constexpr float GEAR_RATIO = 119.757575;
        static constexpr float ENCODER_COUNTS_TO_WHEEL_RAD = 2.0f * 3.14159265f / ENCODER_COUNTS_PER_REV * GEAR_RATIO;
        static constexpr uint IDLE_STOP_SAMPLES = 3;

        // Config-specific values
        uint encoder_AB_pin_; 
        uint calibration_data_[4]; // relative phase sizes
};

#endif
