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
    const uint encoder_number;
    PIO pio_block;
} QuadratureEncoderConfig;

// substep_state_t type from RP Pico Examples
typedef struct substep_state_t {
    // configuration data:
    uint calibration_data[4]; // relative phase sizes
    uint clocks_per_us;       // save the clk_sys frequency in clocks per us
    uint idle_stop_samples;   // after these samples without transitions, assume the encoder is stopped
    PIO pio;
    uint sm;

    // internal fields to keep track of the previous state:
    uint prev_trans_pos, prev_trans_us;
    uint prev_step_us;
    uint prev_low, prev_high;
    uint idle_stop_sample_count;
    int speed_2_20;
    int stopped;

    // output of the encoder update function:
    int speed;     // estimated speed in substeps per second
    uint position; // estimated position in substeps

    uint raw_step; // raw step count
} substep_state_t;

class QuadratureEncoder{
    public:
        QuadratureEncoder(const QuadratureEncoderConfig& config);
        void initialize();
        void execute();
        float get_wheel_speed const();
        
    private:
        substep_state_t state_{}; // Make sure these are all zeros

        uint step{}; // current position in raw quadrature steps. This is useful to handle steps

        // Below will be passed thru configuration 
        // variables used to keep track of incremental calibration
        uint calib_last_us, calib_last_step, calib_count;
        uint calib_sum[4], calib_data[4];

        // position reset value
        uint position_reset;

        // Harware specific constants and offline calculations 
        static constexpr uint32_t ENCODER_COUNTS_PER_REV = 12;
        static constexpr float GEAR_RATIO = 119.757575;
        static constexpr float PI = 3.14159265;
        static constexpr float ENCODER_COUNTS_TO_WHEEL_RAD = 2.0f * PI / ENCODER_COUNTS_PER_REV * GEAR_RATIO;

        // Config-specific constants
        const uint encoder_AB_pin_;

        float wheel_speed_radps_{0.0f};
};

#endif
