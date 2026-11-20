#ifndef QUADRATUREENCODER_HPP
#define QUADRATUREENCODER_HPP

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/timer.h"

enum class QuadratureEncoderInitStatus {READY, TOO_MANY_ENCODERS, NO_STATE_MACHINE, BAD_PIN};

typedef struct QuadratureEncoderConfig{
    uint encoder_AB_pin; // Location of Pin A. Pin B = Pin A + 1
    uint calibration_data[4]; // relative phase sizes
    bool invert;
} QuadratureEncoderConfig;

// substep_state to handle RP examples code
typedef struct substep_state_t { 
    // output of the encoder execute function:
    uint pos_substep{};        // estimated position in substeps
    uint raw_step{};           // raw step count

    // relative phase sizes for calibrating
    uint calibration_data[4]; 

    // internal fields to keep track of the previous state:
    uint prev_trans_pos{}, prev_trans_us{};
    uint prev_step_us{};
    uint prev_low{}, prev_high{};
    uint idle_stop_sample_count{};
    int speed_2_20{};
    int stopped{};
} substep_state_t;

class QuadratureEncoder{
    public:
        explicit QuadratureEncoder(const QuadratureEncoderConfig& config);
        QuadratureEncoderInitStatus initialize();
        void execute();
        float get_wheel_speed() const { return wheel_speed_radps_; }
        substep_state_t get_state() const { return state_; }
        
    private:
        void read_pio_data(uint *step, uint *step_us, uint *transition_us, int *forward);
        uint get_step_start_transition_pos(uint step) const;
        int substep_calc_speed(int delta_substep, int delta_us) const;

        inline static uint n_encoders_ = 0; // Increments for each constructor called (decrements when destroyed)
        uint encoder_AB_pin_; 
        substep_state_t state_{}; // Make sure these are all zeros on init
        uint encoder_number_{};   // Unique to each instance
        PIO pio_block_{};
        uint state_machine_{};
        uint clock_cycles_per_us_{};
        bool invert_{};
        float wheel_speed_radps_{0.0f};

        // Harware specific constants and offline calculations 
        static constexpr uint32_t ENCODER_EDGES_PER_REV = 12;
        static constexpr float GEAR_RATIO = 119.757575; // in : out
        static constexpr float SUBSTEPS_TO_WHEEL_RAD = 2.0f * 3.14159265f / (ENCODER_EDGES_PER_REV * 64.0f * GEAR_RATIO);
        static constexpr uint IDLE_STOP_SAMPLES = 10;
};

#endif
