#include "QuadratureEncoder.hpp"
#include "quadrature_encoder_substep.pio.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"

QuadratureEncoder::QuadratureEncoder(const QuadratureEncoderConfig& config){
    n_encoders_++;
    encoder_number_ = n_encoders_ - 1; // Zero indexed 

    // Handle configuration data
    invert_ = config.invert;
    encoder_AB_pin_ = config.encoder_AB_pin;
    for (int i{}; i < 4; ++i){
        state_.calibration_data[i] = config.calibration_data[i];
    }
}

EncoderStatus QuadratureEncoder::initialize() {
    int forward;

    // Figure out which state machine to use
    state_machine_ = encoder_number_ % 4; 

    // Figure out which PIO block to use
    if (encoder_number_ < 4){
        pio_block_ = pio0;
    } else if (encoder_number_ < 8){
        pio_block_ = pio1;
    } else {
        return EncoderStatus::TOO_MANY_ENCODERS;
    }
    
    // Claim PIO Block & state_machines if unclaimed
    // TODO: No check to see if program loaded right
    if (pio_can_add_program(pio_block_, &quadrature_encoder_substep_program)){
        pio_add_program(pio_block_, &quadrature_encoder_substep_program);
        for (int i = 0; i < 4; i++){
            if (!pio_sm_is_claimed(pio_block_, i)){
                // claim all SMs to be used by this program
                pio_sm_claim(pio_block_, i);
            } else {
                return EncoderStatus::NO_STATE_MACHINE;
            }
        }
    }

    // TODO: at check for pin in some range
    
    // start "stopped" so that we don't use stale data to compute speeds
    state_.stopped = 1;

    // cache the PIO cycles per us
    clock_cycles_per_us_ = (clock_get_hz(clk_sys) + 500000) / 1000000;

    // initialize PIO program using this encoder's state_machine
    quadrature_encoder_substep_program_init(pio_block_, state_machine_, encoder_AB_pin_);
    
    // initialize the previous state
    read_pio_data(&state_.raw_step, &state_.prev_step_us, &state_.prev_trans_us, &forward);
    state_.pos_substep = get_step_start_transition_pos(state_.raw_step) + 32;
    state_.prev_low = get_step_start_transition_pos(state_.raw_step);
    state_.prev_high = get_step_start_transition_pos(state_.raw_step + 1);

    return EncoderStatus::READY;
}

// read the PIO data and update the speed / position estimate
// This is basically substep_update() in the example code
void QuadratureEncoder::execute() {
    uint new_raw_step, step_us, transition_us, transition_pos, low, high;
    int forward, speed_high, speed_low;

    // read the current encoder state_from the PIO
    read_pio_data(&new_raw_step, &step_us, &transition_us, &forward);

    // from the current step we can get the low and high boundaries in substeps
    // of the current position
    low = get_step_start_transition_pos(new_raw_step);
    high = get_step_start_transition_pos(new_raw_step + 1);

    // if we were not stopped, but the last transition was more than
    // "idle_stop_samples" ago, we are stopped now
    if (new_raw_step == state_.raw_step)
        state_.idle_stop_sample_count++;
    else
        state_.idle_stop_sample_count = 0;

    if (!state_.stopped && state_.idle_stop_sample_count >= IDLE_STOP_SAMPLES) {
        wheel_speed_radps_ = 0;
        state_.speed_2_20 = 0;
        state_.stopped = 1;
    }

    // when we are at a different step now, there is certainly a transition
    if (state_.raw_step != new_raw_step) {
        // the transition position depends on the direction of the move
        transition_pos = forward ? low : high;

        // if we are not stopped, that means there is valid previous transition
        // we can use to estimate the current speed
        if (!state_.stopped)
            state_.speed_2_20 = substep_calc_speed(transition_pos - state_.prev_trans_pos, transition_us - state_.prev_trans_us);

        // if we have a transition, we are not stopped now
        state_.stopped = 0;
        // save the timestamp and position of this transition to use later to
        // estimate speed
        state_.prev_trans_pos = transition_pos;
        state_.prev_trans_us = transition_us;
    }

    // if we are stopped, speed is zero and the position estimate remains
    // constant. If we are not stopped, we have to update the position and speed
    if (!state_.stopped) {
        // although the current step doesn't give us a precise position, it does
        // give boundaries to the position, which together with the last
        // transition gives us boundaries for the speed value. This can be very
        // useful especially in two situations:
        // - we have been stopped for a while and start moving quickly: although
        //   we only have one transition initially, the number of steps we moved
        //   can already give a non-zero speed estimate
        // - we were moving but then stop: without any extra logic we would just
        //   keep the last speed for a while, but we know from the step
        //   boundaries that the speed must be decreasing

        // if there is a transition between the last sample and now, and that
        // transition is closer to now than the previous sample time, we should
        // use the slopes from the last sample to the transition as these will
        // have less numerical issues and produce a tighter boundary
        if (state_.prev_trans_us > state_.prev_step_us && 
            (int)(state_.prev_trans_us - state_.prev_step_us) > (int)(step_us - state_.prev_trans_us)) {
            speed_high = substep_calc_speed(state_.prev_trans_pos - state_.prev_low, state_.prev_trans_us - state_.prev_step_us);
            speed_low = substep_calc_speed(state_.prev_trans_pos - state_.prev_high, state_.prev_trans_us - state_.prev_step_us);
        } else {
            // otherwise use the slopes from the last transition to now
            speed_high = substep_calc_speed(high - state_.prev_trans_pos, step_us - state_.prev_trans_us);
            speed_low = substep_calc_speed(low - state_.prev_trans_pos, step_us - state_.prev_trans_us);
        }
        // make sure the current speed estimate is between the maximum and
        // minimum values obtained from the step slopes
        if (state_.speed_2_20 > speed_high)
            state_.speed_2_20 = speed_high;
        if (state_.speed_2_20 < speed_low)
            state_.speed_2_20 = speed_low;

        // convert the speed units from "sub-steps per 2^20 us" to "sub-steps
        // per second"
        const float speed_substepps = static_cast<float>((state_.speed_2_20 * 62500LL) >> 16);
        
        // convert from substeps / sec to rad / s
        wheel_speed_radps_ = (invert_ ? -1.0f : 1.0f) * speed_substepps * SUBSTEPS_TO_WHEEL_RAD;

        // estimate the current position by applying the speed estimate to the
        // most recent transition
        state_.pos_substep = state_.prev_trans_pos + (((int64_t)state_.speed_2_20 * (step_us - transition_us)) >> 20);

        // make sure the position estimate is between "low" and "high", as we
        // can be sure the actual current position must be in this range
        if ((int)(state_.pos_substep - high) > 0)
            state_.pos_substep = high;
        else if ((int)(state_.pos_substep - low) < 0)
            state_.pos_substep = low;
    }

    // save the current values to use on the next sample
    state_.prev_low = low;
    state_.prev_high = high;
    state_.raw_step = new_raw_step;
    state_.prev_step_us = step_us;
}


// Helper functions from quadrature_encoder_substep in pico-examples
void QuadratureEncoder::read_pio_data(uint *step, uint *step_us, uint *transition_us, int *forward) {
    int cycles;

    // get the raw data from the PIO state_machine
    quadrature_encoder_substep_get_counts(pio_block_, state_machine_, step, &cycles, step_us);

    // when the PIO program detects a transition, it sets cycles to either zero
    // (when step is incrementing) or 2^31 (when step is decrementing) and keeps
    // decrementing it on each 13 clock loop. We can use this information to get
    // the time and direction of the last transition
    if (cycles < 0) {
        cycles = -cycles;
        *forward = 1;
    } else {
        cycles = 0x80000000 - cycles;
        *forward = 0;
    }
    *transition_us = *step_us - ((cycles * 13) / clock_cycles_per_us_);
}

// get the sub-step position of the start of a step
uint QuadratureEncoder::get_step_start_transition_pos(uint step) const {
    return ((step << 6) & 0xFFFFFF00) | state_.calibration_data[step & 3];
}

// compute speed in "sub-steps per 2^20 us" from a delta substep position and
// delta time in microseconds. This unit is cheaper to compute and use, so we
// only convert to "sub-steps per second" once per update, at most
int QuadratureEncoder::substep_calc_speed(int delta_substep, int delta_us) const {
    return ((int64_t) delta_substep << 20) / delta_us;
} 
