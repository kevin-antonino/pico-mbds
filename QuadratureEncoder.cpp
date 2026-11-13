#include QuadratureEncoder.hpp

QuadratureEncoder::QuadratureEncoder(const QuadratureEncoderConfig& config){
    // Put configuration checks in here
    // AB must be a GPIO / eligable pin
    // Encoder number MUST be 0-3
    // PIO Block has to be either PIO0 or PIO1 
    
    // Get unique encoder number (0-3)
    num_encoders_++;
    encoder_number_ = num_encoders_ - 1; // Zero indexed 
    state_machine_ = encoder_number_ % 4; 
    
    // start "stopped" so that we don't use stale data to compute speeds
    state_.stopped = 1;
    
    // start with equal phase size calibration
    calibration_data_[0] = 0;
    calibration_data_[1] = 64;
    calibration_data_[2] = 128;
    calibration_data_[3] = 192;
    
    // cache the PIO cycles per us
    clock_cycles_per_us_ = (clock_get_hz(clk_sys) + 500000) / 1000000;
}

    void initialize() {
        // Figure out which PIO block to use
        if (encoder_number_ < 4){
            pio_block_ = pio0;
        } else if (encoder_number_ < 8){
            pio_block_ = pio1;
        } else {
            // raise an error
        }
        
        // Claim PIO Block & state machines if unclaimed
        if (pio_can_add_program(pio_block_, &pico_encoder_program)){
            pio_add_program(pio_block_, &pico_encoder_program);
            for (int i = 0; i < 4; i++){
                if (!pio_sm_is_claimed(pio_block_, i)){
                    // claim all SMs to be used by this program
                    pio_sm_claim(pio_block_, i);
                } else {
                    // raise an Error
                }
            }
        }
        
        // initialize PIO program using this encoder's state machine
        quadrature_encoder_substep_program_init(pio_block_, state_machine_, pin_a);
        
        // initialize the previous state
        read_pio_data(&state_.substep_count, &state.prev_step_us, &state.prev_trans_us, &forward);
        state_.pos_substep = get_step_start_transition_pos(state_, state_.substep_count) + 32;
    }
    
void read_pio_data(uint *step, uint *step_us, uint *transition_us, int *forward) {
    int cycles;

    // get the raw data from the PIO state machine
    quadrature_encoder_substep_get_counts(&pio_block_, &state_machine_, step, &cycles, step_us);

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
uint get_step_start_transition_pos(uint step) {
    return ((step << 6) & 0xFFFFFF00) | state_.calibration_data[step & 3];
}

// compute speed in "sub-steps per 2^20 us" from a delta substep position and
// delta time in microseconds. This unit is cheaper to compute and use, so we
// only convert to "sub-steps per second" once per update, at most
static int substep_calc_speed(int delta_substep, int delta_us) {
    return ((int64_t) delta_substep << 20) / delta_us;
} 

// read the PIO data and update the speed / position estimate
// This is basically substep_update() in the example
void execute() {
    uint new_substep_count, step_us, transition_us, transition_pos, low, high;
    int forward, speed_high, speed_low;

    // read the current encoder state from the PIO
    read_pio_data(&new_substep_count, &step_us, &transition_us, &forward);

    // from the current step we can get the low and high boundaries in substeps
    // of the current position
    low = get_step_start_transition_pos(new_substep_count);
    high = get_step_start_transition_pos(new_substep_count + 1);

    // if we were not stopped, but the last transition was more than
    // "idle_stop_samples" ago, we are stopped now
    if (new_substep_count == state_.substep_count)
        state_.idle_stop_sample_count++;
    else
        state_.idle_stop_sample_count = 0;

    if (!state_.stopped && state.idle_stop_sample_count >= IDLE_STOP_SAMPLES) {
        state_.speed = 0;
        state_.speed_2_20 = 0;
        state_.stopped = 1;
    }

    // when we are at a different step now, there is certainly a transition
    if (state_.substep_count != new_substep_count) {
        // the transition position depends on the direction of the move
        transition_pos = forward ? low : high;

        // if we are not stopped, that means there is valid previous transition
        // we can use to estimate the current speed
        if (!state_.stopped)
            state_.speed_2_20 = substep_calc_speed(transition_pos - state.prev_trans_pos, transition_us - state.prev_trans_us);

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
        if (state_.prev_trans_us > state.prev_step_us && 
            (int)(state_.prev_trans_us - state.prev_step_us) > (int)(step_us - state.prev_trans_us)) {
            speed_high = substep_calc_speed(state_.prev_trans_pos - state_.prev_low, state.prev_trans_us - state_.prev_step_us);
            speed_low = substep_calc_speed(state_.prev_trans_pos - state_.prev_high, state.prev_trans_us - state_.prev_step_us);
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
        state_.speed = (state.speed_2_20 * 62500LL) >> 16;

        // estimate the current position by applying the speed estimate to the
        // most recent transition
        state_.pos_substep = state.prev_trans_pos + (((int64_t)state.speed_2_20 * (step_us - transition_us)) >> 20);

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
    state_.substep_count = new_substep_count;
    state_.prev_step_us = step_us;
}

