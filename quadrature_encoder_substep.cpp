/**
 * Copyright (c) 2023 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Modified and adapted for use with Delta Debugger
 * P.Beal - 2025/02/22
 */
#include "quadrature_encoder_substep.hpp"

/**
 * the quadrature encoder code starts here
 */

/**
 * internal helper functions (not to be used by user code)
 */

void substep_state_t::read_pio_data(uint *step, uint *step_us, uint *transition_us, int *forward)
{
    int cycles;

    // get the raw data from the PIO state machine
    quadrature_encoder_substep_get_counts(this->pio, this->sm, step, &cycles, step_us);

    // when the PIO program detects a transition, it sets cycles to either zero
    // (when step is incrementing) or 2^31 (when step is decrementing) and keeps
    // decrementing it on each 13 clock loop. We can use this information to get
    // the time and direction of the last transition
    if (cycles < 0)
    {
        cycles = -cycles;
        *forward = 1;
    }
    else
    {
        cycles = 0x80000000 - cycles;
        *forward = 0;
    }
    *transition_us = *step_us - ((cycles * 13) / this->clocks_per_us);
}

// get the sub-step position of the start of a step
uint substep_state_t::get_step_start_transition_pos(uint step)
{
    return ((step << 6) & 0xFFFFFF00) | this->calibration_data[step & 3];
}

// compute speed in "sub-steps per 2^20 us" from a delta substep position and
// delta time in microseconds. This unit is cheaper to compute and use, so we
// only convert to "sub-steps per second" once per update, at most
int substep_state_t::substep_calc_speed(int delta_substep, int delta_us)
{
    return ((int64_t)delta_substep << 20) / delta_us;
}

/**
 * main functions to be used by user code
 */
substep_state_t::substep_state_t(uint8_t pin_ab)
{
    PIO pio = pio0;
    int sm;
    // Find a free SM on one of the PIO's
    sm = pio_claim_unused_sm(pio, false); // don't panic
    // Try pio1 if SM not found
    if (sm < 0)
    {
        pio = pio1;
        sm = pio_claim_unused_sm(pio, true); // panic if no SM is free
    }

    substep_state_t(pin_ab, pio, sm);
}

substep_state_t::substep_state_t(uint8_t pin_ab, PIO pio, int sm)
{
    this->pin_ab = pin_ab;
    this->pio = pio;
    this->sm = sm;
}

substep_state_t::~substep_state_t()
{
    //free(this);
}

// initialize the substep state structure and start PIO code
void substep_state_t::substep_init()
{
    int forward;

    // set all fields to zero by default
    //memset(this, 0, sizeof(substep_state_t));

    // initialize the PIO program (and save the PIO reference)
    quadrature_encoder_substep_program_init(this->pio, this->sm, this->pin_ab);

    printf("quadrature encoder substep program init complete\n");
    
    // start with equal phase size calibration
    this->calibration_data[0] = 0;
    this->calibration_data[1] = 64;
    this->calibration_data[2] = 128;
    this->calibration_data[3] = 192;

    this->idle_stop_samples = 3;

    // start "stopped" so that we don't use stale data to compute speeds
    this->stopped = 1;

    // cache the PIO cycles per us
    this->clocks_per_us = (clock_get_hz(clk_sys) + 500000) / 1000000;

    // initialize the "previous state"
    read_pio_data(&this->raw_step, &this->prev_step_us, &this->prev_trans_us, &forward);

    this->position = get_step_start_transition_pos(this->raw_step) + 32;
}

// read the PIO data and update the speed / position estimate
void substep_state_t::substep_update()
{
    uint step, step_us, transition_us, transition_pos, low, high;
    int forward, speed_high, speed_low;

    // read the current encoder state from the PIO
    read_pio_data(&step, &step_us, &transition_us, &forward);

    // from the current step we can get the low and high boundaries in substeps
    // of the current position
    low = get_step_start_transition_pos(step);
    high = get_step_start_transition_pos(step + 1);

    // if we were not stopped, but the last transition was more than
    // "idle_stop_samples" ago, we are stopped now
    if (step == this->raw_step)
        this->idle_stop_sample_count++;
    else
        this->idle_stop_sample_count = 0;

    if (!this->stopped && this->idle_stop_sample_count >= this->idle_stop_samples)
    {
        this->speed = 0;
        this->speed_2_20 = 0;
        this->stopped = 1;
    }

    // when we are at a different step now, there is certainly a transition
    if (this->raw_step != step)
    {
        // the transition position depends on the direction of the move
        transition_pos = forward ? low : high;

        // if we are not stopped, that means there is valid previous transition
        // we can use to estimate the current speed
        if (!this->stopped)
            this->speed_2_20 = substep_calc_speed(transition_pos - this->prev_trans_pos, transition_us - this->prev_trans_us);

        // if we have a transition, we are not stopped now
        this->stopped = 0;
        // save the timestamp and position of this transition to use later to
        // estimate speed
        this->prev_trans_pos = transition_pos;
        this->prev_trans_us = transition_us;
    }

    // if we are stopped, speed is zero and the position estimate remains
    // constant. If we are not stopped, we have to update the position and speed
    if (!this->stopped)
    {
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
        if (this->prev_trans_us > this->prev_step_us &&
            (int)(this->prev_trans_us - this->prev_step_us) > (int)(step_us - this->prev_trans_us))
        {
            speed_high = substep_calc_speed(this->prev_trans_pos - this->prev_low, this->prev_trans_us - this->prev_step_us);
            speed_low = substep_calc_speed(this->prev_trans_pos - this->prev_high, this->prev_trans_us - this->prev_step_us);
        }
        else
        {
            // otherwise use the slopes from the last transition to now
            speed_high = substep_calc_speed(high - this->prev_trans_pos, step_us - this->prev_trans_us);
            speed_low = substep_calc_speed(low - this->prev_trans_pos, step_us - this->prev_trans_us);
        }
        // make sure the current speed estimate is between the maximum and
        // minimum values obtained from the step slopes
        if (this->speed_2_20 > speed_high)
            this->speed_2_20 = speed_high;
        if (this->speed_2_20 < speed_low)
            this->speed_2_20 = speed_low;

        // convert the speed units from "sub-steps per 2^20 us" to "sub-steps
        // per second"
        this->speed = (this->speed_2_20 * 62500LL) >> 16;

        // estimate the current position by applying the speed estimate to the
        // most recent transition
        this->position = this->prev_trans_pos + (((int64_t)this->speed_2_20 * (step_us - transition_us)) >> 20);

        // make sure the position estimate is between "low" and "high", as we
        // can be sure the actual current position must be in this range
        if ((int)(this->position - high) > 0)
            this->position = high;
        else if ((int)(this->position - low) < 0)
            this->position = low;
    }

    // save the current values to use on the next sample
    this->prev_low = low;
    this->prev_high = high;
    this->raw_step = step;
    this->prev_step_us = step_us;
}

// function to measure the difference between the different steps on the encoder
void substep_state_t::substep_calibrate_phases(PIO pio, uint sm)
{
#define sample_count 1024
// #define SHOW_ALL_SAMPLES
#ifdef SHOW_ALL_SAMPLES
    static int result[sample_count];
    int i;
#endif
    int index, cycles, clocks_per_us, calib[4];
    uint cur_us, last_us, step_us, step, last_step;
    int64_t sum[4], total;

    memset(sum, 0, sizeof(sum));

    clocks_per_us = (clock_get_hz(clk_sys) + 500000) / 1000000;

    // keep reading the PIO state in a tight loop to get all steps and use the
    // transition measures of the PIO code to measure the time of each step
    last_step = -10;
    index = -10;
    while (index < sample_count)
    {

        quadrature_encoder_substep_get_counts(pio, sm, &step, &cycles, &step_us);

        // wait until we have a transition
        if (step == last_step)
            continue;

        // synchronize the index with the lower 2 bits of the current step
        if (index < 0 && index > -4 && (step & 3) == 1)
            index = 0;

        // convert the "time since last transition" to an absolute microsecond
        // timestamp
        if (cycles > 0)
        {
            printf("error: expected forward motion\n");
            return;
        }
        cur_us = step_us + (cycles * 13) / clocks_per_us;

        // if the index is already synchronized, use the step size
        if (index >= 0)
        {
#ifdef SHOW_ALL_SAMPLES
            result[index] = cur_us - last_us;
#endif
            sum[(step - 1) & 3] += cur_us - last_us;
        }
        index++;

        last_step = step;
        last_us = cur_us;
    }

#ifdef SHOW_ALL_SAMPLES
    printf("full sample table:\n");
    for (i = 0; i < sample_count; i++)
    {
        printf("%d ", result[i]);
        if ((i & 3) == 3)
            printf("\n");
    }
#endif

    // scale the sizes to a total of 256 to be used as sub-steps
    total = sum[0] + sum[1] + sum[2] + sum[3];
    calib[0] = (sum[0] * 256 + total / 2) / total;
    calib[1] = ((sum[0] + sum[1]) * 256 + total / 2) / total;
    calib[2] = ((sum[0] + sum[1] + sum[2]) * 256 + total / 2) / total;

    // print calibration information
    printf("calibration command:\n\n");
    printf("\tsubstep_set_calibration_data(&state, %d, %d, %d);\n\n",
           calib[0], calib[1], calib[2]);
}

// set the phase size calibration, use the "substep_calibrate_phases" function
// to get the values. Many encoders (especially low cost ones) have phases that
// don't have the same size. To get good substep accuracy, the code should know
// about this. This is specially important at low speeds with encoders that have
// big phase size differences
void substep_state_t::substep_set_calibration_data(int step0, int step1, int step2)
{
    this->calibration_data[0] = 0;
    this->calibration_data[1] = step0;
    this->calibration_data[2] = step1;
    this->calibration_data[3] = step2;
}

void substep_state_t::begin(void)
{

    pio_add_program(this->pio, &quadrature_encoder_substep_program);
    substep_init();
}

/*  // example calibration code, uncomment to calibrate the encoder:

    //    // - turn on a DC motor at 50% PWM
    //    init_pwm();
    //    set_pwm(-0.5);
    //    // - wait for the motor to reach a reasonably stable speed
    //    sleep_ms(2000);
    //    // - run the phase size calibration code
    //    substep_calibrate_phases(pio, sm);
    //    // - stop the motor
    //    set_pwm(0);

    // replace this with the output of the calibration function
    substep_set_calibration_data(64, 128, 192);

    uint last_position = 0;
    int last_speed = 0;
    uint last_raw_step = 0;
    while (1)
    {

        // read the PIO and update the state data
        substep_update();

        if (last_position != state.position || last_speed != state.speed || last_raw_step != state.raw_step)
        {
            // print out the result
            printf("pos: %-10d  speed: %-10d  raw_steps: %-10d\n", state.position, state.speed, state.raw_step);
            last_position = state.position;
            last_speed = state.speed;
            last_raw_step = state.raw_step;
        }

        // run at roughly 100Hz
        sleep_ms(10);
    }
}
*/