#pragma once

#include "FreeRTOS.h" /* Must come first. */
#include "task.h"     /* RTOS task related API prototypes. */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "hardware/pwm.h"
#include <pico/divider.h>

#include "quadrature_encoder_substep.pio.h"

//
// ---- quadrature encoder interface with sub-step accuracy
//
// At low speeds, or high sample rates, a quadrature encoder can produce a small
// number of steps per sample, which makes any speed estimate made from those
// counts to be very noisy.
//
// This code uses a PIO program to count steps like the standard quadrature
// encoder example, but also mark the timestamp of the last step transition,
// which then allows the sampling code to measure the actual speed by computing
// the time passed between transitions and the "distance" traveled. See
// README.md for more details

class substep_state_t
{
public:
    /**
     * main functions to be used by user code
     */

    /*!
     * \brief Constructor
     *
     * \param pin_ab: GPIO pin to connect the A phase of the encoder. The B phase must be connected to the next pin.
     * \param pio: pio selected
     * \param sm: state machine selected
     */
    substep_state_t(uint8_t pin_ab, PIO pio, int sm);

    /*!
     * \brief Constructor
     *
     * \param pin_ab: GPIO pin to connect the A phase of the encoder. The B phase must be connected to the next pin.
     */
    substep_state_t(uint8_t pin_ab);

    /*!
     * \brief Release memory (as needed):
     */
    virtual ~substep_state_t();

    void begin();

    // read the PIO data and update the speed / position estimate
    void substep_update();

    // function to measure the difference between the different steps on the encoder
    void substep_calibrate_phases(PIO pio, uint sm);

    // set the phase size calibration, use the "substep_calibrate_phases" function
    // to get the values. Many encoders (especially low cost ones) have phases that
    // don't have the same size. To get good substep accuracy, the code should know
    // about this. This is specially important at low speeds with encoders that have
    // big phase size differences
    void substep_set_calibration_data(int step0, int step1, int step2);

    int getSpeed() { return speed; }

    uint getPosition() { return position; }

    uint getRawStep() { return raw_step; }

    uint last_position = 0;
    int last_speed = 0;
    uint last_raw_step = 0;

    PIO pio;
    uint sm;
    uint pin_ab;

private:
    /**
     * internal helper functions (not to be used by user code)
     */

    // initialize the substep state structure and start PIO code
    void substep_init();

    void read_pio_data(uint *step, uint *step_us, uint *transition_us, int *forward);

    // get the sub-step position of the start of a step
    uint get_step_start_transition_pos(uint step);

    // compute speed in "sub-steps per 2^20 us" from a delta substep position and
    // delta time in microseconds. This unit is cheaper to compute and use, so we
    // only convert to "sub-steps per second" once per update, at most
    int substep_calc_speed(int delta_substep, int delta_us);

    // configuration data:
    uint calibration_data[4]; // relative phase sizes
    uint clocks_per_us;       // save the clk_sys frequency in clocks per us
    uint idle_stop_samples;   // after these samples without transitions, assume the encoder is stopped


    // internal fields to keep track of the previous state:
    uint prev_trans_pos,
        prev_trans_us;
    uint prev_step_us;
    uint prev_low, prev_high;
    uint idle_stop_sample_count;
    int speed_2_20;
    int stopped;

    // output of the encoder update function:
    int speed;     // estimated speed in substeps per second
    uint position; // estimated position in substeps
    uint raw_step; // raw step count
};

/*
// this section is an optional DC motor control code to help test an encoder
// attached to a DC motor (and calibrate phase sizes)

const int dir_pin = 5;
const int pwm_pin = 7;

static void set_pwm(float value)
{
    int ivalue = value * 6250;
    if (ivalue < 0) {
        gpio_put(dir_pin, true);
        pwm_set_gpio_level(pwm_pin, 6250+ivalue);
    } else {
        gpio_put(dir_pin, false);
        pwm_set_gpio_level(pwm_pin, ivalue);
    }
}

static void init_pwm(void)
{
    gpio_init(dir_pin);
    gpio_set_dir(dir_pin, true);
    gpio_init(pwm_pin);
    gpio_set_dir(pwm_pin, true);

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv_int(&cfg, 1);
    pwm_config_set_wrap(&cfg, 6250);
    pwm_init(pwm_gpio_to_slice_num(pwm_pin), &cfg, true);

    gpio_set_function(pwm_pin, GPIO_FUNC_PWM);
    set_pwm(0);
}
*/
