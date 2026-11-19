#include <stdio.h>
#include "pico/stdlib.h"
#include "QuadratureEncoder.hpp"

int main() {
    stdio_init_all();
    sleep_ms(2000);

    QuadratureEncoderConfig config;

    // Set up config
    config.encoder_AB_pin = 15;
    config.calibration_data[0] = 0;
    config.calibration_data[1] = 64;
    config.calibration_data[2] = 128;
    config.calibration_data[3] = 192;
    config.invert = false;

    // VCC pin to 3.3v
    uint VCC_PIN = 0;
    gpio_init(VCC_PIN);
    gpio_set_dir(VCC_PIN, GPIO_OUT);
    gpio_put(VCC_PIN, 1);

    QuadratureEncoder encoder(config);
    EncoderStatus status = encoder.initialize();
    if (status != EncoderStatus::Ready) {
        printf("encoder init failed: %d\n", static_cast<int>(status));
        while (true) tight_loop_contents();
    }

    constexpr float RAD_TO_DEG = 57.2957795f;
    absolute_time_t next = get_absolute_time();
    int tick = 0;

    while (true) {
        encoder.execute();                // 100 Hz

        if (++tick >= 10) {               // print at 10 Hz
            tick = 0;
            printf("Speed in deg/s: %10.1f\n", encoder.get_wheel_speed() * RAD_TO_DEG);
            substep_state_t state = encoder.get_state();
            printf("pos: %-10u raw_steps: %-10d\n", state.pos_substep, (int)state.raw_step);
        }

        next = delayed_by_ms(next, 20);   // fixed period, no drift
        sleep_until(next);
    }
}
