class Speedometer{
    public:
        Speedometer(const SpeedometerConfig& config) : 
        encoder_AB_pin_{config.encoder_AB_pin}, 
        encoder_number_{config.encoder_number},
        pio_block_{config.pio_block}{
            // Put configuration checks in here
            // AB must be a GPIO / eligable pin
            // Encoder number MUST be 0-3
            // PIO Block has to be either PIO0 or PIO1 
        }

        void initialize(){
            quadrature_encoder_program_init(pio_block_, encoder_number_, encoder_AB_pin_, 0);
        }

        float calculate_wheel_speed(){
            reading_.count = quadrature_encoder_get_count(pio_block_, encoder_number_);
            reading_.timestamp_us = time_us_64();
            reading_.delta = reading_.count - prev_reading.count;

            if (reading_.delta != 0){
                float dt_s = <static_cast>float(reading_.timestamp_us - prev_reading_.timestamp_us)*1e6f;
                wheel_speed_radps_ = reading_.delta * COUNTS_TO_RAD / dt_s * GEAR_RATIO;
                prev_reading_ = reading_;
            }
            return wheel_speed_radps_;
        }
};
