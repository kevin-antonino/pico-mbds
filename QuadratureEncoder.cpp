class QuadratureEncoder{
    public:
        QuadratureEncoder(const QuadratureEncoderConfig& config) : 
        encoder_AB_pin_{config.encoder_AB_pin}, 
        encoder_number_{config.encoder_number},
        pio_block_{config.pio_block}{
            // Put configuration checks in here
            // AB must be a GPIO / eligable pin
            // Encoder number MUST be 0-3
            // PIO Block has to be either PIO0 or PIO1 
        }

        void initialize(){
            // pio_add_program(pio, &quadrature_encoder_program); MUST be called before initialize()
            quadrature_encoder_program_init(pio_block_, encoder_number_, encoder_AB_pin_, 0);
            prev_sample_.count = quadrature_encoder_get_count(pio_block_, encoder_number_);
            prev_sample_.timestamp_us = time_us_64();
        }

        void execute(){
            current_sample_.count = quadrature_encoder_get_count(pio_block_, encoder_number_);
            current_sample_.timestamp_us = time_us_64();
            int delta_count = current_sample_.count - prev_sample_.count;

            if (delta_count != 0){
                float dt_s = <static_cast>float(current_sample_.timestamp_us - prev_sample_.timestamp_us)*1e-6f;
                if (dt_s > 0){
                    wheel_speed_radps_ = delta_count * ENCODER_COUNTS_TO_WHEEL_RAD / dt_s;
                    prev_sample_ = current_sample_; 
                }
            }
            // TODO: add 0 speed timeout
        }

        float get_wheel_speed() const {
            return wheel_speed_radps_;
        }
};
