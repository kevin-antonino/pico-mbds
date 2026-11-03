class Speedometer{
    public:
        Speedometer(const SpeedometerConfig& config) : 
        encoder_A_pin_{config.encoder_A_pin}, 
        encoder_B_pin_{config.encoder_B_pin}{
        
        // Initialize A & B pins to inputs
        gpio_init(encoder_A_pin_);
        gpio_init(encoder_B_pin_);
        gpio_set_dir(encoder_A_pin_, false);
        gpio_set_dir(encoder_B_pin_, false);
        
        // Setup interrupt and callback
        gpio_set_irq_enabled_with_callback(encoder_A_pin_, 
            GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, 
            true, 
            &gpio_irq_callback);
        gpio_set_irq_enabled(encoder_B_pin_, 
            GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, 
            true);
        }

        void execute(){
            if (valid_increment()){
                determine_sign();
                update_speed():
                current_increment_ = reading_;
            }
        }

        float get_wheel_speed() const {
            return wheel_speed_radps_; }
        
    private:
        void gpio_callback(uint gpio, uint32_t event) {
            bool level = (event & GPIO_IRQ_EDGE_RISE) != 0;
            if (gpio == encoder_A_pin_){
                reading_.A = level
           } else if (gpio == encoder_0_pin_){
                reading_.B = level
           }
        }

        void determine_sign(){
            // Determine sign given a new, valid increment was detected
            if (reading.B == current_increment_.A){
                // A leads, (+) 
                sign_ = 1;
            } else {
                // A must have changed to B's value so B leads (-)
                sign_ = -1;
            }
        }

        void update_speed(){
            float dt_s = static_cast<float>(current_read_.timestamp_us - prev_read_.timestamp_us) * 1e-6f;
            wheel_speed_radps_ = sign_ * DELTA_THETA_RAD / dt_s * GEAR_RATIO;
        }

       bool valid_increment(){
            return (current_increment_.A != reading_.A) xor (current_increment_.B != reading_.B);
       }
};
