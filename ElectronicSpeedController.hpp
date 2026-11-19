#include "QuadratureEncoder.hpp"

class enum ElectronicSpeedControllerMode{ACTIVE, INACTIVE, LOCKOUT};
class enum ElectronicSpeedControllerInitStatus{READY, BAD_ENCODER, BAD_CONFIG};

typedef struct ElectronicSpeedControllerConfig{
    uint pwm_pin;
    uint dir_pin;
}

class ElectronicSpeedController{
    public:
        ElectronicSpeedController();
        ElectronicSpeedControllerInitStatus initialize();
        void execute();
        void set_speed_command(float cmd_radps);
    
    private:
        void calculate_pwm_and_direction();
        void apply_limits();
        void write_output_pins();

        QuadratureEncoder encoder_;
        ElectronicSpeedControllerMode mode_{ControllerMode::INACTIVE}; 
        float speed_cmd_radps_{0}; 
        uint pwm_cmd_{0};
        uint direction_{1};
        float error_{0};
        float error_int_{0};
        uint n_frames_sat_{0};
        uint n_frames_lockout_{0};

        // config
        uint pwm_pin_;
        uint dir_pin_;

        // Tuning Params
        constexpr float KP = 1;
        constexpr float KI = 0.1;

        // Motor Parameters
        static constexpr float SUPPLY_VOLTAGE_V = 5.0f; // Voltage of power supply
        static constexpr float REF_VOLTAGE_V = 4.5f; // Voltage for spec values
        static constexpr float I_MOTOR_kgm = 1.0f; // Motor MOI
        static constexpr float STALL_TORQUE_Nm = 1.8f / 100.0f;
        static constexpr float NO_LOAD_SPEED_radps = 150 / 60 * 2 * 3.14159265;
        
        // Feed Forward Params
        static constexpr float KM = STALL_TORQUE / REF_VOLTAGE_V;
        static constexpr float KB = REF_VOLTAGE_V / NO_LOAD_SPEED_radps; // Back EMF Constant
        static constexpr float KFF = KM * KB / I_MOTOR_kgm;

        // Timing
        static constexpr FREQ_Hz = 100;
        static constexpr DELTA_TIME = 1 / FREQ_Hz;

        // Saturations and timeouts
        static constexpr uint SATURATION_TIMEOUT_FRAMES = 50;
        static constexpr uint RESET_TIMOUT_FRAMES = 200;
        // Counter max is 65,535 = 2^16 levels. 2.29kHz for the 150 MHz clock
        static constexpr uint PWM_COUNTS_PER_CYCLE = 4096; // 12 bits => 36.6 kHz
        static constexpr uint MAX_PWM = static_cast<uint>(REF_VOLTAGE_V / SUPPLY_VOLTAGE_V * PWM_COUNTS_PER_CYCLE); // PWM Duty cycle max 0-256
        static constexpr float MAX_SPEED_CMD_radps = 120 * 2 * 3.14159265
};
