#include "QuadratureEncoder.hpp"
#include "hardware/pwm.h"

enum class WheelSpeedControllerMode{ACTIVE, INACTIVE, LOCKOUT};
enum class WheelSpeedControllerInitStatus{READY, BAD_ENCODER, BAD_PIN};

typedef struct WheelSpeedControllerConfig{
    uint pwm_pin;
    uint dir_pin;
    QuadratureEncoderConfig encoder_config;
} WheelSpeedControllerConfig;

typedef struct WheelSpeedControllerTelemetry{
    float wheel_speed_radps{0};
    WheelSpeedControllerMode mode{WheelSpeedControllerMode::INACTIVE}; 
    float speed_cmd_radps{0}; 
    uint pwm_cmd{0};
    uint direction{1};
    float error{0};
    float error_int{0};
    uint n_frames_sat{0};
    uint n_frames_lockout{0};
} WheelSpeedControllerTelemetry;

class WheelSpeedController{
    public:
        WheelSpeedController() = default;
        WheelSpeedControllerInitStatus initialize(const WheelSpeedControllerConfig& config);
        void execute();
        void set_speed_command(float cmd_radps);
        WheelSpeedControllerMode get_mode() const { return mode_; };
        WheelSpeedControllerTelemetry get_telemetry() const;
        void lockout();
    
    private:
        void calculate_pwm_and_direction();
        void calculate_error();
        void limit_output();
        void write_output_pins();

        QuadratureEncoder encoder_;
        WheelSpeedControllerMode mode_{WheelSpeedControllerMode::INACTIVE}; 
        float speed_cmd_radps_{0}; 
        uint pwm_cmd_{0};
        uint direction_{1};
        uint last_cmd_dir_{1};
        float error_{0};
        float error_int_{0};
        uint n_frames_sat_{0};
        uint n_frames_lockout_{0};

        // config
        uint pwm_pin_;
        uint dir_pin_;

        // Tuning Params
        static constexpr float KP = 0.5;
        static constexpr float KI = 0.5;

        // Motor Parameters
        static constexpr float SUPPLY_VOLTAGE_V = 5.0f; // Voltage of power supply
        static constexpr float REF_VOLTAGE_V = 4.5f; // Voltage for spec values
        static constexpr float I_MOTOR_kgm = 1.0f; // Motor MOI
        static constexpr float STALL_TORQUE_Nm = 1.8f / 100.0f;
        static constexpr float NO_LOAD_SPEED_radps = 150.0f / 60.0f * 2.0f * 3.14159265f;
        
        // Feed Forward Params
        static constexpr float KM = STALL_TORQUE_Nm / REF_VOLTAGE_V;
        static constexpr float KB = REF_VOLTAGE_V / NO_LOAD_SPEED_radps; // Back EMF Constant
        static constexpr float KFF = KB;

        // Timing
        static constexpr uint FREQ_Hz = 100;
        static constexpr float DELTA_TIME = 1 / static_cast<float>(FREQ_Hz);

        // Saturations and timeouts
        static constexpr uint SATURATION_TIMEOUT_FRAMES = 50;
        static constexpr uint RESET_TIMOUT_FRAMES = 200;
        // Counter max is 65,535 = 2^16 levels. 2.29kHz for the 150 MHz clock
        static constexpr uint PWM_COUNTS_PER_CYCLE = 4096; // 12 bits => 36.6 kHz
        static constexpr uint MAX_PWM = static_cast<uint>(REF_VOLTAGE_V / SUPPLY_VOLTAGE_V * PWM_COUNTS_PER_CYCLE); // PWM Duty cycle max 0-256
        static constexpr float MAX_SPEED_CMD_radps = NO_LOAD_SPEED_radps;
        static constexpr float MIN_SPEED_CMD_radps =  3.14159265f / 2.0f;
};
