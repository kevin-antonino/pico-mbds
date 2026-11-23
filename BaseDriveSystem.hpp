#include "WheelSpeedController.hpp"

enum class BaseDriveSystemInitStatus{ READY, FAILED };
enum class BaseDriveSystemMode{ ACTIVE, INACTIVE, LOCKOUT };

class BaseDriveSystem{
    public:
        BaseDriveSystem();
        void initialize();
        void execute();
        void set_wheel_speed_commands(const std::array<float, 4> &wheel_speed_cmds_radps);
        std::array<float, 4> get_wheel_speeds() const { return wheel_speeds_radps_; };
        BaseDriveSystemTelemetry get_telemetry() const;
    
    private:
        std::array<WheelSpeedController, 4> wheel_ctl_;
        std::array<float, 4> wheel_speeds_radps_;
        std::array<float, 4> wheel_speed_cmds_radps_;
        BaseDriveSystemMode mode_;
};
