#include "ControllerConfiguration.hpp"

BaseDriveSystem::BaseDriveSystem(){
}

BaseDriveSystemInitStatus BaseDriveSystem::initialize(){
    for (int i{}; i<4; ++i) {
        WheelSpeedControllerInitStatus status = wheel_ctl_[i].initialize(BASE_DRIVE_CONFIG[i]);
        if (status != WheelSpeedControllerInitStatus::READY){
            return BaseDriveSystemInitStatus::FAILED;
        }
    }
}

void BaseDriveSystem::execute() {
    // Current plan is to just force everything into the same mode for safety 
    if (mode_ == BaseDriveSystemMode::ACTIVE){
        for (int i{}; i<4; ++i) {
            wheel_ctl_[i].execute();
            const WheelSpeedControllerMode wheel_mode =  wheel_ctl_[i].get_mode();
            if (wheel_mode == WheelSpeedControllerMode::LOCKOUT) {
                for (int i{}; i<4; ++i) {
                    wheel_ctl_[i].lockout();
                }
                mode_ == BaseDriveSystemMode::LOCKOUT;
                return;
            }
        }
    } else if (mode_ == BaseDriveSystemMode::LOCKOUT){
        for (int i{}; i<4; ++i) {
            wheel_ctl_[i].execute();
            if (wheel_mode == WheelSpeedControllerMode::ACTIVE) {
                mode_ == BaseDriveSystemMode::ACTIVE;
            }
        }
    }
}
