//
// Created by LHospital
//

#include "../include/PIDController/PID_Controller.h"

PID_Controller::PID_Controller() {
    E_prev_ = 0;
}

PID_Controller::PID_Controller(double kp, double kd, double ki) {
    kp_ = kp;
    kd_ = kd;
    ki_ = ki;
    E_prev_ = 0;
}

void PID_Controller::setPID(double kp, double kd, double ki) {
    kp_ = kp;
    kd_ = kd;
    ki_ = ki;
}

double PID_Controller::compute(double err_now) {
    double E_now = -10 * err_now;
    double pid_out  = kp_*E_now + kd_*(E_now - E_prev_);
    E_prev_ = E_now;
    // 计算角度
    double angle_out = pid_out + 90;

    if(angle_out >= 150) {
        angle_out = 150;
    }
    if(angle_out <= 30) {
        angle_out = 30;
    }

    return angle_out;
}