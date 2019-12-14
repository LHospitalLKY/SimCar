//
// Created by LHospital
//
#ifndef _PID_CONTROLLER_H
#define _PID_CONTROLLER_H 

#include <iostream>

class PID_Controller {
public:
    PID_Controller();
    PID_Controller(double kp, double kd, double ki);
    ~PID_Controller() = default;

    /**
     * @brief: 设置PID参数
     * @pram: kp, kd, ki
     * @return:
    **/
    void setPID(double kp, double kd, double ki);

    /**
     * @brief: 根据距离车道中心的偏移，计算横向控制量
     * @pram: double err
     * @return: double controll_value;
    **/
    double compute(double err_now);

private:
    double kp_;
    double kd_;
    double ki_;
    double E_prev_;
};

#endif	// _PID_CONTROLLER_H