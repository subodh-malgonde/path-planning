#include "pid.h"

PID::PID(){}

PID::~PID(){}

void PID::init(double Kp, double Kd, double Ki){
    kp = Kp;
    kd = Kd;
    ki = Ki;

    d_error = 0;
    p_error = 0;
    i_error = 0;
}

double PID::getControl(double new_speed, double target_speed){
    double new_error =  target_speed - new_speed;
    d_error = p_error - new_error;
    p_error = new_error;
    i_error += p_error;

    return kp*p_error + kd*d_error + ki*i_error;
}
