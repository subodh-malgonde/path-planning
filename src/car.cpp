#include "car.h"

Car::Car(){
    id = 0;
    x = 0;
    y = 0;
    vx = 0;
    vy = 0;
    s = 0;
    d = 0;
    speed = 0;
    yaw = 0;
}

Car::~Car(){}

void Car::init(int i_d, double pos_x, double pos_y, double v_x, double v_y, double pos_s, double pos_d){
    id = i_d;
    x = pos_x;
    y = pos_y;
    s = pos_s;
    d = pos_d;
    vx = v_x;
    vy = v_y;
    speed = sqrt(vx*vx + vy*vy);
}

void Car::initEgo(double pos_x, double pos_y, double pos_s, double pos_d, double car_speed, double car_yaw){
    x = pos_x;
    y = pos_y;
    s = pos_s;
    d = pos_d;
    speed = car_speed;
    yaw = car_yaw;
}

void Car::setTrajectory(vector<double> &pts_s, vector<double> &pts_d){
    trajectory_s = pts_s;
    trajectory_d = pts_d;
}
