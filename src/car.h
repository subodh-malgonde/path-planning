#ifndef Car_H
#define Car_H

#include <iostream>
#include "math.h"
#include <vector>
#include "constants.h"

using namespace std;

class Car {

    public:
        /**
        * Constructor.
        */
        Car();

        /**
        * Destructor.
        */
        virtual ~Car();

        int id;
        double x;
        double y;
        double s;
        double d;
        double vx;
        double vy;
        double speed;
        double yaw;

        vector<double> trajectory_s;
        vector<double> trajectory_d;

        string state = KEEP_LANE;

//        const vector<string> states = {KEEP_LANE, LANE_CHANGE_LEFT, LANE_CHANGE_RIGHT, PREPARE_LANE_CHANGE_LEFT, PREPARE_LANE_CHANGE_RIGHT};

        void init(int i_d, double pos_x, double pos_y, double v_x, double v_y, double pos_s, double pos_d);
        void initEgo(double pos_x, double pos_y, double pos_s, double pos_d, double car_speed, double car_yaw);
        void setTrajectory(vector<double> &pts_s, vector<double> &pts_d);

        bool positionAt(double t);

        bool checkCollision(Car car);
};

#endif /* Car_H */