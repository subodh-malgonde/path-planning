#include "environment.h"
#include <iostream>
#include <math.h>
#include "cost_functions.h"
#include "constants.h"

using namespace std;

Environment::Environment(){
    pid.init(0.01, 0.009, 0);
}

Environment::~Environment(){}

void Environment::update(std::vector<Car> &other_cars, Car &main_car){
    cars = other_cars;
    ego = main_car;
}

bool Environment::canAccelerate(){
    // todo add more complex logic like
    // speed of the car in front
    // check for max velocity
    if(ego.speed < 40.0){
        return true;
    }else{
        return false;
    }
}

void Environment::UpdateEgo(double pos_x, double pos_y, double pos_s, double pos_d, double car_speed, double car_yaw){
    if(ego.state != KEEP_LANE){
        if(fabs(2 + 4*lane - pos_d) < 1.0){
            cout << "resetting ego state to keep lane" << endl;
            ego.state = KEEP_LANE;
        }
    }
    ego.initEgo(pos_x, pos_y, pos_s, pos_d, car_speed, car_yaw);
}

vector<double> Environment::getNewState(){

    vector<double> keep_lane_cost;
    vector<double> lane_change_right_cost;
    vector<double> lane_change_left_cost;

    if(ego.state == KEEP_LANE){
        // todo compute keep lane cost
        // if cost > 0, then compute changing lanes cost
        //
        keep_lane_cost = keepLaneCost(ego, lane, ego_speed, cars);
        cout << "keep lane cost, speed " << keep_lane_cost[0] << ", " << keep_lane_cost[1] << endl;

        if(keep_lane_cost[0] > 0){
            lane_change_left_cost = laneChangeLeftCost(ego, lane, ego_speed, cars);
            lane_change_right_cost = laneChangeRightCost(ego, lane, ego_speed, cars);
            cout << "change lane left cost " << lane_change_left_cost[0] << endl;
            cout << "change lane right cost " << lane_change_right_cost[0] << endl;

            if((lane_change_left_cost[0] < keep_lane_cost[0]) & (lane_change_left_cost[0] <= lane_change_right_cost[0])){
                lane -= 1;
                ego.state = LANE_CHANGE_LEFT;
                target_speed = lane_change_left_cost[1];
            }else if ((lane_change_right_cost[0] < keep_lane_cost[0]) & (lane_change_right_cost[0] <= lane_change_left_cost[0])){
                lane += 1;
                ego.state = LANE_CHANGE_RIGHT;
                target_speed = lane_change_right_cost[1];
            }else{
                target_speed = keep_lane_cost[1];
            }
        }else{
            target_speed = keep_lane_cost[1];
        }
    }else if ((ego.state == LANE_CHANGE_LEFT) | (ego.state == LANE_CHANGE_RIGHT)){
        // do nothing until the lane is changed
    }

    cout << "initial ego speed " << ego_speed << endl;

    ego_speed += pid.getControl(ego_speed, target_speed);
//    if(ego_speed < target_speed - 1.0){
//        ego_speed += 0.2;
//    }else{
//        ego_speed -= 0.2;
//    }
    cout << "final ego speed " << ego_speed << endl;
    return {ego_speed, lane};
}