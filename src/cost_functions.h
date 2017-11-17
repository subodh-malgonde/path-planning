#ifndef CostFunctionsH
#define CostFunctionsH

#include <math.h>
#include <vector>
#include "car.h"
#include "spline.h"
#include "constants.h"

using namespace std;


template <class T>
void printVector(std::vector<T> &values){
    for(int i = 0; i < values.size(); i++){
        std::cout << values[i] << ", ";
    }
    std::cout << std::endl;
}

double speed_cost(double speed){
    // cost of going at the lower speed should be high
    return (speed - MAX_SPEED)*(speed - MAX_SPEED);
}

bool checkCollision(Car ego, Car car){
    bool collision = false;

    for(int i = 0; i < ego.trajectory_s.size(); ++i){
        double s = ego.trajectory_s[i];
        double d = ego.trajectory_d[i];
        double s1 = car.trajectory_s[i];
        double d1 = car.trajectory_d[i];
        if((fabs(s - s1) < 15) & (fabs(d - d1) < 1.5)){
            collision = true;
            cout << "collision with car s, d, speed: " << car.s << ", " << car.d << ", " << car.speed << endl;
            cout << "ego s, d, speed: " << ego.d << ", " << ego.s << ", " << ego.speed << endl;

            cout << "ego trajectory:" << endl;
            cout << "s:" << endl;
            printVector(ego.trajectory_s);

            cout << "d:" << endl;
            printVector(ego.trajectory_d);

            cout << "car trajectory:" << endl;
            cout << "s:" << endl;
            printVector(car.trajectory_s);

            cout << "d:" << endl;
            printVector(car.trajectory_d);
            break;
        }
    }
    return collision;
}

void setPredictedTrajectory(vector<double> &pts_s, vector<double> &pts_d, Car &ego, double ref_speed){

    tk::spline spline_function;
    spline_function.set_points(pts_s, pts_d);

    vector<double> trajectory_s;
    vector<double> trajectory_d;

    double s = ego.s;
    double d = ego.d;

    for(int j = 1; j < 100; j++){
        s += ref_speed*TIME_PER_POINT;
        d = spline_function(s);

        trajectory_s.push_back(s);
        trajectory_d.push_back(d);
    }

    ego.setTrajectory(trajectory_s, trajectory_d);
}

double getLaneTargetSpeed(double lane, Car ego, const vector<Car> &cars){

    double max_speed = 0;

    double next_s = ego.s + BUFFER_DISTANCE;

    for(int i = 0; i < cars.size(); ++i){
        Car car = cars[i];
        if((fabs(2 + 4*lane - car.d) < 1.0) & (ego.s < car.s) & (car.s < next_s)){
            max_speed = car.speed;
            next_s = car.s;
        }
    }
    if(next_s < ego.s + BUFFER_DISTANCE){
        max_speed = min(max_speed, MAX_SPEED);
    }else{
        max_speed = MAX_SPEED;
    }
    return max_speed;

}


vector<Car> getCollidingCars(const vector<Car> &cars, const Car &ego, const double target_lane){
    vector<Car> collidingCars;

    for(unsigned int i = 0; i < cars.size(); ++i){
        Car car = cars[i];
        if((fabs(2 + 4*target_lane - car.d) < 2) & (fabs(car.s - ego.s) < LANE_CHANGE_BUFFER_DISTANCE)){
            collidingCars.push_back(car);
        }else if((fabs(ego.d - car.d) < 2) &  (car.s < ego.s + LANE_CHANGE_BUFFER_DISTANCE)){
            collidingCars.push_back(car);
        }
    }
    return collidingCars;
}


vector<double> laneChangeLeftCost(Car ego, double lane, double ref_speed, const vector<Car> &cars){
    double cost = 0;
    if(lane ==0){
        cost = 1e+5;
    }else{

        vector<Car> collidingCars = getCollidingCars(cars, ego, lane - 1);

        if(collidingCars.size() > 0){
            //generate trajectory
            // check for collision
            vector<double> pts_s;
            vector<double> pts_d;

            pts_s.push_back(ego.s - 60);
            pts_d.push_back(2 + 4*lane);

            pts_s.push_back(ego.s - 30);
            pts_d.push_back(2 + 4*lane);

            pts_s.push_back(ego.s);
            pts_d.push_back(2 + 4*lane);

            pts_s.push_back(ego.s + 20);
            pts_d.push_back(2 + 4*(lane - 1));

            pts_s.push_back(ego.s + 40);
            pts_d.push_back(2 + 4*(lane - 1));

            pts_s.push_back(ego.s + 80);
            pts_d.push_back(2 + 4*(lane - 1));


            setPredictedTrajectory(pts_s, pts_d, ego, ref_speed);

            for(int i = 0; i < collidingCars.size(); ++i){
                if(checkCollision(ego, collidingCars[i])){
                    cost = 1e+5;
                    break;
                }
            }

        }
    }
    if(cost < 1){
        double max_speed = getLaneTargetSpeed(lane - 1, ego, cars);
        return {speed_cost(max_speed), max_speed};
    }else{
        return {cost, 0};
    }
}

vector<double> laneChangeRightCost(Car ego, double lane, double ref_speed, vector<Car> cars){
    double cost = 0;
    if(lane == 2){
        cost = 1e+5;
    }else{

        vector<Car> collidingCars = getCollidingCars(cars, ego, lane + 1);

        if(collidingCars.size() > 0){

            //generate trajectory
            // check for collision
            vector<double> pts_s;
            vector<double> pts_d;

            pts_s.push_back(ego.s - 60);
            pts_d.push_back(2 + 4*lane);

            pts_s.push_back(ego.s - 30);
            pts_d.push_back(2 + 4*lane);

            pts_s.push_back(ego.s);
            pts_d.push_back(2 + 4*lane);

            pts_s.push_back(ego.s + 20);
            pts_d.push_back(2 + 4*(lane + 1));

            pts_s.push_back(ego.s + 40);
            pts_d.push_back(2 + 4*(lane + 1));

            pts_s.push_back(ego.s + 80);
            pts_d.push_back(2 + 4*(lane + 1));

            setPredictedTrajectory(pts_s, pts_d, ego, ref_speed);

            for(int i = 0; i < collidingCars.size(); ++i){
                if(checkCollision(ego, collidingCars[i])){
                    cost = 1e+5;
                    break;
                }
            }
        }
    }
    if(cost < 1){
        double max_speed = getLaneTargetSpeed(lane + 1, ego, cars);
        return {speed_cost(max_speed), max_speed};
    }else{
        return {cost, 0};
    }
}

vector<double> keepLaneCost(Car ego, double lane, double ref_speed, vector<Car> cars){
    double target_speed = MAX_SPEED;

    for(int i = 0; i < cars.size(); ++i){
        Car car = cars[i];

        if(((2 + 4*lane - 2) < car.d) & (car.d < (2 + 4*lane + 2)) & (car.s > ego.s) & (fabs(car.s - ego.s) < 50.0)){
            // check s;

            double car_speed = sqrt(car.vx*car.vx + car.vy*car.vy);
            cout << "car in the same lane as ego. s: " << car.s << ", speed: " << car_speed << endl;
            cout << "ego s: " << ego.s << ", speed: " << ref_speed << endl;

            // check the result 3 seconds ahead of us
            double ego_s = (ego.s + (ref_speed)*3.0);
            double car_s = (car.s + car_speed*3.0);
            if(ego_s > car_s - BUFFER_DISTANCE){
                target_speed = car_speed;
                break;
            }
        }
    }

    return {speed_cost(target_speed), target_speed};
}

#endif