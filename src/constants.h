#ifndef CONSTANTS_H
#define CONSTANTS_H

using namespace std;

const double MAX_ACCELERATION = 10.0;
const double MIN_ACCELERATION = -10.0;
const double TIME_PER_POINT = 0.02;
const double BUFFER_DISTANCE = 30.0; // buffer distance in meters
const double LANE_CHANGE_BUFFER_DISTANCE = 50.0; // buffer distance in meters for lane change
const double MAX_SPEED = 48.0/2.24; // max speed in meters per second

const string LANE_CHANGE_LEFT = "LCL";
const string LANE_CHANGE_RIGHT= "LCR";
const string PREPARE_LANE_CHANGE_LEFT= "PLCL";
const string PREPARE_LANE_CHANGE_RIGHT = "PLCR";
const string KEEP_LANE = "KL";

#endif