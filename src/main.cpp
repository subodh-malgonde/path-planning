#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "car.h"
#include "constants.h"
#include "environment.h"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

vector<double> getSpeedFrenet(const double &px, const double &py, const double &vx, const double &vy, const vector<double> &maps_x, const vector<double> &maps_y, const vector<double> &maps_dx, const vector<double> &maps_dy){
    //
    double yaw = atan2(vy, vx);
    int i = NextWaypoint(px, py, yaw, maps_x, maps_y);
    double v_d = maps_dx[i]*vx + maps_dy[i]*vy;
    double v_s = sqrt(vx*vx + vy*vy - v_d*v_d);
    return {v_s, v_d};
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  Environment environment;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &environment](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Yaw is in degrees
          	car_yaw = deg2rad(car_yaw);

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	environment.cars.clear();
          	
//          	std::cout << "SENSOR FUSION " << std::endl;

          	for(int i=0; i < sensor_fusion.size(); i++){
          	    auto data = sensor_fusion[i];
          	    Car car = Car();
          	    double px = data[1];
          	    double py = data[2];
          	    double vx = data[3];
          	    double vy = data[4];
          	    double s = data[5];
          	    double d = data[6];

          	    car.init(data[0], px, py, vx, vy, s, d);

          	    std::vector<double> trajectory_s;
          	    std::vector<double> trajectory_d;
          	    std::vector<double> v_s_d = getSpeedFrenet(px, py, vx, vy, map_waypoints_x, map_waypoints_y, map_waypoints_dx, map_waypoints_dy);

                // build the car's trajectory for the next 2 seconds
          	    for(int j = 1; j < 100; j++){
          	        s += v_s_d[0]*TIME_PER_POINT;
          	        d += v_s_d[1]*TIME_PER_POINT;

          	        trajectory_s.push_back(s);
          	        trajectory_d.push_back(d);
          	    }
          	    car.setTrajectory(trajectory_s, trajectory_d);

          	    environment.cars.push_back(car);
          	}
//          	std::cout << "END SENSOR FUSION " << std::endl;




//            std::cout << "previous path length: "<< previous_path_x.size() << std::endl;
//            std::cout << "x, y, s, d, yaw, car speed: "<< car_x << ", " << car_y << ", " << car_s  << ", " << car_d << ", " << car_yaw << ", " << car_speed << std::endl;

          	vector<double> ptsx;
          	vector<double> ptsy;

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

              double pos_x;
              double pos_y;
              double pos_s;
              double pos_d;
              double ref_x;
              double ref_y ;

              double angle;

              int path_size = previous_path_x.size();

              for(int i = 0; i < path_size; i++)
              {
                  next_x_vals.push_back(previous_path_x[i]);
                  next_y_vals.push_back(previous_path_y[i]);
              }

              if(path_size < 2)
              {
                  pos_x = car_x;
                  pos_y = car_y;
                  angle = car_yaw;
                  pos_s = car_s;
                  pos_d = car_d;

                  ref_x = car_x - cos(car_yaw);
                  ref_y = car_y - sin(car_yaw);

                  environment.UpdateEgo(car_x, car_y, car_s, car_d, car_speed, car_yaw);
              }
              else
              {
                  pos_x = previous_path_x[path_size-1];
                  pos_y = previous_path_y[path_size-1];

                  ref_x = previous_path_x[path_size-2];
                  ref_y = previous_path_y[path_size-2];
                  angle = atan2(pos_y-ref_y,pos_x-ref_x);

                  vector<double> pos_s_d = getFrenet(pos_x, pos_y, angle, map_waypoints_x, map_waypoints_y);
                  pos_s = pos_s_d[0];
                  pos_d = pos_s_d[1];

                  environment.UpdateEgo(car_x, car_y, car_s, car_d, car_speed, car_yaw);
              }

              vector<double> new_state = environment.getNewState();
          	  double new_speed = new_state[0];
          	  double new_lane = new_state[1];

              ptsx.push_back(ref_x);
              ptsx.push_back(pos_x);
              ptsy.push_back(ref_y);
              ptsy.push_back(pos_y);

              vector<double> next_xy = getXY(pos_s + 30, (2 + 4*new_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
              ptsx.push_back(next_xy[0]);
              ptsy.push_back(next_xy[1]);

              next_xy = getXY(pos_s + 60, (2 + 4*new_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
              ptsx.push_back(next_xy[0]);
              ptsy.push_back(next_xy[1]);

              next_xy = getXY(pos_s + 90, (2 + 4*new_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
              ptsx.push_back(next_xy[0]);
              ptsy.push_back(next_xy[1]);


              for(int i=0; i<ptsx.size(); i++){

                double shift_x = ptsx[i] - pos_x;
                double shift_y = ptsy[i] - pos_y;

                ptsx[i] = shift_x*cos(angle) + shift_y*sin(angle);
                ptsy[i] = shift_y*cos(angle) - shift_x*sin(angle);
              }

              tk::spline spline_function;
              spline_function.set_points(ptsx,ptsy);

              double target_x = 60.0;
              double target_y = spline_function(target_x);
              double target_distance = distance(target_x, 0.0, target_y, 0.0);
              double N = (target_distance)/(0.02*new_speed);

          	double x_local = 0.0;
          	double y_local = 0.0;

          	for(int i = 0; i < 50 - path_size; i++){

          	    x_local += target_x/N;
          	    y_local += spline_function(x_local);

          	    double x_global = pos_x + x_local*cos(angle) - y_local*sin(angle);
          	    double y_global = pos_y + y_local*cos(angle) + x_local*sin(angle);

            	next_x_vals.push_back(x_global);
          	    next_y_vals.push_back(y_global);
          	}
            std::cout << "############## END ############"<< car_speed << std::endl;


          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
