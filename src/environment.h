#ifndef Environment_H
#define Environment_H

#include <vector>
#include "car.h"
#include "pid.h"

class Environment {

    public:
      /**
      * Constructor.
      */
      Environment();

      /**
      * Destructor.
      */
      virtual ~Environment();

      void update(std::vector<Car> &other_cars, Car &main_car);

      bool canAccelerate();

      PID pid;

      std::vector<double> getNewState();
      void UpdateEgo(double pos_x, double pos_y, double pos_s, double pos_d, double car_speed, double car_yaw);

      std::vector<Car> cars;
      Car ego;
      double target_speed;
      double ego_speed = 0.0;
      double lane = 1.0;
};

#endif /* Environment_H */