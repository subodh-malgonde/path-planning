#ifndef PIDH
#define PIDH

class PID{
    public:
        PID();
        virtual ~PID();

        double kp;
        double kd;
        double ki;

        double p_error;
        double d_error;
        double i_error;
        double set_speed;

        double getControl(double new_speed, double target_speed);
        void init(double kp, double kd, double ki);

};
#endif