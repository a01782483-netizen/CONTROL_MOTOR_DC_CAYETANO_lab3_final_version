#ifndef __PID_H__
#define __PID_H__

class PID
{
public:
    PID();
    void setup(float gains[3], float dt, float min_duty = -100.0f, float max_duty = 100.0f); // dt [s] seconds
    float calculate(float error);
    float getError();

private:
    float _kp, _ki, _kd, _dt;
    float _prev_error = 0;
    float _integral = 0;
    float _min_duty = -100.0f;
    float _max_duty = 100.0f;
};

#endif // __PID_H__