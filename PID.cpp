#include "PID.h"

PID::PID()
{
}

void PID::setup(float gains[3], float dt, float min_duty, float max_duty)
{
    _kp = gains[0];
    _ki = gains[1];
    _kd = gains[2];
    _dt = dt;

    _min_duty = min_duty;
    _max_duty = max_duty;

    _integral = 0.0f;
    _prev_error = 0.0f;
}

float PID::calculate(float error)
{
    float max_duty = _max_duty;
    float min_duty = _min_duty;

    float p = _kp * error; // integral term = y[n] = y[n-1] +(dt/2)(e[n]+e[n-1])
    float i = _ki * _integral;
    float d = _kd * (error - _prev_error) / _dt;

    // antiwindup
    float d_integral = _dt * (error + _prev_error) / 2.0f;
    float d_i = _ki * d_integral;

    float u = p + i + d;

    float u_test = u + d_i;

    if ((u_test >= min_duty && u_test <= max_duty) ||
        (u_test > max_duty && d_i < 0.0f) ||
        (u_test < min_duty && d_i > 0.0f))
    {
        _integral += d_integral;
        u = u_test;
    }

    if (u <= min_duty)
    {
        u = min_duty;
    }
    else if (u >= max_duty)
    {
        u = max_duty;
    }

    _prev_error = error;
    return u;
}

float PID::getError()
{
    return _prev_error;
}
