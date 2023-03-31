#ifndef PID_H
#define PID_H

#include <Arduino.h>

struct PID_params
{
    float error = 0;
    float error_prev = 0;
    float delta_error = 0;
    float total_error = 0;
    float control_signal = 0;
    float set_point = 0;
    float KP = 0;
    float KI = 0;
    float KD = 0;
    float feedback = 0;
    float proportional = 0;
    float integral = 0;
    float derivative = 0;
    unsigned long time = 0;
    float time_sampling = 100;
    float maxControl = 40;
    float minControl = -40;
};

float PID(PID_params param);

float PID(PID_params param){
    if (millis() - param.time > param.time_sampling)
    {
        param.time = millis();
        param.error = param.set_point - param.feedback;
        param.delta_error = param.error - param.error_prev;
        param.total_error = param.total_error + param.error;

        if(param.total_error >= param.maxControl) param.total_error = param.maxControl;
        else if(param.total_error <= param.minControl) param.total_error = param.minControl;

        param.proportional = param.KP * param.error;
        param.integral = param.KI * param.total_error * param.time_sampling;
        param.derivative = (param.KD/param.time_sampling)*param.delta_error;
        param.control_signal = param.proportional + param.integral + param.derivative;

        if(param.control_signal >= param.maxControl) param.control_signal = param.maxControl;
        else if(param.control_signal <= param.minControl) param.control_signal = param.minControl;

        param.error_prev = param.error;

    }
    
    return int(param.control_signal);
}

#endif