#include "pid.h"

void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float dt,
              float output_min, float output_max)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->dt = dt;
    pid->output_min = output_min;
    pid->output_max = output_max;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_derivative = 0.0f;
}

float PID_Compute(PID_Controller *pid, float setpoint, float measurement)
{
    float error = setpoint - measurement;

    pid->integral += error * pid->dt;

    /* Anti-windup: clamp integral */
    if (pid->integral > pid->output_max)
        pid->integral = pid->output_max;
    else if (pid->integral < pid->output_min)
        pid->integral = pid->output_min;

    /* Derivada com filtro passa-baixa (alpha=0.3) para amortecer ruido.
     * Sem filtro, KD alto amplifica spikes do sensor e causa oscilacao. */
    float raw_derivative = (error - pid->prev_error) / pid->dt;
    float derivative = pid->prev_derivative * 0.7f + raw_derivative * 0.3f;
    pid->prev_derivative = derivative;
    pid->prev_error = error;

    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;

    if (output > pid->output_max)
        output = pid->output_max;
    else if (output < pid->output_min)
        output = pid->output_min;

    return output;
}

void PID_Reset(PID_Controller *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_derivative = 0.0f;
}

void PID_SetGains(PID_Controller *pid, float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
}
