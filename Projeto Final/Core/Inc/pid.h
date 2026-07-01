#ifndef PID_H
#define PID_H

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float dt;
    float integral;
    float prev_error;
    float prev_derivative;
    float output_min;
    float output_max;
} PID_Controller;

void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float dt,
              float output_min, float output_max);
float PID_Compute(PID_Controller *pid, float setpoint, float measurement);
void PID_Reset(PID_Controller *pid);
void PID_SetGains(PID_Controller *pid, float Kp, float Ki, float Kd);

#endif /* PID_H */
