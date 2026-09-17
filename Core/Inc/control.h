#ifndef CONTROL_H
#define CONTROL_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

//Sensoriamento
#define SENSOR_COUNT        8

// Placeholder de calibração( PQ não tem como eu fazer a calibracao de casa ne)
// Tem que trocar pelos valores certos
#define SENSOR_MIN_DEFAULT  400
#define SENSOR_MAX_DEFAULT  3600

#define POSITION_CENTER     3500
#define PWM_MAX_DUTY         4199
#define BASE_SPEED_DUTY      2500

//PID

typedef struct {
    float Kp;
    float Ki;
    float Kd;

    float integral;
    float integral_max;
    float prev_error;

    float output;
} PID_t;

extern volatile uint16_t line_sensor_raw[SENSOR_COUNT];

void Control_Init(PID_t *pid, float kp, float ki, float kd, float integral_max);

bool LineSensor_GetPosition(int32_t *position_out);

float PID_Step(PID_t *pid, int32_t error);

void Motors_ApplyPID(float pid_output);

#endif
