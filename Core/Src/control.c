#include "control.h"
#include "tim.h"

static const int32_t sensor_weights[SENSOR_COUNT] = {
    0, 1000, 2000, 3000, 4000, 5000, 6000, 7000
};

static int32_t sensor_min[SENSOR_COUNT];
static int32_t sensor_max[SENSOR_COUNT];


#define LINE_DETECT_THRESHOLD  200

static int32_t clamp_i32(int32_t v, int32_t min, int32_t max)
{
    if (v < min) return min;
    if (v > max) return max;
    return v;
}

static float clamp_f(float v, float min, float max)
{
    if (v < min) return min;
    if (v > max) return max;
    return v;
}

void Control_Init(PID_t *pid, float kp, float ki, float kd, float integral_max)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->integral = 0.0f;
    pid->integral_max = integral_max;
    pid->prev_error = 0.0f;
    pid->output = 0.0f;

    for (int i = 0; i < SENSOR_COUNT; i++) {
        sensor_min[i] = SENSOR_MIN_DEFAULT;
        sensor_max[i] = SENSOR_MAX_DEFAULT;
    }

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
}

bool LineSensor_GetPosition(int32_t *position_out)
{
    int64_t weighted_sum = 0;
    int64_t value_sum = 0;
    bool line_seen = false;

    for (int i = 0; i < SENSOR_COUNT; i++) {
        int32_t raw = line_sensor_raw[i];
        int32_t range = sensor_max[i] - sensor_min[i];
        int32_t normalized;
        if (range <= 0) {
            normalized = 0;
        } else {
            normalized = ((raw - sensor_min[i]) * 1000) / range;
            normalized = clamp_i32(normalized, 0, 1000);
        }

        if (normalized > LINE_DETECT_THRESHOLD) {
            line_seen = true;
        }

        weighted_sum += (int64_t)normalized * sensor_weights[i];
        value_sum += normalized;
    }

    if (!line_seen || value_sum == 0) {
        return false;
    }

    *position_out = (int32_t)(weighted_sum / value_sum);
    return true;
}

float PID_Step(PID_t *pid, int32_t error)
{
    float error_f = (float)error;

    float p_term = pid->Kp * error_f;

    pid->integral += error_f;
    pid->integral = clamp_f(pid->integral, -pid->integral_max, pid->integral_max);
    float i_term = pid->Ki * pid->integral;

    float d_term = pid->Kd * (error_f - pid->prev_error);
    pid->prev_error = error_f;

    pid->output = p_term + i_term + d_term;
    return pid->output;
}

void Motors_ApplyPID(float pid_output)
{
    int32_t left_duty  = (int32_t)(BASE_SPEED_DUTY + pid_output);
    int32_t right_duty = (int32_t)(BASE_SPEED_DUTY - pid_output);

    if (left_duty < 0) {
        HAL_GPIO_WritePin(MOTOR1_DIR_GPIO_Port, MOTOR1_DIR_Pin, GPIO_PIN_SET);
        left_duty = -left_duty;
    } else {
        HAL_GPIO_WritePin(MOTOR1_DIR_GPIO_Port, MOTOR1_DIR_Pin, GPIO_PIN_RESET);
    }
    left_duty = clamp_i32(left_duty, 0, PWM_MAX_DUTY);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, left_duty);

    if (right_duty < 0) {
        HAL_GPIO_WritePin(MOTOR2_DIR_GPIO_Port, MOTOR2_DIR_Pin, GPIO_PIN_SET);
        right_duty = -right_duty;
    } else {
        HAL_GPIO_WritePin(MOTOR2_DIR_GPIO_Port, MOTOR2_DIR_Pin, GPIO_PIN_RESET);
    }
    right_duty = clamp_i32(right_duty, 0, PWM_MAX_DUTY);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, right_duty);
}
