#include "control.h"
#include "tim.h"

static const int32_t sensor_weights[SENSOR_COUNT] = {
    0, 1000, 2000, 3000, 4000, 5000, 6000, 7000
};

static int32_t sensor_min[SENSOR_COUNT];
static int32_t sensor_max[SENSOR_COUNT];

// Calibracao anterior, para restaurar sensores que calibrarem mal
static int32_t backup_min[SENSOR_COUNT];
static int32_t backup_max[SENSOR_COUNT];

int32_t base_speed_duty = BASE_SPEED_DUTY;

// Ultima posicao em que a linha foi vista (usada quando a linha some, ex. curva de 90 graus)
static int32_t last_position = POSITION_CENTER;


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

int32_t LineSensor_Normalize(int i, int32_t raw)
{
    int32_t range = sensor_max[i] - sensor_min[i];
    if (range <= 0) {
        return 0;
    }
    int32_t n = clamp_i32(((raw - sensor_min[i]) * 1000) / range, 0, 1000);
#if SENSOR_INVERT
    n = 1000 - n;
#endif
    return n;
}

void LineSensor_ResetMemory(void)
{
    last_position = POSITION_CENTER;
}

bool LineSensor_GetPosition(int32_t *position_out)
{
    int64_t weighted_sum = 0;
    int64_t value_sum = 0;
    bool line_seen = false;

    for (int i = 0; i < SENSOR_COUNT; i++) {
        int32_t normalized = LineSensor_Normalize(i, line_sensor_raw[i]);

        // ignora o "quase branco": senao todos os sensores fora da linha
        // puxam a media para o centro e reduzem a correcao
        if (normalized < SENSOR_NOISE_FLOOR) {
            normalized = 0;
        }

        if (normalized > LINE_DETECT_THRESHOLD) {
            line_seen = true;
        }

        weighted_sum += (int64_t)normalized * sensor_weights[i];
        value_sum += normalized;
    }

    if (line_seen && value_sum > 0) {
        last_position = (int32_t)(weighted_sum / value_sum);
        *position_out = last_position;
        return true;
    }

    // Linha perdida: em vez de ir reto, finge que a linha esta na ponta do lado
    // em que foi vista por ultimo -> o PID continua virando para aquele lado.
    if (last_position < POSITION_CENTER) {
        *position_out = sensor_weights[0];
    } else if (last_position > POSITION_CENTER) {
        *position_out = sensor_weights[SENSOR_COUNT - 1];
    } else {
        *position_out = POSITION_CENTER;
    }
    return false;
}

float PID_Step(PID_t *pid, int32_t error)
{
    float error_f = (float)error;

    float p_term = pid->Kp * error_f;

    // Anti-windup: limita o TERMO I (em duty), nao a soma do erro.
    // Antes a soma era limitada a 1000 e saturava em menos de 1 tick (erro chega a 3500).
    if (pid->Ki != 0.0f) {
        float ki_abs = (pid->Ki < 0.0f) ? -pid->Ki : pid->Ki;
        float lim = pid->integral_max / ki_abs;
        pid->integral = clamp_f(pid->integral + error_f, -lim, lim);
    } else {
        pid->integral = 0.0f;
    }
    float i_term = pid->Ki * pid->integral;

    float d_term = pid->Kd * (error_f - pid->prev_error);
    pid->prev_error = error_f;

    pid->output = p_term + i_term + d_term;
    return pid->output;
}

static void motor_write(GPIO_TypeDef *dir_port, uint16_t dir_pin,
                        TIM_HandleTypeDef *htim, int32_t duty)
{
    if (duty < 0) {
        HAL_GPIO_WritePin(dir_port, dir_pin, GPIO_PIN_SET);
        duty = -duty;
    } else {
        HAL_GPIO_WritePin(dir_port, dir_pin, GPIO_PIN_RESET);
    }
    duty = clamp_i32(duty, 0, PWM_MAX_DUTY);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, duty);
}

// Motor 2 (TIM3 PB5 / DIR PB4) = ESQUERDO, Motor 1 (TIM2 PB3 / DIR PB2) = DIREITO
void Motors_Set(int32_t left_duty, int32_t right_duty)
{
    motor_write(MOTOR2_DIR_GPIO_Port, MOTOR2_DIR_Pin, &htim3, left_duty);
    motor_write(MOTOR1_DIR_GPIO_Port, MOTOR1_DIR_Pin, &htim2, right_duty);
}

void Motors_Stop(void)
{
    Motors_Set(0, 0);
}

void Motors_ApplyPID(float pid_output)
{
    // limita antes de converter: evita overflow do int32 se o PID explodir
    float out = clamp_f(pid_output, -2.0f * PWM_MAX_DUTY, 2.0f * PWM_MAX_DUTY);
    int32_t left_duty  = (int32_t)(base_speed_duty + out);
    int32_t right_duty = (int32_t)(base_speed_duty - out);
    left_duty  = clamp_i32(left_duty,  -MAX_REVERSE_DUTY, PWM_MAX_DUTY);
    right_duty = clamp_i32(right_duty, -MAX_REVERSE_DUTY, PWM_MAX_DUTY);
    Motors_Set(left_duty, right_duty);
}

void PID_Reset(PID_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output = 0.0f;
}

void LineSensor_CalibReset(void)
{
    for (int i = 0; i < SENSOR_COUNT; i++) {
        backup_min[i] = sensor_min[i];
        backup_max[i] = sensor_max[i];
        sensor_min[i] = 4095;
        sensor_max[i] = 0;
    }
}

void LineSensor_CalibSample(void)
{
    for (int i = 0; i < SENSOR_COUNT; i++) {
        int32_t raw = line_sensor_raw[i];
        if (raw < sensor_min[i]) sensor_min[i] = raw;
        if (raw > sensor_max[i]) sensor_max[i] = raw;
    }
}

uint8_t LineSensor_CalibFinish(void)
{
    uint8_t rejected = 0;
    for (int i = 0; i < SENSOR_COUNT; i++) {
        if (sensor_max[i] - sensor_min[i] < CAL_MIN_RANGE) {
            sensor_min[i] = backup_min[i];
            sensor_max[i] = backup_max[i];
            rejected |= (uint8_t)(1u << i);
        }
    }
    return rejected;
}

int32_t LineSensor_GetMin(int i) { return sensor_min[i]; }
int32_t LineSensor_GetMax(int i) { return sensor_max[i]; }
