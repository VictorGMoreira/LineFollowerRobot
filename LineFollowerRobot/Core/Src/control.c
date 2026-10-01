#include "control.h"
#include "motor.h"

static const int32_t sensor_weights[SENSOR_COUNT] = {
    0, 1000, 2000, 3000, 4000, 5000, 6000, 7000
};

static int32_t sensor_min[SENSOR_COUNT];
static int32_t sensor_max[SENSOR_COUNT];

// Calibracao anterior, para restaurar sensores que calibrarem mal
static int32_t backup_min[SENSOR_COUNT];
static int32_t backup_max[SENSOR_COUNT];

int32_t vel_base = VEL_BASE_PADRAO;

// Memoria da linha perdida
static int32_t ultimo_erro_valido = 0;
static bool    linha_estava_perdida = false;

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

    Motor_Init();
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
    ultimo_erro_valido = 0;
    linha_estava_perdida = false;
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

        if (normalized > LIMIAR_LINHA) {
            line_seen = true;
        }

#if SENSOR_ORDEM_INVERTIDA
        weighted_sum += (int64_t)normalized * sensor_weights[SENSOR_COUNT - 1 - i];
#else
        weighted_sum += (int64_t)normalized * sensor_weights[i];
#endif
        value_sum += normalized;
    }

    if (line_seen && value_sum > 0) {
        *position_out = (int32_t)(weighted_sum / value_sum);
        return true;
    }

    *position_out = POSITION_CENTER;
    return false;
}

int32_t Linha_ErroPerdida(void)
{
    if (ultimo_erro_valido > MARGEM_LADO)  return ERRO_MAX;     // linha saiu por um lado: gira forte
    if (ultimo_erro_valido < -MARGEM_LADO) return -ERRO_MAX;
    return ultimo_erro_valido;                                  // quase reto (falha na fita): segue
}

int32_t Linha_Erro(PID_t *pid, bool *visivel)
{
    int32_t position;
    bool seen = LineSensor_GetPosition(&position);
    int32_t erro;

    if (seen) {
        erro = POSITION_CENTER - position;
        if (linha_estava_perdida) {
            // reencontrou: sem salto no termo derivativo
            pid->prev_error = (float)erro;
        }
        ultimo_erro_valido = erro;
        linha_estava_perdida = false;
    } else {
        erro = Linha_ErroPerdida();
        linha_estava_perdida = true;
    }

    *visivel = seen;
    return erro;
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

void Motors_Set(int32_t u_esq, int32_t u_dir)
{
    Motor_Comando(u_esq, u_dir);
}

void Motors_Stop(void)
{
    Motor_Freio();
}

// Ultimo comando de cada roda depois da rampa (antes da zona morta)
static int32_t rampa_esq = 0;
static int32_t rampa_dir = 0;

void Motors_ResetRampa(void)
{
    rampa_esq = 0;
    rampa_dir = 0;
}

// Reduzir a forca: imediato. Aumentar: no maximo SLEW_MAX por tick.
// Inverter o sentido: vai a 0 na hora e sobe no sentido novo com o limite.
static int32_t rampa(int32_t atual, int32_t alvo)
{
    if (SLEW_MAX <= 0) {
        return alvo;
    }
    if ((atual > 0 && alvo < 0) || (atual < 0 && alvo > 0)) {
        atual = 0;
    }
    int32_t mag_atual = (atual < 0) ? -atual : atual;
    int32_t mag_alvo  = (alvo < 0) ? -alvo : alvo;
    if (mag_alvo <= mag_atual) {
        return alvo;
    }
    if (mag_alvo - mag_atual <= SLEW_MAX) {
        return alvo;
    }
    return (alvo > 0) ? mag_atual + SLEW_MAX : -(mag_atual + SLEW_MAX);
}

void Motors_ApplyPID(float pid_output, int32_t erro)
{
    // saida do PID na escala antiga (duty 4199) -> U; ganhos continuam iguais
    float c = clamp_f(pid_output * PID_PARA_U, -2.0f * U_MAX, 2.0f * U_MAX);
    float c_abs = (c < 0.0f) ? -c : c;

    float vb = (float)vel_base;
    if (K_REDUCAO_CURVA > 0.0f) {
        float erro_abs = (float)((erro < 0) ? -erro : erro);
        float reduzida = vb - K_REDUCAO_CURVA * erro_abs;
        float piso = (VEL_MIN_CURVA < vel_base) ? (float)VEL_MIN_CURVA : vb;
        vb = (reduzida < piso) ? piso : reduzida;
    }

    // Roda de fora nunca passa de vel_base; a diferenca entre as rodas continua 2*c
    float fora   = vb;
    float dentro = vb - 2.0f * c_abs;
    if (dentro < -(float)REV_MAX) {
        dentro = -(float)REV_MAX;
    }

    // c > 0 equivale ao antigo "esq = base + c": esquerda e a roda de fora
    int32_t alvo_esq = (c >= 0.0f) ? (int32_t)fora : (int32_t)dentro;
    int32_t alvo_dir = (c >= 0.0f) ? (int32_t)dentro : (int32_t)fora;

    rampa_esq = rampa(rampa_esq, alvo_esq);
    rampa_dir = rampa(rampa_dir, alvo_dir);
    Motor_Comando(rampa_esq, rampa_dir);
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
