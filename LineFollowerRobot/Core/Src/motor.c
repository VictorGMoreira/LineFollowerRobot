#include "motor.h"
#include "tim.h"
#include "robot_config.h"

/*
 * DRV8833 com os dois pinos de cada ponte em PWM (xIN1 = canal A, xIN2 = canal B):
 *
 *   xIN1 xIN2   motor
 *    0    0     solto (coast)
 *    1    0     sentido + (corrente OUT1 -> OUT2 / OUT3 -> OUT4)
 *    0    1     sentido -
 *    1    1     freio
 *
 * Slow decay (MOTOR_DECAY_LENTO = 1):
 *   +: A = 100%,  B = PWM com CCR = periodo - forca   (freio no resto do ciclo)
 *   -: A = PWM com CCR = periodo - forca,  B = 100%
 * Fast decay (MOTOR_DECAY_LENTO = 0):
 *   +: A = forca, B = 0                               (solto no resto do ciclo)
 *   -: A = 0,     B = forca
 * Freio: A = B = 100% (CCR = periodo, saida sempre alta em PWM mode 1)
 */

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t           canal_a;    // xIN1
    uint32_t           canal_b;    // xIN2
    bool               invertido;
    int32_t            dz;
} Motor_t;

static Motor_t motores[2] = {
    { &MOTOR_ESQ_TIM, MOTOR_ESQ_CANAL_A, MOTOR_ESQ_CANAL_B,
      MOTOR_ESQ_INVERTIDO != 0, DZ_ESQ },
    { &MOTOR_DIR_TIM, MOTOR_DIR_CANAL_A, MOTOR_DIR_CANAL_B,
      MOTOR_DIR_INVERTIDO != 0, DZ_DIR },
};

static int32_t clamp_u(int32_t u)
{
    if (u > U_MAX)  return U_MAX;
    if (u < -U_MAX) return -U_MAX;
    return u;
}

static void motor_pwm(Motor_t *m, uint32_t ccr_a, uint32_t ccr_b)
{
    __HAL_TIM_SET_COMPARE(m->htim, m->canal_a, ccr_a);
    __HAL_TIM_SET_COMPARE(m->htim, m->canal_b, ccr_b);
}

// Aplica um comando ja compensado (-U_MAX..U_MAX) no hardware
static void motor_saida(Motor_t *m, int32_t u)
{
    u = clamp_u(u);
    if (m->invertido) {
        u = -u;
    }

    uint32_t periodo = __HAL_TIM_GET_AUTORELOAD(m->htim) + 1u;
    uint32_t forca   = ((uint32_t)((u < 0) ? -u : u) * periodo) / U_MAX;

    if (u == 0) {
        motor_pwm(m, periodo, periodo);
        return;
    }

#if MOTOR_DECAY_LENTO
    if (u > 0) {
        motor_pwm(m, periodo, periodo - forca);
    } else {
        motor_pwm(m, periodo - forca, periodo);
    }
#else
    if (u > 0) {
        motor_pwm(m, forca, 0u);
    } else {
        motor_pwm(m, 0u, forca);
    }
#endif
}

// |u| < U_MIN_ATIVO -> 0; |u| de 1..U_MAX -> dz..U_MAX, mantendo o sinal
static int32_t compensar(int32_t u, int32_t dz)
{
    int32_t mag = (u < 0) ? -u : u;
    if (mag < U_MIN_ATIVO) {
        return 0;
    }
    if (mag > U_MAX) {
        mag = U_MAX;
    }
    mag = dz + ((mag - 1) * (U_MAX - dz)) / (U_MAX - 1);
    return (u < 0) ? -mag : mag;
}

void Motor_Init(void)
{
    for (int i = 0; i < 2; i++) {
        HAL_TIM_PWM_Start(motores[i].htim, motores[i].canal_a);
        HAL_TIM_PWM_Start(motores[i].htim, motores[i].canal_b);
    }
    Motor_Freio();
}

void Motor_Comando(int32_t u_esq, int32_t u_dir)
{
    motor_saida(&motores[0], compensar(clamp_u(u_esq), motores[0].dz));
    motor_saida(&motores[1], compensar(clamp_u(u_dir), motores[1].dz));
}

void Motor_ComandoBruto(int32_t u_esq, int32_t u_dir)
{
    motor_saida(&motores[0], u_esq);
    motor_saida(&motores[1], u_dir);
}

void Motor_SetDZ(int32_t dz_esq, int32_t dz_dir)
{
    motores[0].dz = dz_esq;
    motores[1].dz = dz_dir;
}

void Motor_GetDZ(int32_t *dz_esq, int32_t *dz_dir)
{
    *dz_esq = motores[0].dz;
    *dz_dir = motores[1].dz;
}

void Motor_Freio(void)
{
    motor_saida(&motores[0], 0);
    motor_saida(&motores[1], 0);
}

void Motor_Solto(void)
{
    for (int i = 0; i < 2; i++) {
        motor_pwm(&motores[i], 0u, 0u);
    }
}
