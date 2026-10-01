#include "motor.h"
#include "tim.h"
#include "robot_config.h"

/*
 * DRV8833, pino PWM = IN1/IN3, pino GPIO = IN2/IN4:
 *
 *   PWM  GPIO   motor
 *    0    0     solto (coast)
 *    1    0     sentido A
 *    0    1     sentido B
 *    1    1     freio
 *
 * Sentido A: GPIO LOW  + PWM com CCR = forca             -> fast decay (solto no resto do ciclo)
 * Sentido B: GPIO HIGH + PWM com CCR = periodo - forca   -> slow decay (freio no resto do ciclo)
 *            (o motor so e acionado na parte BAIXA do PWM, por isso o CCR e invertido)
 * Freio:     GPIO HIGH + CCR = periodo (saida 100% alta em PWM mode 1)
 */

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t           canal;
    GPIO_TypeDef      *dir_port;
    uint16_t           dir_pin;
    bool               invertido;
    int32_t            dz;
    bool               dir_ok;     // pino GPIO de direcao configurado como saida?
} Motor_t;

static Motor_t motores[2] = {
    { &MOTOR_ESQ_TIM, MOTOR_ESQ_CANAL, MOTOR_ESQ_DIR_PORT, MOTOR_ESQ_DIR_PIN,
      MOTOR_ESQ_INVERTIDO != 0, DZ_ESQ, false },
    { &MOTOR_DIR_TIM, MOTOR_DIR_CANAL, MOTOR_DIR_DIR_PORT, MOTOR_DIR_DIR_PIN,
      MOTOR_DIR_INVERTIDO != 0, DZ_DIR, false },
};

static int32_t clamp_u(int32_t u)
{
    if (u > U_MAX)  return U_MAX;
    if (u < -U_MAX) return -U_MAX;
    return u;
}

static bool pino_e_saida(GPIO_TypeDef *port, uint16_t pin)
{
    uint32_t idx = (uint32_t)__builtin_ctz(pin);
    return ((port->MODER >> (2u * idx)) & 0x3u) == 0x1u;    // 01 = general purpose output
}

// Aplica um comando ja compensado (-U_MAX..U_MAX) no hardware
static void motor_saida(Motor_t *m, int32_t u)
{
    u = clamp_u(u);
    if (m->invertido) {
        u = -u;
    }

    uint32_t periodo = __HAL_TIM_GET_AUTORELOAD(m->htim) + 1u;

    if (!m->dir_ok) {
        // Sem pino de direcao so existe o sentido A (GPIO fica em LOW pelo pull-down da
        // DRV8833): nao ha re nem freio. Negativo e zero viram "solto".
        uint32_t ccr = (u > 0) ? ((uint32_t)u * periodo) / U_MAX : 0u;
        __HAL_TIM_SET_COMPARE(m->htim, m->canal, ccr);
        return;
    }

    if (u == 0) {
        HAL_GPIO_WritePin(m->dir_port, m->dir_pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(m->htim, m->canal, periodo);
    } else if (u > 0) {
        HAL_GPIO_WritePin(m->dir_port, m->dir_pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(m->htim, m->canal, ((uint32_t)u * periodo) / U_MAX);
    } else {
        uint32_t forca = ((uint32_t)(-u) * periodo) / U_MAX;
        HAL_GPIO_WritePin(m->dir_port, m->dir_pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(m->htim, m->canal, periodo - forca);
    }
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
        motores[i].dir_ok = pino_e_saida(motores[i].dir_port, motores[i].dir_pin);
        HAL_TIM_PWM_Start(motores[i].htim, motores[i].canal);
    }
    Motor_Freio();
}

bool Motor_DirOk(int lado)
{
    return motores[lado ? 1 : 0].dir_ok;
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

void Motor_Freio(void)
{
    motor_saida(&motores[0], 0);
    motor_saida(&motores[1], 0);
}

void Motor_Solto(void)
{
    for (int i = 0; i < 2; i++) {
        if (motores[i].dir_ok) {
            HAL_GPIO_WritePin(motores[i].dir_port, motores[i].dir_pin, GPIO_PIN_RESET);
        }
        __HAL_TIM_SET_COMPARE(motores[i].htim, motores[i].canal, 0u);
    }
}
