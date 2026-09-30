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

// 0 = linha da valor ALTO no ADC (padrao). Se na pagina do ESP as barras
// DESCEREM em cima da linha, troque para 1.
#define SENSOR_INVERT        0

// Valores calibrados abaixo disso contam como 0 na media (ruido do branco)
#define SENSOR_NOISE_FLOOR   50

// Faixa minima (max - min) aceita na calibracao; abaixo disso o sensor
// mantem a calibracao anterior
#define CAL_MIN_RANGE        900

#define POSITION_CENTER     3500
#define PWM_MAX_DUTY         4199
#define BASE_SPEED_DUTY      2500

// Quanto a roda de dentro pode girar para tras nas curvas fechadas.
// 0 = nunca inverte (so freia). Valores altos dao curva mais rapida mas
// picos de corrente maiores no driver quando o DIR troca.
#define MAX_REVERSE_DUTY     1500
#define PIVOT_TURN_DUTY      3000

//PID

typedef struct {
    float Kp;
    float Ki;
    float Kd;

    float integral;
    float integral_max;     // limite do termo I em unidades de duty (|Ki*integral| <= integral_max)
    float prev_error;

    float output;
} PID_t;

// Estados do robo (compartilhado com main.c e comm.c)
typedef enum {
    STATE_INIT,
    STATE_SEGUINDO_LINHA,
    STATE_LINHA_PERDIDA,
    STATE_PARADO_SEGURANCA,
    STATE_PARADO,           // parado por comando (STOP)
    STATE_TESTE_MOTOR,      // teste de motor por tempo (MOTOR l r ms)
    STATE_CALIBRANDO        // coletando min/max dos sensores (CAL START)
} RobotState_t;

extern volatile uint16_t line_sensor_raw[SENSOR_COUNT];
extern volatile RobotState_t robot_state;
extern PID_t pid;
extern int32_t base_speed_duty;     // antes era so BASE_SPEED_DUTY, agora ajustavel via serial

void Control_Init(PID_t *pid, float kp, float ki, float kd, float integral_max);

// Sempre escreve uma posicao 0..7000. Retorna false se a linha nao esta visivel:
// nesse caso a posicao e a ponta (0 ou 7000) do lado em que a linha foi vista por ultimo.
bool LineSensor_GetPosition(int32_t *position_out);

// Valor calibrado de um sensor: 0 (fundo) .. 1000 (linha)
int32_t LineSensor_Normalize(int i, int32_t raw);

// Esquece o ultimo lado em que a linha foi vista (chamado no RUN)
void LineSensor_ResetMemory(void);

float PID_Step(PID_t *pid, int32_t error);

void Motors_ApplyPID(float pid_output);

// duty com sinal: -PWM_MAX_DUTY..PWM_MAX_DUTY (negativo = inverte DIR)
void Motors_Set(int32_t left_duty, int32_t right_duty);
void Motors_Stop(void);
void Motors_PivotRecover(void)
{
    if (last_position < POSITION_CENTER) {
        // linha vista pela última vez à esquerda -> pivô pra esquerda
        Motors_Set(-PIVOT_TURN_DUTY, PIVOT_TURN_DUTY);
    } else if (last_position > POSITION_CENTER) {
        // linha vista pela última vez à direita -> pivô pra direita
        Motors_Set(PIVOT_TURN_DUTY, -PIVOT_TURN_DUTY);
    } else {
        // nunca viu a linha ainda (ex.: logo depois do boot) -> não inventa lado
        Motors_Stop();
    }
}


void PID_Reset(PID_t *pid);

// Calibracao dos sensores: reset -> amostra varias vezes passando o robo sobre a linha -> finish
void LineSensor_CalibReset(void);
void LineSensor_CalibSample(void);
// Sensores com faixa < CAL_MIN_RANGE voltam para a calibracao anterior.
// Retorna mascara de bits dos sensores rejeitados (bit i = sensor i).
uint8_t LineSensor_CalibFinish(void);
int32_t LineSensor_GetMin(int i);
int32_t LineSensor_GetMax(int i);

#endif
