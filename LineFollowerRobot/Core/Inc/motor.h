#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

// Acionamento DRV8833 com 2 pinos PWM por motor (xIN1 e xIN2).
// Comandos na escala U: -U_MAX..+U_MAX (ver robot_config.h). 0 = freio.

// Liga o PWM dos dois motores e deixa em freio
void Motor_Init(void);

// Com compensacao de zona morta (uso normal)
void Motor_Comando(int32_t u_esq, int32_t u_dir);

// Sem compensacao (medicao da zona morta)
void Motor_ComandoBruto(int32_t u_esq, int32_t u_dir);

// Zona morta de cada motor em U (comeca com DZ_ESQ/DZ_DIR; muda pelo comando DZ, so em RAM)
void Motor_SetDZ(int32_t dz_esq, int32_t dz_dir);
void Motor_GetDZ(int32_t *dz_esq, int32_t *dz_dir);

// Freio: os dois pinos em HIGH
void Motor_Freio(void);

// Solto (coast): os dois pinos em LOW. Usado na calibracao, para arrastar o robo com a mao.
void Motor_Solto(void);

#endif
