#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

// Acionamento DRV8833 com 1 pino PWM + 1 pino GPIO por motor.
// Comandos na escala U: -U_MAX..+U_MAX (ver robot_config.h). 0 = freio.

// Liga o PWM dos dois motores e deixa em freio
void Motor_Init(void);

// false se o pino GPIO de direcao do motor nao esta configurado como saida
// (lado 0 = esquerdo, 1 = direito)
bool Motor_DirOk(int lado);

// Com compensacao de zona morta (uso normal)
void Motor_Comando(int32_t u_esq, int32_t u_dir);

// Sem compensacao (medicao da zona morta)
void Motor_ComandoBruto(int32_t u_esq, int32_t u_dir);

// Freio: os dois pinos em HIGH
void Motor_Freio(void);

// Solto (coast): os dois pinos em LOW. Usado na calibracao, para arrastar o robo com a mao.
void Motor_Solto(void);

#endif
