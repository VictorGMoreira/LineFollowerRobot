#ifndef COMM_H
#define COMM_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

// Link serial com o ESP32-C3 (calibracao)
// USART1: PA9 = TX (-> RX do ESP), PA10 = RX (<- TX do ESP), 115200 8N1
// Implementado direto nos registradores (o HAL UART nao esta no projeto).
// ATENCAO: se um dia habilitar USART1 no CubeMX, remova o USART1_IRQHandler de comm.c.
#define COMM_BAUDRATE   115200

void Comm_Init(void);

// Chamar no loop principal: le bytes recebidos e executa comandos completos
void Comm_Process(void);

// Envia uma linha (adiciona "\n"). Nao bloqueia: se o buffer encher, descarta.
void Comm_SendLine(const char *line);

// true se o watchdog (comando WD) esta ligado e nenhuma linha chegou dentro do tempo
bool Comm_LinkTimedOut(void);

// Contador de ticks (500 Hz) do teste de motor; decrementado em main.c
extern volatile uint32_t motor_test_ticks;

#endif
