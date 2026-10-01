#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

/*
 * Parametros ajustaveis do acionamento dos motores e da logica de linha.
 *
 * Escala "U": comando de motor com sinal, -U_MAX..+U_MAX.
 *   +U_MAX = 100% para frente, -U_MAX = 100% para tras, 0 = freio.
 *   O motor.c converte para o CCR lendo o ARR do timer em tempo de execucao.
 */

/* ------------------------------------------------------------------ */
/* Escala                                                              */
/* ------------------------------------------------------------------ */
#define U_MAX               1000    /* [U] 100% de duty */

/* Converte a saida do PID (escala antiga, duty 0..4199) para U.
 * Assim os ganhos Kp/Ki/Kd continuam com o mesmo efeito de antes. */
#define PID_PARA_U          ((float)U_MAX / 4199.0f)

/* Velocidade base padrao: 595 U = mesmo valor antigo (2500 de 4199). Ajustavel com SPEED. */
#define VEL_BASE_PADRAO     595     /* [U] */

/* ------------------------------------------------------------------ */
/* Pinagem DRV8833 (1 pino PWM + 1 pino GPIO por motor)                */
/* ------------------------------------------------------------------ */
/* Lados conferidos no robo em 30/09 (teste MOTOR com rodas no ar). */

/* ESQUERDO: IN3 = PB3 (TIM2_CH2, PWM)   IN4 = PB6 (GPIO_Output "MOTOR_DIR_R" no CubeMX)
 * Se o PB6 deixar de ser saida no CubeMX, o motor.c detecta, desliga a re e o freio
 * desse motor (0 vira "solto") e avisa pela UART.
 * (o label "MOTOR_DIR_R" do CubeMX ficou com R de antes; o motor e o esquerdo) */
#define MOTOR_ESQ_TIM       htim2
#define MOTOR_ESQ_CANAL     TIM_CHANNEL_2
#define MOTOR_ESQ_DIR_PORT  MOTOR_DIR_R_GPIO_Port
#define MOTOR_ESQ_DIR_PIN   MOTOR_DIR_R_Pin

/* DIREITO: IN1 = PB5 (TIM3_CH2, PWM)   IN2 = PB4 (GPIO) */
#define MOTOR_DIR_TIM       htim3
#define MOTOR_DIR_CANAL     TIM_CHANNEL_2
#define MOTOR_DIR_DIR_PORT  GPIOB
#define MOTOR_DIR_DIR_PIN   GPIO_PIN_4

/* Sentido de cada motor:
 *   0 = comando positivo usa GPIO LOW + PWM          -> frente em FAST decay, re em SLOW decay
 *   1 = comando positivo usa GPIO HIGH + PWM invertido -> frente em SLOW decay, re em FAST decay
 * Se uma roda girar ao contrario, troque o valor dela.
 * Para ter slow decay andando para frente: inverta os fios OUT do motor na DRV8833 e ponha 1. */
#define MOTOR_ESQ_INVERTIDO 1
#define MOTOR_DIR_INVERTIDO 1

/* ------------------------------------------------------------------ */
/* Zona morta                                                          */
/* ------------------------------------------------------------------ */
/* Menor comando que faz cada roda vencer a inercia do robo (medir com MODO_MEDIR_DZ).
 * |u| de 1..U_MAX e mapeado para DZ..U_MAX. Valor provisorio. */
#define DZ_ESQ              300     /* [U] */
#define DZ_DIR              300     /* [U] */

/* |u| abaixo disso vira 0 (freio), antes da compensacao. Evita a roda vibrar
 * pulando de +DZ para -DZ quando o comando cruza o zero. 20 = 2% de U_MAX. */
#define U_MIN_ATIVO         20      /* [U] */

/* ------------------------------------------------------------------ */
/* Mistura das rodas (curva nunca mais rapida que a reta)              */
/* ------------------------------------------------------------------ */
/* Roda de fora = vel_base; roda de dentro = vel_base - 2*|c|, limitada a -REV_MAX. */
#define REV_MAX             600     /* [U] re maxima da roda de dentro (60% de U_MAX) */

/* Opcional: vel_base_efetiva = vel_base - K_REDUCAO_CURVA * |erro|, com piso VEL_MIN_CURVA.
 * 0 = desligado. Ex.: 0.1 -> com erro 3500 reduz 350 U. */
#define K_REDUCAO_CURVA     0.0f    /* [U por unidade de erro de posicao] */
#define VEL_MIN_CURVA       300     /* [U] */

/* ------------------------------------------------------------------ */
/* Rampa (limite de variacao do comando)                               */
/* ------------------------------------------------------------------ */
/* Enquanto segue a linha, cada roda so pode AUMENTAR a forca em no maximo SLEW_MAX
 * por tick (2 ms). Reduzir a forca e imediato. Na inversao de sentido o comando vai
 * direto a 0 (freio) e sobe no sentido novo com o mesmo limite.
 * Aplicado antes da compensacao de zona morta. STOP, freio e teste MOTOR nao passam por aqui.
 * 150 -> 0 a 100% em ~14 ms. 0 = desligado. */
#define SLEW_MAX            150     /* [U por tick de 2 ms] */

/* ------------------------------------------------------------------ */
/* Linha perdida                                                       */
/* ------------------------------------------------------------------ */
/* Sensor calibrado (0..1000) acima disso "ve" a linha. Todos abaixo = linha perdida. */
#define LIMIAR_LINHA        200     /* [0..1000, escala calibrada] */

/* Erro com a linha na ponta da barra (posicao 0 ou 7000, centro 3500). */
#define ERRO_MAX            3500    /* [unidades de posicao] */

/* Ao perder a linha: se |ultimo erro valido| > MARGEM_LADO, usa erro = +-ERRO_MAX para
 * o lado onde a linha saiu (gira forte). Senao, mantem o ultimo erro (falha na fita,
 * andando quase reto). 1050 = 30% de ERRO_MAX. */
#define MARGEM_LADO         1050    /* [unidades de posicao] */

/* ------------------------------------------------------------------ */
/* Modo de medicao da zona morta                                       */
/* ------------------------------------------------------------------ */
/* 1 = o comando RUN faz a rampa de medicao em vez de seguir linha:
 * os dois motores sobem de 0 a U_MAX, DZ_PASSO a cada DZ_PASSO_MS, SEM compensacao.
 * Cada passo manda "DZ <u>" pela UART (aparece no log da pagina do ESP). STOP interrompe. */
#define MODO_MEDIR_DZ       0
#define DZ_PASSO            10      /* [U] */
#define DZ_PASSO_MS         300     /* [ms] */

#endif
