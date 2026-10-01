#include "comm.h"
#include "control.h"
#include "motor.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>

/*
 * Protocolo ASCII, uma linha por comando, terminada em '\n' (o '\r' e ignorado).
 * Separadores: espaco ou virgula. Comando nao diferencia maiusculas/minusculas.
 *
 *   PING                 -> PONG
 *   STATUS               -> STATUS <estado> KP <kp> KI <ki> KD <kd> SPEED <u>
 *   STOP                 -> OK STOP                      (para os motores na hora)
 *   RUN                  -> OK RUN                       (zera o PID e volta a seguir linha)
 *   PID <kp> <ki> <kd>   -> OK PID <kp> <ki> <kd>        (cada ganho entre -10000 e 10000)
 *   PID?                 -> PID <kp> <ki> <kd>
 *   KP|KI|KD <valor>     -> OK PID <kp> <ki> <kd>        (muda so um coeficiente)
 *   SPEED <u>            -> OK SPEED <u>                 (velocidade base, 0..1000 = 0..100%)
 *   MOTOR <esq> <dir> <ms> -> OK MOTOR <esq> <dir> <ms>  (u -1000..1000, negativo = re, 0 = freio,
 *                                                          com compensacao de zona morta; ms 1..10000)
 *                           ... e ao terminar: EVT MOTOR DONE
 *   SENS                 -> SENS R <8 brutos 0..4095> N <8 calibrados 0..1000> POS <0..7000> LINE <1|0>
 *                           (LINE 0 = linha perdida; POS e a ponta do ultimo lado visto)
 *   CAL START            -> OK CAL START                 (motores parados, arraste o robo sobre a linha)
 *   CAL STOP             -> CAL MIN <8 valores> MAX <8 valores> [REJ <indices>]
 *                           (sensores com faixa < CAL_MIN_RANGE ficam com a calibracao anterior)
 *   CAL?                 -> CAL MIN ... MAX ... [WARN]
 *   WD <ms>              -> OK WD <ms>                   (watchdog do link: se nao chegar NENHUMA linha
 *                           em <ms> com o robo andando, ele para e manda EVT WD STOP. 0 = desliga)
 *   HB                   -> (sem resposta)               (heartbeat, so alimenta o watchdog)
 *
 * Eventos espontaneos: HELLO LINEFOLLOWER (boot), EVT MOTOR DONE, EVT LOST STOP (girou 2 s
 * sem achar a linha), EVT WD STOP, EVT ADC RESTART.
 *
 * Erros: ERR <motivo>
 */

#define RX_BUF_SIZE     128
#define TX_BUF_SIZE     512
#define LINE_MAX        96
#define MOTOR_TEST_MAX_MS   10000
#define TICK_MS             2       // TIM4 a 500 Hz
#define GAIN_LIMIT          10000.0f
#define WD_MAX_MS           60000

volatile uint32_t motor_test_ticks = 0;

static uint32_t wd_timeout_ms = 0;      // 0 = watchdog desligado
static uint32_t last_rx_ms = 0;         // HAL_GetTick() da ultima linha recebida

static volatile uint8_t  rx_buf[RX_BUF_SIZE];
static volatile uint16_t rx_head = 0, rx_tail = 0;
static volatile uint8_t  tx_buf[TX_BUF_SIZE];
static volatile uint16_t tx_head = 0, tx_tail = 0;

static char     line_buf[LINE_MAX];
static uint16_t line_len = 0;
static bool     line_overflow = false;

/* ------------------------------------------------------------------ */
/* Hardware                                                            */
/* ------------------------------------------------------------------ */

void Comm_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();

    // PA9 = USART1_TX, PA10 = USART1_RX (AF7). Pull-up no RX pra nao ler lixo com o ESP desligado.
    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    USART1->CR1 = 0;
    USART1->CR2 = 0;
    USART1->CR3 = 0;
    // Oversampling 16: BRR = PCLK2 / baud (USART1 fica no APB2 = 84 MHz)
    USART1->BRR = (HAL_RCC_GetPCLK2Freq() + COMM_BAUDRATE / 2) / COMM_BAUDRATE;
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;

    // Prioridade abaixo do TIM4 (0) para nao atrasar o tick de controle
    HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    Comm_SendLine("HELLO LINEFOLLOWER");
}

void USART1_IRQHandler(void)
{
    uint32_t sr = USART1->SR;

    // Ler DR depois do SR tambem limpa ORE/FE/NE
    if (sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE)) {
        uint8_t b = (uint8_t)USART1->DR;
        uint16_t next = (rx_head + 1) % RX_BUF_SIZE;
        if (next != rx_tail) {
            rx_buf[rx_head] = b;
            rx_head = next;
        }
    }

    if ((USART1->CR1 & USART_CR1_TXEIE) && (sr & USART_SR_TXE)) {
        if (tx_tail == tx_head) {
            USART1->CR1 &= ~USART_CR1_TXEIE;
        } else {
            USART1->DR = tx_buf[tx_tail];
            tx_tail = (tx_tail + 1) % TX_BUF_SIZE;
        }
    }
}

static uint16_t tx_free(void)
{
    return (uint16_t)((tx_tail + TX_BUF_SIZE - tx_head - 1) % TX_BUF_SIZE);
}

void Comm_SendLine(const char *line)
{
    size_t len = strlen(line);
    if (len + 1 > tx_free()) {
        return;     // melhor perder a resposta do que travar o loop de controle
    }
    for (size_t i = 0; i < len; i++) {
        tx_buf[tx_head] = (uint8_t)line[i];
        tx_head = (tx_head + 1) % TX_BUF_SIZE;
    }
    tx_buf[tx_head] = '\n';
    tx_head = (tx_head + 1) % TX_BUF_SIZE;

    __disable_irq();
    USART1->CR1 |= USART_CR1_TXEIE;
    __enable_irq();
}

/* ------------------------------------------------------------------ */
/* Formatacao (printf do newlib-nano nao imprime float por padrao)     */
/* ------------------------------------------------------------------ */

static char *fmt_float(char *out, float v)
{
    // 5 casas decimais: suficiente para Ki pequenos
    char *p = out;
    if (v < 0) {
        *p++ = '-';
        v = -v;
    }
    uint32_t scaled = (uint32_t)(v * 100000.0f + 0.5f);
    sprintf(p, "%lu.%05lu", (unsigned long)(scaled / 100000UL), (unsigned long)(scaled % 100000UL));
    return out;
}

static const char *state_name(RobotState_t s)
{
    switch (s) {
        case STATE_INIT:             return "INIT";
        case STATE_SEGUINDO_LINHA:   return "RUN";
        case STATE_LINHA_PERDIDA:    return "LOST";
        case STATE_PARADO_SEGURANCA: return "SAFE_STOP";
        case STATE_PARADO:           return "STOP";
        case STATE_TESTE_MOTOR:      return "MOTOR_TEST";
        case STATE_CALIBRANDO:       return "CALIB";
        default:                     return "?";
    }
}

/* ------------------------------------------------------------------ */
/* Comandos                                                            */
/* ------------------------------------------------------------------ */

static bool parse_float(const char *tok, float *out)
{
    if (tok == NULL) return false;
    char *end;
    float v = strtof(tok, &end);
    if (end == tok || *end != '\0') return false;
    // rejeita nan/inf (v != v so e verdade para NaN) e valores absurdos
    if (v != v || v > GAIN_LIMIT || v < -GAIN_LIMIT) return false;
    *out = v;
    return true;
}

static bool parse_int(const char *tok, int32_t *out)
{
    if (tok == NULL) return false;
    char *end;
    long v = strtol(tok, &end, 10);
    if (end == tok || *end != '\0') return false;
    *out = (int32_t)v;
    return true;
}

static void reply_pid(const char *prefix)
{
    char kp[16], ki[16], kd[16], out[80];
    snprintf(out, sizeof(out), "%s %s %s %s", prefix,
             fmt_float(kp, pid.Kp), fmt_float(ki, pid.Ki), fmt_float(kd, pid.Kd));
    Comm_SendLine(out);
}

static void reply_cal(uint8_t rejected)
{
    char out[128];
    int n = snprintf(out, sizeof(out), "CAL MIN");
    for (int i = 0; i < SENSOR_COUNT; i++) {
        n += snprintf(out + n, sizeof(out) - n, " %ld", (long)LineSensor_GetMin(i));
    }
    n += snprintf(out + n, sizeof(out) - n, " MAX");
    bool weak = false;
    for (int i = 0; i < SENSOR_COUNT; i++) {
        n += snprintf(out + n, sizeof(out) - n, " %ld", (long)LineSensor_GetMax(i));
        if (LineSensor_GetMax(i) - LineSensor_GetMin(i) < CAL_MIN_RANGE) weak = true;
    }
    if (rejected) {
        n += snprintf(out + n, sizeof(out) - n, " REJ");
        for (int i = 0; i < SENSOR_COUNT; i++) {
            if (rejected & (1u << i)) {
                n += snprintf(out + n, sizeof(out) - n, " %d", i);
            }
        }
    } else if (weak) {
        snprintf(out + n, sizeof(out) - n, " WARN");
    }
    Comm_SendLine(out);
}

static void cmd_stop(void)
{
    Motors_Stop();
    motor_test_ticks = 0;
    robot_state = STATE_PARADO;
    Comm_SendLine("OK STOP");
}

static void handle_line(char *line)
{
    const char *sep = " ,\t";
    char *cmd = strtok(line, sep);
    if (cmd == NULL) return;

    for (char *c = cmd; *c; c++) *c = (char)toupper((unsigned char)*c);

    char *a1 = strtok(NULL, sep);
    char *a2 = strtok(NULL, sep);
    char *a3 = strtok(NULL, sep);

    if (strcmp(cmd, "PING") == 0) {
        Comm_SendLine("PONG");
    }
    else if (strcmp(cmd, "STOP") == 0) {
        cmd_stop();
    }
    else if (strcmp(cmd, "HB") == 0) {
        // so alimenta o watchdog (last_rx_ms ja foi atualizado em Comm_Process)
    }
    else if (strcmp(cmd, "WD") == 0) {
        int32_t ms;
        if (!parse_int(a1, &ms) || ms < 0 || ms > WD_MAX_MS) {
            Comm_SendLine("ERR WD USE: WD <0..60000 ms>");
            return;
        }
        wd_timeout_ms = (uint32_t)ms;
        char out[32];
        snprintf(out, sizeof(out), "OK WD %ld", (long)ms);
        Comm_SendLine(out);
    }
    else if (strcmp(cmd, "RUN") == 0) {
        motor_test_ticks = 0;
        PID_Reset(&pid);
        LineSensor_ResetMemory();
        robot_state = STATE_SEGUINDO_LINHA;
        Comm_SendLine("OK RUN");
    }
    else if (strcmp(cmd, "STATUS") == 0) {
        char kp[16], ki[16], kd[16], out[112];
        snprintf(out, sizeof(out), "STATUS %s KP %s KI %s KD %s SPEED %ld",
                 state_name(robot_state),
                 fmt_float(kp, pid.Kp), fmt_float(ki, pid.Ki), fmt_float(kd, pid.Kd),
                 (long)vel_base);
        Comm_SendLine(out);
    }
    else if (strcmp(cmd, "PID?") == 0) {
        reply_pid("PID");
    }
    else if (strcmp(cmd, "PID") == 0) {
        float kp, ki, kd;
        if (a1 == NULL) {
            reply_pid("PID");
        } else if (parse_float(a1, &kp) && parse_float(a2, &ki) && parse_float(a3, &kd)) {
            pid.Kp = kp;
            pid.Ki = ki;
            pid.Kd = kd;
            PID_Reset(&pid);
            reply_pid("OK PID");
        } else {
            Comm_SendLine("ERR PID USE: PID <kp> <ki> <kd> (-10000..10000)");
        }
    }
    else if (strcmp(cmd, "KP") == 0 || strcmp(cmd, "KI") == 0 || strcmp(cmd, "KD") == 0) {
        float v;
        if (!parse_float(a1, &v)) {
            Comm_SendLine("ERR VALUE");
            return;
        }
        if (cmd[1] == 'P') pid.Kp = v;
        else if (cmd[1] == 'I') pid.Ki = v;
        else pid.Kd = v;
        PID_Reset(&pid);
        reply_pid("OK PID");
    }
    else if (strcmp(cmd, "SPEED") == 0) {
        int32_t v;
        if (!parse_int(a1, &v) || v < 0 || v > U_MAX) {
            Comm_SendLine("ERR SPEED 0..1000");
            return;
        }
        vel_base = v;
        char out[32];
        snprintf(out, sizeof(out), "OK SPEED %ld", (long)v);
        Comm_SendLine(out);
    }
    else if (strcmp(cmd, "MOTOR") == 0) {
        int32_t l, r, ms;
        if (!parse_int(a1, &l) || !parse_int(a2, &r) || !parse_int(a3, &ms) ||
            l < -U_MAX || l > U_MAX ||
            r < -U_MAX || r > U_MAX ||
            ms < 1 || ms > MOTOR_TEST_MAX_MS) {
            Comm_SendLine("ERR MOTOR USE: MOTOR <-1000..1000> <-1000..1000> <1..10000 ms>");
            return;
        }
        robot_state = STATE_TESTE_MOTOR;
        motor_test_ticks = (uint32_t)((ms + TICK_MS - 1) / TICK_MS);
        Motors_Set(l, r);
        char out[48];
        snprintf(out, sizeof(out), "OK MOTOR %ld %ld %ld", (long)l, (long)r, (long)ms);
        Comm_SendLine(out);
    }
    else if (strcmp(cmd, "SENS") == 0) {
        uint16_t raw[SENSOR_COUNT];
        for (int i = 0; i < SENSOR_COUNT; i++) {
            raw[i] = line_sensor_raw[i];     // copia: o DMA continua escrevendo
        }
        char out[128];
        int n = snprintf(out, sizeof(out), "SENS R");
        for (int i = 0; i < SENSOR_COUNT; i++) {
            n += snprintf(out + n, sizeof(out) - n, " %u", (unsigned)raw[i]);
        }
        n += snprintf(out + n, sizeof(out) - n, " N");
        for (int i = 0; i < SENSOR_COUNT; i++) {
            n += snprintf(out + n, sizeof(out) - n, " %ld", (long)LineSensor_Normalize(i, raw[i]));
        }
        int32_t pos;
        bool seen = LineSensor_GetPosition(&pos);
        if (!seen) {
            pos = POSITION_CENTER - Linha_ErroPerdida();    // posicao que o PID esta usando
        }
        snprintf(out + n, sizeof(out) - n, " POS %ld LINE %d", (long)pos, seen ? 1 : 0);
        Comm_SendLine(out);
    }
    else if (strcmp(cmd, "CAL?") == 0) {
        reply_cal(0);
    }
    else if (strcmp(cmd, "CAL") == 0) {
        if (a1 != NULL) {
            for (char *c = a1; *c; c++) *c = (char)toupper((unsigned char)*c);
        }
        if (a1 != NULL && strcmp(a1, "START") == 0) {
            Motor_Solto();
            motor_test_ticks = 0;
            LineSensor_CalibReset();
            robot_state = STATE_CALIBRANDO;
            Comm_SendLine("OK CAL START");
        } else if (a1 != NULL && strcmp(a1, "STOP") == 0) {
            if (robot_state != STATE_CALIBRANDO) {
                Comm_SendLine("ERR CAL NOT RUNNING");
                return;
            }
            robot_state = STATE_PARADO;
            reply_cal(LineSensor_CalibFinish());
        } else {
            reply_cal(0);
        }
    }
    else {
        Comm_SendLine("ERR UNKNOWN");
    }
}

void Comm_Process(void)
{
    while (rx_tail != rx_head) {
        char c = (char)rx_buf[rx_tail];
        rx_tail = (rx_tail + 1) % RX_BUF_SIZE;

        if (c == '\r') continue;

        if (c == '\n') {
            last_rx_ms = HAL_GetTick();
            if (line_overflow) {
                Comm_SendLine("ERR TOO LONG");
            } else if (line_len > 0) {
                line_buf[line_len] = '\0';
                handle_line(line_buf);
            }
            line_len = 0;
            line_overflow = false;
            continue;
        }

        if (line_len < LINE_MAX - 1) {
            line_buf[line_len++] = c;
        } else {
            line_overflow = true;
        }
    }
}

bool Comm_LinkTimedOut(void)
{
    return wd_timeout_ms > 0 && (HAL_GetTick() - last_rx_ms) > wd_timeout_ms;
}
