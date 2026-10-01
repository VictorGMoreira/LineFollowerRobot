/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "control.h"
#include "comm.h"
#include "motor.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

volatile uint16_t line_sensor_raw[SENSOR_COUNT];
PID_t pid;

volatile bool tick_flag = false;

/* Largada: 0 = liga PARADO e so anda com o comando RUN (seguro para calibrar).
 * Para competicao sem o ESP, coloque o atraso em ms (ex.: 3000 = anda 3 s apos ligar). */
#define AUTO_START_DELAY_MS   0

/* quantos ticks seguidos sem ver linha até considerar "perdida de fato"
 * (10 ticks a 500Hz = 20ms — ajuste se achar sensível demais ou de menos) */
#define LOST_DEBOUNCE_TICKS   10

/* Linha perdida NUNCA para o robo (prova de velocidade): ver Linha_Erro em control.c */

#define TICK_MS               2                 /* TIM4 a 500 Hz */

static uint32_t     lost_ticks = 0;             /* ticks seguidos sem ver linha */
static RobotState_t prev_state = STATE_INIT;    /* estado no tick anterior */

/* RobotState_t agora fica em control.h (comm.c tambem usa) */
volatile RobotState_t robot_state = STATE_INIT;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void Sensors_StartADC(void)
{
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)line_sensor_raw, SENSOR_COUNT);
    /* O DMA circular atualiza line_sensor_raw sozinho; as interrupcoes de meia/fim de
     * transferencia so ocupariam a CPU (a cada poucos us) e travariam o loop e a UART. */
    __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_HT | DMA_IT_TC);
    HAL_NVIC_DisableIRQ(DMA2_Stream0_IRQn);
}

/* Se o ADC der overrun, o DMA para e os sensores congelam: reinicia. */
static void Sensors_CheckADC(void)
{
    if (__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_OVR)) {
        HAL_ADC_Stop_DMA(&hadc1);
        __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_OVR);
        Sensors_StartADC();
        Comm_SendLine("EVT ADC RESTART");
    }
}

#if MODO_MEDIR_DZ
/* Rampa de medicao da zona morta: sobe DZ_PASSO a cada DZ_PASSO_MS, sem compensacao */
static uint32_t dz_ticks = 0;

static void DZ_Passo(void)
{
    const uint32_t ticks_por_passo = DZ_PASSO_MS / TICK_MS;
    int32_t u = (int32_t)(dz_ticks / ticks_por_passo) * DZ_PASSO;

    if (dz_ticks % ticks_por_passo == 0) {
        if (u > U_MAX) {
            Motor_Freio();
            robot_state = STATE_PARADO;
            Comm_SendLine("DZ FIM");
            return;
        }
        char msg[24];
        snprintf(msg, sizeof(msg), "DZ %ld", (long)u);
        Comm_SendLine(msg);
    }
    Motor_ComandoBruto(u, u);
    dz_ticks++;
}
#endif

static bool Robot_IsMoving(RobotState_t s)
{
    return s == STATE_SEGUINDO_LINHA || s == STATE_LINHA_PERDIDA || s == STATE_TESTE_MOTOR;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  Sensors_StartADC();
  Control_Init(&pid, 0.1f, 0.0f, 0.05f, 1000.0f);
  Comm_Init();                /* tem que vir DEPOIS do MX_USART1_UART_Init: arma a recepcao */
  if (!Motor_DirOk(0)) {
      Comm_SendLine("ERR MOTOR ESQ: pino de direcao (PB6) nao e saida no CubeMX - sem re/freio");
  }
  if (!Motor_DirOk(1)) {
      Comm_SendLine("ERR MOTOR DIR: pino de direcao (PB4) nao e saida no CubeMX - sem re/freio");
  }
  HAL_TIM_Base_Start_IT(&htim4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
      Comm_Process();
      Sensors_CheckADC();

      if (tick_flag) {
          tick_flag = false;

          /* Watchdog do link (comando WD): sem nada do ESP por muito tempo -> para */
          if (Robot_IsMoving(robot_state) && Comm_LinkTimedOut()) {
              Motors_Stop();
              motor_test_ticks = 0;
              robot_state = STATE_PARADO;
              Comm_SendLine("EVT WD STOP");
          }

          /* Entrou em seguir linha vindo de outro estado (RUN, largada): zera o contador */
          if (robot_state == STATE_SEGUINDO_LINHA &&
              prev_state != STATE_SEGUINDO_LINHA && prev_state != STATE_LINHA_PERDIDA) {
              lost_ticks = 0;
              Motors_ResetRampa();    /* parte de 0: as rodas estavam em freio */
#if MODO_MEDIR_DZ
              dz_ticks = 0;
#endif
          }

          switch (robot_state) {

              case STATE_INIT:
                  Motors_Stop();
#if AUTO_START_DELAY_MS > 0
                  if (HAL_GetTick() >= AUTO_START_DELAY_MS) {
                      PID_Reset(&pid);
                      LineSensor_ResetMemory();
                      robot_state = STATE_SEGUINDO_LINHA;
                  }
#else
                  robot_state = STATE_PARADO;     /* espera o RUN */
#endif
                  break;

              case STATE_SEGUINDO_LINHA:
              case STATE_LINHA_PERDIDA: {
#if MODO_MEDIR_DZ
                  DZ_Passo();
#else
                  /* Linha perdida: Linha_Erro devolve o ultimo erro valido ou +-ERRO_MAX
                   * (ver MARGEM_LADO). Nunca para os motores por perda de linha. */
                  bool seen;
                  int32_t error = Linha_Erro(&pid, &seen);

                  if (seen) {
                      lost_ticks = 0;
                      robot_state = STATE_SEGUINDO_LINHA;
                  } else if (++lost_ticks >= LOST_DEBOUNCE_TICKS) {
                      robot_state = STATE_LINHA_PERDIDA;     /* so informativo (STATUS) */
                  }

                  float pid_output = PID_Step(&pid, error);
                  Motors_ApplyPID(pid_output, error);
#endif
                  break;
              }

              case STATE_PARADO:
              case STATE_PARADO_SEGURANCA:
                  Motors_Stop();
                  break;

              case STATE_TESTE_MOTOR:
                  /* os motores ja foram ligados pelo comando MOTOR; aqui so conta o tempo */
                  if (motor_test_ticks > 0) {
                      motor_test_ticks--;
                  }
                  if (motor_test_ticks == 0) {
                      Motors_Stop();
                      robot_state = STATE_PARADO;
                      Comm_SendLine("EVT MOTOR DONE");
                  }
                  break;

              case STATE_CALIBRANDO:
                  Motor_Solto();      /* rodas soltas para arrastar o robo com a mao */
                  LineSensor_CalibSample();
                  break;

              default:
                  Motors_Stop();
                  break;
          }

          prev_state = robot_state;
      }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4) {
        tick_flag = true;
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
