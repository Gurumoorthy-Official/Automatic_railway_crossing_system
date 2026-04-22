/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Railway Gate Controller - FreeRTOS - Blue Pill
  *                   Pins:
  *                     PA1  = IR_OBSTACLE
  *                     PA2  = IR_EXIT
  *                     PA3  = IR_APPROACH
  *                     PA6  = Servo PWM (TIM3 CH1)
  *                     PB0  = ROAD_RED LED
  *                     PB1  = TRAIN_RED LED
  *                     PB10 = TRAIN_GREEN LED
  *                     PB12 = ERROR_LED (external red)
  *                     PC13 = DEBUG_LED (active LOW, onboard)
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "cmsis_os.h"
#include <stdbool.h>

/* Private typedef -----------------------------------------------------------*/
typedef enum {
    STATE_IDLE = 0,
    STATE_APPROACH,
    STATE_TRAIN_PASSING
} SystemState;

/* Private define ------------------------------------------------------------*/
#define IR_OBSTACLE_Pin         GPIO_PIN_1
#define IR_OBSTACLE_GPIO_Port   GPIOA

#define IR_EXIT_Pin             GPIO_PIN_2
#define IR_EXIT_GPIO_Port       GPIOA

#define IR_APPROACH_Pin         GPIO_PIN_3
#define IR_APPROACH_GPIO_Port   GPIOA

#define ROAD_RED_Pin            GPIO_PIN_0
#define ROAD_RED_GPIO_Port      GPIOB

#define TRAIN_RED_Pin           GPIO_PIN_1
#define TRAIN_RED_GPIO_Port     GPIOB

#define TRAIN_GREEN_Pin         GPIO_PIN_10
#define TRAIN_GREEN_GPIO_Port   GPIOB

#define ERROR_LED_Pin           GPIO_PIN_12
#define ERROR_LED_GPIO_Port     GPIOB

#define DEBUG_LED_Pin           GPIO_PIN_13
#define DEBUG_LED_GPIO_Port     GPIOC

#define SERVO_OPEN_PULSE    500U   // 0°
#define SERVO_CLOSED_PULSE  1500U  // 90°

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

// RTOS task handles
osThreadId_t sensorTaskHandle;
osThreadId_t gateTaskHandle;
osThreadId_t ledTaskHandle;

// Task attributes
const osThreadAttr_t sensorTask_attributes = {
    .name = "sensorTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t) osPriorityNormal,
};
const osThreadAttr_t gateTask_attributes = {
    .name = "gateTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t) osPriorityNormal,
};
const osThreadAttr_t ledTask_attributes = {
    .name = "ledTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityNormal,
};

// RTOS primitives
osSemaphoreId_t trainApproachSem;
osSemaphoreId_t trainExitSem;

// Shared state
volatile SystemState systemState   = STATE_IDLE;
volatile uint8_t     obstaclePresent = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
void SensorTask(void *argument);
void GateTask(void *argument);
void LedTask(void *argument);

/* USER CODE BEGIN 0 */

// ---- Servo helpers ----
void ServoSetPulse(uint32_t pulse)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse);
}

void ServoOpen(void)
{
    int32_t current = (int32_t)__HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1);
    int32_t target  = (int32_t)SERVO_OPEN_PULSE;
    for (int32_t p = current; p >= target; p -= 2)  // smaller step = smoother
    {
        ServoSetPulse((uint32_t)p);
        osDelay(8);  // shorter delay = faster but still smooth
    }
    ServoSetPulse(SERVO_OPEN_PULSE);
}

void ServoClose(void)
{
    int32_t current = (int32_t)__HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1);
    int32_t target  = (int32_t)SERVO_CLOSED_PULSE;
    for (int32_t p = current; p <= target; p += 2)  // smaller step = smoother
    {
        ServoSetPulse((uint32_t)p);
        osDelay(8);
    }
    ServoSetPulse(SERVO_CLOSED_PULSE);
}

// ---- Stable sensor read (active LOW, 7 samples) ----
bool StableLow(GPIO_TypeDef *port, uint16_t pin)
{
    int count = 0;
    for (int i = 0; i < 7; i++)
    {
        if (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET) count++;
        osDelay(5);
    }
    return (count >= 6);
}

/* USER CODE END 0 */

/* ================================================================
   MAIN
   ================================================================ */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_TIM3_Init();

    // Start servo PWM and set gate open at boot
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    ServoSetPulse(SERVO_OPEN_PULSE);

    // Initial LED state
    HAL_GPIO_WritePin(ROAD_RED_GPIO_Port,    ROAD_RED_Pin,    GPIO_PIN_RESET); // off
    HAL_GPIO_WritePin(TRAIN_RED_GPIO_Port,   TRAIN_RED_Pin,   GPIO_PIN_SET);   // on
    HAL_GPIO_WritePin(TRAIN_GREEN_GPIO_Port, TRAIN_GREEN_Pin, GPIO_PIN_RESET); // off
    HAL_GPIO_WritePin(ERROR_LED_GPIO_Port,   ERROR_LED_Pin,   GPIO_PIN_RESET); // off
    HAL_GPIO_WritePin(DEBUG_LED_GPIO_Port,   DEBUG_LED_Pin,   GPIO_PIN_SET);   // off (active low)

    // Init RTOS kernel
    osKernelInitialize();

    // Create semaphores
    trainApproachSem = osSemaphoreNew(1, 0, NULL);
    trainExitSem     = osSemaphoreNew(1, 0, NULL);

    if (trainApproachSem == NULL) Error_Handler();
    if (trainExitSem == NULL)     Error_Handler();

    // Create tasks
    sensorTaskHandle = osThreadNew(SensorTask, NULL, &sensorTask_attributes);
    gateTaskHandle   = osThreadNew(GateTask,   NULL, &gateTask_attributes);
    ledTaskHandle    = osThreadNew(LedTask,    NULL, &ledTask_attributes);

    // Start scheduler — never returns
    osKernelStart();

    while (1) {}
}

/* ================================================================
   SENSOR TASK — monitors IR sensors, updates state
   ================================================================ */
void SensorTask(void *argument)
{
    osDelay(2000); // ignore unstable startup readings

    for (;;)
    {
        switch (systemState)
        {
            case STATE_IDLE:
                if (StableLow(IR_APPROACH_GPIO_Port, IR_APPROACH_Pin))
                {
                    systemState = STATE_APPROACH;
                    osSemaphoreRelease(trainApproachSem);
                }
                break;

            case STATE_APPROACH:
                if (StableLow(IR_OBSTACLE_GPIO_Port, IR_OBSTACLE_Pin))
                    obstaclePresent = 1;
                else
                    obstaclePresent = 0;
                break;

            case STATE_TRAIN_PASSING:
                if (StableLow(IR_EXIT_GPIO_Port, IR_EXIT_Pin))
                {
                    systemState = STATE_IDLE;
                    osSemaphoreRelease(trainExitSem);
                }
                break;
        }

        osDelay(10);
    }
}

/* ================================================================
   GATE TASK — controls servo and LEDs based on state
   ================================================================ */
void GateTask(void *argument)
{
    for (;;)
    {
        // Wait for train approach signal
        osSemaphoreAcquire(trainApproachSem, osWaitForever);

        // Blink road red LED 5 times as warning
        for (int i = 0; i < 5; i++)
        {
            HAL_GPIO_WritePin(ROAD_RED_GPIO_Port, ROAD_RED_Pin, GPIO_PIN_SET);
            osDelay(300);
            HAL_GPIO_WritePin(ROAD_RED_GPIO_Port, ROAD_RED_Pin, GPIO_PIN_RESET);
            osDelay(300);
        }
        HAL_GPIO_WritePin(ROAD_RED_GPIO_Port, ROAD_RED_Pin, GPIO_PIN_SET); // keep on

        // Wait until obstacle clears, then close gate
        while (1)
        {
            if (obstaclePresent)
            {
                // Obstacle present — keep gate open
                ServoOpen();
                osDelay(500);
            }
            else
            {
                osDelay(200); // settle
                if (!obstaclePresent)
                {
                    // Safe to close
                    ServoClose();
                    systemState = STATE_TRAIN_PASSING;

                    HAL_GPIO_WritePin(TRAIN_RED_GPIO_Port,   TRAIN_RED_Pin,   GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(TRAIN_GREEN_GPIO_Port, TRAIN_GREEN_Pin, GPIO_PIN_SET);
                    break;
                }
            }
        }

        // Wait for train to fully exit
        osSemaphoreAcquire(trainExitSem, osWaitForever);
        osDelay(500); // debounce

        // Open gate and reset LEDs
        ServoOpen();
        HAL_GPIO_WritePin(ROAD_RED_GPIO_Port,    ROAD_RED_Pin,    GPIO_PIN_RESET);
        HAL_GPIO_WritePin(TRAIN_RED_GPIO_Port,   TRAIN_RED_Pin,   GPIO_PIN_SET);
        HAL_GPIO_WritePin(TRAIN_GREEN_GPIO_Port, TRAIN_GREEN_Pin, GPIO_PIN_RESET);
    }
}

/* ================================================================
   LED TASK — blinks debug LED to show system is alive
   ================================================================ */
void LedTask(void *argument)
{
    for (;;)
    {
        HAL_GPIO_TogglePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin);
        osDelay(1000);
    }
}

/* ================================================================
   CUBEMX GENERATED FUNCTIONS
   ================================================================ */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI_DIV2;
    RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL16;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

static void MX_TIM3_Init(void)
{
    TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig      = {0};
    TIM_OC_InitTypeDef      sConfigOC          = {0};

    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 63;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 19999;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK) Error_Handler();

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) Error_Handler();

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) Error_Handler();

    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();

    HAL_TIM_MspPostInit(&htim3);
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    // Default output states
    HAL_GPIO_WritePin(DEBUG_LED_GPIO_Port,   DEBUG_LED_Pin,   GPIO_PIN_SET);   // off (active low)
    HAL_GPIO_WritePin(ROAD_RED_GPIO_Port,    ROAD_RED_Pin,    GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TRAIN_RED_GPIO_Port,   TRAIN_RED_Pin,   GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TRAIN_GREEN_GPIO_Port, TRAIN_GREEN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ERROR_LED_GPIO_Port,   ERROR_LED_Pin,   GPIO_PIN_RESET);

    // PC13 — DEBUG_LED
    GPIO_InitStruct.Pin   = DEBUG_LED_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DEBUG_LED_GPIO_Port, &GPIO_InitStruct);

    // PA1, PA2, PA3 — IR sensors
    GPIO_InitStruct.Pin  = IR_OBSTACLE_Pin | IR_EXIT_Pin | IR_APPROACH_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // PB0 — ROAD_RED
    GPIO_InitStruct.Pin   = ROAD_RED_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ROAD_RED_GPIO_Port, &GPIO_InitStruct);

    // PB1 — TRAIN_RED
    GPIO_InitStruct.Pin   = TRAIN_RED_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(TRAIN_RED_GPIO_Port, &GPIO_InitStruct);

    // PB10 — TRAIN_GREEN
    GPIO_InitStruct.Pin   = TRAIN_GREEN_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(TRAIN_GREEN_GPIO_Port, &GPIO_InitStruct);

    // PB12 — ERROR_LED
    GPIO_InitStruct.Pin   = ERROR_LED_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ERROR_LED_GPIO_Port, &GPIO_InitStruct);
}

void Error_Handler(void)
{
    __disable_irq();
    HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, GPIO_PIN_SET); // solid on
    while (1)
    {
        HAL_GPIO_TogglePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin);
        HAL_Delay(100);
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
