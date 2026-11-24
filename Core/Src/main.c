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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdarg.h>
/* No project_app.h or project_*.c files required. */

#define PG_W 80
#define PG_H 24
#define PG_AUDIO_RATE 16000U
#define PG_AUDIO_HALF 256U
#define PG_AUDIO_SIZE 512U
#define PG_GHOST_MAX 9000U /* 5 minutes at 30 Hz */
#define PG_TRAFFIC_MAX 9U
#define PG_SCORE_COUNT 5U
#define PG_TICK_SENSOR_HZ 100U
#define PG_TICK_GAME_HZ 30U
#define PG_TICK_RENDER_HZ 30U

/* Tunable algorithm settings. */
#define PG_KALMAN_Q 0.06f
#define PG_KALMAN_R 1.0f
#define PG_STEER_DEG_RANGE 28.0f
#define PG_SENSOR_CALIBRATION_SAMPLES 64U
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    PG_MENU = 0, PG_RUNNING = 1, PG_PAUSED = 2,
    PG_GAME_OVER = 3, PG_REPLAY = 4
} PG_Mode;

typedef enum { PG_FX_CRASH = 1 } PG_SoundEvent;

typedef struct {
    float rawRoll, rawPitch, roll, pitch;
    uint32_t tick;
    uint8_t valid;
} PG_Tilt;

typedef struct {
    int8_t x;    /* column */
    int8_t y;    /* row; -1 means inactive */
    uint8_t kind;
} PG_Car;

typedef struct {
    PG_Mode mode;
    int8_t playerX, ghostX;
    uint16_t speed;
    uint32_t score, frame, seed;
    uint32_t scores[PG_SCORE_COUNT];
    PG_Car traffic[PG_TRAFFIC_MAX];
    uint32_t sensorTick;
    float rawRoll, filteredRoll, rawPitch, filteredPitch;
    uint8_t sensorOk, flashOk, ghostAvailable, collision;
    uint32_t ghostFrameCount;
    uint32_t replayMismatch;
} PG_View;

/* Kept small enough for 16 KB flash slots, with CRC on the entire record. */
typedef struct {
    uint32_t magic, version, sequence;
    uint32_t scores[PG_SCORE_COUNT];
    uint32_t bestSeed;
    uint32_t bestLength;
    uint32_t bestRunHash;
    int8_t positions[PG_GHOST_MAX];
    uint32_t crc32;
} PG_Save;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Five testpoint outputs are already configured as push-pull GPIO in
 * supplied CubeMX main.c. Probe the named Arduino header pins with a logic
 * analyzer. Set PG_TRACE_ENABLED=0 to reclaim pins. */
#ifndef PG_TRACE_ENABLED
#define PG_TRACE_ENABLED 1
#endif
#if PG_TRACE_ENABLED
#define PG_TRACE_Audio_Pin ARD_D10_Pin
#define PG_TRACE_Audio_GPIO_Port ARD_D10_GPIO_Port
#define PG_TRACE_Sensor_Pin ARD_D4_Pin
#define PG_TRACE_Sensor_GPIO_Port ARD_D4_GPIO_Port
#define PG_TRACE_Game_Pin ARD_D8_Pin
#define PG_TRACE_Game_GPIO_Port ARD_D8_GPIO_Port
#define PG_TRACE_Render_Pin ARD_D5_Pin
#define PG_TRACE_Render_GPIO_Port ARD_D5_GPIO_Port
#define PG_TRACE_Storage_Pin ARD_D9_Pin
#define PG_TRACE_Storage_GPIO_Port ARD_D9_GPIO_Port
#define PG_TRACE_BEGIN(name) HAL_GPIO_WritePin(PG_TRACE_##name##_GPIO_Port, PG_TRACE_##name##_Pin, GPIO_PIN_SET)
#define PG_TRACE_END(name) HAL_GPIO_WritePin(PG_TRACE_##name##_GPIO_Port, PG_TRACE_##name##_Pin, GPIO_PIN_RESET)
#else
#define PG_TRACE_BEGIN(task_name) ((void)0)
#define PG_TRACE_END(task_name)   ((void)0)
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

DFSDM_Channel_HandleTypeDef hdfsdm1_channel2;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

OSPI_HandleTypeDef hospi1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for audioTask */
osThreadId_t audioTaskHandle;
const osThreadAttr_t audioTask_attributes = {
  .name = "audioTask",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for sensorTask */
osThreadId_t sensorTaskHandle;
const osThreadAttr_t sensorTask_attributes = {
  .name = "sensorTask",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for gameTask */
osThreadId_t gameTaskHandle;
const osThreadAttr_t gameTask_attributes = {
  .name = "gameTask",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for renderTask */
osThreadId_t renderTaskHandle;
const osThreadAttr_t renderTask_attributes = {
  .name = "renderTask",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for storageTask */
osThreadId_t storageTaskHandle;
const osThreadAttr_t storageTask_attributes = {
  .name = "storageTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */
volatile size_t debugFreeHeap = 0;
volatile size_t debugMinFreeHeap = 0;

/* Game-wide state and RTOS synchronization handles. */
/* The *single owner* of each hardware peripheral is documented in README. */
osMessageQueueId_t pgTiltQueue, pgSoundQueue, pgAudioHalfQueue, pgSaveQueue;
osSemaphoreId_t pgAudioSemaphore, pgUartSemaphore, pgButtonSemaphore;
osMutexId_t pgViewMutex, pgFlashMutex, pgSaveMutex;
volatile uint16_t pgAudioSpeed = 0;
volatile uint8_t pgFlashMapped = 0, pgFlashReady = 0;
volatile uint32_t pgAudioRefills = 0, pgAudioLate = 0, pgAudioUnderruns = 0;
volatile uint32_t pgAudioStartError = 0, pgSensorErrors = 0, pgSensorReadings = 0;
volatile uint32_t pgGameFrames = 0, pgRenderFrames = 0, pgUartErrors = 0;
volatile uint32_t pgFlashErrors = 0, pgSavedGames = 0, pgTiltToScreenMs = 0;
/* Flash boot diagnostics, visible in PuTTY and the debugger.
 * Step identifies the last attempted phase; FailStep + FailOp identify
 * the first operation that failed.  These operations do not erase flash. */
volatile uint32_t pgFlashStep = 0, pgFlashFailStep = 0, pgFlashFailOp = 0;
volatile uint32_t pgFlashHalError = 0, pgFlashHalState = 0;
volatile uint32_t pgFlashAddress = 0;
volatile uint8_t pgFlashId[3] = {0U, 0U, 0U};
volatile uint8_t pgFlashStatus = 0xFFU;
volatile uint32_t pgAvgFpsX100 = 0;
volatile float pgRollRawVariance = 0, pgRollFilteredVariance = 0;
volatile float pgPitchRawVariance = 0, pgPitchFilteredVariance = 0;
volatile uint32_t pgReplayMismatch = 0;
PG_Tilt pgTiltLog[512];
volatile uint32_t pgTiltLogHead = 0;
volatile uint8_t pgCpuPercent[5] = {0};
static PG_View pgView;
static PG_Save pgCurrentSave; /* score table and best-run trace */
static int8_t pgCurrentRun[PG_GHOST_MAX];
static uint16_t pgAudioBuffer[PG_AUDIO_SIZE] __attribute__((aligned(4)));
static uint32_t pgCurrentCount = 0;
static uint32_t pgCurrentHash = 2166136261UL;
static uint8_t pgIsInitialized = 0;
static PG_Save pgStorageSnapshot; /* Avoid 9 KB stack allocations. */
static uint8_t pgImuStartupOk = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_OCTOSPI1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI3_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_USB_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM7_Init(void);
void StartDefaultTask(void *argument);
void audioTask1(void *argument);
void sensorTask1(void *argument);
void gameTask1(void *argument);
void renderTask1(void *argument);
void storageTask1(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ================================================================
 * APPLICATION IMPLEMENTATION (all code intentionally in main.c)
 * Section 1: Game logic
 * Section 2: LSM6DSL I2C reader, recovery and Kalman filter
 * Section 3: QSPI flash, scores, ghost and sound effect storage
 * Section 4: DAC audio synthesis and mixing
 * Section 5: RTOS shared routines, renderer, callbacks and profiling
 * ================================================================ */

/* ======== GAME ======== */
#include <string.h>

static int32_t pg_clamp(int32_t v, int32_t low, int32_t high)
{
    return (v < low) ? low : (v > high ? high : v);
}

uint32_t PG_SeedHash(uint32_t state, uint32_t datum)
{
    /* FNV-1a on integer bytes, independent of endianness. */
    for (unsigned i = 0; i < 4; i++) {
        state = (state ^ (datum & 0xFFU)) * 16777619U;
        datum >>= 8U;
    }
    return state;
}

void PG_InsertScore(uint32_t scores[PG_SCORE_COUNT], uint32_t score)
{
    for (unsigned i = 0; i < PG_SCORE_COUNT; i++) {
        if (score > scores[i]) {
            for (unsigned j = PG_SCORE_COUNT - 1; j > i; j--)
                scores[j] = scores[j - 1];
            scores[i] = score;
            return;
        }
    }
}

static uint32_t pg_random(uint32_t *seed)
{
    uint32_t x = *seed;
    if (!x) x = 0xA764C943U;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *seed = x;
    return x;
}

void PG_GameReset(PG_View *v, uint32_t seed, PG_Mode mode)
{
    uint32_t oldScores[PG_SCORE_COUNT];
    uint8_t flashOk = v->flashOk;
    uint8_t ghostAvailable = v->ghostAvailable;
    uint32_t ghostLength = v->ghostFrameCount;
    for (unsigned j = 0; j < PG_SCORE_COUNT; j++) oldScores[j] = v->scores[j];
    memset(v, 0, sizeof(*v));
    for (unsigned j = 0; j < PG_SCORE_COUNT; j++) v->scores[j] = oldScores[j];
    for (unsigned i = 0; i < PG_TRAFFIC_MAX; i++) v->traffic[i].y = -1;
    v->mode = mode;
    v->playerX = 40;
    v->ghostX = -1;
    v->speed = 35;
    v->seed = seed ? seed : 0x12345678U;
    v->flashOk = flashOk;
    v->ghostAvailable = ghostAvailable;
    v->ghostFrameCount = ghostLength;
}

/* Traffic is a seeded function of frame number, independent of tilt input.
 * This ensures traffic can be reconstructed when replaying the best run. */
static uint32_t pg_frame_random(uint32_t seed, uint32_t frame)
{
    uint32_t state = seed ^ (frame * 0x9E3779B9U);
    return pg_random(&state);
}

void PG_GameStep(PG_View *v, const PG_Tilt *tilt, const int8_t *replay,
                 uint32_t replayLength, int8_t *record, uint32_t *recordCount,
                 uint32_t *runHash)
{
    if (v->mode != PG_RUNNING && v->mode != PG_REPLAY) return;
    if (tilt) {
        v->sensorTick = tilt->tick;
        v->sensorOk = tilt->valid;
        v->rawRoll = tilt->rawRoll;
        v->rawPitch = tilt->rawPitch;
        v->filteredRoll = tilt->roll;
        v->filteredPitch = tilt->pitch;
    }
    if (v->mode == PG_RUNNING && tilt && tilt->valid) {
        float ratio = tilt->roll / PG_STEER_DEG_RANGE;
        if (ratio > 1.0f) ratio = 1.0f;
        if (ratio < -1.0f) ratio = -1.0f;
        v->playerX = (int8_t)pg_clamp(40 + (int32_t)(ratio * 14.0f), 26, 54);
        v->speed = (uint16_t)pg_clamp(40 - (int32_t)(tilt->pitch * 2.0f), 0, 100);
    }
    if (v->mode == PG_REPLAY) {
        if (v->frame >= replayLength || replay == NULL) {
            v->mode = PG_MENU;
            return;
        }
        v->playerX = replay[v->frame];
        v->ghostX = v->playerX;
        v->speed = 60; /* visual replay only; traffic timing depends on frame */
    } else if (record && recordCount && *recordCount < PG_GHOST_MAX) {
        record[*recordCount] = v->playerX;
        ++*recordCount;
    }
    /* Every frame advances traffic, with density increasing over time. */
    uint32_t spawnInterval = (v->frame / 600U < 8U) ? (17U - v->frame / 600U) : 9U;
    if ((v->frame % spawnInterval) == 0U) {
        uint32_t dice = pg_frame_random(v->seed, v->frame);
        unsigned slot = PG_TRAFFIC_MAX;
        for (unsigned j = 0; j < PG_TRAFFIC_MAX; j++) {
            if (v->traffic[j].y < 0) { slot = j; break; }
        }
        if (slot < PG_TRAFFIC_MAX) {
            v->traffic[slot].x = (int8_t)(30 + 10 * (dice % 3U));
            v->traffic[slot].y = 3;
            v->traffic[slot].kind = (uint8_t)((dice >> 8) % 3U);
        }
    }
    for (unsigned j = 0; j < PG_TRAFFIC_MAX; j++) {
        PG_Car *c = &v->traffic[j];
        if (c->y < 0) continue;
        /* Ignore spawn row on spawn frame and move all traffic one row/frame. */
        c->y += 1;
        if (c->y >= 22) { c->y = -1; continue; }
        if (v->mode == PG_RUNNING && c->y >= 19 && c->y <= 21 &&
            (int32_t)v->playerX >= (int32_t)c->x - 2 &&
            (int32_t)v->playerX <= (int32_t)c->x + 2) {
            v->mode = PG_GAME_OVER;
            v->collision = 1;
        }
    }
    if (v->mode == PG_RUNNING) v->score += (1U + v->speed / 10U);
    if (runHash) {
        *runHash = PG_SeedHash(*runHash, (uint8_t)v->playerX);
        *runHash = PG_SeedHash(*runHash, pg_frame_random(v->seed, v->frame));
    }
    v->frame++;
    if (v->mode == PG_RUNNING && recordCount && *recordCount >= PG_GHOST_MAX)
        v->mode = PG_GAME_OVER; /* bounded RAM; complete 5-min trace */
    if (v->mode == PG_REPLAY && v->frame >= replayLength)
        v->mode = PG_MENU;
}

/* ======== SENSOR / KALMAN ======== */
#include <math.h>

/* LSM6DSL: WHO_AM_I 0x6A; 104 Hz, +/-2 g, BDU + auto-increment. */
#define IMU_WHO_AM_I 0x0FU
#define IMU_CTRL1_XL 0x10U
#define IMU_CTRL3_C  0x12U
#define IMU_OUT_XL   0x28U
static uint16_t imuAddr = 0U;

int PG_IMU_Init(void)
{
    uint8_t who = 0U, value;
    imuAddr = 0U; /* Do not reuse a stale address after I2C failure. */
    for (unsigned a = 0; a < 2; ++a) {
        uint16_t address = (uint16_t)((a ? 0x6BU : 0x6AU) << 1);
        if (HAL_I2C_Mem_Read(&hi2c2, address, IMU_WHO_AM_I, I2C_MEMADD_SIZE_8BIT,
                              &who, 1, 100) == HAL_OK && who == 0x6AU) {
            imuAddr = address;
            break;
        }
    }
    if (!imuAddr) return 0;
    value = 0x44U; /* BDU + IF_INC */
    if (HAL_I2C_Mem_Write(&hi2c2, imuAddr, IMU_CTRL3_C, I2C_MEMADD_SIZE_8BIT,
                           &value, 1, 100) != HAL_OK) return 0;
    value = 0x40U; /* ODR_XL = 104 Hz; FS_XL = +/-2 g. */
    if (HAL_I2C_Mem_Write(&hi2c2, imuAddr, IMU_CTRL1_XL, I2C_MEMADD_SIZE_8BIT,
                           &value, 1, 100) != HAL_OK) return 0;
    if (HAL_I2C_Mem_Write(&hi2c2, imuAddr, 0x11U, I2C_MEMADD_SIZE_8BIT,
                           &value, 1, 100) != HAL_OK) return 0;
    return 1;
}

int PG_IMU_Read(float *roll, float *pitch)
{
    uint8_t bytes[6];
    if (!imuAddr || !roll || !pitch ||
        HAL_I2C_Mem_Read(&hi2c2, imuAddr, IMU_OUT_XL, I2C_MEMADD_SIZE_8BIT,
                         bytes, sizeof(bytes), 15) != HAL_OK) return 0;
    const float scale = 0.000061f; /* 0.061 mg/LSB for +/-2g. */
    int16_t x = (int16_t)((uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8));
    int16_t y = (int16_t)((uint16_t)bytes[2] | ((uint16_t)bytes[3] << 8));
    int16_t z = (int16_t)((uint16_t)bytes[4] | ((uint16_t)bytes[5] << 8));
    float ax = x * scale, ay = y * scale, az = z * scale;
    float sq = ax * ax + ay * ay + az * az;
    if (sq < 0.04f || sq > 3.0f) return 0; /* invalid / violent motion */
    /* Use a gravity-vector tilt angle with no +/-180 degree discontinuity.
     * For a board laid flat, roll is near zero regardless of Z axis sign.
     * ±90 degrees is more than sufficient for the ±28-degree steering range. */
    *roll = atan2f(ay, sqrtf(ax * ax + az * az)) * 57.2957795f;
    *pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.2957795f;
    return 1;
}

float PG_KalmanUpdate(float z, float *x, float *p, float q, float r)
{
    *p += q;
    float k = *p / (*p + r);
    *x += k * (z - *x);
    *p *= (1.0f - k);
    return *x;
}

/* ======== FLASH / CRC ======== */
#include <string.h>

/* MX25R6435F 8 MiB. Dedicated flash layout: do NOT share with other data. */
#define NOR_BYTES        0x800000UL
#define NOR_SECTOR       4096UL
#define NOR_PAGE         256UL
#define NOR_FX_ADDR      0x000000UL
#define NOR_FX_DATA      0x000100UL
#define NOR_FX_LENGTH    2048U
#define NOR_SLOT_A       0x7F8000UL /* 16 KiB each */
#define NOR_SLOT_B       0x7FC000UL
#define NOR_SLOT_BYTES   0x4000UL
#define NOR_FX_MAGIC     0x44554658UL /* DUFX */
#define NOR_SAVE_MAGIC   0x45435345UL /* ECSE */
#define NOR_SAVE_VERSION 1UL
#define NOR_MEMORY_BASE  0x90000000UL /* STM32L4+ OCTOSPI1 aperture */

/* Failure operation codes:
 * 1=HAL command, 2=receive, 3=transmit, 4=wait timeout/status,
 * 5=JEDEC mismatch, 6=write-enable absent, 7=record verification,
 * 8=bad input, 9=HAL memory map, 10=HAL unmap. */
static int nor_failed(uint32_t op)
{
    pgFlashFailStep = pgFlashStep;
    pgFlashFailOp = op;
    pgFlashHalError = (uint32_t)hospi1.ErrorCode;
    pgFlashHalState = (uint32_t)hospi1.State;
    return 0;
}

static uint8_t norMapped = 0;
static uint32_t norLastSequence = 0;
static uint8_t fxData[NOR_FX_LENGTH];
static PG_Save recordScratch; /* never on a small RTOS task stack */

typedef struct { uint32_t magic, version, length, crc; } FlashFxHeader;

uint32_t PG_Crc32(const void *data, size_t length)
{
    const uint8_t *p = (const uint8_t *)data;
    uint32_t crc = 0xFFFFFFFFUL;
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint32_t)p[i];
        for (unsigned bit = 0; bit < 8; bit++)
            crc = (crc >> 1) ^ (0xEDB88320UL & (0U - (crc & 1U)));
    }
    return ~crc;
}

static OSPI_RegularCmdTypeDef nor_command(uint8_t opcode)
{
    OSPI_RegularCmdTypeDef c = {0};
    c.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
    c.FlashId = HAL_OSPI_FLASH_ID_1;
    c.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
    c.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
    c.Instruction = opcode;
    c.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
    c.AddressMode = HAL_OSPI_ADDRESS_NONE;
    c.AddressSize = HAL_OSPI_ADDRESS_24_BITS;
    c.AddressDtrMode = HAL_OSPI_ADDRESS_DTR_DISABLE;
    c.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
    c.DataMode = HAL_OSPI_DATA_NONE;
    c.DataDtrMode = HAL_OSPI_DATA_DTR_DISABLE;
    c.DummyCycles = 0;
    c.DQSMode = HAL_OSPI_DQS_DISABLE;
    c.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;
    return c;
}

static int nor_cmdonly(uint8_t op)
{
    OSPI_RegularCmdTypeDef c = nor_command(op);
    if (HAL_OSPI_Command(&hospi1, &c, 100U) != HAL_OK)
        return nor_failed(1U);
    return 1;
}

static int nor_status(uint8_t *status)
{
    OSPI_RegularCmdTypeDef c = nor_command(0x05U);
    c.DataMode = HAL_OSPI_DATA_1_LINE;
    c.NbData = 1;
    if (HAL_OSPI_Command(&hospi1, &c, 100U) != HAL_OK)
        return nor_failed(1U);
    if (HAL_OSPI_Receive(&hospi1, status, 100U) != HAL_OK)
        return nor_failed(2U);
    pgFlashStatus = *status;
    return 1;
}

static int nor_wait(uint32_t timeoutMs)
{
    uint32_t started = HAL_GetTick();
    uint8_t s = 1;
    do {
        if (!nor_status(&s)) return 0;
        if ((s & 1U) == 0) return 1;
        osDelay(1);
    } while ((HAL_GetTick() - started) < timeoutMs);
    return nor_failed(4U);
}

static int nor_wren(void)
{
    uint8_t status;
    if (!nor_cmdonly(0x06U) || !nor_status(&status)) return 0;
    if (!(status & 2U)) return nor_failed(6U);
    return 1;
}

/* Use bounded indirect reads for controlled transactions and diagnostics.
 * Earlier reads at 0x7F8000 failed because OCTOSPI DeviceSize was configured
 * for only 4 MiB; the required DeviceSize for this 8 MiB chip is 23.
 * Keep chunked reads to limit per-transaction latency and identify any future
 * failing address in pgFlashAddress. This routine never programs or erases. */
#define NOR_READ_CHUNK 256U
static int nor_read(uint32_t address, void *out, uint32_t n)
{
    pgFlashAddress = address;
    if (!out || address > NOR_BYTES || n > NOR_BYTES - address || !n)
        return nor_failed(8U);

    uint8_t *dst = (uint8_t *)out;
    while (n != 0U) {
        uint32_t chunk = (n > NOR_READ_CHUNK) ? NOR_READ_CHUNK : n;
        pgFlashAddress = address;
        OSPI_RegularCmdTypeDef c = nor_command(0x03U);
        c.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
        c.Address = address;
        c.DataMode = HAL_OSPI_DATA_1_LINE;
        c.NbData = chunk;
        if (HAL_OSPI_Command(&hospi1, &c, 200U) != HAL_OK)
            return nor_failed(1U);
        if (HAL_OSPI_Receive(&hospi1, dst, 500U) != HAL_OK)
            return nor_failed(2U);
        address += chunk;
        dst += chunk;
        n -= chunk;
    }
    return 1;
}

static int nor_erase4k(uint32_t address)
{
    pgFlashAddress = address;
    if ((address & (NOR_SECTOR - 1)) || address >= NOR_BYTES)
        return nor_failed(8U);
    if (!nor_wren()) return 0;
    OSPI_RegularCmdTypeDef c = nor_command(0x20U);
    c.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
    c.Address = address;
    if (HAL_OSPI_Command(&hospi1, &c, 100U) != HAL_OK)
        return nor_failed(1U);
    return nor_wait(3000U);
}

static int nor_write(uint32_t address, const void *data, uint32_t length)
{
    const uint8_t *p = (const uint8_t *)data;
    pgFlashAddress = address;
    if (address > NOR_BYTES || length > NOR_BYTES - address)
        return nor_failed(8U);
    while (length) {
        uint32_t take = NOR_PAGE - (address % NOR_PAGE);
        if (take > length) take = length;
        if (!nor_wren()) return 0;
        OSPI_RegularCmdTypeDef c = nor_command(0x02U);
        c.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
        c.Address = address;
        c.DataMode = HAL_OSPI_DATA_1_LINE;
        c.NbData = take;
        pgFlashAddress = address;
        if (HAL_OSPI_Command(&hospi1, &c, 100U) != HAL_OK)
            return nor_failed(1U);
        if (HAL_OSPI_Transmit(&hospi1, (uint8_t *)(uintptr_t)p, 200U) != HAL_OK)
            return nor_failed(3U);
        if (!nor_wait(500U)) return 0;
        address += take;
        p += take;
        length -= take;
    }
    return 1;
}

int PG_FlashInit(void)
{
    uint8_t id[3] = {0U, 0U, 0U};
    pgFlashFailStep = pgFlashFailOp = 0U;
    pgFlashHalError = pgFlashHalState = 0U;
    pgFlashId[0] = pgFlashId[1] = pgFlashId[2] = 0U;

    /* Non-destructive startup recovery: a previous reset may have left the
     * MX25R6435F in deep power-down or enhanced-performance mode.
     * 0xAB wakes the chip; 0xFF exits performance-read mode;
     * 0x66 followed by 0x99 resets volatile command state, not user data. */
    pgFlashStep = 1U;
    if (!nor_cmdonly(0xABU)) return 0;
    osDelay(1U);
    pgFlashStep = 2U;
    if (!nor_cmdonly(0xFFU) || !nor_cmdonly(0x66U) ||
        !nor_cmdonly(0x99U)) return 0;
    osDelay(1U);

    OSPI_RegularCmdTypeDef c = nor_command(0x9FU);
    c.DataMode = HAL_OSPI_DATA_1_LINE;
    c.NbData = 3U;
    pgFlashStep = 10U;
    if (HAL_OSPI_Command(&hospi1, &c, 100U) != HAL_OK)
        return nor_failed(1U);
    pgFlashStep = 11U;
    if (HAL_OSPI_Receive(&hospi1, id, 100U) != HAL_OK)
        return nor_failed(2U);
    for (unsigned i = 0; i < 3; ++i) pgFlashId[i] = id[i];
    pgFlashStep = 12U;
    if (id[0] != 0xC2U || id[1] != 0x28U || id[2] != 0x17U)
        return nor_failed(5U);
    pgFlashStep = 13U;
    if (!nor_wait(1000U)) return 0;
    pgFlashStep = 14U;
    return 1;
}

static int nor_prepare_effects(void)
{
    FlashFxHeader fx = {0};
    pgFlashStep = 20U;
    /* A failed I/O read must NOT be mistaken for an unformatted sector. */
    if (!nor_read(NOR_FX_ADDR, &fx, sizeof(fx))) return 0;
    if (fx.magic == NOR_FX_MAGIC && fx.version == 1 &&
        fx.length == NOR_FX_LENGTH) {
        pgFlashStep = 21U;
        if (!nor_read(NOR_FX_DATA, fxData, NOR_FX_LENGTH)) return 0;
        if (fx.crc == PG_Crc32(fxData, NOR_FX_LENGTH)) return 1;
    }
    /* Missing/invalid bank: the original firmware creates it in flash.
     * Only permit this in the explicitly reserved first 4 KiB sector. */

    /* One-time deterministic crash SFX bank. Never erase any other sectors. */
    uint32_t rng = 0xACDC1234UL;
    for (uint32_t i = 0; i < NOR_FX_LENGTH; i++) {
        rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
        int32_t sample = ((int32_t)(rng & 0xFFU) - 128) *
                         (int32_t)(NOR_FX_LENGTH - i) / (int32_t)NOR_FX_LENGTH;
        fxData[i] = (uint8_t)(int8_t)sample;
    }
    pgFlashStep = 22U;
    if (!nor_erase4k(NOR_FX_ADDR)) return 0;
    pgFlashStep = 23U;
    if (!nor_write(NOR_FX_DATA, fxData, NOR_FX_LENGTH)) return 0;
    fx.magic = NOR_FX_MAGIC;
    fx.version = 1;
    fx.length = NOR_FX_LENGTH;
    fx.crc = PG_Crc32(fxData, NOR_FX_LENGTH);
    pgFlashStep = 24U;
    return nor_write(NOR_FX_ADDR, &fx, sizeof(fx));
}

int PG_FlashReadBest(PG_Save *out)
{
    if (!out) return nor_failed(8U);
    if (!nor_prepare_effects()) return 0;
    int found = 0;
    for (unsigned slot = 0; slot < 2; slot++) {
        uint32_t address = (slot == 0) ? NOR_SLOT_A : NOR_SLOT_B;
        pgFlashStep = (slot == 0U) ? 30U : 31U;
        /* Read only the fixed header first.  An erased/invalid slot is not
         * worth transferring ~9 KB, and should not be treated as an I/O error. */
        if (!nor_read(address, &recordScratch, offsetof(PG_Save, positions)))
            return 0;
        if (recordScratch.magic != NOR_SAVE_MAGIC ||
            recordScratch.version != NOR_SAVE_VERSION ||
            recordScratch.bestLength > PG_GHOST_MAX)
            continue;
        /* A plausible record needs its full positions array and CRC. */
        if (!nor_read(address, &recordScratch, sizeof(recordScratch))) return 0;
        if (recordScratch.crc32 != PG_Crc32(&recordScratch, offsetof(PG_Save, crc32)))
            continue;
        if (!found || (int32_t)(recordScratch.sequence - norLastSequence) > 0) {
            *out = recordScratch;
            norLastSequence = recordScratch.sequence;
            found = 1;
        }
    }
    /* Empty or uninitialized flash is valid; do not treat it as an IO fault. */
    if (!found) memset(out, 0, sizeof(*out));
    pgFlashStep = 32U;
    return 1;
}

int PG_FlashWriteBest(PG_Save *save)
{
    pgFlashStep = 50U;
    if (!save || sizeof(*save) > NOR_SLOT_BYTES || norMapped)
        return nor_failed(8U);
    save->magic = NOR_SAVE_MAGIC;
    save->version = NOR_SAVE_VERSION;
    save->sequence = norLastSequence + 1U;
    save->crc32 = PG_Crc32(save, offsetof(PG_Save, crc32));
    uint32_t address = (save->sequence & 1U) ? NOR_SLOT_A : NOR_SLOT_B;
    pgFlashStep = 51U;
    for (uint32_t i = 0; i < NOR_SLOT_BYTES; i += NOR_SECTOR)
        if (!nor_erase4k(address + i)) return 0;
    /* Header is programmed last: a power outage cannot make torn data valid. */
    const uint32_t afterFirst = sizeof(*save) - NOR_PAGE;
    pgFlashStep = 52U;
    if (!nor_write(address + NOR_PAGE, (const uint8_t *)save + NOR_PAGE, afterFirst))
        return 0;
    pgFlashStep = 53U;
    if (!nor_write(address, save, NOR_PAGE)) return 0;
    pgFlashStep = 54U;
    if (!nor_read(address, &recordScratch, sizeof(recordScratch))) return 0;
    if (recordScratch.crc32 != PG_Crc32(&recordScratch, offsetof(PG_Save, crc32)))
        return nor_failed(7U);
    norLastSequence = save->sequence;
    pgFlashStep = 55U;
    return 1;
}

int PG_FlashMap(void)
{
    if (norMapped) return 1;
    pgFlashStep = 40U;
    OSPI_RegularCmdTypeDef c = nor_command(0x0BU); /* FAST READ, 1-1-1 */
    c.OperationType = HAL_OSPI_OPTYPE_READ_CFG;
    c.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
    c.Address = 0;
    c.DataMode = HAL_OSPI_DATA_1_LINE;
    c.NbData = 1;
    c.DummyCycles = 8;
    OSPI_MemoryMappedTypeDef mm = {0};
    mm.TimeOutActivation = HAL_OSPI_TIMEOUT_COUNTER_DISABLE;
    if (HAL_OSPI_Command(&hospi1, &c, 100U) != HAL_OK)
        return nor_failed(1U);

    /* STM32L4 HAL requires both READ_CFG and WRITE_CFG before memory mapping.
     * A read-only application still has to configure the write command.  The
     * software never writes to the 0x90000000 memory-mapped address window;
     * NOR programming always uses indirect commands after PG_FlashUnmap(). */
    OSPI_RegularCmdTypeDef w = nor_command(0x02U); /* PAGE PROGRAM, 1-1-1 */
    w.OperationType = HAL_OSPI_OPTYPE_WRITE_CFG;
    w.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
    w.Address = 0U;
    w.DataMode = HAL_OSPI_DATA_1_LINE;
    w.NbData = 1U;
    if (HAL_OSPI_Command(&hospi1, &w, 100U) != HAL_OK)
        return nor_failed(1U);

    pgFlashStep = 41U;
    if (HAL_OSPI_MemoryMapped(&hospi1, &mm) != HAL_OK)
        return nor_failed(9U);
    norMapped = 1;
    pgFlashStep = 42U;
    return 1;
}

int PG_FlashUnmap(void)
{
    if (!norMapped) return 1;
    pgFlashStep = 60U;
    if (HAL_OSPI_Abort(&hospi1) != HAL_OK) return nor_failed(10U);
    norMapped = 0;
    return 1;
}

const int8_t *PG_FlashCrashSamples(void)
{
    if (!norMapped) return NULL;
    return (const int8_t *)(uintptr_t)(NOR_MEMORY_BASE + NOR_FX_DATA);
}

uint32_t PG_FlashCrashLength(void) { return NOR_FX_LENGTH; }

/* ======== AUDIO ======== */
static uint32_t enginePhase = 0U;
static uint32_t bassPhase = 0U;
static uint32_t randomState = 0xD357A63EU;
static uint32_t crashPosition = 0U;
volatile uint32_t pgEffectRemaining = 0U;

void PG_AudioTriggerCrash(void)
{
    crashPosition = 0;
    pgEffectRemaining = PG_FlashCrashLength();
}

void PG_AudioFill(uint16_t *dst, uint32_t length, uint16_t speed,
                  const int8_t *effect, uint32_t effectLength)
{
    uint32_t freq = 55U + ((uint32_t)speed * 5U);
    uint32_t increment = (uint32_t)(((uint64_t)freq << 24) / PG_AUDIO_RATE);
    uint32_t bassInc = increment / 2U;
    for (uint32_t i = 0; i < length; ++i) {
        enginePhase += increment;
        bassPhase += bassInc;
        int32_t v = 2048;
        if (speed > 0U) {
            /* Triangle and low-frequency pulse approximate a motor harmonic. */
            uint32_t p = (enginePhase >> 16) & 0xFFU;
            int32_t triangle = (p < 128U) ? ((int32_t)p * 2 - 128) :
                                              (383 - (int32_t)p * 2);
            int32_t low = (bassPhase & 0x00800000UL) ? 1 : -1;
            v += triangle * (2 + speed / 35U) + low * 80;
        }
        if (pgEffectRemaining) {
            int32_t noise;
            if (effect && crashPosition < effectLength) {
                noise = (int32_t)effect[crashPosition];
            } else {
                randomState ^= randomState << 13;
                randomState ^= randomState >> 17;
                randomState ^= randomState << 5;
                noise = (int32_t)(randomState & 0xFFU) - 128;
                noise = noise * (int32_t)pgEffectRemaining / (int32_t)PG_FlashCrashLength();
            }
            v += noise * 7;
            ++crashPosition;
            --pgEffectRemaining;
        }
        if (v < 0) v = 0;
        if (v > 4095) v = 4095;
        dst[i] = (uint16_t)v;
    }
}

/* ======== RTOS HELPERS / RENDERER / IRQ CALLBACKS ======== */
/* FreeRTOS run-time timer: DWT cycles extended to 64 bits and divided by 128.
 * Call on every context switch so the 32-bit counter cannot wrap unnoticed
 * (120 MHz DWT wraps after ~35.8 s). The returned 32-bit count wraps after
 * ~76 minutes, so a five-minute test is covered. */
static uint64_t pgRunCycles = 0;
static uint32_t pgLastCycle = 0;
void PG_RuntimeStatsInit(void)
{
#if defined(CoreDebug) && defined(DWT) && defined(CoreDebug_DEMCR_TRCENA_Msk)
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
#endif
    pgRunCycles = 0;
    pgLastCycle = 0;
}
uint32_t PG_RuntimeStatsCounter(void)
{
#if defined(DWT)
    uint32_t now = DWT->CYCCNT;
    pgRunCycles += (uint32_t)(now - pgLastCycle);
    pgLastCycle = now;
    return (uint32_t)(pgRunCycles >> 7);
#else
    return HAL_GetTick() * 1000U; /* compile-host fallback */
#endif
}

void PG_RuntimeUpdate(void)
{
#if defined(configGENERATE_RUN_TIME_STATS) && (configGENERATE_RUN_TIME_STATS == 1) && \
    defined(configUSE_TRACE_FACILITY) && (configUSE_TRACE_FACILITY == 1)
    TaskStatus_t stats[10];
    uint32_t total = 0;
    static uint32_t priorTotal = 0;
    static uint32_t prior[5] = {0};
    UBaseType_t n = uxTaskGetSystemState(stats, 10, &total);
    uint32_t dt = total - priorTotal;
    if (dt) {
        for (unsigned k = 0; k < 5; k++) {
            const char *labels[5] = {"audioTask", "sensorTask", "gameTask",
                                      "renderTask", "storageTask"};
            for (UBaseType_t j = 0; j < n; j++) {
                if (strcmp(stats[j].pcTaskName, labels[k]) == 0) {
                    uint32_t used = stats[j].ulRunTimeCounter - prior[k];
                    uint32_t percentage = (uint32_t)((uint64_t)used * 100U / dt);
                    pgCpuPercent[k] = (uint8_t)((percentage > 100U) ? 100U : percentage);
                    prior[k] = stats[j].ulRunTimeCounter;
                    break;
                }
            }
        }
    }
    priorTotal = total;
#else
    /* FreeRTOSConfig.h run-time statistics must be enabled for these values. */
#endif
}

static uint32_t pg_interval(uint32_t *remainder, uint32_t hz)
{
    uint32_t sum = *remainder + osKernelGetTickFreq();
    uint32_t result = sum / hz;
    *remainder = sum % hz;
    return result ? result : 1;
}



/* Called on *DMA IRQ* context, never busy wait in ISR. */
static void pg_audio_irq(uint8_t half)
{
    if (pgIsInitialized && pgAudioHalfQueue && pgAudioSemaphore &&
        osMessageQueuePut(pgAudioHalfQueue, &half, 0U, 0U) == osOK &&
        osSemaphoreRelease(pgAudioSemaphore) == osOK) return;
    ++pgAudioLate;
}

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *h)
{
    if (h->Instance == DAC1) pg_audio_irq(0);
}
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *h)
{
    if (h->Instance == DAC1) pg_audio_irq(1);
}
void HAL_DAC_DMAUnderrunCallbackCh1(DAC_HandleTypeDef *h)
{
    if (h->Instance == DAC1) ++pgAudioUnderruns;
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *h)
{
    if (h->Instance == USART1 && pgUartSemaphore)
        (void)osSemaphoreRelease(pgUartSemaphore);
}
void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
    if (pin == BUTTON_EXTI13_Pin && pgButtonSemaphore)
        (void)osSemaphoreRelease(pgButtonSemaphore);
}



typedef struct { double mean, m2; uint32_t count; } PG_Stats;
static void pg_stat(PG_Stats *s, float value, volatile float *variance)
{
    s->count++;
    double delta = (double)value - s->mean;
    s->mean += delta / (double)s->count;
    s->m2 += delta * ((double)value - s->mean);
    if (s->count > 1U) *variance = (float)(s->m2 / (double)(s->count - 1U));
}





static char pgScreen[PG_H][PG_W];
static char pgPrevious[PG_H][PG_W];
static char pgTx[4096]; /* static: DMA uses it until TxCpltCallback */
static void pg_write(unsigned row, unsigned col, const char *format, ...)
{
    if (row >= PG_H || col >= PG_W) return;
    char buffer[120];
    va_list ap;
    va_start(ap, format);
    (void)vsnprintf(buffer, sizeof(buffer), format, ap);
    va_end(ap);
    size_t length = strlen(buffer);
    if (length > PG_W - col) length = PG_W - col;
    memcpy(&pgScreen[row][col], buffer, length);
}

static void pg_draw(const PG_View *v)
{
    memset(pgScreen, ' ', sizeof(pgScreen));
    pg_write(0, 0, "ECSE444 - NIGHT HIGHWAY   |  USER: start / pause / replay");
    const char *status[] = {"MENU", "DRIVING", "PAUSED", "CRASH", "REPLAY"};
    PG_Mode mode = (v->mode <= PG_REPLAY) ? v->mode : PG_MENU;
    pg_write(1, 0, "MODE:%-8s SCORE:%-8lu SPEED:%-3u FRAME:%-7lu FLASH:%s",
             status[mode], (unsigned long)v->score, (unsigned)v->speed,
             (unsigned long)v->frame, v->flashOk ? "OK" : "OFF");
    /* Avoid newlib-nano's optional (and expensive) floating-point printf. */
    pg_write(2, 0, "ROLL:raw:%+4ld filt:%+4ld  PITCH:raw:%+4ld filt:%+4ld IMU:%s",
             (long)v->rawRoll, (long)v->filteredRoll,
             (long)v->rawPitch, (long)v->filteredPitch,
             v->sensorOk ? "OK" : "NO");
    for (unsigned y = 3; y <= 21; y++) {
        pgScreen[y][24] = '|';
        pgScreen[y][56] = '|';
        if (((y + v->frame / 2U) % 5U) < 3U) {
            pgScreen[y][35] = ':';
            pgScreen[y][46] = ':';
        }
        if (((y + v->frame) % 7U) == 0) {
            pgScreen[y][12] = '.';
            pgScreen[y][65] = '*';
        }
    }
    for (unsigned i = 0; i < PG_TRAFFIC_MAX; i++) {
        /* Don't leave stationary traffic on the screen after replay exits. */
        if (v->mode == PG_MENU) break;
        PG_Car c = v->traffic[i];
        if (c.y >= 3 && c.y <= 21 && c.x >= 26 && c.x <= 54) {
            pgScreen[(unsigned)c.y][(unsigned)c.x] = c.kind == 1 ? 'V' : 'X';
            if (c.x > 26) pgScreen[(unsigned)c.y][(unsigned)c.x - 1] = '[';
            if (c.x < 54) pgScreen[(unsigned)c.y][(unsigned)c.x + 1] = ']';
        }
    }
    if (v->ghostX >= 26 && v->ghostX <= 54 &&
        (v->mode == PG_REPLAY || v->mode == PG_RUNNING))
        pgScreen[20][(unsigned)v->ghostX] = 'g';
    if ((v->mode == PG_RUNNING || v->mode == PG_PAUSED || v->mode == PG_REPLAY) &&
        v->playerX >= 26 && v->playerX <= 54) {
        pgScreen[20][(unsigned)v->playerX] = 'A';
    }
    pg_write(3, 1, "TOP 5");
    for (unsigned i = 0; i < PG_SCORE_COUNT; i++)
        pg_write(5 + i, 1, "%u: %-8lu", i + 1, (unsigned long)v->scores[i]);
    pg_write(12, 1, "FPS:%2lu.%02lu", (unsigned long)(pgAvgFpsX100 / 100U),
             (unsigned long)(pgAvgFpsX100 % 100U));
    pg_write(14, 1, "Audio late:%lu", (unsigned long)pgAudioLate);
    pg_write(15, 1, "CPU:%u/%u/%u/%u/%u", (unsigned)pgCpuPercent[0],
             (unsigned)pgCpuPercent[1], (unsigned)pgCpuPercent[2],
             (unsigned)pgCpuPercent[3], (unsigned)pgCpuPercent[4]);
    pg_write(16, 1, "Latency:%lums", (unsigned long)pgTiltToScreenMs);
    pg_write(17, 1, "Ghost:%lu", (unsigned long)v->ghostFrameCount);
    pg_write(18, 1, "Mismatch:%lu", (unsigned long)v->replayMismatch);
    pg_write(19, 1, "QSPI ID:%02X %02X %02X", (unsigned)pgFlashId[0],
             (unsigned)pgFlashId[1], (unsigned)pgFlashId[2]);
    pg_write(20, 1, "FStep:%lu Fail:%lu/%lu", (unsigned long)pgFlashStep,
             (unsigned long)pgFlashFailStep, (unsigned long)pgFlashFailOp);
    /* Last attempted flash address makes a failed read easy to localize. */
    pg_write(21, 1, "H:%lX S:%02X A:%06lX", (unsigned long)pgFlashHalError,
             (unsigned)pgFlashStatus, (unsigned long)pgFlashAddress);
    pg_write(22, 0, "Tilt the board to steer and accelerate. Avoid oncoming traffic!");
    if (mode == PG_MENU) pg_write(23, 0, "Press USER to start a new run.");
    else if (mode == PG_PAUSED) pg_write(23, 0, "PAUSED - press USER to resume.");
    else if (mode == PG_GAME_OVER) pg_write(23, 0,
                  "CRASH - press USER for best-run ghost replay (or new run).");
    else if (mode == PG_REPLAY) pg_write(23, 0, "REPLAY - press USER to return to menu.");
    else pg_write(23, 0, "Keep your car away from [X] and [V].");
}

static size_t pg_append(size_t offset, const char *str, size_t n)
{
    if (n > sizeof(pgTx) - offset) return SIZE_MAX;
    memcpy(pgTx + offset, str, n);
    return offset + n;
}
static size_t pg_cursor(size_t offset, unsigned row, unsigned col)
{
    char tmp[20];
    int n = snprintf(tmp, sizeof(tmp), "\x1b[%u;%uH", row+1, col+1);
    return n > 0 ? pg_append(offset, tmp, (size_t)n) : SIZE_MAX;
}
static size_t pg_fullscreen(void)
{
    size_t off = pg_append(0, "\x1b[2J\x1b[?25l", sizeof("\x1b[2J\x1b[?25l") - 1);
    for (unsigned y = 0; y < PG_H && off != SIZE_MAX; y++) {
        off = pg_cursor(off, y, 0);
        if (off != SIZE_MAX) off = pg_append(off, pgScreen[y], PG_W);
    }
    return off;
}
static size_t pg_deltas(void)
{
    size_t off = 0;
    for (unsigned y = 0; y < PG_H; ++y) {
        for (unsigned x = 0; x < PG_W;) {
            if (pgScreen[y][x] == pgPrevious[y][x]) { x++; continue; }
            unsigned begin = x;
            while (x < PG_W && pgScreen[y][x] != pgPrevious[y][x]) x++;
            off = pg_cursor(off, y, begin);
            if (off == SIZE_MAX) return SIZE_MAX;
            off = pg_append(off, &pgScreen[y][begin], x - begin);
            if (off == SIZE_MAX) return SIZE_MAX;
        }
    }
    return off;
}
static int pg_uart_dma(size_t count)
{
    if (!count) return 1;
    if (huart1.hdmatx == NULL) {
        ++pgUartErrors; /* Fix HAL_UART_MspInit DMA linkage in CubeMX. */
        return 0;
    }
    if (count > UINT16_MAX ||
        HAL_UART_Transmit_DMA(&huart1, (uint8_t *)pgTx, (uint16_t)count) != HAL_OK) {
        ++pgUartErrors; return 0;
    }
    if (osSemaphoreAcquire(pgUartSemaphore, 100U) != osOK) {
        ++pgUartErrors;
        (void)HAL_UART_AbortTransmit(&huart1);
        return 0;
    }
    return 1;
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

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_DFSDM1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_OCTOSPI1_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_USB_Init();
  MX_DAC1_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  /* LSM6DSL: initial configuration BEFORE starting FreeRTOS.
   * I2C2: 7-bit address 0x6A (fallback 0x6B).
   * CTRL3_C=0x44: block-data-update + auto-increment.
   * CTRL1_XL=0x40: accelerometer 104 Hz, +/-2 g.
   * CTRL2_G =0x40: gyro 104 Hz, 245 dps (optional game telemetry).
   */
  {
    uint8_t who = 0U;
    uint8_t control = 0x44U;
    imuAddr = 0U;
    for (uint32_t address = 0x6AU; address <= 0x6BU; address++) {
      if (HAL_I2C_Mem_Read(&hi2c2, (uint16_t)(address << 1), 0x0FU,
                           I2C_MEMADD_SIZE_8BIT, &who, 1U, 100U) == HAL_OK &&
          who == 0x6AU) {
        imuAddr = (uint16_t)(address << 1);
        break;
      }
    }
    if (imuAddr != 0U &&
        HAL_I2C_Mem_Write(&hi2c2, imuAddr, 0x12U,
                          I2C_MEMADD_SIZE_8BIT, &control, 1U, 100U) == HAL_OK) {
      control = 0x40U;
      if (HAL_I2C_Mem_Write(&hi2c2, imuAddr, 0x10U,
                            I2C_MEMADD_SIZE_8BIT, &control, 1U, 100U) == HAL_OK &&
          HAL_I2C_Mem_Write(&hi2c2, imuAddr, 0x11U,
                            I2C_MEMADD_SIZE_8BIT, &control, 1U, 100U) == HAL_OK) {
        pgImuStartupOk = 1U;
      }
    }
    if (!pgImuStartupOk) ++pgSensorErrors;
  }
/* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* All queues, semaphores and mutexes created before task scheduling. */
    pgTiltQueue = osMessageQueueNew(4U, sizeof(PG_Tilt), NULL);
    pgSoundQueue = osMessageQueueNew(8U, sizeof(uint8_t), NULL);
    pgAudioHalfQueue = osMessageQueueNew(4U, sizeof(uint8_t), NULL);
    pgSaveQueue = osMessageQueueNew(2U, sizeof(uint8_t), NULL);
    pgAudioSemaphore = osSemaphoreNew(4U, 0U, NULL);
    pgUartSemaphore = osSemaphoreNew(1U, 0U, NULL);
    pgButtonSemaphore = osSemaphoreNew(1U, 0U, NULL);
    pgViewMutex = osMutexNew(NULL);
    pgFlashMutex = osMutexNew(NULL);
    pgSaveMutex = osMutexNew(NULL);
    if (!pgTiltQueue || !pgSoundQueue || !pgAudioHalfQueue || !pgSaveQueue ||
        !pgAudioSemaphore || !pgUartSemaphore || !pgButtonSemaphore ||
        !pgViewMutex || !pgFlashMutex || !pgSaveMutex) Error_Handler();
    PG_GameReset(&pgView, 0x42UL, PG_MENU);
    memset(&pgCurrentSave, 0, sizeof(pgCurrentSave));
    pgIsInitialized = 1;
/* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of audioTask */
  audioTaskHandle = osThreadNew(audioTask1, NULL, &audioTask_attributes);

  /* creation of sensorTask */
  sensorTaskHandle = osThreadNew(sensorTask1, NULL, &sensorTask_attributes);

  /* creation of gameTask */
  gameTaskHandle = osThreadNew(gameTask1, NULL, &gameTask_attributes);

  /* creation of renderTask */
  renderTaskHandle = osThreadNew(renderTask1, NULL, &renderTask_attributes);

  /* creation of storageTask */
  storageTaskHandle = osThreadNew(storageTask1, NULL, &storageTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK|RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T7_TRGO;
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_ABOVE_80MHZ;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_channel2.Instance = DFSDM1_Channel2;
  hdfsdm1_channel2.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel2.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel2.Init.OutputClock.Divider = 2;
  hdfsdm1_channel2.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel2.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel2.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel2.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel2.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel2.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel2.Init.Awd.Oversampling = 1;
  hdfsdm1_channel2.Init.Offset = 0;
  hdfsdm1_channel2.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x30A175AB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x30A175AB;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief OCTOSPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OCTOSPI1_Init(void)
{

  /* USER CODE BEGIN OCTOSPI1_Init 0 */

  /* USER CODE END OCTOSPI1_Init 0 */

  OSPIM_CfgTypeDef OSPIM_Cfg_Struct = {0};

  /* USER CODE BEGIN OCTOSPI1_Init 1 */

  /* USER CODE END OCTOSPI1_Init 1 */
  /* OCTOSPI1 parameter configuration*/
  hospi1.Instance = OCTOSPI1;
  hospi1.Init.FifoThreshold = 1;
  hospi1.Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE;
  hospi1.Init.MemoryType = HAL_OSPI_MEMTYPE_MACRONIX;
  /* MX25R6435F is 64 Mbit (8 MiB), requiring 23 address bits.
     The STM32 HAL subtracts one when programming OCTOSPI_DCR1.DEVSIZE. */
  hospi1.Init.DeviceSize = 23;
  hospi1.Init.ChipSelectHighTime = 1;
  hospi1.Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE;
  hospi1.Init.ClockMode = HAL_OSPI_CLOCK_MODE_0;
  hospi1.Init.ClockPrescaler = 5;
  hospi1.Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_NONE;
  hospi1.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_DISABLE;
  hospi1.Init.ChipSelectBoundary = 0;
  hospi1.Init.DelayBlockBypass = HAL_OSPI_DELAY_BLOCK_BYPASSED;
  if (HAL_OSPI_Init(&hospi1) != HAL_OK)
  {
    Error_Handler();
  }
  OSPIM_Cfg_Struct.ClkPort = 1;
  OSPIM_Cfg_Struct.NCSPort = 1;
  OSPIM_Cfg_Struct.IOLowPort = HAL_OSPIM_IOPORT_1_LOW;
  if (HAL_OSPIM_Config(&hospi1, &OSPIM_Cfg_Struct, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OCTOSPI1_Init 2 */

  /* USER CODE END OCTOSPI1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 7499;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 921600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_USB_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, ST25DV04K_RF_DISABLE_Pin|ISM43362_RST_Pin|ISM43362_SPI3_CSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ARD_D10_Pin|ARD_D4_Pin|SPBTLE_RF_RST_Pin|ARD_D9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin
                          |SPSGRF_915_SDN_Pin|ARD_D5_Pin|SPSGRF_915_SPI3_CSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, SPBTLE_RF_SPI3_CSN_Pin|PMOD_RESET_Pin|PMOD_SPI2_SCK_Pin|STSAFE_A110_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ST25DV04K_RF_DISABLE_Pin ISM43362_RST_Pin ISM43362_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = ST25DV04K_RF_DISABLE_Pin|ISM43362_RST_Pin|ISM43362_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_OVRCR_EXTI3_Pin ST25DV04K_GPO_Pin SPSGRF_915_GPIO3_EXTI5_Pin SPBTLE_RF_IRQ_EXTI6_Pin
                           ISM43362_DRDY_EXTI1_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_OVRCR_EXTI3_Pin|ST25DV04K_GPO_Pin|SPSGRF_915_GPIO3_EXTI5_Pin|SPBTLE_RF_IRQ_EXTI6_Pin
                          |ISM43362_DRDY_EXTI1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTTON_EXTI13_Pin VL53L0X_GPIO1_EXTI7_Pin LSM3MDL_DRDY_EXTI8_Pin */
  GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin|VL53L0X_GPIO1_EXTI7_Pin|LSM3MDL_DRDY_EXTI8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D10_Pin ARD_D4_Pin SPBTLE_RF_RST_Pin ARD_D9_Pin */
  GPIO_InitStruct.Pin = ARD_D10_Pin|ARD_D4_Pin|SPBTLE_RF_RST_Pin|ARD_D9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D3_Pin */
  GPIO_InitStruct.Pin = ARD_D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D6_Pin */
  GPIO_InitStruct.Pin = ARD_D6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(ARD_D6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D8_Pin ISM43362_BOOT0_Pin ISM43362_WAKEUP_Pin LED2_Pin
                           SPSGRF_915_SDN_Pin ARD_D5_Pin SPSGRF_915_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin
                          |SPSGRF_915_SDN_Pin|ARD_D5_Pin|SPSGRF_915_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LPS22HB_INT_DRDY_EXTI10_Pin LSM6DSL_INT1_EXTI11_Pin USB_OTG_FS_PWR_EN_Pin ARD_D2_Pin
                           HTS221_DRDY_EXTI15_Pin PMOD_IRQ_EXTI2_Pin */
  GPIO_InitStruct.Pin = LPS22HB_INT_DRDY_EXTI10_Pin|LSM6DSL_INT1_EXTI11_Pin|USB_OTG_FS_PWR_EN_Pin|ARD_D2_Pin
                          |HTS221_DRDY_EXTI15_Pin|PMOD_IRQ_EXTI2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : SPBTLE_RF_SPI3_CSN_Pin PMOD_RESET_Pin PMOD_SPI2_SCK_Pin STSAFE_A110_RESET_Pin */
  GPIO_InitStruct.Pin = SPBTLE_RF_SPI3_CSN_Pin|PMOD_RESET_Pin|PMOD_SPI2_SCK_Pin|STSAFE_A110_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L0X_XSHUT_Pin LED3_WIFI__LED4_BLE_Pin */
  GPIO_InitStruct.Pin = VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OTG_FS_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OTG_FS_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_ID_Pin USB_OTG_FS_DM_Pin USB_OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_ID_Pin|USB_OTG_FS_DM_Pin|USB_OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
	for (;;)
	{
	    debugFreeHeap = xPortGetFreeHeapSize();
	    debugMinFreeHeap = xPortGetMinimumEverFreeHeapSize();
        PG_RuntimeUpdate();

	    osDelay(1000);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_audioTask1 */
/**
* @brief Function implementing the audioTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_audioTask1 */
void audioTask1(void *argument)
{
  /* USER CODE BEGIN audioTask1 */
    (void)argument;
    PG_AudioFill(pgAudioBuffer, PG_AUDIO_SIZE, 0, NULL, 0);
    if (hdac1.DMA_Handle1 == NULL) {
        pgAudioStartError = 3; /* Fix HAL_DAC_MspInit DMA linkage in CubeMX. */
        for (;;) osDelay(1000);
    }
    if (HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *)pgAudioBuffer,
                          PG_AUDIO_SIZE, DAC_ALIGN_12B_R) != HAL_OK) {
        pgAudioStartError = 1;
        for (;;) osDelay(1000);
    }
    if (HAL_TIM_Base_Start(&htim7) != HAL_OK) {
        pgAudioStartError = 2;
        HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
        for (;;) osDelay(1000);
    }
    for (;;) {
        if (osSemaphoreAcquire(pgAudioSemaphore, osWaitForever) != osOK) continue;
        uint8_t half;
        if (osMessageQueueGet(pgAudioHalfQueue, &half, NULL, 0U) != osOK) {
            ++pgAudioLate;
            continue;
        }
        PG_TRACE_BEGIN(Audio);
        uint8_t event;
        while (osMessageQueueGet(pgSoundQueue, &event, NULL, 0U) == osOK)
            if (event == PG_FX_CRASH) PG_AudioTriggerCrash();
        /* Never wait behind an erase: sacrifice one effect segment if busy. */
        const int8_t *fx = NULL;
        uint32_t fxLen = 0;
        int locked = (osMutexAcquire(pgFlashMutex, 0U) == osOK);
        if (locked && pgFlashMapped) {
            fx = PG_FlashCrashSamples();
            fxLen = PG_FlashCrashLength();
        }
        PG_AudioFill(&pgAudioBuffer[(uint32_t)half * PG_AUDIO_HALF],
                     PG_AUDIO_HALF, pgAudioSpeed, fx, fxLen);
        if (locked) (void)osMutexRelease(pgFlashMutex);
        ++pgAudioRefills;
        if (osMessageQueueGetCount(pgAudioHalfQueue) != 0U) ++pgAudioLate;
        PG_TRACE_END(Audio);
    }
/* USER CODE END audioTask1 */
}

/* USER CODE BEGIN Header_sensorTask1 */
/**
* @brief Function implementing the sensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_sensorTask1 */
void sensorTask1(void *argument)
{
  /* USER CODE BEGIN sensorTask1 */
    (void)argument;
    float rollZero = 0.0f, pitchZero = 0.0f;
    float rollState = 0.0f, pitchState = 0.0f;
    float rollP = 1.0f, pitchP = 1.0f;
    uint32_t next = osKernelGetTickCount(), fraction = 0;
    int ready = (int)pgImuStartupOk; /* Initialized in main() */
    if (ready) {
        uint32_t n = 0;
        for (uint32_t i = 0; i < PG_SENSOR_CALIBRATION_SAMPLES; i++) {
            float r, p;
            if (PG_IMU_Read(&r, &p)) { rollZero += r; pitchZero += p; n++; }
            osDelay(10);
        }
        if (n) { rollZero /= n; pitchZero /= n; }
        else { ready = 0; ++pgSensorErrors; }
    }
    PG_Stats statsRR = {0}, statsRF = {0}, statsPR = {0}, statsPF = {0};
    for (;;) {
        PG_TRACE_BEGIN(Sensor);
        if (!ready) {
            ready = PG_IMU_Init();
            if (!ready) { ++pgSensorErrors; osDelay(1000); }
            else {
                rollState = pitchState = 0;
                rollP = pitchP = 1;
                rollZero = pitchZero = 0;
            }
        }
        if (ready) {
            PG_Tilt t = {0};
            float r, p;
            if (PG_IMU_Read(&r, &p)) {
                t.valid = 1;
                t.tick = osKernelGetTickCount();
                t.rawRoll = r - rollZero;
                t.rawPitch = p - pitchZero;
                t.roll = PG_KalmanUpdate(t.rawRoll, &rollState, &rollP,
                                         PG_KALMAN_Q, PG_KALMAN_R);
                t.pitch = PG_KalmanUpdate(t.rawPitch, &pitchState, &pitchP,
                                          PG_KALMAN_Q, PG_KALMAN_R);
                pgTiltLog[pgTiltLogHead & 511U] = t;
                pgTiltLogHead++;
                pg_stat(&statsRR, t.rawRoll, &pgRollRawVariance);
                pg_stat(&statsRF, t.roll, &pgRollFilteredVariance);
                pg_stat(&statsPR, t.rawPitch, &pgPitchRawVariance);
                pg_stat(&statsPF, t.pitch, &pgPitchFilteredVariance);
                if (osMessageQueuePut(pgTiltQueue, &t, 0, 0) != osOK) {
                    PG_Tilt old;
                    (void)osMessageQueueGet(pgTiltQueue, &old, NULL, 0);
                    (void)osMessageQueuePut(pgTiltQueue, &t, 0, 0);
                }
                ++pgSensorReadings;
            } else { ++pgSensorErrors; ready = 0; }
        }
        PG_TRACE_END(Sensor);
        next += pg_interval(&fraction, PG_TICK_SENSOR_HZ);
        if ((int32_t)(osKernelGetTickCount() - next) > 0)
            next = osKernelGetTickCount();
        (void)osDelayUntil(next);
    }
/* USER CODE END sensorTask1 */
}

/* USER CODE BEGIN Header_gameTask1 */
/**
* @brief Function implementing the gameTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_gameTask1 */
void gameTask1(void *argument)
{
  /* USER CODE BEGIN gameTask1 */
    (void)argument;
    PG_Tilt tilt = {0};
    uint32_t next = osKernelGetTickCount(), fraction = 0, buttonAt = 0;
    uint32_t ledOffAt = 0;
    PG_Mode previousMode = PG_MENU;
    for (;;) {
        PG_TRACE_BEGIN(Game);
        PG_Tilt in;
        while (osMessageQueueGet(pgTiltQueue, &in, NULL, 0) == osOK) tilt = in;
        uint32_t now = osKernelGetTickCount();
        int clicked = osSemaphoreAcquire(pgButtonSemaphore, 0) == osOK &&
                      (uint32_t)(now - buttonAt) >= osKernelGetTickFreq()/5U;
        if (clicked) buttonAt = now;
        if (osMutexAcquire(pgViewMutex, osWaitForever) == osOK) {
            if (clicked) {
                if (pgView.mode == PG_MENU) {
                    PG_GameReset(&pgView, now ^ 0x91B8F203U, PG_RUNNING);
                    pgCurrentCount = 0; pgCurrentHash = 2166136261UL;
                } else if (pgView.mode == PG_RUNNING) pgView.mode = PG_PAUSED;
                else if (pgView.mode == PG_PAUSED) pgView.mode = PG_RUNNING;
                else if (pgView.mode == PG_GAME_OVER) {
                    if (pgCurrentSave.bestLength > 0) {
                        PG_GameReset(&pgView, pgCurrentSave.bestSeed, PG_REPLAY);
                        pgCurrentHash = 2166136261UL;
                    } else {
                        PG_GameReset(&pgView, now ^ 0x81D092B3U, PG_RUNNING);
                        pgCurrentCount = 0; pgCurrentHash = 2166136261UL;
                    }
                } else if (pgView.mode == PG_REPLAY) pgView.mode = PG_MENU;
            }
            /* Publish live tilt in every game mode, including MENU/PAUSED.
             * Previously only PG_GameStep refreshed these fields, so MENU
             * displayed stale angles and an ever-increasing latency. */
            pgView.sensorTick = tilt.tick;
            pgView.sensorOk = tilt.valid;
            pgView.rawRoll = tilt.rawRoll;
            pgView.filteredRoll = tilt.roll;
            pgView.rawPitch = tilt.rawPitch;
            pgView.filteredPitch = tilt.pitch;
            previousMode = pgView.mode;
            if (previousMode == PG_RUNNING || previousMode == PG_REPLAY) {
                PG_GameStep(&pgView, &tilt, pgCurrentSave.positions,
                    pgCurrentSave.bestLength, pgCurrentRun, &pgCurrentCount,
                    &pgCurrentHash);
                /* Overlay the recorded best-run car during ordinary play. */
                if (previousMode == PG_RUNNING && pgView.frame != 0U &&
                    pgView.frame <= pgCurrentSave.bestLength)
                    pgView.ghostX = pgCurrentSave.positions[pgView.frame - 1U];
                else if (previousMode == PG_RUNNING)
                    pgView.ghostX = -1;
            }
            if (previousMode == PG_RUNNING && pgView.mode == PG_GAME_OVER) {
                uint8_t crash = PG_FX_CRASH;
                (void)osMessageQueuePut(pgSoundQueue, &crash, 0, 0);
                HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
                ledOffAt = now + osKernelGetTickFreq()/3U;
                (void)osMutexAcquire(pgSaveMutex, osWaitForever);
                uint32_t oldTop = pgCurrentSave.scores[0];
                PG_InsertScore(pgCurrentSave.scores, pgView.score);
                if (pgView.score > oldTop || pgCurrentSave.bestLength == 0U) {
                    pgCurrentSave.bestSeed = pgView.seed;
                    pgCurrentSave.bestLength = pgCurrentCount;
                    pgCurrentSave.bestRunHash = pgCurrentHash;
                    memcpy(pgCurrentSave.positions, pgCurrentRun, pgCurrentCount);
                }
                for (unsigned i = 0; i < PG_SCORE_COUNT; ++i)
                    pgView.scores[i] = pgCurrentSave.scores[i];
                pgView.ghostAvailable = (pgCurrentSave.bestLength > 0);
                pgView.ghostFrameCount = pgCurrentSave.bestLength;
                (void)osMutexRelease(pgSaveMutex);
                uint8_t request = 1;
                (void)osMessageQueuePut(pgSaveQueue, &request, 0, 0);
            }
            if (previousMode == PG_REPLAY && pgView.mode == PG_MENU) {
                if (pgCurrentHash != pgCurrentSave.bestRunHash) ++pgReplayMismatch;
                pgView.replayMismatch = pgReplayMismatch;
            }
            if (ledOffAt && (int32_t)(now - ledOffAt) >= 0) {
                HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
                ledOffAt = 0;
            }
            pgView.flashOk = pgFlashReady;
            pgAudioSpeed = (pgView.mode == PG_RUNNING || pgView.mode == PG_REPLAY)
                         ? pgView.speed : 0;
            ++pgGameFrames;
            (void)osMutexRelease(pgViewMutex);
        }
        PG_TRACE_END(Game);
        next += pg_interval(&fraction, PG_TICK_GAME_HZ);
        if ((int32_t)(osKernelGetTickCount() - next) > 0)
            next = osKernelGetTickCount();
        (void)osDelayUntil(next);
    }
/* USER CODE END gameTask1 */
}

/* USER CODE BEGIN Header_renderTask1 */
/**
* @brief Function implementing the renderTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_renderTask1 */
void renderTask1(void *argument)
{
  /* USER CODE BEGIN renderTask1 */
    (void)argument;
    memset(pgPrevious, 0, sizeof(pgPrevious));
    uint32_t next = osKernelGetTickCount(), fraction = 0;
    uint32_t fpsStart = next, framesSince = 0;
    for (;;) {
        PG_TRACE_BEGIN(Render);
        PG_View snapshot;
        if (osMutexAcquire(pgViewMutex, 0U) == osOK) {
            snapshot = pgView;
            (void)osMutexRelease(pgViewMutex);
            pg_draw(&snapshot);
            size_t bytes = pg_deltas();
            if (bytes == SIZE_MAX || pgRenderFrames == 0U)
                bytes = pg_fullscreen();
            if (bytes != SIZE_MAX && pg_uart_dma(bytes)) {
                memcpy(pgPrevious, pgScreen, sizeof(pgScreen));
                ++pgRenderFrames;
                ++framesSince;
                if (snapshot.sensorOk) {
                    uint32_t ticks = osKernelGetTickCount() - snapshot.sensorTick;
                    pgTiltToScreenMs = (uint32_t)((uint64_t)ticks * 1000U /
                                                  osKernelGetTickFreq());
                }
            } else memset(pgPrevious, 0, sizeof(pgPrevious));
        }
        uint32_t now = osKernelGetTickCount();
        uint32_t span = now - fpsStart;
        if (span >= osKernelGetTickFreq()) {
            pgAvgFpsX100 = (uint32_t)((uint64_t)framesSince *
                     osKernelGetTickFreq() * 100U / span);
            framesSince = 0; fpsStart = now;
        }
        PG_TRACE_END(Render);
        next += pg_interval(&fraction, PG_TICK_RENDER_HZ);
        if ((int32_t)(osKernelGetTickCount() - next) > 0)
            next = osKernelGetTickCount();
        (void)osDelayUntil(next);
    }
/* USER CODE END renderTask1 */
}

/* USER CODE BEGIN Header_storageTask1 */
/**
* @brief Function implementing the storageTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_storageTask1 */
void storageTask1(void *argument)
{
  /* USER CODE BEGIN storageTask1 */
    (void)argument;
    PG_TRACE_BEGIN(Storage);
    int flashOk = PG_FlashInit();
    if (flashOk) {
        if (PG_FlashReadBest(&pgStorageSnapshot)) {
            if (osMutexAcquire(pgSaveMutex, osWaitForever) == osOK) {
                pgCurrentSave = pgStorageSnapshot;
                (void)osMutexRelease(pgSaveMutex);
            }
            if (osMutexAcquire(pgViewMutex, osWaitForever) == osOK) {
                for (unsigned i = 0; i < PG_SCORE_COUNT; i++)
                    pgView.scores[i] = pgStorageSnapshot.scores[i];
                pgView.ghostAvailable = pgStorageSnapshot.bestLength > 0U;
                pgView.ghostFrameCount = pgStorageSnapshot.bestLength;
                (void)osMutexRelease(pgViewMutex);
            }
        } else { flashOk = 0; ++pgFlashErrors; }
        if (flashOk && !PG_FlashMap()) { flashOk = 0; ++pgFlashErrors; }
        else if (flashOk) pgFlashMapped = 1;
    } else ++pgFlashErrors;
    pgFlashReady = flashOk ? 1 : 0;
    PG_TRACE_END(Storage);
    for (;;) {
        uint8_t request;
        if (osMessageQueueGet(pgSaveQueue, &request, NULL, osWaitForever) != osOK)
            continue;
        if (request != 1 || !pgFlashReady) continue;
        /* Collision SFX lasts 128 ms. Allow time for AudioTask to see event. */
        osDelay(50);
        uint32_t started = osKernelGetTickCount();
        while (pgEffectRemaining &&
               osKernelGetTickCount() - started < 2U * osKernelGetTickFreq())
            osDelay(10);
        PG_TRACE_BEGIN(Storage);
        if (osMutexAcquire(pgSaveMutex, osWaitForever) == osOK) {
            pgStorageSnapshot = pgCurrentSave;
            (void)osMutexRelease(pgSaveMutex);
        }
        /* Serialize flash mode transitions and protect against AudioTask reads. */
        if (osMutexAcquire(pgFlashMutex, osWaitForever) == osOK) {
            if (!PG_FlashUnmap()) {
                ++pgFlashErrors;
            } else {
                pgFlashMapped = 0;
                if (PG_FlashWriteBest(&pgStorageSnapshot)) ++pgSavedGames;
                else ++pgFlashErrors;
                if (PG_FlashMap()) pgFlashMapped = 1;
                else { pgFlashReady = 0; ++pgFlashErrors; }
            }
            (void)osMutexRelease(pgFlashMutex);
        }
        PG_TRACE_END(Storage);
    }
/* USER CODE END storageTask1 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
#ifdef USE_FULL_ASSERT
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
