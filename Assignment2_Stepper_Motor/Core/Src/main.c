/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
   	PROJECT: 	Laboratory - ISA template
	Author:		Prof. Dr. Jannis Kreß
	ECU:		STM32 NUCLEO-G431KB
	Version:	1

    Info: After initialization of Peripherals the core is set to sleep mode.
	HAL_ADC_ConvCpltCallback is called when new data is available to be processed.
	Processing of Sensor-Values will be done in this Handler (no active main loop)
  */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"		// for use of standard math functions
#include "stdlib.h"		// for use of abs() function
#include "stdbool.h"	// for use of true/false states
#include "string.h"
#include "stdio.h"
#include "SEGGER_RTT.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

enum StepperMode {
	MODE_IDLE = 0,
	MODE_VELOCITY,
	MODE_POSITION
};

struct StepperState {
	enum StepperMode mode;
	int32_t current_pos;        // microsteps from calibrated zero
	float motor_angle_deg;      // derived from current_pos
	float amr_angle_deg;        // from AMR sensor
	float target_angle_deg;     // position mode target
	int32_t steps_remaining;    // position mode: steps left to go
	int32_t steps_total;        // position mode: total steps for this move (for ramp calc)
	int16_t velocity_pct;       // velocity %*100, range +-10000
	uint16_t current_arr;       // current timer ARR value
	bool enabled;               // motor driver enabled
	bool calibrated;            // zero calibration done
	int8_t direction;           // +1 or -1
};

/* UART RX ring buffer */
#define UART_RX_BUF_SIZE 128
struct UartRxBuf {
	uint8_t data[UART_RX_BUF_SIZE];
	volatile uint16_t head;
	uint16_t tail;
};

/* UART TX buffer */
#define UART_TX_BUF_SIZE 256

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define RAD_TO_DEG (180.0f / M_PI)

/* Stepper motor constants */
#define STEP_ANGLE_DEG       1.8f
#define MICROSTEP_FACTOR     64
#define STEPS_PER_REV        (uint32_t)((360.0f / STEP_ANGLE_DEG) * MICROSTEP_FACTOR)  // 12800
#define DEG_PER_MICROSTEP    (360.0f / STEPS_PER_REV)

#define F_TIMER              650000UL   // Timer 17 counter frequency after prescaler
#define ARR_DEFAULT          299
#define ARR_MIN              10         // max speed clamp
#define ARR_MAX              65535      // min speed (16-bit limit)
#define MAX_SPEED_RPS        1.5f       // motor spec limit

/* Position mode ramp */
#define RAMP_MIN_STEPS       100        // minimum steps before starting decel
#define RAMP_START_ARR       3000       // slow start ARR
#define RAMP_CRUISE_ARR      299        // full speed ARR
#define RAMP_ACCEL_STEPS     200        // steps over which to accelerate

/* Valid angle range */
#define ANGLE_MIN_DEG        0.0f
#define ANGLE_MAX_DEG        180.0f

/* UART protocol */
#define CMD_HEADER           "|SuA|"
#define CMD_HEADER_LEN       5
#define CMD_MODE_INDEX       CMD_HEADER_LEN
#define CMD_ARG_START_INDEX  (CMD_HEADER_LEN + 2)

/* Telemetry rate: send every N ADC callbacks (~500 Hz ADC → 25 Hz telemetry at divisor 20) */
#define TELEMETRY_DIVISOR    20

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/*VARIABLES******************************************************************/
struct ADC_Values {
	uint16_t SIN1;
	uint16_t COS1;
	uint16_t VDIAG1;
};
struct AMR_MinMax {
	uint16_t sin_min;
	uint16_t sin_max;
	uint16_t cos_min;
	uint16_t cos_max;
};
struct AMR_CalibParams {
	float sin_offset;
	float sin_amplitude;
	float cos_offset;
	float cos_amplitude;
};

struct AMR_Normalized {
	float sin;
	float cos;
};
struct XY_Values {
	float Y1;
	float X1;
};

/* Global variables************************************************************/
uint16_t adcVal[8];
struct ADC_Values ADCresult;
struct XY_Values XYresult;

struct AMR_MinMax amr_minmax = {
  .sin_min = UINT16_MAX,
  .sin_max = 0,
  .cos_min = UINT16_MAX,
  .cos_max = 0
};

struct AMR_CalibParams amr_calib;
struct AMR_Normalized amr_normalized;

float amr_angle_deg_raw = 0.0f;
const float amr_angle_deg_offset = 79.6f + 8.1f;
float amr_angle_deg = 0.0f;

/* Stepper state — visible in debugger */
volatile struct StepperState stepper = {
	.mode = MODE_IDLE,
	.current_pos = 0,
	.motor_angle_deg = 0.0f,
	.amr_angle_deg = 0.0f,
	.target_angle_deg = 0.0f,
	.steps_remaining = 0,
	.steps_total = 0,
	.velocity_pct = 0,
	.current_arr = ARR_DEFAULT,
	.enabled = false,
	.calibrated = false,
	.direction = 1
};

/* UART */
struct UartRxBuf uart_rx = { .head = 0, .tail = 0 };
uint8_t uart_rx_byte;
char uart_tx_buf[UART_TX_BUF_SIZE];

/* Command parse buffer */
#define CMD_BUF_SIZE 64
char cmd_buf[CMD_BUF_SIZE];
uint8_t cmd_buf_pos = 0;

/* Telemetry counter */
uint32_t telemetry_counter = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM17_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/*Private Function Prototypes**************************************************/
void calibrateADCs(void);					// calibrating ADC
struct ADC_Values getDMAValues(const uint16_t *dma_values);	// reading ADC values from direct memory access

void writeRTT(struct ADC_Values adc_result,
              const struct AMR_MinMax *minmax,
              float angle_deg_raw,
              float angle_deg);
void writeRTTFloat2(float value);

void amr_update_minmax(struct AMR_MinMax *minmax, uint16_t sin_val, uint16_t cos_val);
struct AMR_CalibParams amr_compute_calibration(const struct AMR_MinMax *minmax);
struct AMR_Normalized amr_normalize(uint16_t sin_val, uint16_t cos_val, const struct AMR_CalibParams *calib);
float amr_calculate_angle(float sin_norm, float cos_norm);

/* UART */
void uart_send(const char *str);
void uart_process_rx(void);
void parse_command(const char *cmd, uint8_t len);
bool parse_i32_field(const char *cmd, uint8_t len, uint8_t *index, int32_t *value);
void log_command_result(const char *cmd, const char *status, const char *reason);

/* Stepper control */
void stepper_set_enabled(bool en);
void stepper_set_direction(int8_t dir);
void stepper_start_velocity(int16_t pct);
void stepper_start_position(float target_deg, uint16_t max_speed_pct);
void stepper_start_calibration(float cal_angle_deg);
void stepper_stop(void);
void stepper_update_motor_angle(void);
uint16_t speed_pct_to_arr(uint16_t pct);

/* Telemetry */
void send_telemetry(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void calibrateADCs(void){
	/*INFOBOX: (CLICK TO OPEN)
	 * This function calibrates the ADC module. Calibration must not be executed while ADC is active.
	 * Calibration called in int main before ADC is activated.
	 */

	if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)		// calibrate ADC1 - single mode
	{
		Error_Handler();
	}
	if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_DIFFERENTIAL_ENDED) != HAL_OK)	// calibrate ADC1 - differential mode
	{
		Error_Handler();
	}
	if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK)		// calibrate ADC2 - single mode
	{
		Error_Handler();
	}
	if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_DIFFERENTIAL_ENDED) != HAL_OK)	// calibrate ADC2 - differential mode
	{
		Error_Handler();
	}
}

struct ADC_Values getDMAValues(const uint16_t *dma_values){
	/*INFOBOX: (CLICK TO OPEN)
	 * This function reads ADC values directly from DMA (RAM) and stores them into result struct for
	 * better readability/handling.
	 */
	struct ADC_Values result;

	result.SIN1 = dma_values[I_SIN1];			//fill struct with ADC values from DMA Array
	result.COS1 = dma_values[I_COS1];
	result.VDIAG1 = dma_values[I_VDIAG1];
	return result;
}

void writeRTT(struct ADC_Values adc_result,
              const struct AMR_MinMax *minmax,
              float angle_deg_raw,
              float angle_deg){
	SEGGER_RTT_printf(0,
		"SIN1=%u (min:%u max:%u) COS1=%u (min:%u max:%u) VDIAG1=%u\r\nangle_raw=",
		adc_result.SIN1,
		minmax->sin_min,
		minmax->sin_max,
		adc_result.COS1,
		minmax->cos_min,
		minmax->cos_max,
		adc_result.VDIAG1);
	writeRTTFloat2(angle_deg_raw);
	SEGGER_RTT_WriteString(0, "° angle=");
	writeRTTFloat2(angle_deg);
	SEGGER_RTT_WriteString(0, "°\r\n");
}

void writeRTTFloat2(float value){
	int32_t centi_degrees = (int32_t)(value * 100.0f + (value >= 0.0f ? 0.5f : -0.5f));
	const char *sign = "";

	if (centi_degrees < 0) {
		sign = "-";
		centi_degrees = -centi_degrees;
	}

	SEGGER_RTT_printf(0, "%s%ld.%02ld", sign, centi_degrees / 100, centi_degrees % 100);
}

void amr_update_minmax(struct AMR_MinMax *minmax, uint16_t sin_val, uint16_t cos_val) {
  if (sin_val < minmax->sin_min) minmax->sin_min = sin_val;
  if (sin_val > minmax->sin_max) minmax->sin_max = sin_val;
  if (cos_val < minmax->cos_min) minmax->cos_min = cos_val;
  if (cos_val > minmax->cos_max) minmax->cos_max = cos_val;
}

struct AMR_CalibParams amr_compute_calibration(const struct AMR_MinMax *minmax) {
	struct AMR_CalibParams result;

	/* Amplitude */
	result.cos_amplitude = (float)(minmax->cos_max - minmax->cos_min) / 2.0f;
	result.sin_amplitude = (float)(minmax->sin_max - minmax->sin_min) / 2.0f;

	/* Offset */
	result.cos_offset = (float)(minmax->cos_max + minmax->cos_min) / 2.0f;
	result.sin_offset = (float)(minmax->sin_max + minmax->sin_min) / 2.0f;
	return result;
}

/*
 * Task 2b: Normalization
 * Returns: range [-1, 1].
 */
struct AMR_Normalized amr_normalize(uint16_t sin_val, uint16_t cos_val, const struct AMR_CalibParams *calib) {
	struct AMR_Normalized result;
	result.sin = ((float)sin_val - calib->sin_offset) / calib->sin_amplitude;
	result.cos = ((float)cos_val - calib->cos_offset) / calib->cos_amplitude;
	return result;
}

/*
 * Task 3: Angle Calculation
 */
float amr_calculate_angle(float sin_norm, float cos_norm) {
	/* atan2: returns [-pi, pi] */
	float angle_rad = atan2f(sin_norm, cos_norm);

	/* Convert to degrees */
	float angle_deg = angle_rad * RAD_TO_DEG;

  /* Bring into range [0, 360] */
	if (angle_deg < 0.0f) {
		angle_deg += 360.0f;
	}

	/* A full rotation of the sensor is only 180° mechanically */
	return angle_deg / 2.0f;
}

void uart_send(const char *str) {
	HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 50);
}

/*==========================================================================
 * UART command parser
 *
 * Protocol: |SuA|<cmd>|<args...>| followed by LF or CRLF
 * V: |SuA|V|±NNNNN|           velocity %*100
 * P: |SuA|P|±NNNNN|NNNNN|     position 0.01deg, max speed %*100
 * C: |SuA|C|NNNNN|            set current position reference to angle 0.01deg
 * E: |SuA|E|0| or |SuA|E|1|  enable/disable
 *==========================================================================*/

void uart_process_rx(void) {
	while (uart_rx.tail != uart_rx.head) {
		char c = uart_rx.data[uart_rx.tail];
		uart_rx.tail = (uart_rx.tail + 1) % UART_RX_BUF_SIZE;

		if (c == '\r' || c == '\n') {
			if (cmd_buf_pos > 0) {
				cmd_buf[cmd_buf_pos] = '\0';
				parse_command(cmd_buf, cmd_buf_pos);
				cmd_buf_pos = 0;
			}
			continue;
		}

		if (cmd_buf_pos < CMD_BUF_SIZE - 1) {
			cmd_buf[cmd_buf_pos++] = c;
		} else {
			cmd_buf[cmd_buf_pos] = '\0';
			log_command_result(cmd_buf, "ERROR", "command too long");
			cmd_buf_pos = 0;
		}
	}
}

void log_command_result(const char *cmd, const char *status, const char *reason) {
	SEGGER_RTT_printf(0, "CMD %s -> %s", cmd, status);
	if (reason != NULL && reason[0] != '\0') {
		SEGGER_RTT_printf(0, " (%s)", reason);
	}
	SEGGER_RTT_WriteString(0, "\r\n");
}

bool parse_i32_field(const char *cmd, uint8_t len, uint8_t *index, int32_t *value) {
	if (*index >= len) {
		return false;
	}

	int32_t sign = 1;
	if (cmd[*index] == '-') {
		sign = -1;
		(*index)++;
	} else if (cmd[*index] == '+') {
		(*index)++;
	}

	if (*index >= len || cmd[*index] < '0' || cmd[*index] > '9') {
		return false;
	}

	int32_t parsed = 0;
	while (*index < len && cmd[*index] >= '0' && cmd[*index] <= '9') {
		parsed = parsed * 10 + (cmd[*index] - '0');
		(*index)++;
	}

	if (*index >= len || cmd[*index] != '|') {
		return false;
	}
	(*index)++;

	*value = sign * parsed;
	return true;
}

void parse_command(const char *cmd, uint8_t len) {
	if (len < CMD_ARG_START_INDEX + 2) {
		log_command_result(cmd, "ERROR", "command too short");
		return;
	}

	if (memcmp(cmd, CMD_HEADER, CMD_HEADER_LEN) != 0) {
		log_command_result(cmd, "ERROR", "invalid header");
		return;
	}

	char mode = cmd[CMD_MODE_INDEX];
	if (cmd[CMD_MODE_INDEX + 1] != '|') {
		log_command_result(cmd, "ERROR", "missing command separator");
		return;
	}

	switch (mode) {
	case 'V': {
		/* |SuA|V|±NNNNN| */
		int32_t vel = 0;
		uint8_t index = CMD_ARG_START_INDEX;
		if (!parse_i32_field(cmd, len, &index, &vel) || index != len) {
			log_command_result(cmd, "ERROR", "invalid velocity format");
		} else if (vel < -10000 || vel > 10000) {
			log_command_result(cmd, "ERROR", "velocity out of range");
		} else if (vel != 0 && !stepper.enabled) {
			log_command_result(cmd, "ERROR", "motor disabled");
		} else {
			stepper_start_velocity((int16_t)vel);
			log_command_result(cmd, "OK", "");
		}
		break;
	}
	case 'P': {
		/* |SuA|P|±NNNNN|NNNNN| */
		int32_t pos = 0;
		int32_t spd = 0;
		uint8_t index = CMD_ARG_START_INDEX;
		if (!parse_i32_field(cmd, len, &index, &pos) ||
		    !parse_i32_field(cmd, len, &index, &spd) ||
		    index != len) {
			log_command_result(cmd, "ERROR", "invalid position format");
		} else if (!stepper.enabled) {
			log_command_result(cmd, "ERROR", "motor disabled");
		} else if (!stepper.calibrated) {
			log_command_result(cmd, "ERROR", "not calibrated");
		} else if (pos < (int32_t)(ANGLE_MIN_DEG * 100.0f) || pos > (int32_t)(ANGLE_MAX_DEG * 100.0f)) {
			log_command_result(cmd, "ERROR", "position out of range");
		} else if (spd < 0 || spd > 10000) {
			log_command_result(cmd, "ERROR", "speed out of range");
		} else {
			float target = (float)pos / 100.0f;
			uint16_t max_spd = (spd > 10000) ? 10000 : (spd < 100 ? 100 : (uint16_t)spd);
			stepper_start_position(target, max_spd);
			log_command_result(cmd, "OK", "");
		}
		break;
	}
	case 'C': {
		/* |SuA|C|NNNNN| */
		int32_t cal = 0;
		uint8_t index = CMD_ARG_START_INDEX;
		if (!parse_i32_field(cmd, len, &index, &cal) || index != len) {
			log_command_result(cmd, "ERROR", "invalid calibration format");
		} else if (cal < (int32_t)(ANGLE_MIN_DEG * 100.0f) || cal > (int32_t)(ANGLE_MAX_DEG * 100.0f)) {
			log_command_result(cmd, "ERROR", "calibration angle out of range");
		} else {
			float cal_deg = (float)cal / 100.0f;
			stepper_start_calibration(cal_deg);
			log_command_result(cmd, "OK", "");
		}
		break;
	}
	case 'E': {
		/* |SuA|E|0| or |SuA|E|1| */
		int32_t en = 0;
		uint8_t index = CMD_ARG_START_INDEX;
		if (!parse_i32_field(cmd, len, &index, &en) || index != len) {
			log_command_result(cmd, "ERROR", "invalid enable format");
		} else if (en != 0 && en != 1) {
			log_command_result(cmd, "ERROR", "enable must be 0 or 1");
		} else {
			stepper_set_enabled(en != 0);
			log_command_result(cmd, "OK", "");
		}
		break;
	}
	default:
		log_command_result(cmd, "ERROR", "unknown command");
		break;
	}
}

/*==========================================================================
 * Stepper motor control
 *==========================================================================*/

uint16_t speed_pct_to_arr(uint16_t pct) {
	/* pct is %*100, range 0..10000 (0% to 100%) */
	if (pct == 0) return ARR_MAX;
	/* Map pct linearly: 100% → ARR_MIN, ~1% → ARR_MAX */
	float n = MAX_SPEED_RPS * ((float)pct / 10000.0f);
	float arr_f = (float)F_TIMER / (2.0f * n * STEPS_PER_REV) - 1.0f;
	if (arr_f < ARR_MIN) return ARR_MIN;
	if (arr_f > ARR_MAX) return ARR_MAX;
	return (uint16_t)arr_f;
}

void stepper_update_motor_angle(void) {
	stepper.motor_angle_deg = (float)stepper.current_pos * DEG_PER_MICROSTEP;
}

void stepper_set_enabled(bool en) {
	stepper.enabled = en;
	HAL_GPIO_WritePin(Enable_GPIO_Port, Enable_Pin, en ? GPIO_PIN_RESET : GPIO_PIN_SET);
	if (!en) {
		stepper.mode = MODE_IDLE;
		stepper.steps_remaining = 0;
		stepper.velocity_pct = 0;
	}
}

void stepper_set_direction(int8_t dir) {
	stepper.direction = dir;
	HAL_GPIO_WritePin(Dir_GPIO_Port, Dir_Pin, dir > 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void stepper_stop(void) {
	stepper.mode = MODE_IDLE;
	stepper.steps_remaining = 0;
	stepper.velocity_pct = 0;
}

void stepper_start_velocity(int16_t pct) {
	if (!stepper.enabled) return;

	if (pct == 0) {
		stepper_stop();
		return;
	}

	stepper.mode = MODE_VELOCITY;
	stepper.velocity_pct = pct;

	int8_t dir = (pct > 0) ? 1 : -1;
	stepper_set_direction(dir);

	uint16_t abs_pct = (uint16_t)(pct > 0 ? pct : -pct);
	stepper.current_arr = speed_pct_to_arr(abs_pct);
	__HAL_TIM_SET_AUTORELOAD(&htim17, stepper.current_arr);
}

void stepper_start_position(float target_deg, uint16_t max_speed_pct) {
	if (!stepper.enabled || !stepper.calibrated) return;

	if (target_deg < ANGLE_MIN_DEG) target_deg = ANGLE_MIN_DEG;
	if (target_deg > ANGLE_MAX_DEG) target_deg = ANGLE_MAX_DEG;

	stepper.target_angle_deg = target_deg;

	float delta_deg = target_deg - stepper.motor_angle_deg;
	int32_t steps = (int32_t)(delta_deg / DEG_PER_MICROSTEP);

	if (steps == 0) {
		stepper_stop();
		return;
	}

	stepper_set_direction(steps > 0 ? 1 : -1);
	stepper.steps_remaining = abs(steps);
	stepper.steps_total = stepper.steps_remaining;
	stepper.velocity_pct = (int16_t)max_speed_pct;
	stepper.mode = MODE_POSITION;

	/* Start slow */
	stepper.current_arr = RAMP_START_ARR;
	__HAL_TIM_SET_AUTORELOAD(&htim17, stepper.current_arr);
}

void stepper_start_calibration(float cal_angle_deg) {
	if (cal_angle_deg < ANGLE_MIN_DEG) cal_angle_deg = ANGLE_MIN_DEG;
	if (cal_angle_deg > ANGLE_MAX_DEG) cal_angle_deg = ANGLE_MAX_DEG;

	stepper_stop();
	stepper.target_angle_deg = cal_angle_deg;
	stepper.current_pos = (int32_t)(cal_angle_deg / DEG_PER_MICROSTEP);
	stepper_update_motor_angle();
	stepper.calibrated = true;
}

/*==========================================================================
 * Telemetry — JSON over UART
 *==========================================================================*/

void send_telemetry(void) {
	/* Build JSON by concatenation — no library needed.
	 * Angles in 0.01 deg (integer), velocity in %*100. */
	const char *mode_str;
	switch (stepper.mode) {
		case MODE_VELOCITY:    mode_str = "V"; break;
		case MODE_POSITION:    mode_str = "P"; break;
		default:               mode_str = "I"; break;
	}

	int32_t motor_cdeg = (int32_t)(stepper.motor_angle_deg * 100.0f);
	int32_t amr_cdeg = (int32_t)(stepper.amr_angle_deg * 100.0f);
	int32_t target_cdeg = (int32_t)(stepper.target_angle_deg * 100.0f);

	/* Compute steps/sec from current ARR */
	int32_t sps = 0;
	if (stepper.mode == MODE_VELOCITY || stepper.mode == MODE_POSITION) {
		sps = (int32_t)(F_TIMER / (2.0f * (stepper.current_arr + 1)));
	}

	int len = snprintf(uart_tx_buf, UART_TX_BUF_SIZE,
		"{\"mode\":\"%s\",\"motor\":%ld,\"amr\":%ld,\"target\":%ld,"
		"\"vel\":%d,\"sps\":%ld,\"en\":%d,\"cal\":%d,\"rem\":%ld}\n",
		mode_str, motor_cdeg, amr_cdeg, target_cdeg,
		stepper.velocity_pct, sps,
		stepper.enabled ? 1 : 0,
		stepper.calibrated ? 1 : 0,
		stepper.steps_remaining);

	if (len > 0 && len < UART_TX_BUF_SIZE)
		uart_send(uart_tx_buf);
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
  MX_ADC2_Init();
  MX_TIM7_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_TIM17_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* Initialize user-defined Peripherals ***************************************************************/
  SEGGER_RTT_Init();
  SEGGER_RTT_WriteString(0, "RTT initialized\r\n");

  uart_send("{\"msg\":\"SuA stepper controller ready\"}\n");
  SEGGER_RTT_WriteString(0, "USART2 initialized\r\n");

  calibrateADCs();													// calibrate ADCs

  HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*)adcVal, 3);		// configure ADC mode
  HAL_TIM_Base_Start_IT(&htim7);									// start timer 7
  HAL_TIM_Base_Start_IT(&htim17);

  /* Motor disabled at startup */
  stepper_set_enabled(false);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uart_process_rx();
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 26;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV8;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T7_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_8;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_3;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_DUALMODE_REGSIMULT;
  multimode.DMAAccessMode = ADC_DMAACCESSMODE_12_10_BITS;
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_1CYCLE;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_DIFFERENTIAL_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 3;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = ENABLE;
  hadc2.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_8;
  hadc2.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_3;
  hadc2.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc2.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_DIFFERENTIAL_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_17;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 650-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  htim7.Init.Prescaler = 13-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 9999;
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
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 10-1;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 299;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

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
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
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
  HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);
  /* USER CODE END USART2_Init 2 */

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
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Enable_GPIO_Port, Enable_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Step_Pin|Dir_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Enable_Pin */
  GPIO_InitStruct.Pin = Enable_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Enable_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Step_Pin Dir_Pin LD2_Pin */
  GPIO_InitStruct.Pin = Step_Pin|Dir_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart2) {
		uart_rx.data[uart_rx.head] = uart_rx_byte;
		uart_rx.head = (uart_rx.head + 1) % UART_RX_BUF_SIZE;
		HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){

	/*INFOBOX: (CLICK TO OPEN)
	 * This ADC callback is called after ADC conversion is done.
	 * 1. ADC values are read from RAM via DMA
	 * 2. values are then normalized
	 * 3. then CORDIC is executing angle measurement
	 * 4. redundancy check between both channels
	 */

	ADCresult = getDMAValues(adcVal);			// save raw ADC values from DMA array into struct

	amr_update_minmax(&amr_minmax, ADCresult.SIN1, ADCresult.COS1);
	amr_calib = amr_compute_calibration(&amr_minmax);
	amr_normalized = amr_normalize(ADCresult.SIN1, ADCresult.COS1, &amr_calib);
	amr_angle_deg_raw = amr_calculate_angle(amr_normalized.sin, amr_normalized.cos);
	amr_angle_deg = amr_angle_deg_raw - amr_angle_deg_offset;
  if (amr_angle_deg < 0.0f) {
    amr_angle_deg += 180.0f;
  }

	stepper.amr_angle_deg = amr_angle_deg;
	writeRTT(ADCresult, &amr_minmax, amr_angle_deg_raw, amr_angle_deg);

	/* Send telemetry at reduced rate */
	telemetry_counter++;
	if (telemetry_counter >= TELEMETRY_DIVISOR) {
		telemetry_counter = 0;
		send_telemetry();
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	/*INFOBOX: (CLICK TO OPEN)
	 * This TIMER callback is called by ISR from TIMER4.
	 * - TIMER 4: toggeling user LED in ERROR-state (every 200ms / 5Hz)
	 */

	if(htim == &htim17){
		if (!stepper.enabled) return;

		if (stepper.mode == MODE_VELOCITY) {
			/* Continuous stepping — toggle the STEP pin */
			HAL_GPIO_TogglePin(Step_GPIO_Port, Step_Pin);

			/* Count only on rising edges (every other toggle) */
			static bool step_high = false;
			step_high = !step_high;
			if (step_high) {
				stepper.current_pos += stepper.direction;
				stepper_update_motor_angle();
			}
		}
		else if (stepper.mode == MODE_POSITION && stepper.steps_remaining > 0) {
			HAL_GPIO_TogglePin(Step_GPIO_Port, Step_Pin);

			static bool pos_step_high = false;
			pos_step_high = !pos_step_high;
			if (pos_step_high) {
				stepper.current_pos += stepper.direction;
				stepper.steps_remaining--;
				stepper_update_motor_angle();

				/* Trapezoidal ramp profile */
				int32_t steps_done = stepper.steps_total - stepper.steps_remaining;
				uint16_t cruise_arr = speed_pct_to_arr((uint16_t)abs(stepper.velocity_pct));

				if (stepper.steps_total < RAMP_MIN_STEPS) {
					/* Short move — just go slow */
					stepper.current_arr = RAMP_START_ARR;
				} else if (steps_done < RAMP_ACCEL_STEPS) {
					/* Accelerating */
					float frac = (float)steps_done / RAMP_ACCEL_STEPS;
					stepper.current_arr = (uint16_t)(RAMP_START_ARR - frac * (RAMP_START_ARR - cruise_arr));
				} else if (stepper.steps_remaining < RAMP_ACCEL_STEPS) {
					/* Decelerating */
					float frac = (float)stepper.steps_remaining / RAMP_ACCEL_STEPS;
					stepper.current_arr = (uint16_t)(RAMP_START_ARR - frac * (RAMP_START_ARR - cruise_arr));
				} else {
					stepper.current_arr = cruise_arr;
				}
				__HAL_TIM_SET_AUTORELOAD(&htim17, stepper.current_arr);

				if (stepper.steps_remaining == 0) {
					stepper_stop();
				}
			}
		}
	}

	if (htim == &htim4){
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);				//toggle Error_LED to visualize error case
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
