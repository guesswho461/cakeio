#include "cake_io.h"

#include <string.h>
#include <stdio.h>
#include <math.h>

#include "main.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_flash.h"
#include "stm32f1xx_hal_tim.h"

#include "usart.h"
#include "i2c.h"
#include "spi.h"
#include "adc.h"
#include "tim.h"

#include "OneWire.h"
#include "DallasTemperature.h"
#include "lcd_hd44780_i2c.h"
#include "vl53l0x_api.h"
#include "max6675.h"
#include "modbus.h"
#include "mcp23017.h"

/* PRIVATE DEFINE */

#define __HAL_DMA_SET_COUNTER(__HANDLE__, __COUNTER__) ((__HANDLE__)->Instance->CNDTR = (uint16_t)(__COUNTER__))
#define __TO_LOGIC_ADDRESS(__ADDR__) (__ADDR__ - 40001)
#define __SET_BIT_TO_0(__DATA__, __BIT__) (__DATA__ &= ~(1 << __BIT__))
#define __SET_BIT_TO_1(__DATA__, __BIT__) (__DATA__ |= 1 << __BIT__)
#define __SET_MEM(__DATA__, __VALUE__, __SIZE__) (memset((void *)__DATA__, __VALUE__, __SIZE__))

#define MODBUS_DEVICE_ADDR 0x01
#define FW_VERSION 1

#define MAX_OW_CNT 2 //has to be less than 5
#define MAX_LCD_CHAR_CNT_PER_LINE 16
#define MAX_LCD_CHAR_CNT (MAX_LCD_CHAR_CNT_PER_LINE << 1)
#define MAX_PORTOCOL_BUF_SIZE 256
#define MAX_MCP23017_BIT_SIZE 16
#define MAX_ANALOG_PIN_CNT 4
#define MAX_ANALOG_DATA_SIZE (MAX_ANALOG_PIN_CNT * 5)
#define ASCII_SPACE 32
#define DEFAULT_GATE_CMD_MAX 180

#define PAR_MEM_FLASH_ADD 0x0801FC00

#define PROTOCOL_UART_HANDLE huart2
#define PROTOCOL_UART_INST USART2
#define PROTOCOL_UART_DMA_RX_HANDLE hdma_usart2_rx
#define OW_UART_HANDLE huart3
#define I2C_HANDLE hi2c2
#define OVEN_THERMO_HANDLE hspi2
#define OVEN_THERMO_CS_PIN SPI2_CS_Pin
#define OVEN_THERMO_CS_PORT SPI2_CS_GPIO_Port
#define ADC_HANDLE hadc1
#define TIMER_HANDLE htim3
#define BOWL_TOF_I2C_HANDLE hi2c1

#define BOWL_TOF_I2C_ADDR 0x52
#define BATTER_TOF_I2C_ADDR 0x53
#define LCD_I2C_ADDR 0x27
#define MCP_0_I2C_ADDR 0x20
#define MCP_1_I2C_ADDR 0x21

#define BIT_IDX_COIN_1_CMD				5
#define BIT_IDX_COIN_2_CMD				6
#define BIT_IDX_BRAKE_CMD				4
#define BIT_IDX_LED_1_CMD				3
#define BIT_IDX_LED_2_CMD				2
#define BIT_IDX_LED_3_CMD				1
#define BIT_IDX_DEPRESSURELIZE_CMD		0
#define BIT_IDX_FRIG_CMD				7
#define BIT_IDX_VACUUM_CMD				6
#define BIT_IDX_OVEN_CMD				5

#define BIT_IDX_MOT_01_SERVO_ON_CMD		0
#define BIT_IDX_MOT_02_SERVO_ON_CMD		1
#define BIT_IDX_MOT_03_SERVO_ON_CMD		2
#define BIT_IDX_MOT_04_SERVO_ON_CMD		3
#define BIT_IDX_MOT_05_SERVO_ON_CMD		4
#define BIT_IDX_MOT_06_SERVO_ON_CMD		5
#define BIT_IDX_MOT_07_SERVO_ON_CMD		6
#define BIT_IDX_MOT_08_SERVO_ON_CMD		7

#define MAX_PAR_MEM_SIZE 					0x0100 //the size has to be 2^N
//the address should not be greater than PAR_MEM_SIZE
//system
#define PAR_ADDR_FW_VER 					0x0000
#define PAR_ADDR_SERIAL_NUM_H 				0x0001
#define PAR_ADDR_SERIAL_NUM_L 				0x0002
#define PAR_ADDR_SYS_CMD					0x0003
#define PAR_ADDR_SYS_PW						0x0004
#define PAR_ADDR_SYS_CYCLIC_TIME 			0x0005
#define PAR_ADDR_SYS_ERROR_CODE 			0x0006
#define PAR_ADDR_SYS_HEARTBEAT 				0x0007

//status
#define PAR_ADDR_BOWL_CNT 					0x0010 //I2C 0x52, SDA PB11, SCL PB10
#define PAR_ADDR_BATTER_CNT 				0x0011 //I2C 0x53, SDA PB11, SCL PB10
#define PAR_ADDR_OVEN_TEMP 					0x0012 //SPI, MOSI PB15, MISO PB14, SCL PB13, CS PB12
#define PAR_ADDR_FRIG_TEMP 					0x0013 //OW, TX PC10
#define PAR_ADDR_MACH_TEMP 					0x0014 //OW, TX PC10
#define PAR_ADDR_BOWL_READY					0x0015 //PC00
#define PAR_ADDR_FLOW_1_CNT					0x0016 //PC01
#define PAR_ADDR_FLOW_2_CNT					0x0017 //PC02
#define PAR_ADDR_COIN_1_CNT					0x0018 //PC03
#define PAR_ADDR_COIN_2_CNT					0x0019 //PC04
#define PAR_ADDR_TP_MODE					0x001A //PC05
#define PAR_ADDR_TP_JOG_SPD					0x001B //ADC1, IN6, PA06
#define PAR_ADDR_TP_JOG_TARGET				0x001C //PA00, PA01, PA04, PA05, PA08, PA09, PA10
#define PAR_ADDR_TP_JOG_DIR					0x001D //PC08
#define PAR_ADDR_ALARM_RESET				0x001E //PC09
#define PAR_ADDR_CLEAN_1					0x001F //PC11
#define PAR_ADDR_CLEAN_2					0x0020 //PC12
#define PAR_ADDR_CLEAN_3					0x002A //PC13

//command
#define PAR_ADDR_COIN_1_CMD					0x0080 //I2C 0x20, 1st MCP23017, PORTB, PIN 5
#define PAR_ADDR_COIN_2_CMD					0x0081 //I2C 0x20, 1st MCP23017, PORTB, PIN 6
#define PAR_ADDR_BRAKE_CMD					0x0082 //I2C 0x20, 1st MCP23017, PORTB, PIN 4
#define PAR_ADDR_LED_1_CMD					0x0083 //I2C 0x20, 1st MCP23017, PORTB, PIN 3
#define PAR_ADDR_LED_2_CMD					0x0084 //I2C 0x20, 1st MCP23017, PORTB, PIN 2
#define PAR_ADDR_LED_3_CMD					0x0085 //I2C 0x20, 1st MCP23017, PORTB, PIN 1
#define PAR_ADDR_DEPRESSURELIZE_CMD			0x0086 //I2C 0x20, 1st MCP23017, PORTB, PIN 0
#define PAR_ADDR_FRIG_CMD					0x0087 //I2C 0x20, 1st MCP23017, PORTA, PIN 7
#define PAR_ADDR_VACUUM_CMD					0x0088 //I2C 0x20, 1st MCP23017, PORTA, PIN 6
#define PAR_ADDR_OVEN_CMD					0x0089 //I2C 0x20, 1st MCP23017, PORTA, PIN 5
#define PAR_ADDR_GATE_CMD					0x008A //TIM3, CH1, PC06
#define PAR_ADDR_GATE_CMD_MAX				0x008B
#define PAR_ADDR_GATE_CMD_MIN				0x008C

#define PAR_ADDR_MOT_01_SERVO_ON_CMD		0x0090 //I2C 0x21, 2nd MCP23017, PORTB, PIN 0
#define PAR_ADDR_MOT_02_SERVO_ON_CMD		0x0091 //I2C 0x21, 2nd MCP23017, PORTB, PIN 1
#define PAR_ADDR_MOT_03_SERVO_ON_CMD		0x0092 //I2C 0x21, 2nd MCP23017, PORTB, PIN 2
#define PAR_ADDR_MOT_04_SERVO_ON_CMD		0x0093 //I2C 0x21, 2nd MCP23017, PORTB, PIN 3
#define PAR_ADDR_MOT_05_SERVO_ON_CMD		0x0094 //I2C 0x21, 2nd MCP23017, PORTB, PIN 4
#define PAR_ADDR_MOT_06_SERVO_ON_CMD		0x0095 //I2C 0x21, 2nd MCP23017, PORTB, PIN 5
#define PAR_ADDR_MOT_07_SERVO_ON_CMD		0x0096 //I2C 0x21, 2nd MCP23017, PORTB, PIN 6
#define PAR_ADDR_MOT_08_SERVO_ON_CMD		0x0097 //I2C 0x21, 2nd MCP23017, PORTB, PIN 7

/* PRIVATE DEFINE */

/* PRIVATE PAR DECLARE */

struct TOF {
	VL53L0X_Dev_t dev;
	uint16_t mem_addr;
	GPIO_TypeDef *x_pin_port;
	uint16_t x_pin;
};

struct MCP_DIO {
	bool is_output;
	bool valid;
	uint16_t mem_addr;
	uint8_t port;
	uint8_t bit_idx;
};

struct MCP23017 {
	MCP23017_HandleTypeDef hdev;
	I2C_HandleTypeDef *p_i2c;
	uint16_t i2c_addr;
	uint8_t port_a_iodir;
	uint8_t port_b_iodir;
	struct MCP_DIO dio_cfg[MAX_MCP23017_BIT_SIZE];
};

struct STM32_GPIO {
	int16_t mem_bit_idx;
	bool is_output;
	GPIO_TypeDef *port;
	uint16_t pin;
	uint16_t mem_addr;
};

struct PWM {
	TIM_HandleTypeDef *p_tim;
	uint32_t ch;
	uint16_t mem_addr;
	uint16_t max_addr;
	uint16_t min_addr;
};

/* PRIVATE PAR DECLARE */

/* PRIVATE PAR */

uint16_t par_mem[MAX_PAR_MEM_SIZE] = { 0 };
OneWire_HandleTypeDef ow;
DallasTemperature_HandleTypeDef dt;
uint8_t ow_cnt = 0;
char lcd_char_buf[MAX_LCD_CHAR_CNT] = { ASCII_SPACE };
ThermoCouple oven_thermo;
uint32_t start = 0;
uint32_t end = 0;
Modbus_Conf_t mb_srv;
mbus_t mb_srv_ctx;
uint8_t mb_srv_rx_buf[MAX_PORTOCOL_BUF_SIZE] = { 0 };
uint8_t mb_srv_tx_buf[MAX_PORTOCOL_BUF_SIZE] = { 0 };
uint8_t rx_buf[MAX_PORTOCOL_BUF_SIZE] = { 0 };
uint8_t rx_len = 0;
HAL_StatusTypeDef ret = 0;
uint32_t analog_datas[MAX_ANALOG_DATA_SIZE] = { 0 };

const uint16_t ow_par_add_table[MAX_OW_CNT] = { PAR_ADDR_FRIG_TEMP, PAR_ADDR_MACH_TEMP };

struct TOF tofs[] = {
		{.mem_addr = PAR_ADDR_BOWL_CNT,
		 .x_pin_port = BOWL_TOF_X_GPIO_Port,
		 .x_pin = BOWL_TOF_X_Pin,
		 .dev = { .I2cHandle = &I2C_HANDLE,
				  .I2cDevAddr = 0x52,
				  .target_i2c_addr = BOWL_TOF_I2C_ADDR,
				  .comms_type = 1,
				  .comms_speed_khz = 100,
				  .mode = 2 }},
		{.mem_addr = PAR_ADDR_BATTER_CNT,
		 .x_pin_port = BATTER_TOF_X_GPIO_Port,
		 .x_pin = BATTER_TOF_X_Pin,
		 .dev = { .I2cHandle = &I2C_HANDLE,
				  .I2cDevAddr = 0x52,
				  .target_i2c_addr = BATTER_TOF_I2C_ADDR,
				  .comms_type = 1,
				  .comms_speed_khz = 100,
				  .mode = 2 }},
		NULL };

struct MCP23017 mcps[] = {
		{.p_i2c = &I2C_HANDLE,
		.i2c_addr = MCP_0_I2C_ADDR,
		.port_a_iodir = MCP23017_IODIR_ALL_OUTPUT,
		.port_b_iodir = MCP23017_IODIR_ALL_OUTPUT,
		.dio_cfg[0] = {
				.is_output = true,
				.valid = true,
				.mem_addr = PAR_ADDR_COIN_1_CMD,
				.port = MCP23017_PORTB,
				.bit_idx = BIT_IDX_COIN_1_CMD },
		.dio_cfg[1] = {
				.is_output = true,
				.valid = true,
				.mem_addr = PAR_ADDR_COIN_2_CMD,
				.port = MCP23017_PORTB,
				.bit_idx = BIT_IDX_COIN_2_CMD },
		.dio_cfg[2] = {
				.is_output = true,
				.valid = true,
				.mem_addr = PAR_ADDR_BRAKE_CMD,
				.port = MCP23017_PORTB,
				.bit_idx = BIT_IDX_BRAKE_CMD },
		.dio_cfg[3] = {
				.is_output = true,
				.valid = true,
				.mem_addr = PAR_ADDR_LED_1_CMD,
				.port = MCP23017_PORTB,
				.bit_idx = BIT_IDX_LED_1_CMD },
		.dio_cfg[4] = {
				.is_output = true,
				.valid = true,
				.mem_addr = PAR_ADDR_LED_2_CMD,
				.port = MCP23017_PORTB,
				.bit_idx = BIT_IDX_LED_2_CMD },
		.dio_cfg[5] = {
				.is_output = true,
				.valid = true,
				.mem_addr = PAR_ADDR_LED_3_CMD,
				.port = MCP23017_PORTB,
				.bit_idx = BIT_IDX_LED_3_CMD },
		.dio_cfg[6] = {
				.is_output = true,
				.valid = true,
				.mem_addr = PAR_ADDR_DEPRESSURELIZE_CMD,
				.port = MCP23017_PORTB,
				.bit_idx = BIT_IDX_DEPRESSURELIZE_CMD },
		.dio_cfg[7] = {
				.is_output = true,
				.valid = true,
				.mem_addr = PAR_ADDR_FRIG_CMD,
				.port = MCP23017_PORTA,
				.bit_idx = BIT_IDX_FRIG_CMD },
		.dio_cfg[8] = {
				.is_output = true,
				.valid = true,
				.mem_addr = PAR_ADDR_VACUUM_CMD,
				.port = MCP23017_PORTA,
				.bit_idx = BIT_IDX_VACUUM_CMD },
		.dio_cfg[9] = {
				.is_output = true,
				.valid = true,
				.mem_addr = PAR_ADDR_OVEN_CMD,
				.port = MCP23017_PORTA,
				.bit_idx = BIT_IDX_OVEN_CMD }},
		{.p_i2c = &I2C_HANDLE,
		.i2c_addr = MCP_1_I2C_ADDR,
		.port_a_iodir = MCP23017_IODIR_ALL_OUTPUT,
		.port_b_iodir = MCP23017_IODIR_ALL_OUTPUT,
		.dio_cfg[0] = {
				.is_output = true,
				.valid = true,
				.mem_addr = PAR_ADDR_MOT_01_SERVO_ON_CMD,
				.port = MCP23017_PORTB,
				.bit_idx = BIT_IDX_MOT_01_SERVO_ON_CMD },
		.dio_cfg[1] = {
				.is_output = true,
				.valid = true,
				.mem_addr = PAR_ADDR_MOT_02_SERVO_ON_CMD,
				.port = MCP23017_PORTB,
				.bit_idx = BIT_IDX_MOT_02_SERVO_ON_CMD },
		.dio_cfg[2] = {
				.is_output = true,
				.valid = true,
				.mem_addr = PAR_ADDR_MOT_03_SERVO_ON_CMD,
				.port = MCP23017_PORTB,
				.bit_idx = BIT_IDX_MOT_03_SERVO_ON_CMD },
		.dio_cfg[3] = {
				.is_output = true,
				.valid = true,
				.mem_addr = PAR_ADDR_MOT_04_SERVO_ON_CMD,
				.port = MCP23017_PORTB,
				.bit_idx = BIT_IDX_MOT_04_SERVO_ON_CMD },
		.dio_cfg[4] = {
				.is_output = true,
				.valid = true,
				.mem_addr = PAR_ADDR_MOT_05_SERVO_ON_CMD,
				.port = MCP23017_PORTB,
				.bit_idx = BIT_IDX_MOT_05_SERVO_ON_CMD },
		.dio_cfg[5] = {
				.is_output = true,
				.valid = true,
				.mem_addr = PAR_ADDR_MOT_06_SERVO_ON_CMD,
				.port = MCP23017_PORTB,
				.bit_idx = BIT_IDX_MOT_06_SERVO_ON_CMD },
		.dio_cfg[6] = {
				.is_output = true,
				.valid = true,
				.mem_addr = PAR_ADDR_MOT_07_SERVO_ON_CMD,
				.port = MCP23017_PORTB,
				.bit_idx = BIT_IDX_MOT_07_SERVO_ON_CMD },
		.dio_cfg[7] = {
				.is_output = true,
				.valid = true,
				.mem_addr = PAR_ADDR_MOT_08_SERVO_ON_CMD,
				.port = MCP23017_PORTB,
				.bit_idx = BIT_IDX_MOT_08_SERVO_ON_CMD }},
		NULL };

struct PWM pwms[] = {
		{ .p_tim = &TIMER_HANDLE,
		  .ch = TIM_CHANNEL_1,
		  .mem_addr = PAR_ADDR_GATE_CMD,
		  .max_addr = PAR_ADDR_GATE_CMD_MAX,
		  .min_addr = PAR_ADDR_GATE_CMD_MIN },
		NULL };

struct STM32_GPIO stm32_gpios[] = {
		{ .mem_bit_idx = -1,
		  .is_output = false,
		  .port = ALARM_RESET_GPIO_Port,
		  .pin = ALARM_RESET_Pin,
		  .mem_addr = PAR_ADDR_ALARM_RESET },
		{ .mem_bit_idx = -1,
		  .is_output = false,
		  .port = BOWL_READY_GPIO_Port,
		  .pin = BOWL_READY_Pin,
		  .mem_addr = PAR_ADDR_BOWL_READY },
		{ .mem_bit_idx = -1,
		  .is_output = false,
		  .port = FLOW_1_CNT_GPIO_Port,
		  .pin = FLOW_1_CNT_Pin,
		  .mem_addr = PAR_ADDR_FLOW_1_CNT },
		{ .mem_bit_idx = -1,
		  .is_output = false,
		  .port = FLOW_2_CNT_GPIO_Port,
		  .pin = FLOW_2_CNT_Pin,
		  .mem_addr = PAR_ADDR_FLOW_2_CNT },
		{ .mem_bit_idx = -1,
		  .is_output = false,
		  .port = COIN_1_CNT_GPIO_Port,
		  .pin = COIN_1_CNT_Pin,
		  .mem_addr = PAR_ADDR_COIN_1_CNT },
		{ .mem_bit_idx = -1,
		  .is_output = false,
		  .port = COIN_2_CNT_GPIO_Port,
		  .pin = COIN_2_CNT_Pin,
		  .mem_addr = PAR_ADDR_COIN_2_CNT },
		{ .mem_bit_idx = -1,
		  .is_output = false,
		  .port = TP_MODE_GPIO_Port,
		  .pin = TP_MODE_Pin,
		  .mem_addr = PAR_ADDR_TP_MODE },
		{ .mem_bit_idx = -1,
		  .is_output = false,
		  .port = TP_JOG_DIR_GPIO_Port,
		  .pin = TP_JOG_DIR_Pin,
		  .mem_addr = PAR_ADDR_TP_JOG_DIR },
		{ .mem_bit_idx = 0,
		  .is_output = false,
		  .port = TP_JOG_TARGET_0_GPIO_Port,
		  .pin = TP_JOG_TARGET_0_Pin,
		  .mem_addr = PAR_ADDR_TP_JOG_TARGET },
		{ .mem_bit_idx = 1,
		  .is_output = false,
		  .port = TP_JOG_TARGET_1_GPIO_Port,
		  .pin = TP_JOG_TARGET_1_Pin,
		  .mem_addr = PAR_ADDR_TP_JOG_TARGET },
		{ .mem_bit_idx = 2,
		  .is_output = false,
		  .port = TP_JOG_TARGET_2_GPIO_Port,
		  .pin = TP_JOG_TARGET_2_Pin,
		  .mem_addr = PAR_ADDR_TP_JOG_TARGET },
		{ .mem_bit_idx = 3,
		  .is_output = false,
		  .port = TP_JOG_TARGET_3_GPIO_Port,
		  .pin = TP_JOG_TARGET_3_Pin,
		  .mem_addr = PAR_ADDR_TP_JOG_TARGET },
		{ .mem_bit_idx = 4,
		  .is_output = false,
		  .port = TP_JOG_TARGET_4_GPIO_Port,
		  .pin = TP_JOG_TARGET_4_Pin,
		  .mem_addr = PAR_ADDR_TP_JOG_TARGET },
		{ .mem_bit_idx = 5,
		  .is_output = false,
		  .port = TP_JOG_TARGET_5_GPIO_Port,
		  .pin = TP_JOG_TARGET_5_Pin,
		  .mem_addr = PAR_ADDR_TP_JOG_TARGET },
		{ .mem_bit_idx = 6,
		  .is_output = false,
		  .port = TP_JOG_TARGET_6_GPIO_Port,
		  .pin = TP_JOG_TARGET_6_Pin,
		  .mem_addr = PAR_ADDR_TP_JOG_TARGET },
		{ .mem_bit_idx = -1,
		  .is_output = false,
		  .port = CLEAN_1_GPIO_Port,
		  .pin = CLEAN_1_Pin,
		  .mem_addr = PAR_ADDR_CLEAN_1 },
		{ .mem_bit_idx = -1,
		  .is_output = false,
		  .port = CLEAN_2_GPIO_Port,
		  .pin = CLEAN_2_Pin,
		  .mem_addr = PAR_ADDR_CLEAN_2 },
		{ .mem_bit_idx = -1,
		  .is_output = false,
		  .port = CLEAN_3_GPIO_Port,
		  .pin = CLEAN_3_Pin,
		  .mem_addr = PAR_ADDR_CLEAN_3 },
		NULL };

/* PRIVATE PAR */

/* PRIVATE FUNCTION PROTOTYPE */

uint8_t dt_init(OneWire_HandleTypeDef *p_ow,
		DallasTemperature_HandleTypeDef *p_dt,
		UART_HandleTypeDef *p_uart);
void get_ow_temperature(DallasTemperature_HandleTypeDef *p_dt,
		uint16_t *p_mem,
		const uint16_t *p_add_table);
uint32_t protocol_parser(uint8_t *p_uart_rx_buf, uint32_t size);
void set_flash(uint32_t dst, uint8_t *p_src, uint32_t size);
void get_flash(uint8_t *p_dst, uint32_t src, uint32_t size);
void set_lcd(char *p_buf);
uint16_t tofs_init(struct TOF *p_tofs);
void tof_cyclic(struct TOF *p_tofs, uint16_t *p_mem);
void oven_thermo_init(ThermoCouple *p_thermo);
uint16_t get_oven_temperature(ThermoCouple *p_thermo);
mbus_t modbus_server_init(Modbus_Conf_t *p_mb_srv,
		uint8_t devaddr,
		uint8_t *p_rx_buf,
		uint16_t rx_buf_size,
		uint8_t *p_tx_buf,
		uint16_t tx_buf_size);
uint16_t user_mbus_read(uint32_t la);
uint16_t user_mbus_write(uint32_t la, uint16_t value);
int user_mbus_send(const mbus_t mb_ctx,
		const uint8_t *data,
		const uint16_t size);
void mcp_init(struct MCP23017 *p_mcps);
void mcp_cyclic(struct MCP23017 *p_mcps, uint16_t *p_mem);
void set_dio(uint16_t cmd, uint8_t *p_data, uint8_t bit_idx);
void stm32_dio_cyclic(struct STM32_GPIO *p_cfgs, uint16_t *p_mem);
void pwm_init(struct PWM *p_pwms);
void pwm_cyclic(struct PWM *p_pwms, const uint16_t *p_mem);

/* PRIVATE FUNCTION PROTOTYPE */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) {
		__HAL_UART_CLEAR_IDLEFLAG(huart);
		__HAL_DMA_DISABLE(huart->hdmarx);

		rx_len = MAX_PORTOCOL_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);

		for (int i = 0; i < rx_len; ++i) {
			mbus_poll(mb_srv_ctx, rx_buf[i]);
		}

		__HAL_DMA_SET_COUNTER(huart->hdmarx, MAX_PORTOCOL_BUF_SIZE);
		__HAL_DMA_ENABLE(huart->hdmarx);
	}
}

void cake_io_init() {
	__SET_MEM(lcd_char_buf, ASCII_SPACE, sizeof(char) * MAX_LCD_CHAR_CNT);
	__SET_MEM(par_mem, 0, sizeof(uint16_t) * MAX_PAR_MEM_SIZE);
	par_mem[PAR_ADDR_FW_VER] = FW_VERSION;
	*(par_mem + PAR_ADDR_GATE_CMD_MAX) = DEFAULT_GATE_CMD_MAX;
	*(par_mem + PAR_ADDR_GATE_CMD_MIN) = 0;

	ow_cnt = dt_init(&ow, &dt, &OW_UART_HANDLE);
	tofs_init(tofs);
	mcp_init(mcps);
	lcdInit(&I2C_HANDLE, (uint8_t) LCD_I2C_ADDR, (uint8_t) 4, (uint8_t) 20);
	oven_thermo_init(&oven_thermo);

	HAL_ADC_Start_DMA(&ADC_HANDLE, analog_datas, MAX_ANALOG_DATA_SIZE);

	HAL_TIM_Base_Start(&TIMER_HANDLE);
	pwm_init(pwms);

	mb_srv_ctx = modbus_server_init(&mb_srv, MODBUS_DEVICE_ADDR, mb_srv_rx_buf,
	MAX_PORTOCOL_BUF_SIZE, mb_srv_tx_buf, MAX_PORTOCOL_BUF_SIZE);
	__HAL_UART_ENABLE_IT(&PROTOCOL_UART_HANDLE, UART_IT_IDLE); // enable idle line interrupt
	HAL_UART_Receive_DMA(&PROTOCOL_UART_HANDLE, rx_buf, MAX_PORTOCOL_BUF_SIZE);
}

void cake_io_cyclic() {
	start = HAL_GetTick();

	get_ow_temperature(&dt, par_mem, ow_par_add_table);
	tof_cyclic(tofs, par_mem);
	*(par_mem + PAR_ADDR_OVEN_TEMP) = get_oven_temperature(&oven_thermo);

	mcp_cyclic(mcps, par_mem);
	*(par_mem + PAR_ADDR_TP_JOG_SPD) = (uint16_t)analog_datas[0];

	stm32_dio_cyclic(stm32_gpios, par_mem);

	pwm_cyclic(pwms, par_mem);

	sprintf(lcd_char_buf, "%d %d %d %d %d %d %d %d %d %d %d",
			*(par_mem + PAR_ADDR_TP_MODE),
			*(par_mem + PAR_ADDR_TP_JOG_SPD),
			*(par_mem + PAR_ADDR_TP_JOG_TARGET),
			*(par_mem + PAR_ADDR_TP_JOG_DIR),
			*(par_mem + PAR_ADDR_ALARM_RESET),
			*(par_mem + PAR_ADDR_CLEAN_1),
			*(par_mem + PAR_ADDR_CLEAN_2),
			*(par_mem + PAR_ADDR_CLEAN_3),
			*(par_mem + PAR_ADDR_BOWL_READY),
			*(par_mem + PAR_ADDR_FLOW_1_CNT),
			*(par_mem + PAR_ADDR_GATE_CMD));
	set_lcd(lcd_char_buf);

	end = HAL_GetTick();
	*(par_mem + PAR_ADDR_SYS_CYCLIC_TIME) = end - start;
	*(par_mem + PAR_ADDR_SYS_HEARTBEAT) = ~*(par_mem + PAR_ADDR_SYS_HEARTBEAT);
}

void set_flash(uint32_t dst, uint8_t *p_src, uint32_t size) {
	HAL_FLASH_Unlock();
	//the size will be 2^N
	for (uint32_t i = 0; i < (size >> 3); i++) {
		uint32_t add = dst + i;
		uint64_t data = *((uint64_t*) (p_src + i));
		//double word is 64bit
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, add, data);
	}
	HAL_FLASH_Lock();
}

void get_flash(uint8_t *p_dst, uint32_t src, uint32_t size) {
	memcpy((void*) p_dst, (void*) src, size);
}

uint8_t dt_init(OneWire_HandleTypeDef *p_ow,
		DallasTemperature_HandleTypeDef *p_dt, UART_HandleTypeDef *p_uart) {
	uint8_t cnt = 0;
	CurrentDeviceAddress address;
	OW_Begin(p_ow, p_uart);
	OW_Reset(p_ow);
	DT_SetOneWire(p_dt, p_ow);
	DT_Begin(p_dt);
	cnt = DT_GetDeviceCount(p_dt);
	if (cnt > MAX_OW_CNT) {
		cnt = MAX_OW_CNT;
	}
	for (uint8_t i = 0; i < cnt; ++i) {
		if (DT_GetAddress(&dt, address, i)) {
			DT_SetResolution(&dt, address, 12, true);
		}
	}
	return cnt;
}

void get_ow_temperature(DallasTemperature_HandleTypeDef *p_dt, uint16_t *p_mem,
		const uint16_t *p_add_table) {
	DT_RequestTemperatures(p_dt);
	for (uint8_t i = 0; i < ow_cnt; ++i) {
		*(p_mem + *(p_add_table + i)) = (uint16_t) round(
				DT_GetTempCByIndex(p_dt, i));
	}
}

void set_lcd(char *p_buf) {
	lcdCursorHome();
	lcdPrintStr((uint8_t*) p_buf, MAX_LCD_CHAR_CNT_PER_LINE);
	lcdSetCursorPosition(0, 1);
	lcdPrintStr((uint8_t*) (p_buf + MAX_LCD_CHAR_CNT_PER_LINE),
	MAX_LCD_CHAR_CNT_PER_LINE);
}

uint16_t tofs_init(struct TOF *p_tofs) {
	uint16_t ret = 0;
	uint8_t vhv_setting = 0;
	uint8_t phase_cal = 0;
	uint32_t ref_spad_cnt = 0;
	uint8_t is_apar_spad = 0;
	FixPoint1616_t signal_limit = (FixPoint1616_t) (0.25 * 65536);
	FixPoint1616_t sigma_limit = (FixPoint1616_t) (18 * 65536);
	uint32_t timing_budget = 33000;
	uint8_t prerange_vcsel_period = 14;
	uint8_t final_range_vcsel_period = 10;
//	for (struct TOF *p_tof = p_tofs; p_tof->dev.I2cHandle != NULL; p_tof++) {
//		HAL_GPIO_WritePin(p_tof->x_pin_port, p_tof->x_pin, GPIO_PIN_RESET);
//	}
//	HAL_Delay(50);
	for (struct TOF *p_tof = p_tofs; p_tof->dev.I2cHandle != NULL; p_tof++) {
//		HAL_GPIO_WritePin(p_tof->x_pin_port, p_tof->x_pin, GPIO_PIN_SET);
//		HAL_Delay(50);
		p_tof->dev.api_st = VL53L0X_RdWord(&p_tof->dev,
		VL53L0X_REG_IDENTIFICATION_MODEL_ID, &p_tof->dev.model_id);
		if (p_tof->dev.api_st != VL53L0X_ERROR_NONE) {
			ret = 1;
			break;
		}
		if (p_tof->dev.target_i2c_addr != p_tof->dev.I2cDevAddr) {
			p_tof->dev.api_st = VL53L0X_SetDeviceAddress(&p_tof->dev,
					p_tof->dev.target_i2c_addr);
			if (p_tof->dev.api_st != VL53L0X_ERROR_NONE) {
				ret = 2;
				break;
			}
			p_tof->dev.I2cDevAddr = p_tof->dev.target_i2c_addr;
		}
		p_tof->dev.api_st = VL53L0X_DataInit(&p_tof->dev);
		if (p_tof->dev.api_st != VL53L0X_ERROR_NONE) {
			ret = 3;
			break;
		}
		p_tof->dev.api_st = VL53L0X_StaticInit(&p_tof->dev);
		if (p_tof->dev.api_st != VL53L0X_ERROR_NONE) {
			ret = 4;
			break;
		}
		p_tof->dev.api_st = VL53L0X_GetDeviceInfo(&p_tof->dev,
				&p_tof->dev.basic_info);
		if (p_tof->dev.api_st != VL53L0X_ERROR_NONE) {
			ret = 5;
			break;
		}
		p_tof->dev.api_st = VL53L0X_PerformRefCalibration(&p_tof->dev, &vhv_setting,
				&phase_cal);
		if (p_tof->dev.api_st) {
			ret = 6;
			break;
		}
		p_tof->dev.api_st = VL53L0X_PerformRefSpadManagement(&p_tof->dev,
				&ref_spad_cnt, &is_apar_spad);
		if (p_tof->dev.api_st) {
			ret = 7;
			break;
		}
		p_tof->dev.api_st = VL53L0X_SetDeviceMode(&p_tof->dev,
		VL53L0X_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode
		if (p_tof->dev.api_st) {
			ret = 8;
			break;
		}
		p_tof->dev.api_st = VL53L0X_SetLimitCheckEnable(&p_tof->dev,
		VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1); // Enable Sigma limit
		if (p_tof->dev.api_st) {
			ret = 9;
			break;
		}
		p_tof->dev.api_st = VL53L0X_SetLimitCheckEnable(&p_tof->dev,
		VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1); // Enable Signa limit
		if (p_tof->dev.api_st) {
			ret = 10;
			break;
		}
		/* Ranging configuration */
		switch (p_tof->dev.mode) {
		default:
		case 0:
			/* LONG_RANGE*/
			signal_limit = (FixPoint1616_t) (0.1 * 65536);
			sigma_limit = (FixPoint1616_t) (60 * 65536);
			timing_budget = 33000;
			prerange_vcsel_period = 18;
			final_range_vcsel_period = 14;
			break;
		case 1:
			/* HIGH_ACCURACY */
			signal_limit = (FixPoint1616_t) (0.25 * 65536);
			sigma_limit = (FixPoint1616_t) (18 * 65536);
			timing_budget = 200000;
			prerange_vcsel_period = 14;
			final_range_vcsel_period = 10;
			break;
		case 2:
			/* HIGH_SPEED */
			signal_limit = (FixPoint1616_t) (0.25 * 65536);
			sigma_limit = (FixPoint1616_t) (32 * 65536);
			timing_budget = 20000;
			prerange_vcsel_period = 14;
			final_range_vcsel_period = 10;
			break;
		}
		p_tof->dev.api_st = VL53L0X_SetLimitCheckValue(&p_tof->dev,
		VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, signal_limit);
		if (p_tof->dev.api_st) {
			ret = 11;
			break;
		}
		p_tof->dev.api_st = VL53L0X_SetLimitCheckValue(&p_tof->dev,
		VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, sigma_limit);
		if (p_tof->dev.api_st) {
			ret = 12;
			break;
		}
		p_tof->dev.api_st = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(
				&p_tof->dev, timing_budget);
		if (p_tof->dev.api_st) {
			ret = 13;
			break;
		}
		p_tof->dev.api_st = VL53L0X_SetVcselPulsePeriod(&p_tof->dev,
		VL53L0X_VCSEL_PERIOD_PRE_RANGE, prerange_vcsel_period);
		if (p_tof->dev.api_st) {
			ret = 14;
			break;
		}
		p_tof->dev.api_st = VL53L0X_SetVcselPulsePeriod(&p_tof->dev,
		VL53L0X_VCSEL_PERIOD_FINAL_RANGE, final_range_vcsel_period);
		if (p_tof->dev.api_st) {
			ret = 15;
			break;
		}
		p_tof->dev.api_st = VL53L0X_PerformRefCalibration(&p_tof->dev, &vhv_setting,
				&phase_cal);
		if (p_tof->dev.api_st) {
			ret = 16;
			break;
		}
		p_tof->dev.LeakyFirst = 1;
		p_tof->dev.Present = 1;
	}
	return ret;
}

void tof_cyclic(struct TOF *p_tofs, uint16_t *p_mem) {
	for (struct TOF *p_tof = p_tofs; p_tof->dev.I2cHandle != NULL; p_tof++) {
		p_tof->dev.api_st = VL53L0X_PerformSingleRangingMeasurement(&p_tof->dev,
				&p_tof->dev.meas_data);
		*(par_mem + p_tof->mem_addr) = p_tof->dev.meas_data.RangeMilliMeter;
	}
}

void oven_thermo_init(ThermoCouple *p_thermo) {
	p_thermo->Thx_CS_Pin = OVEN_THERMO_CS_PIN;
	p_thermo->Thx_CS_Port = OVEN_THERMO_CS_PORT;
	p_thermo->hspi = &OVEN_THERMO_HANDLE;
}

uint16_t get_oven_temperature(ThermoCouple *p_thermo) {
	uint16_t temp = 0;
	ReadThermoCouple(p_thermo);
	if (p_thermo->connected) {
		temp = (uint16_t) round(p_thermo->Thx_celcius);
	}
	return temp;
}

mbus_t modbus_server_init(Modbus_Conf_t *p_mb_srv, uint8_t devaddr,
		uint8_t *p_rx_buf, uint16_t rx_buf_size, uint8_t *p_tx_buf,
		uint16_t tx_buf_size) {
	mbus_t mbus = 0;
	/* Device slave address */
	p_mb_srv->devaddr = devaddr;
	/* Device count coils(discrete outputs) */
	p_mb_srv->coils = 8;
	/* Device count discrete (discrete inputs) */
	p_mb_srv->discrete = 8;
	/* Just ptr on any external object, you can get it by context */
	p_mb_srv->device = (void*) 0;
	/* This buffer for data content when we sending data */
	p_mb_srv->sendbuf = p_tx_buf;
	p_mb_srv->sendbuf_sz = tx_buf_size;
	/* This that function for sending some data (use sendbuf for buf) */
	p_mb_srv->send = user_mbus_send;
	/* This buffer for data content when we recv some data (no more then sz) */
	p_mb_srv->recvbuf = p_rx_buf;
	p_mb_srv->recvbuf_sz = rx_buf_size;
	//Read callback function ( read by logical address)
	p_mb_srv->read = user_mbus_read;
	//Write callback function
	p_mb_srv->write = user_mbus_write;
	mbus = mbus_open(p_mb_srv);
	return mbus;
}

uint16_t user_mbus_read(uint32_t la) {
	uint32_t addr = __TO_LOGIC_ADDRESS(la);
	uint16_t value = 0;
	if (addr < MAX_PAR_MEM_SIZE) {
		value = *(par_mem + addr);
	}
	return value;
}

uint16_t user_mbus_write(uint32_t la, uint16_t value) {
	uint32_t addr = __TO_LOGIC_ADDRESS(la);
	if (addr < MAX_PAR_MEM_SIZE) {
		*(par_mem + addr) = value;
	}
	return value;
}

int user_mbus_send(const mbus_t mb_ctx, const uint8_t *data,
		const uint16_t size) {
	HAL_UART_Transmit(&PROTOCOL_UART_HANDLE, data, size, 1);
	return 0;
}

void mcp_init(struct MCP23017 *p_mcps) {
	for (struct MCP23017 *p_mcp = p_mcps; p_mcp->p_i2c != NULL; p_mcp++) {
		mcp23017_init(&p_mcp->hdev, p_mcp->p_i2c, p_mcp->i2c_addr);
		mcp23017_iodir(&p_mcp->hdev, MCP23017_PORTA, p_mcp->port_a_iodir);
		mcp23017_iodir(&p_mcp->hdev, MCP23017_PORTB, p_mcp->port_b_iodir);
	}
}

void set_do(uint16_t cmd, uint8_t *p_data, uint8_t bit_idx) {
	if (cmd > 0) {
		__SET_BIT_TO_1(*p_data, bit_idx);
	} else {
		__SET_BIT_TO_0(*p_data, bit_idx);
	}
}

void mcp_cyclic(struct MCP23017 *p_mcps, uint16_t *p_mem) {
	for (struct MCP23017 *p_mcp = p_mcps; p_mcp->p_i2c != NULL; p_mcp++) {
		for (int i = 0; i < MAX_MCP23017_BIT_SIZE; i++) {
			struct MCP_DIO *p_cfg = p_mcp->dio_cfg + i;
			if (p_cfg->valid == true) {
				if (p_cfg->is_output == true) {
					set_do(*(p_mem + p_cfg->mem_addr),
							&p_mcp->hdev.gpio[p_cfg->port], p_cfg->bit_idx);
				}
			}
		}
		mcp23017_write_gpio(&p_mcp->hdev, MCP23017_PORTA);
		mcp23017_write_gpio(&p_mcp->hdev, MCP23017_PORTB);
	}
}

void stm32_dio_cyclic(struct STM32_GPIO *p_cfgs, uint16_t *p_mem) {
	for (struct STM32_GPIO *p_cfg = p_cfgs; p_cfg->port != NULL; p_cfg++) {
		if (p_cfg->is_output == false) {
			if (p_cfg->mem_bit_idx < 0) {
				*(p_mem + p_cfg->mem_addr) = (uint16_t) HAL_GPIO_ReadPin(
						p_cfg->port, p_cfg->pin);
			} else {
				set_do((uint16_t) HAL_GPIO_ReadPin(p_cfg->port, p_cfg->pin),
					   (p_mem + p_cfg->mem_addr),
					   p_cfg->mem_bit_idx);
			}
		}
	}
}

void pwm_init(struct PWM *p_pwms) {
	for (struct PWM *p_pwm = p_pwms; p_pwm->p_tim != NULL; p_pwm++) {
		HAL_TIM_PWM_Start(p_pwm->p_tim, p_pwm->ch);
	}
}

void pwm_cyclic(struct PWM *p_pwms, const uint16_t *p_mem) {
	for (struct PWM *p_pwm = p_pwms; p_pwm->p_tim != NULL; p_pwm++) {
		uint16_t cmd = *(p_mem + p_pwm->mem_addr);
		uint16_t max = *(par_mem + p_pwm->max_addr);
		uint16_t min = *(par_mem + p_pwm->min_addr);
		if (cmd > max) cmd = max;
		else if (cmd < min) cmd = min;
		__HAL_TIM_SetCompare(p_pwm->p_tim, p_pwm->ch, cmd);
	}
}
