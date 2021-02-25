#include "cake_io.h"

#include <string.h>
#include <stdio.h>
#include <math.h>

#include "usart.h"
#include "i2c.h"
#include "stm32f1xx_hal_flash.h"
#include "spi.h"
#include "stm32f1xx_hal_tim.h"

#include "OneWire.h"
#include "DallasTemperature.h"
#include "lcd_hd44780_i2c.h"
#include "vl53l0x_api.h"
#include "max6675.h"
#include "modbus.h"

/* PRIVATE DEFINE */
#define __HAL_DMA_SET_COUNTER(__HANDLE__, __COUNTER__)   ((__HANDLE__)->Instance->CNDTR = (uint16_t)(__COUNTER__))

#define MODBUS_DEVICE_ADDR 0x01
#define FW_VERSION 1
#define PAR_VERSION 1

#define MAX_OW_CNT 2 //has to be less than 5
#define MAX_LCD_CHAR_CNT_PER_LINE 16
#define MAX_LCD_CHAR_CNT (MAX_LCD_CHAR_CNT_PER_LINE << 1)

#define PAR_MEM_SIZE 512 //the size has to be 2^N
#define PAR_MEM_FLASH_ADD 0x0801FC00

#define PROTOCOL_UART_HANDLE huart2
#define PROTOCOL_UART_INST USART2
#define PROTOCOL_UART_DMA_RX_HANDLE hdma_usart2_rx
#define OW_UART_HANDLE huart3
#define I2C_HANDLE hi2c2
#define OVEN_THERMO_HANDLE hspi2
#define OVEN_THERMO_CS_PIN SPI2_CS_Pin
#define OVEN_THERMO_CS_PORT SPI2_CS_GPIO_Port

#define CUP_TOF_I2C_ADDR 0x52
#define BATTER_TOF_I2C_ADDR 0x53

#define SET_TO_ZERO(ptr, size) memset((void *)ptr, 0, size);
#define ASCII_SPACE 32

#define ROTOCOL_BUF_SIZE 256

/* PRIVATE DEFINE */

/* PRIVATE PAR */
const uint16_t ow_par_add_table[MAX_OW_CNT] = { PAR_ADDR_FRIG_TEMP,
PAR_ADDR_MACH_TEMP };

uint16_t par_mem[PAR_MEM_SIZE];

OneWire_HandleTypeDef ow;
DallasTemperature_HandleTypeDef dt;

uint8_t ow_cnt = 0;
char lcd_char_buf[MAX_LCD_CHAR_CNT] = { ASCII_SPACE };

VL53L0X_Dev_t tofs[] = { { .I2cHandle = &I2C_HANDLE, .I2cDevAddr = 0x52,
		.target_i2c_addr = CUP_TOF_I2C_ADDR, .comms_type = 1, .comms_speed_khz =
				100, .mode = 1 }, NULL };

ThermoCouple oven_thermo;

uint32_t start, end, duration;

Modbus_Conf_t mb_srv;
mbus_t mb_srv_ctx;
uint8_t mb_srv_rx_buf[ROTOCOL_BUF_SIZE];
uint8_t mb_srv_tx_buf[ROTOCOL_BUF_SIZE];

uint8_t rx_buf[ROTOCOL_BUF_SIZE];
uint8_t rx_len;

/* PRIVATE PAR */

/* PRIVATE FUNCTION PROTOTYPE */

void par_mem_init();
uint8_t dt_init(OneWire_HandleTypeDef *p_ow,
		DallasTemperature_HandleTypeDef *p_dt, UART_HandleTypeDef *p_uart);
void get_ow_temperature(DallasTemperature_HandleTypeDef *p_dt, uint16_t *p_mem,
		const uint16_t *p_add_table);
uint32_t protocol_parser(uint8_t *p_uart_rx_buf, uint32_t size);
void set_flash(uint32_t dst, uint8_t *p_src, uint32_t size);
void get_flash(uint8_t *p_dst, uint32_t src, uint32_t size);
void set_lcd(char *p_buf);
uint16_t tofs_init(VL53L0X_Dev_t *p_tofs);
void oven_thermo_init(ThermoCouple *p_thermo);
uint16_t get_oven_temperature(ThermoCouple *p_thermo);
mbus_t modbus_server_init(Modbus_Conf_t *p_mb_srv, uint8_t devaddr,
		uint8_t *p_rx_buf, uint16_t rx_buf_size, uint8_t *p_tx_buf,
		uint16_t tx_buf_size);
uint16_t user_mbus_read(uint32_t la);
uint16_t user_mbus_write(uint32_t la, uint16_t value);
int user_mbus_send(const mbus_t mb_ctx, const uint8_t *data,
		const uint16_t size);

/* PRIVATE FUNCTION PROTOTYPE */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) {
		__HAL_UART_CLEAR_IDLEFLAG(huart);
		__HAL_DMA_DISABLE(huart->hdmarx);

		rx_len = ROTOCOL_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);

		for (int i = 0; i < rx_len; ++i) {
			mbus_poll(mb_srv_ctx, rx_buf[i]);
		}

		__HAL_DMA_SET_COUNTER(huart->hdmarx, ROTOCOL_BUF_SIZE);
		__HAL_DMA_ENABLE(huart->hdmarx);
	}
}

void cake_io_init() {
	par_mem_init();
	memset((void*) lcd_char_buf, ASCII_SPACE, MAX_LCD_CHAR_CNT);

	ow_cnt = dt_init(&ow, &dt, &OW_UART_HANDLE);
	tofs_init(tofs);
	lcdInit(&I2C_HANDLE, (uint8_t) 0x27, (uint8_t) 4, (uint8_t) 20);
	oven_thermo_init(&oven_thermo);
	mb_srv_ctx = modbus_server_init(&mb_srv, MODBUS_DEVICE_ADDR, mb_srv_rx_buf,
	ROTOCOL_BUF_SIZE, mb_srv_tx_buf, ROTOCOL_BUF_SIZE);

	__HAL_UART_ENABLE_IT(&PROTOCOL_UART_HANDLE, UART_IT_IDLE); // enable idle line interrupt
	HAL_UART_Receive_DMA(&PROTOCOL_UART_HANDLE, rx_buf,
	ROTOCOL_BUF_SIZE);
}

void cake_io_cyclic() {
	start = HAL_GetTick();

	get_ow_temperature(&dt, par_mem, ow_par_add_table);
	tofs[0].api_st = VL53L0X_PerformSingleRangingMeasurement(&tofs[0],
			&tofs[0].meas_data);
	*(par_mem + PAR_ADDR_CUP_CNT) = tofs[0].meas_data.RangeMilliMeter;
	*(par_mem + PAR_ADDR_OVEN_TEMP) = get_oven_temperature(&oven_thermo);

	duration = end - start;

	sprintf(lcd_char_buf, "%d %d %d %d %ld",
			*(par_mem + *(ow_par_add_table + 0)),
			*(par_mem + *(ow_par_add_table + 1)), *(par_mem + PAR_ADDR_CUP_CNT),
			*(par_mem + PAR_ADDR_OVEN_TEMP), duration);
	set_lcd(lcd_char_buf);

	end = HAL_GetTick();
}

void par_mem_init() {
	SET_TO_ZERO(par_mem, sizeof(uint16_t) * PAR_MEM_SIZE);
	par_mem[PAR_ADDR_FW_VER] = FW_VERSION;
	par_mem[PAR_ADDR_PAR_VER] = PAR_VERSION;
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

uint16_t tofs_init(VL53L0X_Dev_t *p_tofs) {
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
	for (VL53L0X_Dev_t *p_tof = p_tofs; p_tof->I2cHandle != NULL; p_tof++) {
		p_tof->api_st = VL53L0X_RdWord(p_tof,
		VL53L0X_REG_IDENTIFICATION_MODEL_ID, &p_tof->model_id);
		if (p_tof->api_st != VL53L0X_ERROR_NONE) {
			ret = 1;
			break;
		}
		if (p_tof->target_i2c_addr != p_tof->I2cDevAddr) {
			p_tof->api_st = VL53L0X_SetDeviceAddress(p_tof,
					p_tof->target_i2c_addr);
			if (p_tof->api_st != VL53L0X_ERROR_NONE) {
				ret = 2;
				break;
			}
			p_tof->I2cDevAddr = p_tof->target_i2c_addr;
		}
		p_tof->api_st = VL53L0X_DataInit(p_tof);
		if (p_tof->api_st != VL53L0X_ERROR_NONE) {
			ret = 3;
			break;
		}
		p_tof->api_st = VL53L0X_StaticInit(p_tof);
		if (p_tof->api_st != VL53L0X_ERROR_NONE) {
			ret = 4;
			break;
		}
		p_tof->api_st = VL53L0X_GetDeviceInfo(p_tof, &p_tof->basic_info);
		if (p_tof->api_st != VL53L0X_ERROR_NONE) {
			ret = 5;
			break;
		}
		p_tof->api_st = VL53L0X_PerformRefCalibration(p_tof, &vhv_setting,
				&phase_cal);
		if (p_tof->api_st) {
			ret = 6;
			break;
		}
		p_tof->api_st = VL53L0X_PerformRefSpadManagement(p_tof, &ref_spad_cnt,
				&is_apar_spad);
		if (p_tof->api_st) {
			ret = 7;
			break;
		}
		p_tof->api_st = VL53L0X_SetDeviceMode(p_tof,
		VL53L0X_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode
		if (p_tof->api_st) {
			ret = 8;
			break;
		}
		p_tof->api_st = VL53L0X_SetLimitCheckEnable(p_tof,
		VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1); // Enable Sigma limit
		if (p_tof->api_st) {
			ret = 9;
			break;
		}
		p_tof->api_st = VL53L0X_SetLimitCheckEnable(p_tof,
		VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1); // Enable Signa limit
		if (p_tof->api_st) {
			ret = 10;
			break;
		}
		/* Ranging configuration */
		switch (p_tof->mode) {
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
		p_tof->api_st = VL53L0X_SetLimitCheckValue(p_tof,
		VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, signal_limit);
		if (p_tof->api_st) {
			ret = 1;
			break;
		}
		p_tof->api_st = VL53L0X_SetLimitCheckValue(p_tof,
		VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, sigma_limit);
		if (p_tof->api_st) {
			ret = 12;
			break;
		}
		p_tof->api_st = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(p_tof,
				timing_budget);
		if (p_tof->api_st) {
			ret = 13;
			break;
		}
		p_tof->api_st = VL53L0X_SetVcselPulsePeriod(p_tof,
		VL53L0X_VCSEL_PERIOD_PRE_RANGE, prerange_vcsel_period);
		if (p_tof->api_st) {
			ret = 14;
			break;
		}
		p_tof->api_st = VL53L0X_SetVcselPulsePeriod(p_tof,
		VL53L0X_VCSEL_PERIOD_FINAL_RANGE, final_range_vcsel_period);
		if (p_tof->api_st) {
			ret = 15;
			break;
		}
		p_tof->api_st = VL53L0X_PerformRefCalibration(p_tof, &vhv_setting,
				&phase_cal);
		if (p_tof->api_st) {
			ret = 16;
			break;
		}
		p_tof->LeakyFirst = 1;
		p_tof->Present = 1;
	}
	return ret;
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
	uint32_t addr = la - 40000;
	uint16_t value = 0;
	if (addr < PAR_MEM_SIZE) {
		value = *(par_mem + addr);
	}
	return value;
}

uint16_t user_mbus_write(uint32_t la, uint16_t value) {
	return value;
}

int user_mbus_send(const mbus_t mb_ctx, const uint8_t *data,
		const uint16_t size) {
	HAL_UART_Transmit(&PROTOCOL_UART_HANDLE, data, size, 1);
    return 0;
}
