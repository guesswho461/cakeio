#ifndef __CAKE_IO_H__
#define __CAKE_IO_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

#define PAR_ADDR_FW_VER 		0x0000
#define PAR_ADDR_PAR_VER 		0x0001

#define PAR_ADDR_CUP_CNT 		0x0083
#define PAR_ADDR_BATTER_CNT 	0x0084
#define PAR_ADDR_OVEN_TEMP 		0x0085
#define PAR_ADDR_FRIG_TEMP 		0x0086
#define PAR_ADDR_MACH_TEMP 		0x0087

void cake_io_init();
void cake_io_cyclic();

#ifdef __cplusplus
}
#endif

#endif /* __CAKE_IO_H__ */
