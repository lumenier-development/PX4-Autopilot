/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file board_config.h
 *
 * holybro KakuteH7 internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#include <stm32_gpio.h>

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/

/* Lumenier H743 - NDAA GPIOs ************************************************************************/

/* LEDs are driven with push open drain to support Anode to 5V or 3.3V */

#define GPIO_nLED_BLUE  /* PE3 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_PULLUP|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN3)

#define BOARD_HAS_CONTROL_STATUS_LEDS	1
#define BOARD_ARMED_STATE_LED		LED_BLUE

/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that
 * can be used by the Px4 Firmware in the adc driver
 */

/* ADC defines to be used in sensors.cpp to read from a particular channel */

#define ADC1_CH(n)                  (n)

/* Define GPIO pins used as ADC N.B. Channel numbers must match below */
#define PX4_ADC_GPIO  \
	/* PC0 */  GPIO_ADC123_INP10_0, \
	/* PC1 */  GPIO_ADC123_INP11_0, \
	/* PC4 */  GPIO_ADC12_INP4_0, \
	/* PC5 */  GPIO_ADC12_INP8_0

/* Define Channel numbers must match above GPIO pin IN(n)*/
#define ADC_BATTERY_VOLTAGE_CHANNEL        /* PC0 */  ADC1_CH(10)
#define ADC_BATTERY_CURRENT_CHANNEL        /* PC1 */  ADC1_CH(11)
#define ADC_AIRSPEED_IN_CHANNEL            /* PC4 */  ADC1_CH(4)
#define ADC_RSSI_IN_CHANNEL                /* PC5 */  ADC1_CH(8)

#define ADC_CHANNELS \
	((1 << ADC_BATTERY_VOLTAGE_CHANNEL)       | \
	 (1 << ADC_BATTERY_CURRENT_CHANNEL)       | \
	 (1 << ADC_AIRSPEED_IN_CHANNEL)           | \
	 (1 << ADC_RSSI_IN_CHANNEL))

#define SYSTEM_ADC_BASE STM32_ADC1_BASE

/* By Providing BOARD_ADC_USB_CONNECTED (using the px4_arch abstraction)
 * this board support the ADC system_power interface, and therefore
 * provides the true logic GPIO BOARD_ADC_xxxx macros.
*/
#define BOARD_ADC_USB_CONNECTED (px4_arch_gpioread(GPIO_OTGFS_VBUS))

#define UAVCAN_NUM_IFACES_RUNTIME 1
#define UAVCAN_WAIT_MSR

/* PWM */
#define DIRECT_PWM_OUTPUT_CHANNELS  12
#define BOARD_HAS_PWM  DIRECT_PWM_OUTPUT_CHANNELS

/* Spare GPIO */
#define GPIO_VIDEO_PWR     /* PB2  */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN2)
#define GPIO_VIDEO_CAM     /* PB1  */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN1)

/* Tone alarm output */
#define GPIO_TONE_ALARM_IDLE    /* PA8 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN8)
#define GPIO_TONE_ALARM_GPIO    /* PA8 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN8)

/* High-resolution timer */
#define HRT_TIMER               2  /* use timer2 for the HRT */
#define HRT_TIMER_CHANNEL       1  /* use capture/compare channel 1 */
#define HRT_PPM_CHANNEL         /* T2C3 */  3  /* use capture/compare channel 3 */
#define GPIO_PPM_IN             /* PA2 T2C3 */ GPIO_TIM2_CH3IN_1

#define BOARD_NUM_IO_TIMERS	3

/* RC Serial port */

/* SD Card */
#define SDIO_SLOTNO                    0  /* Only one slot */
#define SDIO_MINOR                     0
#define MMCSD_DETECT	/* PE15 */     (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTE|GPIO_PIN15)
#define GPIO_SDMMC1_NCD		  	MMCSD_DETECT

/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE 5120

/* This board provides the board_on_reset interface */

#define BOARD_HAS_ON_RESET 1

#define PX4_GPIO_INIT_LIST { \
		PX4_ADC_GPIO,                     \
		GPIO_CAN1_TX,                     \
		GPIO_CAN1_RX,                     \
		GPIO_VIDEO_PWR,			  \
		GPIO_VIDEO_CAM,			  \
		GPIO_TONE_ALARM_IDLE,             \
		PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_I2C1_SCL), \
		PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_I2C1_SDA), \
		PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_I2C4_SCL), \
		PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_I2C4_SDA), \
	}

#define BOARD_ENABLE_CONSOLE_BUFFER

__BEGIN_DECLS

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

extern void stm32_spiinitialize(void);

extern int stm32_i2c_sensors_initialize(void);

extern int stm32_spi_bus_initialize(void);

extern void stm32_usbinitialize(void);

extern void board_peripheral_reset(int ms);

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
