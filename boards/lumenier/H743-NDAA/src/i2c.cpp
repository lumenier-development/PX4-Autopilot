/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
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

#ifndef MODULE_NAME
#define MODULE_NAME "i2c_init"
#endif

#define IIS2MDC_I2C_BUS_SPEED 400000 	// I2C bus speed for IIS2MDC
#define IIS2DMC_I2C_ADDR 0x1E 		// I2C address for IIS2MDC
#define IIS2MDC_ADDR_OFFSET_X_REG_L  0x45

#include <px4_arch/i2c_hw_description.h>
#include <px4_platform_common/log.h>
#include <arch/board/board.h>
#include <drivers/magnetometer/st/iis2mdc/iis2mdc.h>

constexpr px4_i2c_bus_t px4_i2c_buses[I2C_BUS_MAX_BUS_ITEMS] = {
	initI2CBusInternal(1),
	initI2CBusExternal(4),
};

/**
 * @brief Custom code for LUX H743-NDAA to initialize and calibrate the LIS2MDL magnetometer sensor.
 *
 * @param [in] NONE
 *
 * @return 0 on success, non-zero on failure.
 */
__EXPORT int stm32_i2c_sensors_initialize() {
	struct i2c_master_s *i2c1 = NULL;
	const struct i2c_config_s iis2mdc_config = {400000, IIS2DMC_I2C_ADDR, 7};

	uint8_t iis2mdc_offset_addr = IIS2MDC_ADDR_OFFSET_X_REG_L;
	uint8_t buffer[2] = {};

	i2c1 = stm32_i2cbus_initialize(1);
	if (i2c1 == NULL) {
		PX4_ERR("Failed to initialize I2C bus 1 for IIS2MDC");
		return 0;
	}

	// Set register A
	uint8_t data_to_write[2] = {IIS2MDC_ADDR_CFG_REG_A, MD_CONTINUOUS | ODR_100 | COMP_TEMP_EN};
	i2c_write(i2c1, &iis2mdc_config, data_to_write, 2);

	// Set register B
	data_to_write[0] = IIS2MDC_ADDR_CFG_REG_B;
	data_to_write[1] = OFF_CANC;
	i2c_write(i2c1, &iis2mdc_config, data_to_write, 2);

	// Set register C
	data_to_write[0] = IIS2MDC_ADDR_CFG_REG_C;
	data_to_write[1] = BDU;
	i2c_write(i2c1, &iis2mdc_config, data_to_write, 2);

	// Read the offset register
	i2c_writeread(i2c1, &iis2mdc_config, &iis2mdc_offset_addr, 1, buffer, 1);
	PX4_INFO("IIS2MDC offset before write: %02x", buffer[0]);

	/*
	data_to_write[0] = IIS2MDC_ADDR_OFFSET_X_REG_L;
	data_to_write[1] = 1;
	i2c_write(i2c1, &iis2mdc_config, data_to_write, 2);
	i2c_writeread(i2c1, &iis2mdc_config, &iis2mdc_offset_addr, 1, buffer, 1);
	PX4_INFO("IIS2MDC offset after write: %02x", buffer[0]);
	*/

	return 0;
}
