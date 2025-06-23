/****************************************************************************
 *
 *   Copyright (C) 2019-2022 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/px4_config.h>
#include <px4_platform/gpio.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <unistd.h>
#include <cstdio>

#include <nuttx/spi/spi.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/nxffs.h>
#include <arch/board/board.h>
#include <systemlib/px4_macros.h>

#include <arm_internal.h>
#include <chip.h>
#include <stm32_gpio.h>
#include "board_config.h"

#include <px4_arch/spi_hw_description.h>
#include <arch/board/board.h>
#include <px4_platform_common/log.h>
#include <drivers/drv_sensor.h>
#include <nuttx/spi/spi.h>

#ifndef MODULE_NAME
#define MODULE_NAME "spi_init"
#endif

constexpr px4_spi_bus_t px4_spi_buses[SPI_BUS_MAX_BUS_ITEMS] = {
	initSPIBus(SPI::Bus::SPI1, {
		initSPIDevice(DRV_IMU_DEVTYPE_ICM42688P, SPI::CS{GPIO::PortA, GPIO::Pin4}, SPI::DRDY{GPIO::PortE, GPIO::Pin14}),
	}),
	initSPIBus(SPI::Bus::SPI2, {
		initSPIDevice(SPIDEV_FLASH(0), SPI::CS{GPIO::PortB, GPIO::Pin12}),
	}),
};

static constexpr bool unused = validateSPIConfig(px4_spi_buses);

/************************************************************************************
 * Name: stm32_spi_bus_initialize
 *
 * Description:
 *   Called to configure SPI buses on LUX F765 - NDAA board.
 *
 ************************************************************************************/
__EXPORT int stm32_spi_bus_initialize(void)
{
	struct spi_dev_s *spi;
	struct mtd_dev_s *mtd;
	int ret = OK;
	#if defined(CONFIG_FS_NXFFS) || defined(CONFIG_FS_LITTLEFS)
	char devname[12];
	#endif

	/* Configure SPI-based devices */
	/* Configure Flash on SPI2 */
	spi = stm32_spibus_initialize(FLASH_SPI_BUS);
	if (!spi)
	{
		ferr("ERROR: Failed to initialize SPI port 2\n");
		return -ENODEV;
	}

	/* Now bind the SPI interface to the W25 SPI FLASH driver */

	mtd = w25_initialize(spi);
	if (!mtd)
	{
		ferr("ERROR: Failed to bind SPI port 2 to the W25 FLASH driver\n");
		return -ENODEV;
	}


	#ifdef CONFIG_FS_NXFFS
		/* Initialize to provide NXFFS on the MTD interface */

		ret = nxffs_initialize(mtd);
		if (ret < 0)
		{
			ferr("ERROR: NXFFS initialization failed: %d\n", -ret);
			return ret;
		}

		/* Mount the file system at /mnt/w25 */

		snprintf(devname, 12, "/mnt/w25%c", 'a' + 0);
		ret = nx_mount(NULL, devname, "nxffs", 0, NULL);
		if (ret < 0)
		{
			ferr("ERROR: Failed to mount the NXFFS volume: %d\n", ret);
			return ret;
		}

	#elif defined(CONFIG_FS_LITTLEFS)
		/* Initialize to provide LittleFS on the MTD interface */
		/* Configure the device with no partition support */

		snprintf(devname, sizeof(devname), "/dev/w25%s","lfs");

		ret = register_mtddriver(devname, mtd, 0755, NULL);

		if (ret != OK) {
			PX4_ERR("register_mtddriver() failed: %d", ret);

		} else {
			ret = nx_mount(devname, "/mnt/w25q", "littlefs", 0, NULL);

			if (ret < 0) {
				ret = nx_mount(devname, "/mnt/w25q", "littlefs", 0,
						"forceformat");

				if (ret < 0) {
					PX4_ERR("W25 mount failure: %d", ret);

				} else {
					PX4_INFO("W25 was forceformatted!");
				}
			}
		}
	#else
		/* And use the FTL layer to wrap the MTD driver as a block driver */
		ret = ftl_initialize(0, mtd);
		if (ret < 0)
		{
			ferr("ERROR: Initialize the FTL layer\n");
			return ret;
		}
	#endif

	return OK;
}
