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

#include <nuttx/spi/spi.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/nxffs.h>
#include <arch/board/board.h>
#include <systemlib/px4_macros.h>
#include <nuttx/mmcsd.h>

#include <arm_internal.h>
#include <chip.h>
#include <stm32_gpio.h>
#include "board_config.h"

#include <px4_arch/spi_hw_description.h>
#include <drivers/drv_sensor.h>
#include <nuttx/spi/spi.h>

constexpr px4_spi_bus_t px4_spi_buses[SPI_BUS_MAX_BUS_ITEMS] = {
	initSPIBus(SPI::Bus::SPI1, {
		initSPIDevice(DRV_IMU_DEVTYPE_ICM42688P, SPI::CS{GPIO::PortA, GPIO::Pin4}, SPI::DRDY{GPIO::PortE, GPIO::Pin14}),
	}),
	initSPIBus(SPI::Bus::SPI2, {
		initSPIDevice(SPIDEV_FLASH(0), SPI::CS{GPIO::PortB, GPIO::Pin12}),
	}),
	initSPIBus(SPI::Bus::SPI3, {
		initSPIDevice(DRV_OSD_DEVTYPE_ATXXXX, SPI::CS{GPIO::PortA, GPIO::Pin15}),
	}),
	initSPIBus(SPI::Bus::SPI4, {
		initSPIDevice(SPIDEV_MMCSD(0), SPI::CS{GPIO::PortE, GPIO::Pin12}),
	}),
};

static constexpr bool unused = validateSPIConfig(px4_spi_buses);

/************************************************************************************
 * Name: stm32_spi_bus_initialize
 *
 * Description:
 *   Called to configure SPI buses on PX4FMU board.
 *
 ************************************************************************************/
static struct spi_dev_s *spi_expansion;

__EXPORT int stm32_spi_bus_initialize(void)
{
	struct spi_dev_s *spi;
	struct mtd_dev_s *mtd;
	int ret = OK;
	#ifdef CONFIG_FS_NXFFS
	char devname[12];
	#endif

	/* Configure SPI-based devices */

	/* Configure MMCSD on SPI4 */
	/* Get the external SPI port */
	spi_expansion = stm32_spibus_initialize(4);

	if (!spi_expansion) {
		syslog(LOG_ERR, "[boot] FAILED to initialize SPI port %d\n", 1);
		return -ENODEV;
	}

	#ifdef CONFIG_MMCSD
		ret = mmcsd_spislotinitialize(CONFIG_NSH_MMCSDMINOR, CONFIG_NSH_MMCSDSLOTNO, spi_expansion);

		if (ret != OK) {
			syslog(LOG_ERR, "[boot] FAILED to bind SPI port 1 to the MMCSD driver\n");
			return -ENODEV;
		}

	#endif

	/* Configure Flash on SPI2 */
	spi = stm32_spibus_initialize(2);
	if (!spi)
	{
		ferr("ERROR: Failed to initialize SPI port 2\n");
		return -ENODEV;
	}

	/* Now bind the SPI interface to the W25 SPI FLASH driver */

	mtd = w25_initialize(spi);
	if (!mtd)
	{
		ferr("ERROR: Failed to bind SPI port 2 to the SST 25 FLASH driver\n");
		return -ENODEV;
	}

	#ifndef CONFIG_FS_NXFFS
		/* And use the FTL layer to wrap the MTD driver as a block driver */

		ret = ftl_initialize(0, mtd);
		if (ret < 0)
		{
			ferr("ERROR: Initialize the FTL layer\n");
			return ret;
		}
	#else
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

	#endif

	return OK;
}
