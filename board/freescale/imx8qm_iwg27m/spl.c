/*
 * Copyright 2020 iWave System Technologies Pvt Ltd.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <dm.h>
#include <spl.h>
#include <dm/uclass.h>
#include <dm/device.h>
#include <dm/uclass-internal.h>
#include <dm/device-internal.h>
#include <dm/lists.h>
#include <bootm.h>

DECLARE_GLOBAL_DATA_PTR;

/* IWG27M: MIPI-DSI: Correcting Reset Power Sequence for 1080p LCD */
void lcd_reset(void);

void spl_board_init(void)
{
	struct udevice *dev;

	uclass_find_first_device(UCLASS_MISC, &dev);

	for (; dev; uclass_find_next_device(&dev)) {
		if (device_probe(dev))
			continue;
	}

	board_early_init_f();

	/* IWG27M: MIPI-DSI: Correcting Reset Power Sequence for 1080p LCD */
	lcd_reset();

	timer_init();

	preloader_console_init();

	puts("Normal Boot\n");
}

void spl_board_prepare_for_boot(void)
{
	board_quiesce_devices();
}

#ifdef CONFIG_SPL_LOAD_FIT
int board_fit_config_name_match(const char *name)
{
	/* Just empty function now - can't decide what to choose */
	debug("%s: %s\n", __func__, name);

	return 0;
}
#endif

void board_init_f(ulong dummy)
{
	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

	arch_cpu_init();

	board_init_r(NULL, 0);
}
