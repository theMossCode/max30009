/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

#include "max30009/max30009.h"

int main(void)
{
	const uint8_t fifo_afull_threshold = 5;
	uint8_t status_1 = 0;
	uint8_t bioz_fifo_raw[3];
	bool pll_locked = false;
	max30009_conf_t default_conf = F_BIOZ_500KHZ_CONF;

	if(max30009_init(&default_conf)){
		return 1;
	}

	if(max30009_set_fifo_afull_threshold(fifo_afull_threshold)){
		return 1;
	}

	if(max30009_bioz_enable_mux()){
		return 1;
	}

	if(max30009_conf_assign_electrodes(MAX30009_BIP_ASSIGN_EL2A, MAX30009_BIN_ASSIGN_EL3A, 
										MAX30009_DRVP_ASSIGN_EL1, MAX30009_DRVN_ASSIGN_EL4)){
		return 1;
	}

	if(max30009_set_stimulus_mode(MAX30009_STIMULUS_MODE_SINEWAVE_VOLTAGE)){
		return 1;
	}

	if(max30009_enable_pll(true)){
		return 1;
	}

	while(1){

		if(max30009_get_status_1(&status_1)){
			continue;
		}

		printk("Status 1: 0x%02x", status_1);

		if((status_1 & (1 << FREQ_LOCK_SHIFT) || status_1 & (1 << PHASE_LOCK_SHIFT)) && !pll_locked){
			pll_locked = true;
			if(max30009_start_bioz()){
				printk("Start BioZ measurement fail");
				return 1;
			}

			printk("PLL locked ");
		}
		else{
			pll_locked = false;

			printk("PLL Sync lost");
		}

		if(pll_locked){
			if(status_1 & (1 << FIFO_DATA_RDY_SHIFT) && !(status_1 & (1 << PWR_RDY_SHIFT))){
				if(max30009_read_fifo(bioz_fifo_raw, sizeof(bioz_fifo_raw))){
					continue;
				}

				printk("Fifo Raw %02x, %02x, %02x", bioz_fifo_raw[0], bioz_fifo_raw[1], bioz_fifo_raw[2]);

				int32_t fifo_data = 0;
				int fifo_tag = max30009_parse_fifo_data(bioz_fifo_raw, &fifo_data);
				printk("FIFO tag %d, data %d", fifo_tag, fifo_data);
			}
		}

		k_sleep(K_MSEC(100));
	}

	return 0;
}
