/*
 * Copyright (c) 2017 IpTronix S.r.l.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __VM3011_H__
#define __VM3011_H__

#include <zephyr/types.h>
#include <device.h>
#include <drivers/gpio.h>
#include <drivers/i2c.h>
// #include "nrfx_pdm.h"


/* User-programmable Registers */
#define VM3011_REG_I2C_CNTRL                 0x00
#define VM3011_REG_WOS_PGA_GAIN              0x01
#define VM3011_REG_WOS_FILTER                0x02
#define VM3011_REG_WOS_PGA_MIN_THR           0x03
#define VM3011_REG_WOS_PGA_MAX_THR           0x04
#define VM3011_REG_WOS_THRESHOLD             0x05

#define VM3011_DOUT_CLEAR_Pos               (4)
#define VM3011_WDT_DELAY_Pos             	(1)
#define VM3011_WOS_BANDPASS_HPF_Pos         (2)
#define VM3011_FAST_MODE_CNT_Pos            (5)
#define VM3011_WOS_RMS_Pos                  (5)

#define VM3011_DOUT_CLEAR_Msk               (0x01 << 4)
#define VM3011_WDT_DELAY_Msk             	(0x03 << 1)
#define VM3011_WDT_ENABLE_Msk               (0x01 << 0)

#define VM3011_WOS_PGA_GAIN_Msk             (0x1F << 0)
#define VM3011_WOS_BANDPASS_LPF_Msk         (0x03 << 0)
#define VM3011_WOS_BANDPASS_HPF_Msk         (0x03 << 2)

#define VM3011_FAST_MODE_CNT_Msk            (0x03 << 5)
#define VM3011_WOS_PGA_MIN_THR_Msk          (0x1F << 0)

#define VM3011_WOS_RMS_Msk                  (0x01 << 5)
#define VM3011_WOS_PGA_MAX_THR_Msk          (0x1F << 0)

#define VM3011_WOS_THRESHOLD_Msk            (0x07 << 0)

#if (CONFIG_VM3011_MONO)
#define VM3011_PDM_MODE NRF_PDM_MODE_MONO
#else
#define VM3011_PDM_MODE NRF_PDM_MODE_STEREO
#endif /* CONFIG_VM3011_MONO */

#if (CONFIG_VM3011_WDT_ENABLE)

#if defined(CONFIG_VM3011_WDT_DELAY_8_MS)
#define VM3011_WDT_DELAY		0x00
#elif defined(CONFIG_VM3011_WDT_DELAY_16_MS)
#define VM3011_WDT_DELAY		0x01
#elif defined(CONFIG_VM3011_WDT_DELAY_32_MS)
#define VM3011_WDT_DELAY		0x02
#elif defined(CONFIG_VM3011_WDT_DELAY_64_MS)
#define VM3011_WDT_DELAY		0x03
#endif

#else
#define VM3011_WDT_DELAY		0x00
#endif /* CONFIG_VM3011_WDT_ENABLE */

#if (CONFIG_VM3011_FAST_MODE_COUNT_ENABLE)

#if defined(CONFIG_VM3011_WINDOW_COMP_2_TRIPS)
#define VM3011_FAST_MODE_CNT		0x01
#elif defined(CONFIG_VM3011_WINDOW_COMP_4_TRIPS)
#define VM3011_FAST_MODE_CNT		0x02
#elif defined(CONFIG_VM3011_WINDOW_COMP_6_TRIPS)
#define VM3011_FAST_MODE_CNT		0x03
#endif

#else
/* 0x00 to disable */
#define VM3011_FAST_MODE_CNT		0x00
#endif /* CONFIG_VM3011_WDT_ENABLE */

#if defined(CONFIG_VM3011_LPF_2_KHZ)
#define VM3011_LPF_FREQUENCY		0x00
#elif defined(CONFIG_VM3011_LPF_4_KHZ)
#define VM3011_LPF_FREQUENCY		0x01
#elif defined(CONFIG_VM3011_LPF_6_KHZ)
#define VM3011_LPF_FREQUENCY		0x02
#elif defined(CONFIG_VM3011_LPF_8_KHZ)
#define VM3011_LPF_FREQUENCY		0x03
#else
#define VM3011_LPF_FREQUENCY		0x00
#endif

#if defined(CONFIG_VM3011_HPF_200_KHZ)
#define VM3011_HPF_FREQUENCY		0x00
#elif defined(CONFIG_VM3011_HPF_300_KHZ)
#define VM3011_HPF_FREQUENCY		0x01
#elif defined(CONFIG_VM3011_HPF_400_KHZ)
#define VM3011_HPF_FREQUENCY		0x02
#elif defined(CONFIG_VM3011_HPF_800_KHZ)
#define VM3011_HPF_FREQUENCY		0x03
#else
#define VM3011_HPF_FREQUENCY		0x03
#endif

/* Programmable Gain Amplifier gain LUT calculator */
#define VM3011_PGA_MAX_DB            91.5
#define VM3011_PGA_DB_STEPS          1.5
#define VM3011_PGA_TO_DB(x)          VM3011_PGA_MAX_DB - ((float)((x) & VM3011_WOS_PGA_GAIN_Msk) * VM3011_PGA_DB_STEPS)
#define VM3011_DB_TO_PGA(x)          (uint8_t)((VM3011_PGA_MAX_DB - (x)) / VM3011_PGA_DB_STEPS)   

#define VM3011_STANDBY_MODE_MIN 200000UL
#define VM3011_STANDBY_MODE_MAX 250000UL

#define VM3011_LOW_POWER_MODE_MIN 350000UL
#define VM3011_LOW_POWER_MODE_MAX 900000UL

#define VM3011_NORMAL_MODE_MIN 1100000UL
#define VM3011_NORMAL_MODE_MAX 4000000UL

typedef enum{
	ZPL,
	STANDBY,
	LOW_POWER,
	NORMAL
}dmic_op_mode;

typedef enum{
	HZ_4800 = 4800UL,
    HZ_6000 = 6000UL,
    HZ_8000 = 8000UL,
    HZ_9600 = 9600UL,
    HZ_11025 = 11025UL,
    HZ_12000 = 12000UL,
    HZ_16000 = 16000UL,
    HZ_32000 = 32000UL,
    HZ_44100 = 44100UL,
    HZ_48000 = 48000UL
}dmic_out_sample_rate;

struct vm3011_config {
    const char *i2c_dev_label;
	uint16_t i2c_address;

    gpio_pin_t data_pin;
    gpio_pin_t clk_pin;
    bool lr_pin_level;

#if defined(CONFIG_VM3011_INT)
    const char *gpio_port;
    gpio_pin_t dout_pin;
    gpio_dt_flags_t dout_flags;
#endif
};

struct vm3011_data{
	const struct device *i2c_dev;
	uint16_t i2c_address;

    dmic_op_mode vm3011_current_mode;
    nrfx_pdm_config_t vm3011_pdm_configs;
};

#endif /* __VM3011_H__ */
