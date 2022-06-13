/* adxl362.c - ADXL362 Three-Axis Digital Accelerometers */

/*
 * Copyright (c) 2017 IpTronix S.r.l.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT vesper_vm3011

#include <kernel.h>
#include <string.h>
#include <init.h>
#include <drivers/gpio.h>
#include <sys/byteorder.h>
#include <sys/__assert.h>
#include <drivers/i2c.h>
#include <audio/dmic.h>
#include "nrfx_pdm.h"

#include "vm3011.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(vm3011, LOG_LEVEL_DBG);

#define PDM_PERIPHERAL_RATIO_64 64UL
#define PDM_PERIPHERAL_RATIO_80 80UL

static struct vm3011_data vm3011_data;
static int16_t pcm_buffer[2][CONFIG_VM3011_PDM_BUFFER_SIZE];
static uint8_t pcm_buffer_in_use = 0;
static bool pcm_buffer_avalable = false;

static const struct vm3011_config vm3011_cfg = {
	.i2c_dev_label = DT_INST_BUS_LABEL(0),
	.i2c_address = DT_INST_REG_ADDR(0),
	.data_pin = DT_INST_PROP(0, data_pin),
	.clk_pin = DT_INST_PROP(0, clk_pin),
	.lr_pin_level = DT_INST_PROP(0, lr_select),
#if defined(CONFIG_VM3011_INT)
	.gpio_port = DT_INST_GPIO_LABEL(0, dout_gpios),
	.dout_pin = DT_INST_GPIO_PIN(0, dout_gpios),
	.dout_flags = DT_INST_GPIO_FLAGS(0, dout_gpios),
#endif
};

static inline int vm3011_get_reg(const struct device *dev, uint8_t register_adress, uint8_t *read_buf)
{
	const struct vm3011_data *dmic_data = dev->data;
	return i2c_reg_read_byte(dmic_data->i2c_dev, dmic_data->i2c_address, register_adress, read_buf);
}

static inline int vm3011_set_reg(const struct device *dev, uint8_t register_adress, uint8_t value)
{
	const struct vm3011_data *dmic_data = dev->data;
	return i2c_reg_write_byte(dmic_data->i2c_dev, dmic_data->i2c_address, register_adress, value);
}

static int vm3011_clear_dout(const struct device *dev)
{
	uint8_t value = 0xFF & VM3011_DOUT_CLEAR_Msk;
	uint8_t buff;
	int ret;

	ret = vm3011_get_reg(dev, VM3011_REG_I2C_CNTRL, &buff);
	if (ret < 0) {
		LOG_ERR("Couldn't read register %d, error %d", VM3011_REG_I2C_CNTRL, ret);
		return ret;
	}

	buff |= value;
	ret = vm3011_set_reg(dev, VM3011_REG_I2C_CNTRL, buff);
	if (ret < 0) {
		LOG_ERR("Couldn't write register %d, error %d", VM3011_REG_I2C_CNTRL, ret);
		return ret;
	}

	return 0;
}

static int vm3011_wdt_configure(const struct device *dev, uint8_t wdt_dly, bool enable)
{
	uint8_t value = (wdt_dly << VM3011_WDT_DELAY_Pos) & VM3011_WDT_DELAY_Msk;
	uint8_t buff;
	int ret;

	if(enable){
		value |= 0xFF & VM3011_WDT_ENABLE_Msk;
	}else{
		value |= 0x0 & VM3011_WDT_ENABLE_Msk;
	}

	ret = vm3011_get_reg(dev, VM3011_REG_I2C_CNTRL, &buff);
	if (ret < 0) {
		LOG_ERR("Couldn't read register %d, error %d", VM3011_REG_I2C_CNTRL, ret);
		return ret;
	}

	buff &= ~(VM3011_WDT_DELAY_Msk | VM3011_WDT_ENABLE_Msk);
	buff |= value;
	ret = vm3011_set_reg(dev, VM3011_REG_I2C_CNTRL, buff);
	if (ret < 0) {
		LOG_ERR("Couldn't write register %d, error %d", VM3011_REG_I2C_CNTRL, ret);
		return ret;
	}

	return 0;
}

static int vm3011_wos_pga_min_thr_set(const struct device *dev, float pga_gain_db)
{
	uint8_t pga_min_gain;
	uint8_t buff;
	int ret;

	ret = vm3011_get_reg(dev, VM3011_REG_WOS_PGA_MIN_THR, &buff);
	if (ret < 0) {
		LOG_ERR("Couldn't read register %d, error %d", VM3011_REG_WOS_PGA_MIN_THR, ret);
		return ret;
	}

	pga_min_gain = VM3011_DB_TO_PGA(pga_gain_db);

	buff &= ~VM3011_WOS_PGA_MIN_THR_Msk;
	buff |= pga_min_gain & VM3011_WOS_PGA_MIN_THR_Msk;

	ret = vm3011_set_reg(dev, VM3011_REG_WOS_PGA_MIN_THR, buff);
	if (ret < 0) {
		LOG_ERR("Couldn't write register %d, error %d", VM3011_REG_WOS_PGA_MIN_THR, ret);
		return ret;
	}

	return 0;
}

static int vm3011_wos_pga_max_thr_set(const struct device *dev, float pga_gain_db)
{
	uint8_t pga_max_gain;
	uint8_t buff;
	int ret;

	ret = vm3011_get_reg(dev, VM3011_REG_WOS_PGA_MAX_THR, &buff);
	if (ret < 0) {
		LOG_ERR("Couldn't read register %d, error %d", VM3011_REG_WOS_PGA_MAX_THR, ret);
		return ret;
	}

	pga_max_gain = VM3011_DB_TO_PGA(pga_gain_db);
	
	buff &= ~VM3011_WOS_PGA_MAX_THR_Msk;
	buff |= pga_max_gain & VM3011_WOS_PGA_MAX_THR_Msk;

	ret = vm3011_set_reg(dev, VM3011_REG_WOS_PGA_MAX_THR, buff);
	if (ret < 0) {
		LOG_ERR("Couldn't write register %d, error %d", VM3011_REG_WOS_PGA_MAX_THR, ret);
		return ret;
	}

	return 0;
}

static int vm3011_bpf_set( const struct device *dev, uint8_t wos_lpf_freq, uint8_t wos_hpf_freq)
{
	uint8_t buff;
	int ret;

	buff = (wos_lpf_freq & VM3011_WOS_BANDPASS_LPF_Msk) |
		   ((wos_hpf_freq << VM3011_WOS_BANDPASS_HPF_Pos) & VM3011_WOS_BANDPASS_HPF_Msk);

	ret = vm3011_set_reg(dev, VM3011_REG_WOS_FILTER, buff);
	if (ret < 0) {
		LOG_ERR("Couldn't write register %d, error %d", VM3011_REG_WOS_FILTER, ret);
		return ret;
	}

	return 0;

}

static int vm3011_fast_mode_cnt_set( const struct device *dev, uint8_t fast_mode_cnt)
{
	uint8_t buff;
	int ret;

	ret = vm3011_get_reg(dev, VM3011_REG_WOS_PGA_MIN_THR, &buff);
	if (ret < 0) {
		LOG_ERR("Couldn't read register %d, error %d", VM3011_REG_WOS_PGA_MIN_THR, ret);
		return ret;
	}

	buff &= ~VM3011_FAST_MODE_CNT_Msk;
	buff |= (fast_mode_cnt << VM3011_FAST_MODE_CNT_Pos) & VM3011_FAST_MODE_CNT_Msk;

	ret = vm3011_set_reg(dev, VM3011_REG_WOS_PGA_MIN_THR, buff);
	if (ret < 0) {
		LOG_ERR("Couldn't write register %d, error %d", VM3011_REG_WOS_PGA_MIN_THR, ret);
		return ret;
	}

	return 0;
}

static int vm3011_wos_rms_set( const struct device *dev, uint8_t value)
{
	uint8_t buff;
	int ret;

	ret = vm3011_get_reg(dev, VM3011_REG_WOS_PGA_MAX_THR, &buff);
	if (ret < 0) {
		LOG_ERR("Couldn't read register %d, error %d", VM3011_REG_WOS_PGA_MAX_THR, ret);
		return ret;
	}

	buff &= ~VM3011_WOS_RMS_Msk;
	buff |= (value << VM3011_WOS_RMS_Pos) & VM3011_WOS_RMS_Msk;

	ret = vm3011_set_reg(dev, VM3011_REG_WOS_PGA_MAX_THR, buff);
	if (ret < 0) {
		LOG_ERR("Couldn't write register %d, error %d", VM3011_REG_WOS_PGA_MAX_THR, ret);
		return ret;
	}

	return 0;
}

static int vm3011_wos_dout_thresh_set( const struct device *dev, uint8_t wos_thresh)
{
	uint8_t buff;
	int ret;

	/* WOS_THRESH register value should not be 0 */
	if(!wos_thresh)
	{
		LOG_WRN("Invalid WOS_THRESH value");
		return -EINVAL;
	}

	buff = wos_thresh & VM3011_WOS_THRESHOLD_Msk;

	ret = vm3011_set_reg(dev, VM3011_REG_WOS_THRESHOLD, buff);
	if (ret < 0) {
		LOG_ERR("Couldn't write register %d, error %d", VM3011_REG_WOS_THRESHOLD, ret);
		return ret;
	}

	return 0;
}

static int vm3011_amb_sound_lvl_get(const struct device *dev, float *amb_sound_lvl)
{
	uint8_t buff;
	int ret;

	ret = vm3011_get_reg(dev, VM3011_REG_WOS_PGA_GAIN, &buff);
	if (ret < 0) {
		LOG_ERR("Couldn't read register %d, error %d", VM3011_REG_WOS_PGA_GAIN, ret);
		return ret;
	}

	/* Translate PGA gain to DB using the LUT */
	*amb_sound_lvl = VM3011_PGA_TO_DB(buff);


	return 0;
}

static void vm3011_pdm_handler(nrfx_pdm_evt_t const * p_evt)
{
	uint32_t ret;
	if(p_evt->error != NRFX_PDM_NO_ERROR)
	{
		LOG_ERR("PDM buffer overflow");
	}

	if(p_evt->buffer_requested){
		pcm_buffer_in_use ^= 1;
		ret = nrfx_pdm_buffer_set(&pcm_buffer[pcm_buffer_in_use][0], CONFIG_VM3011_PDM_BUFFER_SIZE);
		if(ret != NRFX_SUCCESS)
		{
			LOG_ERR("Failed to set buffer, error %d", ret);
		}
	}
	if(p_evt->buffer_released){
		pcm_buffer_avalable = true;
		//do something with released buffer
	}
}

static int vm3011_configure(const struct device *dev,
				 struct dmic_cfg *cfg)
{
	int ret;
	struct vm3011_data *dmic_data = dev->data;
	const struct vm3011_config *config = dev->config;
	uint32_t audio_sample_rate;

	/* Initialise with default parameters */
	nrfx_pdm_config_t pdm_param = 
			NRFX_PDM_DEFAULT_CONFIG(config->clk_pin, config->data_pin);

	/* Configure NRFX PDM struct parameters */
	pdm_param.mode = VM3011_PDM_MODE;
	if(pdm_param.mode == NRF_PDM_MODE_STEREO){
		/* If stereo, left channel always on falling CLK edge */
		pdm_param.gain_l = NRF_PDM_GAIN_DEFAULT;
		pdm_param.gain_r = NRF_PDM_GAIN_DEFAULT;
		pdm_param.edge = NRF_PDM_EDGE_LEFTFALLING;
	}else{
		/* If mono, choose rising or falling CLK edge 
		 * based on the lr_pin_level */
		if(!config->lr_pin_level){
			/* If LOW, the dmic works as a LEFT channel only */
			pdm_param.gain_l = NRF_PDM_GAIN_MAXIMUM;
			pdm_param.edge = NRF_PDM_EDGE_LEFTFALLING;
		}else{
			/* If HIGH, the dmic works as a RIGHT channel only */
			pdm_param.gain_r = NRF_PDM_GAIN_MAXIMUM;
			pdm_param.edge = NRF_PDM_EDGE_LEFTRISING;
		}
	}

	audio_sample_rate = cfg->streams->pcm_rate;

	switch (audio_sample_rate)
	{
		case HZ_4800:
			dmic_data->vm3011_current_mode = LOW_POWER;
		#if NRF_PDM_HAS_RATIO_CONFIG
			/* For 4.8kHz we need ratio of 80x.
			* Otherwise we will put the sensor in STANDBY mode.
			* Resulting clock frequency is about 384kHz */
			pdm_param.ratio = NRF_PDM_RATIO_80X;
		#endif
			pdm_param.clock_freq = 0x030DB000UL;
			break;

		case HZ_6000:
			dmic_data->vm3011_current_mode = LOW_POWER;
		#if NRF_PDM_HAS_RATIO_CONFIG
			/* Resulting clock frequency is about 384kHz */
			pdm_param.ratio = NRF_PDM_RATIO_64X;
		#endif
			pdm_param.clock_freq = 0x030DB000UL;
			break;

		case HZ_8000:
			dmic_data->vm3011_current_mode = LOW_POWER;
		#if NRF_PDM_HAS_RATIO_CONFIG
			/* Resulting clock frequency is about 512kHz */
			pdm_param.ratio = NRF_PDM_RATIO_64X;
		#endif
			pdm_param.clock_freq = 0x04104000UL;
			break;

		case HZ_9600:
			dmic_data->vm3011_current_mode = LOW_POWER;
		#if NRF_PDM_HAS_RATIO_CONFIG
			/* Resulting clock frequency is about 614.4kHz */
			pdm_param.ratio = NRF_PDM_RATIO_64X;
		#endif
			pdm_param.clock_freq = 0x04DE5000UL;
			break;

		case HZ_11025:
			dmic_data->vm3011_current_mode = LOW_POWER;
		#if NRF_PDM_HAS_RATIO_CONFIG
			/* Resulting clock frequency is about 705.6kHz */
			pdm_param.ratio = NRF_PDM_RATIO_64X;
		#endif
			pdm_param.clock_freq = 0x05954000UL;
			break;

		case HZ_12000:
			dmic_data->vm3011_current_mode = LOW_POWER;
		#if NRF_PDM_HAS_RATIO_CONFIG
			/* Resulting clock frequency is about 768kHz */
			pdm_param.ratio = NRF_PDM_RATIO_64X;
		#endif
			pdm_param.clock_freq = 0x06123000UL;
			break;

		case HZ_16000:
			dmic_data->vm3011_current_mode = NORMAL;
		#if NRF_PDM_HAS_RATIO_CONFIG
			/* 1.28 MHz sampling with ratio 80x
			* This is equal to 16kHz output sample rate */
			pdm_param.ratio = NRF_PDM_RATIO_80X;
		#endif
			pdm_param.clock_freq = NRF_PDM_FREQ_1280K;
			break;

		case HZ_32000:
			dmic_data->vm3011_current_mode = NORMAL;
		#if NRF_PDM_HAS_RATIO_CONFIG
			/* Resulting clock frequency is about 2.048MHz */
			pdm_param.ratio = NRF_PDM_RATIO_64X;
		#endif
			pdm_param.clock_freq = 0x0FE03000UL;
			break;

		case HZ_44100:
			dmic_data->vm3011_current_mode = NORMAL;
		#if NRF_PDM_HAS_RATIO_CONFIG
			/* Resulting clock frequency is about 2.822MHz */
			pdm_param.ratio = NRF_PDM_RATIO_64X;
		#endif
			pdm_param.clock_freq = 0x15A02000UL;
			break;

		case HZ_48000:
			dmic_data->vm3011_current_mode = NORMAL;
		#if NRF_PDM_HAS_RATIO_CONFIG
			/* Resulting clock frequency is about 3.072MHz */
			pdm_param.ratio = NRF_PDM_RATIO_64X;
		#endif
			pdm_param.clock_freq = 0x17734000UL;
			break;

		default:
			break;
	}

	// nrfx_pdm_uninit();

	ret = nrfx_pdm_init(&pdm_param, vm3011_pdm_handler);
	if (ret != NRFX_SUCCESS) {
		LOG_ERR("Failed to inititialise the PDM peripheral, error %d", ret);
		return ret;
	}

	if(!irq_is_enabled(PDM0_IRQn)){
		irq_enable(PDM0_IRQn);
	}

	/* Copy the pdm struct in internal vm3011 data struct */
	memcpy(&dmic_data->vm3011_pdm_configs, &pdm_param, sizeof(pdm_param));

	return 0;
}

static int vm3011_get_command(const struct device *dev,
			       enum dmic_trigger cmd)
{
	int ret = 0;
	struct vm3011_data *dmic_data = dev->data;
	nrfx_pdm_config_t *pdm_param = &dmic_data->vm3011_pdm_configs;
	
	switch (cmd) {

		case DMIC_TRIGGER_STOP:
			/* Disable the pdm sampling and enter ZPL */
			ret = nrfx_pdm_stop();
			if (ret != NRFX_SUCCESS) {
				LOG_ERR("Failed to stop PDM sampling, error %d", ret);
                return ret;
			}
			dmic_data->vm3011_current_mode = ZPL;
			break;

		case DMIC_TRIGGER_PAUSE:
			/* Enter Standby mode */
			nrfx_pdm_uninit();

		#if NRF_PDM_HAS_RATIO_CONFIG
			pdm_param->ratio = NRF_PDM_RATIO_64X;
		#endif
			/* About 220kHz. Clock frequencies between
			 * 200kHz - 250kHz place the device in Standby. */
			pdm_param->clock_freq = 0x01C10000UL;

			ret = nrfx_pdm_init(pdm_param, vm3011_pdm_handler);
			if (ret != NRFX_SUCCESS) {
				LOG_ERR("Failed to inititialise the PDM peripheral, error %d", ret);
                return ret;
			}
			
			ret = nrfx_pdm_start();
			if (ret != NRFX_SUCCESS) {
				LOG_ERR("Failed to start PDM sampling, error %d", ret);
                return ret;
			}
			
			dmic_data->vm3011_current_mode = STANDBY;
			break;

		case DMIC_TRIGGER_START:
			/* The frequency is already set from vm3011_configure.
			 * If vm3011_configure was not first called, do nothing. */
			if(dmic_data->vm3011_current_mode == NORMAL || 
			   dmic_data->vm3011_current_mode == LOW_POWER){

				ret = nrfx_pdm_start();
				if (ret != NRFX_SUCCESS) {
					LOG_ERR("Failed to start PDM sampling, error %d", ret);
               		return ret;
				}
			}else{
				LOG_WRN("Must configure the vm3011 before starting sampling");
			}
			break;

		case DMIC_TRIGGER_RESET:
			//disable clock
			//clear buffer
			//clear dout
			ret = vm3011_clear_dout(dev);
			if (ret < 0) {
				LOG_ERR("Couldn't clear dout, error %d", ret);
           		return ret;

			}
			break;	

		default:
			break;
	}
	return 0;
}

static int vm3011_read_pdm(const struct device *dev, uint8_t stream,
			    void **buffer,
			    size_t *size, int32_t timeout)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(stream);
	ARG_UNUSED(timeout);
	if(pcm_buffer_avalable) {
		/* Return pointer to 0th address of sampled buffer */
		*buffer = &pcm_buffer[pcm_buffer_in_use^1][0];
		/* Return the size of the buffer in bytes */
		*size = sizeof(pcm_buffer[0]);
		pcm_buffer_avalable = false;
	}
	else {
		*buffer = NULL;
		*size = 0;
	}
	return 0;
}
static const struct _dmic_ops vm3011_api_funcs = {
	.configure = vm3011_configure,
	.trigger = vm3011_get_command,
	.read  = vm3011_read_pdm,
};

static int vm3011_chip_init(const struct device *dev)
{
	uint8_t buff;
	int ret;

	/* Wait for ZPL to settle properly before
	 * initiating communication. */
	k_sleep(K_MSEC(1000));

	/* As the device does not contain a chip ID
	 * register, read the I2C_CNTRL register.
	 * The value 0x00 should return. */
	ret = vm3011_get_reg(dev, VM3011_REG_I2C_CNTRL, &buff);
	if (ret < 0) {
		LOG_ERR("Couldn't read register %d, error %d", VM3011_REG_I2C_CNTRL, ret);
		return ret;
	}

	if(buff == 0x00){
		LOG_DBG("VM3011 dmic detected");
	}else{
		LOG_ERR("Device not found");
		return -ENXIO; 
	}
	
	ret = vm3011_wdt_configure(dev, VM3011_WDT_DELAY, IS_ENABLED(CONFIG_VM3011_WDT_ENABLE));
	if(ret){
		return ret;
	}

	ret = vm3011_bpf_set(dev, VM3011_LPF_FREQUENCY, VM3011_HPF_FREQUENCY);
	if(ret){
		return ret;
	}

	ret = vm3011_fast_mode_cnt_set(dev, VM3011_FAST_MODE_CNT);
	if(ret){
		return ret;
	}

	ret = vm3011_wos_rms_set(dev, CONFIG_VM3011_WOS_RMS_LPF);
	if(ret){
		return ret;
	}

	/* 45dB SPL is the average perceived loudness of a quiet library */
	ret = vm3011_wos_pga_min_thr_set(dev, (float)45);
	if(ret){
		return ret;
	}

	/* 90dB SPL is the average perceived loudness of a truck or loud speaker */
	ret = vm3011_wos_pga_max_thr_set(dev, (float)90);
	if(ret){
		return ret;
	}

	ret = vm3011_wos_dout_thresh_set(dev, CONFIG_VM3011_WOS_DOUT_THRESHOLD);
	if(ret){
		return ret;
	}

	return 0;
}

static int vm3011_init(const struct device *dev)
{
	const struct vm3011_config *config = dev->config;
	struct vm3011_data *dmic_data = dev->data;

	dmic_data->i2c_dev = device_get_binding(config->i2c_dev_label);
	if(!dmic_data->i2c_dev) {
		LOG_ERR("I2C device not found: %s", config->i2c_dev_label);
		return -EINVAL;
	}

	dmic_data->i2c_address = config->i2c_address;

	if (vm3011_chip_init(dev) < 0) {
		return -EINVAL;
	}

	dmic_data->vm3011_current_mode = ZPL;
	IRQ_CONNECT(PDM0_IRQn, 6, nrfx_pdm_irq_handler, NULL, 0);

	return 0;
}

DEVICE_DT_INST_DEFINE(0, vm3011_init, NULL,
		    &vm3011_data, &vm3011_cfg, POST_KERNEL,
		    CONFIG_AUDIO_DMIC_INIT_PRIORITY, &vm3011_api_funcs);
