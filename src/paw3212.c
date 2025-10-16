/*/*

 * Copyright (c) 2022 The ZMK Contributors * Copyright (c) 2022 The ZMK Contributors

 * *

 * SPDX-License-Identifier: MIT * SPDX-License-Identifier: MIT

 */ */



#define DT_DRV_COMPAT pixart_paw3212#define DT_DRV_COMPAT pixart_paw3212



#include <zephyr/kernel.h>#include <zephyr/kernel.h>

#include <zephyr/sys/byteorder.h>#include <zephyr/sys/byteorder.h>

#include <zephyr/input/input.h>#include <zephyr/input/input.h>

#include <zephyr/pm/device.h>#include <zephyr/pm/device.h>

#include <zmk/keymap.h>#include <zmk/keymap.h>

#include <zmk/events/activity_state_changed.h>#include <zmk/events/activity_state_changed.h>

#include "paw3212.h"#include "paw3212.h"



#include <zephyr/logging/log.h>#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(paw3212, CONFIG_PAW3212_LOG_LEVEL);LOG_MODULE_REGISTER(paw3212, CONFIG_PAW3212_LOG_LEVEL);



//////// Sensor initialization steps definition ////////////////// Sensor initialization steps definition //////////

enum paw3212_init_step {// init is done in non-blocking manner (i.e., async), a //

	ASYNC_INIT_STEP_POWER_UP,// delayable work is defined for this purpose           //

	ASYNC_INIT_STEP_VERIFY_ID,enum paw3212_init_step {

	ASYNC_INIT_STEP_CONFIGURE,	ASYNC_INIT_STEP_POWER_UP,

	ASYNC_INIT_STEP_VERIFY_ID,

	ASYNC_INIT_STEP_COUNT	ASYNC_INIT_STEP_CONFIGURE,

};

	ASYNC_INIT_STEP_COUNT

/* Timings (in ms) needed in between steps */};

static const int32_t async_init_delay[ASYNC_INIT_STEP_COUNT] = {

	[ASYNC_INIT_STEP_POWER_UP]  = 1,/* Timings (in ms) needed in between steps to allow each step finishes succussfully. */

	[ASYNC_INIT_STEP_VERIFY_ID] = 0,// - Since MCU is not involved in the sensor init process, i is allowed to do other tasks.

	[ASYNC_INIT_STEP_CONFIGURE] = 0,//   Thus, k_sleep or delayed schedule can be used.

};static const int32_t async_init_delay[ASYNC_INIT_STEP_COUNT] = {

	[ASYNC_INIT_STEP_POWER_UP]  = 10,

static int paw3212_async_init_power_up(const struct device *dev);	[ASYNC_INIT_STEP_VERIFY_ID] = 0,

static int paw3212_async_init_verify_id(const struct device *dev);	[ASYNC_INIT_STEP_CONFIGURE] = 0,

static int paw3212_async_init_configure(const struct device *dev);};



static int (* const async_init_fn[ASYNC_INIT_STEP_COUNT])(const struct device *dev) = {static int paw3212_async_init_power_up(const struct device *dev);

	[ASYNC_INIT_STEP_POWER_UP]  = paw3212_async_init_power_up,static int paw3212_async_init_verify_id(const struct device *dev);

	[ASYNC_INIT_STEP_VERIFY_ID] = paw3212_async_init_verify_id,static int paw3212_async_init_configure(const struct device *dev);

	[ASYNC_INIT_STEP_CONFIGURE] = paw3212_async_init_configure,

};static int (* const async_init_fn[ASYNC_INIT_STEP_COUNT])(const struct device *dev) = {

	[ASYNC_INIT_STEP_POWER_UP]  = paw3212_async_init_power_up,

//////// Function definitions //////////	[ASYNC_INIT_STEP_VERIFY_ID] = paw3212_async_init_verify_id,

	[ASYNC_INIT_STEP_CONFIGURE] = paw3212_async_init_configure,

static int16_t expand_s12(int16_t x)};

{

	/* Left shifting of negative values is undefined behavior, so we cannot//////// Function definitions //////////

	 * depend on automatic integer promotion (it will convert int16_t to int).

	 * To expand sign we cast s16 to unsigned int and left shift it thenstatic int pmw3610_read(const struct device *dev, uint8_t addr, uint8_t *value, uint8_t len) {

	 * cast it to signed integer and do the right shift. Since type is	const struct pixart_config *cfg = dev->config;

	 * signed compiler will perform arithmetic shift. This last operation	const struct spi_buf tx_buf = { .buf = &addr, .len = sizeof(addr) };

	 * is implementation defined in C but defined in the compiler's	const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };

	 * documentation and common for modern compilers and CPUs.	struct spi_buf rx_buf[] = {

	 */		{ .buf = NULL, .len = sizeof(addr), },

	return ((signed int)((unsigned int)x << 20)) >> 20;		{ .buf = value, .len = len, },

}	};

	const struct spi_buf_set rx = { .buffers = rx_buf, .count = ARRAY_SIZE(rx_buf) };

static int spi_cs_ctrl(const struct device *dev, bool enable)	return spi_transceive_dt(&cfg->spi, &tx, &rx);

{}

	const struct pixart_config *config = dev->config;

	int err;static int pmw3610_read_reg(const struct device *dev, uint8_t addr, uint8_t *value) {

	return pmw3610_read(dev, addr, value, 1);

	if (!enable) {}

		k_busy_wait(T_NCS_SCLK);

	}static int pmw3610_write_reg(const struct device *dev, uint8_t addr, uint8_t value) {

	const struct pixart_config *cfg = dev->config;

	err = gpio_pin_set_dt(&config->cs_gpio, (int)enable);	uint8_t write_buf[] = {addr | SPI_WRITE_BIT, value};

	const struct spi_buf tx_buf = { .buf = write_buf, .len = sizeof(write_buf), };

	if (err) {	const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1, };

		LOG_ERR("SPI CS ctrl failed");	return spi_write_dt(&cfg->spi, &tx);

	}}



	if (enable) {static int pmw3610_write(const struct device *dev, uint8_t reg, uint8_t val) {

		k_busy_wait(T_NCS_SCLK);	pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_ENABLE);

	}	k_sleep(K_USEC(T_CLOCK_ON_DELAY_US));



	return err;    int err = pmw3610_write_reg(dev, reg, val);

}    if (unlikely(err != 0)) {

        return err;

static int reg_read(const struct device *dev, uint8_t reg, uint8_t *buf)    }

{

	int err;    pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_DISABLE);

	const struct pixart_config *config = dev->config;    return 0;

}

	__ASSERT_NO_MSG((reg & SPI_WRITE_BIT) == 0);

static int paw3212_set_cpi(const struct device *dev, uint32_t cpi)

	err = spi_cs_ctrl(dev, true);{

	if (err) {	int err;

		return err;

	}	if ((cpi > PAW3212_CPI_MAX) || (cpi < PAW3212_CPI_MIN)) {

		LOG_ERR("CPI %" PRIu32 " out of range", cpi);

	/* Write register address. */		return -EINVAL;

	const struct spi_buf tx_buf = {	}

		.buf = &reg,

		.len = 1	uint8_t regval = cpi / PAW3212_CPI_STEP;

	};

	const struct spi_buf_set tx = {	LOG_DBG("Set CPI: %u (requested: %u, reg:0x%" PRIx8 ")",

		.buffers = &tx_buf,		regval * PAW3212_CPI_STEP, cpi, regval);

		.count = 1

	};	err = reg_write(dev, PAW3212_REG_WRITE_PROTECT, PAW3212_WPMAGIC);

	if (err) {

	err = spi_write_dt(&config->spi, &tx);		LOG_ERR("Cannot disable write protect");

	if (err) {		return err;

		LOG_ERR("Reg read failed on SPI write");	}

		return err;

	}	err = reg_write(dev, PAW3212_REG_CPI_X, regval);

	if (err) {

	k_busy_wait(T_SRAD);		LOG_ERR("Failed to change x CPI");

		return err;

	/* Read register value. */	}

	struct spi_buf rx_buf = {

		.buf = buf,	err = reg_write(dev, PAW3212_REG_CPI_Y, regval);

		.len = 1,	if (err) {

	};		LOG_ERR("Failed to change y CPI");

	const struct spi_buf_set rx = {		return err;

		.buffers = &rx_buf,	}

		.count = 1,

	};	err = reg_write(dev, PAW3212_REG_WRITE_PROTECT, 0);

	if (err) {

	err = spi_read_dt(&config->spi, &rx);		LOG_ERR("Cannot enable write protect");

	if (err) {	}

		LOG_ERR("Reg read failed on SPI read");

		return err;	return err;

	}}



	err = spi_cs_ctrl(dev, false);static int pmw3610_set_axis(const struct device *dev, bool swap_xy, bool inv_x, bool inv_y) {

	if (err) {    LOG_INF("Setting axis swap_xy: %s inv_x: %s inv_y: %s",

		return err;            swap_xy ? "yes" : "no", inv_x ? "yes" : "no", inv_y ? "yes" : "no");

	}

    uint8_t value;

	k_busy_wait(T_SRX);    int err = pmw3610_read_reg(dev, PMW3610_REG_RES_STEP, &value);

    if (err) {

	return 0;        LOG_ERR("Can't read res step %d", err);

}        return err;

    }

static int reg_write(const struct device *dev, uint8_t reg, uint8_t val)    LOG_INF("Get res step register (reg value 0x%x)", value);

{

	int err;    // Convert axis to register value

	const struct pixart_config *config = dev->config;    // Set prefered RES_STEP

    //   BIT 7: SWAP_XY

	__ASSERT_NO_MSG((reg & SPI_WRITE_BIT) == 0);    //   BIT 6: INV_X

    //   BIT 5: INV_Y

	err = spi_cs_ctrl(dev, true);#if IS_ENABLED(CONFIG_PMW3610_SWAP_XY)

	if (err) {    value |= (1 << 7);

		return err;#else

	}    if (swap_xy) { value |= (1 << 7); } else { value &= ~(1 << 7); }

#endif

	uint8_t buf[] = {#if IS_ENABLED(CONFIG_PMW3610_INVERT_X)

		SPI_WRITE_BIT | reg,    value |= (1 << 6);

		val#else

	};    if (inv_x) { value |= (1 << 6); } else { value &= ~(1 << 6); }

	const struct spi_buf tx_buf = {#endif

		.buf = buf,#if IS_ENABLED(CONFIG_PMW3610_INVERT_Y)

		.len = ARRAY_SIZE(buf)    value |= (1 << 5);

	};#else

	const struct spi_buf_set tx = {    if (inv_y) { value |= (1 << 5); } else { value &= ~(1 << 5); }

		.buffers = &tx_buf,#endif

		.count = 1    LOG_INF("Setting RES_STEP to (reg value 0x%x)", value);

	};

    /* set the axis control register */

	err = spi_write_dt(&config->spi, &tx);    uint8_t addr[] = {0x7F, PMW3610_REG_RES_STEP, 0x7F};

	if (err) {    uint8_t data[] = {0xFF, value, 0x00};

		LOG_ERR("Reg write failed on SPI write");

		return err;	pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_ENABLE);

	}	k_sleep(K_USEC(T_CLOCK_ON_DELAY_US));



	k_busy_wait(T_SCLK_NCS_WR);    /* Write data */

    for (size_t i = 0; i < sizeof(data); i++) {

	err = spi_cs_ctrl(dev, false);        err = pmw3610_write_reg(dev, addr[i], data[i]);

	if (err) {        if (err) {

		return err;            LOG_ERR("Burst write failed on SPI write (data)");

	}            break;

        }

	k_busy_wait(T_SWX);    }

    pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_DISABLE);

	return 0;

}    if (err) {

        LOG_ERR("Failed to set axis");

static int paw3212_set_cpi(const struct device *dev, uint32_t cpi)        return err;

{    }

	int err;

    return 0;

	if ((cpi > PAW3212_CPI_MAX) || (cpi < PAW3212_CPI_MIN)) {}

		LOG_ERR("CPI %" PRIu32 " out of range", cpi);

		return -EINVAL;/* Set sampling rate in each mode (in ms) */

	}static int pmw3610_set_sample_time(const struct device *dev, uint8_t reg_addr, uint32_t sample_time) {

    uint32_t maxtime = 2550;

	uint8_t regval = cpi / PAW3212_CPI_STEP;    uint32_t mintime = 10;

    if ((sample_time > maxtime) || (sample_time < mintime)) {

	LOG_DBG("Set CPI: %u (requested: %u, reg:0x%" PRIx8 ")",        LOG_WRN("Sample time %u out of range [%u, %u]", sample_time, mintime, maxtime);

		regval * PAW3212_CPI_STEP, cpi, regval);        return -EINVAL;

    }

	err = reg_write(dev, PAW3212_REG_WRITE_PROTECT, PAW3212_WPMAGIC);

	if (err) {    uint8_t value = sample_time / mintime;

		LOG_ERR("Cannot disable write protect");    LOG_INF("Set sample time to %u ms (reg value: 0x%x)", sample_time, value);

		return err;

	}    /* The sample time is (reg_value * mintime ) ms. 0x00 is rounded to 0x1 */

    int err = pmw3610_write(dev, reg_addr, value);

	err = reg_write(dev, PAW3212_REG_CPI_X, regval);    if (err) {

	if (err) {        LOG_ERR("Failed to change sample time");

		LOG_ERR("Failed to change x CPI");    }

		return err;

	}    return err;

}

	err = reg_write(dev, PAW3212_REG_CPI_Y, regval);

	if (err) {/* Set downshift time in ms. */

		LOG_ERR("Failed to change y CPI");// NOTE: The unit of run-mode downshift is related to pos mode rate, which is hard coded to be 4 ms

		return err;// The pos-mode rate is configured in pmw3610_async_init_configure

	}static int pmw3610_set_downshift_time(const struct device *dev, uint8_t reg_addr, uint32_t time) {

    uint32_t maxtime;

	err = reg_write(dev, PAW3212_REG_WRITE_PROTECT, 0);    uint32_t mintime;

	if (err) {

		LOG_ERR("Cannot enable write protect");    switch (reg_addr) {

	}    case PMW3610_REG_RUN_DOWNSHIFT:

        /*

	return err;         * Run downshift time = PMW3610_REG_RUN_DOWNSHIFT

}         *                      * 8 * pos-rate (fixed to 4ms)

         */

static int paw3212_set_sleep_timeout(const struct device *dev, uint8_t reg_addr,        maxtime = 8160; // 32 * 255;

				uint32_t timeout_ms)        mintime = 32; // hard-coded in pmw3610_async_init_configure

{        break;

	uint32_t timeout_step_ms;

    case PMW3610_REG_REST1_DOWNSHIFT:

	switch (reg_addr) {        /*

	case PAW3212_REG_SLEEP1:         * Rest1 downshift time = PMW3610_REG_RUN_DOWNSHIFT

		timeout_step_ms = 32;         *                        * 16 * Rest1_sample_period (default 40 ms)

		break;         */

        maxtime = 255 * 16 * CONFIG_PMW3610_REST1_SAMPLE_TIME_MS;

	case PAW3212_REG_SLEEP2:        mintime = 16 * CONFIG_PMW3610_REST1_SAMPLE_TIME_MS;

	case PAW3212_REG_SLEEP3:        break;

		timeout_step_ms = 20480;

		break;    case PMW3610_REG_REST2_DOWNSHIFT:

        /*

	default:         * Rest2 downshift time = PMW3610_REG_REST2_DOWNSHIFT

		LOG_ERR("Not supported");         *                        * 128 * Rest2 rate (default 100 ms)

		return -ENOTSUP;         */

	}        maxtime = 255 * 128 * CONFIG_PMW3610_REST2_SAMPLE_TIME_MS;

        mintime = 128 * CONFIG_PMW3610_REST2_SAMPLE_TIME_MS;

	uint32_t etm = timeout_ms / timeout_step_ms - 1;        break;



	if ((etm < PAW3212_ETM_MIN) || (etm > PAW3212_ETM_MAX)) {    default:

		LOG_WRN("Sleep timeout %" PRIu32 " out of range", timeout_ms);        LOG_ERR("Not supported");

		return -EINVAL;        return -ENOTSUP;

	}    }



	LOG_DBG("Set sleep%d timeout: %u (requested: %u, reg:0x%" PRIx8 ")",    if ((time > maxtime) || (time < mintime)) {

		reg_addr - PAW3212_REG_SLEEP1 + 1,        LOG_WRN("Downshift time %u out of range (%u - %u)", time, mintime, maxtime);

		(etm + 1) * timeout_step_ms,        return -EINVAL;

		timeout_ms,    }

		etm);

    __ASSERT_NO_MSG((mintime > 0) && (maxtime / mintime <= UINT8_MAX));

	int err;

    /* Convert time to register value */

	err = reg_write(dev, PAW3212_REG_WRITE_PROTECT, PAW3212_WPMAGIC);    uint8_t value = time / mintime;

	if (err) {

		LOG_ERR("Cannot disable write protect");    LOG_INF("Set downshift time to %u ms (reg value 0x%x)", time, value);

		return err;

	}    int err = pmw3610_write(dev, reg_addr, value);

    if (err) {

	uint8_t regval;        LOG_ERR("Failed to change downshift time");

    }

	err = reg_read(dev, reg_addr, &regval);

	if (err) {    return err;

		LOG_ERR("Failed to read sleep register");}

		return err;

	}static int pmw3610_set_performance(const struct device *dev, bool enabled) {

    const struct pixart_config *config = dev->config;

	__ASSERT_NO_MSG((etm & PAW3212_ETM_MASK) == etm);    int err = 0;



	regval &= ~PAW3212_ETM_MASK;    if (config->force_awake) {

	regval |= (etm << PAW3212_ETM_POS);        uint8_t value;

        err = pmw3610_read_reg(dev, PMW3610_REG_PERFORMANCE, &value);

	err = reg_write(dev, reg_addr, regval);        if (err) {

	if (err) {            LOG_ERR("Can't read ref-performance %d", err);

		LOG_ERR("Failed to change sleep time");            return err;

		return err;        }

	}        LOG_INF("Get performance register (reg value 0x%x)", value);



	err = reg_write(dev, PAW3212_REG_WRITE_PROTECT, 0);        // Set prefered RUN RATE

	if (err) {        //   BIT 3:   VEL_RUNRATE    0x0: 8ms; 0x1 4ms;

		LOG_ERR("Cannot enable write protect");        //   BIT 2:   POSHI_RUN_RATE 0x0: 8ms; 0x1 4ms;

	}        //   BIT 1-0: POSLO_RUN_RATE 0x0: 8ms; 0x1 4ms; 0x2 2ms; 0x4 Reserved

        uint8_t perf;

	return err;        if (config->force_awake_4ms_mode) {

}            perf = 0x0d; // RUN RATE @ 4ms

        } else {

static int paw3212_set_sample_time(const struct device *dev, uint8_t reg_addr,            // reset bit[3..0] to 0x0 (normal operation)

			      uint32_t sample_time_ms)            perf = value & 0x0F; // RUN RATE @ 8ms

{        }

	uint32_t sample_time_step;

	uint32_t sample_time_min;        if (enabled) {

	uint32_t sample_time_max;            perf |= 0xF0; // set bit[3..0] to 0xF (force awake)

        }

	switch (reg_addr) {        if (perf != value) {

	case PAW3212_REG_SLEEP1:            err = pmw3610_write(dev, PMW3610_REG_PERFORMANCE, perf);

		sample_time_step = 4;            if (err) {

		sample_time_min = 4;                LOG_ERR("Can't write performance register %d", err);

		sample_time_max = 64;                return err;

		break;            }

            LOG_INF("Set performance register (reg value 0x%x)", perf);

	case PAW3212_REG_SLEEP2:        }

	case PAW3212_REG_SLEEP3:        LOG_INF("%s performance mode", enabled ? "enable" : "disable");

		sample_time_step = 64;    }

		sample_time_min = 64;

		sample_time_max = 1024;    return err;

		break;}



	default:static int pmw3610_set_interrupt(const struct device *dev, const bool en) {

		LOG_ERR("Not supported");    const struct pixart_config *config = dev->config;

		return -ENOTSUP;    int ret = gpio_pin_interrupt_configure_dt(&config->irq_gpio,

	}                                              en ? GPIO_INT_LEVEL_ACTIVE : GPIO_INT_DISABLE);

    if (ret < 0) {

	if ((sample_time_ms > sample_time_max) ||        LOG_ERR("can't set interrupt");

	    (sample_time_ms < sample_time_min))	{    }

		LOG_WRN("Sample time %" PRIu32 " out of range", sample_time_ms);    return ret;

		return -EINVAL;}

	}

static int paw3212_async_init_power_up(const struct device *dev)

	uint8_t reg_freq = (sample_time_ms - sample_time_min) / sample_time_step;{

	return reg_write(dev, PAW3212_REG_CONFIGURATION,

	LOG_DBG("Set sleep%d sample time: %u (requested: %u, reg:0x%" PRIx8 ")",			 PAW3212_CONFIG_CHIP_RESET);

		reg_addr - PAW3212_REG_SLEEP1 + 1,}

		(reg_freq * sample_time_step) + sample_time_min,

		sample_time_ms,static int pmw3610_async_init_clear_ob1(const struct device *dev) {

		reg_freq);    return pmw3610_write(dev, PMW3610_REG_OBSERVATION, 0x00);

}

	__ASSERT_NO_MSG(reg_freq <= PAW3212_FREQ_MAX);

static int pmw3610_async_init_check_ob1(const struct device *dev) {

	int err;    uint8_t value;

    int err = pmw3610_read_reg(dev, PMW3610_REG_OBSERVATION, &value);

	err = reg_write(dev, PAW3212_REG_WRITE_PROTECT, PAW3212_WPMAGIC);    if (err) {

	if (err) {        LOG_ERR("Can't do self-test");

		LOG_ERR("Cannot disable write protect");        return err;

		return err;    }

	}

    if ((value & 0x0F) != 0x0F) {

	uint8_t regval;        LOG_ERR("Failed self-test (0x%x)", value);

        return -EINVAL;

	err = reg_read(dev, reg_addr, &regval);    }

	if (err) {

		LOG_ERR("Failed to read sleep register");    uint8_t product_id = 0x01;

		return err;    err = pmw3610_read_reg(dev, PMW3610_REG_PRODUCT_ID, &product_id);

	}    if (err) {

        LOG_ERR("Cannot obtain product id");

	regval &= ~PAW3212_FREQ_MASK;        return err;

	regval |= (reg_freq << PAW3212_FREQ_POS);    }



	err = reg_write(dev, reg_addr, regval);    if (product_id != PMW3610_PRODUCT_ID) {

	if (err) {        LOG_ERR("Incorrect product id 0x%x (expecting 0x%x)!", product_id, PMW3610_PRODUCT_ID);

		LOG_ERR("Failed to change sample time");        return -EIO;

		return err;    }

	}

    return 0;

	err = reg_write(dev, PAW3212_REG_WRITE_PROTECT, 0);}

	if (err) {

		LOG_ERR("Cannot enable write protect");static int paw3212_async_init_configure(const struct device *dev)

	}{

	int err;

	return err;

}	err = reg_write(dev, PAW3212_REG_WRITE_PROTECT, PAW3212_WPMAGIC);

	if (!err) {

static int paw3212_toggle_sleep_modes(const struct device *dev, uint8_t reg_addr1,		uint8_t mouse_option = 0;

			      uint8_t reg_addr2, bool enable)

{		if (IS_ENABLED(CONFIG_PAW3212_ORIENTATION_90)) {

	int err = reg_write(dev, PAW3212_REG_WRITE_PROTECT,			mouse_option |= PAW3212_MOUSE_OPT_XY_SWAP |

			    PAW3212_WPMAGIC);					PAW3212_MOUSE_OPT_Y_INV;

	if (err) {		} else if (IS_ENABLED(CONFIG_PAW3212_ORIENTATION_180)) {

		LOG_ERR("Cannot disable write protect");			mouse_option |= PAW3212_MOUSE_OPT_Y_INV |

		return err;					PAW3212_MOUSE_OPT_X_INV;

	}		} else if (IS_ENABLED(CONFIG_PAW3212_ORIENTATION_270)) {

			mouse_option |= PAW3212_MOUSE_OPT_XY_SWAP |

	uint8_t regval;					PAW3212_MOUSE_OPT_X_INV;

		}

	LOG_DBG("%sable sleep", (enable) ? ("En") : ("Dis"));

		if (IS_ENABLED(CONFIG_PAW3212_12_BIT_MODE)) {

	/* Sleep 1 and Sleep 2 */			mouse_option |= PAW3212_MOUSE_OPT_12BITMODE;

	err = reg_read(dev, reg_addr1, &regval);		}

	if (err) {

		LOG_ERR("Failed to read operation mode register");		err = reg_write(dev, PAW3212_REG_MOUSE_OPTION,

		return err;				mouse_option);

	}	}

	if (!err) {

	uint8_t sleep_enable_mask = BIT(PAW3212_SLP_ENH_POS) |		err = reg_write(dev, PAW3212_REG_WRITE_PROTECT, 0);

				 BIT(PAW3212_SLP2_ENH_POS);	}



	if (enable) {    if (err) {

		regval |= sleep_enable_mask;        LOG_ERR("Config the sensor failed");

	} else {        return err;

		regval &= ~sleep_enable_mask;    }

	}

	return 0;

	err = reg_write(dev, reg_addr1, regval);}

	if (err) {

		LOG_ERR("Failed to %sable sleep", (enable) ? ("en") : ("dis"));

		return err;static void paw3212_async_init(struct k_work *work)

	}{

	struct paw3212_data *data = CONTAINER_OF(work, struct paw3212_data,

	/* Sleep 3 */						 init_work.work);

	err = reg_read(dev, reg_addr2, &regval);	const struct device *dev = data->dev;

	if (err) {

		LOG_ERR("Failed to read configuration register");	LOG_DBG("PAW3212 async init step %d", data->async_init_step);

		return err;

	}	data->err = async_init_fn[data->async_init_step](dev);

	if (data->err) {

	sleep_enable_mask = BIT(PAW3212_SLP3_ENH_POS);		LOG_ERR("PAW3212 initialization failed");

	} else {

	if (enable) {		data->async_init_step++;

		regval |= sleep_enable_mask;

	} else {		if (data->async_init_step == ASYNC_INIT_STEP_COUNT) {

		regval &= ~sleep_enable_mask;			data->ready = true;

	}			LOG_INF("PAW3212 initialized");

		} else {

	err = reg_write(dev, reg_addr2, regval);			k_work_schedule(&data->init_work,

	if (err) {					K_MSEC(async_init_delay[

		LOG_ERR("Failed to %sable sleep", (enable) ? ("en") : ("dis"));						data->async_init_step]));

		return err;		}

	}	}

}

	err = reg_write(dev, PAW3212_REG_WRITE_PROTECT, 0);

	if (err) {static int pmw3610_report_data(const struct device *dev) {

		LOG_ERR("Cannot enable write protect");    struct pixart_data *data = dev->data;

	}    const struct pixart_config *config = dev->config;

    uint8_t buf[PMW3610_BURST_SIZE];

	return err;

}    if (unlikely(!data->ready)) {

        LOG_WRN("Device is not initialized yet");

static int paw3212_set_interrupt(const struct device *dev, const bool en)        return -EBUSY;

{    }

	const struct pixart_config *config = dev->config;

	int ret = gpio_pin_interrupt_configure_dt(&config->irq_gpio,    static int64_t dx = 0;

						      en ? GPIO_INT_LEVEL_ACTIVE : GPIO_INT_DISABLE);    static int64_t dy = 0;

	if (ret < 0) {

		LOG_ERR("can't set interrupt");#if CONFIG_PMW3610_REPORT_INTERVAL_MIN > 0

	}    static int64_t last_smp_time = 0;

	return ret;    static int64_t last_rpt_time = 0;

}    int64_t now = k_uptime_get();

#endif

static int paw3212_async_init_power_up(const struct device *dev)

{	int err = pmw3610_read(dev, PMW3610_REG_MOTION_BURST, buf, PMW3610_BURST_SIZE);

	return reg_write(dev, PAW3212_REG_CONFIGURATION,    if (err) {

			 PAW3212_CONFIG_CHIP_RESET);        return err;

}    }

    // LOG_HEXDUMP_DBG(buf, PMW3610_BURST_SIZE, "buf");

static int paw3212_async_init_verify_id(const struct device *dev)

{// 12-bit two's complement value to int16_t

	int err;// adapted from https://stackoverflow.com/questions/70802306/convert-a-12-bit-signed-number-in-c

#define TOINT16(val, bits) (((struct { int16_t value : bits; }){val}).value)

	uint8_t product_id;

	err = reg_read(dev, PAW3212_REG_PRODUCT_ID, &product_id);    int16_t x = TOINT16((buf[PMW3610_X_L_POS] + ((buf[PMW3610_XY_H_POS] & 0xF0) << 4)), 12);

	if (err) {    int16_t y = TOINT16((buf[PMW3610_Y_L_POS] + ((buf[PMW3610_XY_H_POS] & 0x0F) << 8)), 12);

		LOG_ERR("Cannot obtain product ID");    LOG_DBG("x/y: %d/%d", x, y);

		return err;

	}#ifdef CONFIG_PMW3610_SMART_ALGORITHM

    int16_t shutter = ((int16_t)(buf[PMW3610_SHUTTER_H_POS] & 0x01) << 8)

	LOG_DBG("Product ID: 0x%" PRIx8, product_id);                    + buf[PMW3610_SHUTTER_L_POS];

	if (product_id != PAW3212_PRODUCT_ID) {    if (data->sw_smart_flag && shutter < 45) {

		LOG_ERR("Invalid product ID (0x%" PRIx8")!", product_id);        pmw3610_write(dev, 0x32, 0x00);

		return -EIO;        data->sw_smart_flag = false;

	}    }

    if (!data->sw_smart_flag && shutter > 45) {

	return err;        pmw3610_write(dev, 0x32, 0x80);

}        data->sw_smart_flag = true;

    }

static int paw3212_async_init_configure(const struct device *dev)#endif

{

	int err;#if CONFIG_PMW3610_REPORT_INTERVAL_MIN > 0

    // purge accumulated delta, if last sampled had not been reported on last report tick

	err = reg_write(dev, PAW3212_REG_WRITE_PROTECT, PAW3212_WPMAGIC);    if (now - last_smp_time >= CONFIG_PMW3610_REPORT_INTERVAL_MIN) {

	if (!err) {        dx = 0;

		uint8_t mouse_option = 0;        dy = 0;

    }

		if (IS_ENABLED(CONFIG_PAW3212_ORIENTATION_90)) {    last_smp_time = now;

			mouse_option |= PAW3212_MOUSE_OPT_XY_SWAP |#endif

					PAW3212_MOUSE_OPT_Y_INV;

		} else if (IS_ENABLED(CONFIG_PAW3212_ORIENTATION_180)) {    // accumulate delta until report in next iteration

			mouse_option |= PAW3212_MOUSE_OPT_Y_INV |    dx += x;

					PAW3212_MOUSE_OPT_X_INV;    dy += y;

		} else if (IS_ENABLED(CONFIG_PAW3212_ORIENTATION_270)) {

			mouse_option |= PAW3212_MOUSE_OPT_XY_SWAP |#if CONFIG_PMW3610_REPORT_INTERVAL_MIN > 0

					PAW3212_MOUSE_OPT_X_INV;    // strict to report inerval

		}    if (now - last_rpt_time < CONFIG_PMW3610_REPORT_INTERVAL_MIN) {

        return 0;

		if (IS_ENABLED(CONFIG_PAW3212_12_BIT_MODE)) {    }

			mouse_option |= PAW3212_MOUSE_OPT_12BITMODE;#endif

		}

    // fetch report value

		err = reg_write(dev, PAW3212_REG_MOUSE_OPTION,    int16_t rx = (int16_t)CLAMP(dx, INT16_MIN, INT16_MAX);

				mouse_option);    int16_t ry = (int16_t)CLAMP(dy, INT16_MIN, INT16_MAX);

	}    bool have_x = rx != 0;

	if (!err) {    bool have_y = ry != 0;

		err = reg_write(dev, PAW3212_REG_WRITE_PROTECT, 0);

	}    if (have_x || have_y) {

#if CONFIG_PMW3610_REPORT_INTERVAL_MIN > 0

	return err;        last_rpt_time = now;

}#endif

        dx = 0;

static void paw3212_async_init(struct k_work *work)        dy = 0;

{        if (have_x) {

	struct k_work_delayable *work2 = (struct k_work_delayable *)work;            input_report(dev, config->evt_type, config->x_input_code, rx, !have_y, K_NO_WAIT);

	struct pixart_data *data = CONTAINER_OF(work2, struct pixart_data, init_work);        }

	const struct device *dev = data->dev;        if (have_y) {

            input_report(dev, config->evt_type, config->y_input_code, ry, true, K_NO_WAIT);

	LOG_DBG("PAW3212 async init step %d", data->async_init_step);        }

    }

	data->err = async_init_fn[data->async_init_step](dev);

	if (data->err) {    return err;

		LOG_ERR("PAW3212 initialization failed");}

	} else {

		data->async_init_step++;static void pmw3610_gpio_callback(const struct device *gpiob, struct gpio_callback *cb,

                                  uint32_t pins) {

		if (data->async_init_step == ASYNC_INIT_STEP_COUNT) {    struct pixart_data *data = CONTAINER_OF(cb, struct pixart_data, irq_gpio_cb);

			data->ready = true;    const struct device *dev = data->dev;

			LOG_INF("PAW3212 initialized");    pmw3610_set_interrupt(dev, false);

			paw3212_set_interrupt(dev, true);    k_work_submit(&data->trigger_work);

		} else {}

			k_work_schedule(&data->init_work,

					K_MSEC(async_init_delay[static void pmw3610_work_callback(struct k_work *work) {

						data->async_init_step]));    struct pixart_data *data = CONTAINER_OF(work, struct pixart_data, trigger_work);

		}    const struct device *dev = data->dev;

	}    pmw3610_report_data(dev);

}    pmw3610_set_interrupt(dev, true);

}

static int paw3212_report_data(const struct device *dev)

{static int paw3212_init_irq(const struct device *dev)

	struct pixart_data *data = dev->data;{

	const struct pixart_config *config = dev->config;	int err;

	uint8_t motion_status;	struct paw3212_data *data = dev->data;

	int err;	const struct paw3212_config *config = dev->config;



	if (unlikely(!data->ready)) {	if (!device_is_ready(config->irq_gpio.port)) {

		LOG_WRN("Device is not initialized yet");		LOG_ERR("IRQ GPIO device not ready");

		return -EBUSY;		return -ENODEV;

	}	}



	err = reg_read(dev, PAW3212_REG_MOTION, &motion_status);	err = gpio_pin_configure_dt(&config->irq_gpio, GPIO_INPUT);

	if (err) {	if (err) {

		LOG_ERR("Cannot read motion");		LOG_ERR("Cannot configure IRQ GPIO");

		return err;		return err;

	}	}



	if ((motion_status & PAW3212_MOTION_STATUS_MOTION) != 0) {	gpio_init_callback(&data->irq_gpio_cb, irq_handler,

		uint8_t x_low;			   BIT(config->irq_gpio.pin));

		uint8_t y_low;

		int16_t x;	err = gpio_add_callback(config->irq_gpio.port,

		int16_t y;				&data->irq_gpio_cb);



		if ((motion_status & PAW3212_MOTION_STATUS_DXOVF) != 0) {	if (err) {

			LOG_WRN("X delta overflowed");		LOG_ERR("Cannot add IRQ GPIO callback");

		}	}

		if ((motion_status & PAW3212_MOTION_STATUS_DYOVF) != 0) {

			LOG_WRN("Y delta overflowed");	return err;

		}}



		err = reg_read(dev, PAW3212_REG_DELTA_X_LOW, &x_low);static int paw3212_init(const struct device *dev)

		if (err) {{

			LOG_ERR("Cannot read X delta");	struct paw3212_data *data = dev->data;

			return err;	const struct paw3212_config *config = dev->config;

		}	int err;



		err = reg_read(dev, PAW3212_REG_DELTA_Y_LOW, &y_low);	/* Assert that negative numbers are processed as expected */

		if (err) {	__ASSERT_NO_MSG(-1 == expand_s12(0xFFF));

			LOG_ERR("Cannot read Y delta");

			return err;	k_work_init(&data->trigger_handler_work, trigger_handler);

		}	data->dev = dev;



		if (IS_ENABLED(CONFIG_PAW3212_12_BIT_MODE)) {	if (!spi_is_ready_dt(&config->bus)) {

			uint8_t xy_high;		return -ENODEV;

	}

			err = reg_read(dev, PAW3212_REG_DELTA_XY_HIGH,

				       &xy_high);	if (!device_is_ready(config->cs_gpio.port)) {

			if (err) {		LOG_ERR("SPI CS device not ready");

				LOG_ERR("Cannot read XY delta high");		return -ENODEV;

				return err;	}

			}

	err = gpio_pin_configure_dt(&config->cs_gpio, GPIO_OUTPUT_INACTIVE);

			x = PAW3212_DELTA_X(xy_high, x_low);	if (err) {

			y = PAW3212_DELTA_Y(xy_high, y_low);		LOG_ERR("Cannot configure SPI CS GPIO");

		} else {		return err;

			x = (int8_t)x_low;	}

			y = (int8_t)y_low;

		}	err = paw3212_init_irq(dev);

	if (err) {

		LOG_DBG("x/y: %d/%d", x, y);		return err;

	}

		bool have_x = x != 0;

		bool have_y = y != 0;	k_work_init_delayable(&data->init_work, paw3212_async_init);



		if (have_x || have_y) {	k_work_schedule(&data->init_work,

			if (have_x) {			K_MSEC(async_init_delay[data->async_init_step]));

				input_report(dev, config->evt_type, config->x_input_code, x, !have_y, K_NO_WAIT);

			}	return err;

			if (have_y) {}

				input_report(dev, config->evt_type, config->y_input_code, y, true, K_NO_WAIT);

			}static int paw3212_attr_set(const struct device *dev, enum sensor_channel chan,

		}			    enum sensor_attribute attr,

	}			    const struct sensor_value *val)

{

	return err;	struct paw3212_data *data = dev->data;

}	int err;



static void paw3212_gpio_callback(const struct device *gpiob, struct gpio_callback *cb,	if (unlikely(chan != SENSOR_CHAN_ALL)) {

			uint32_t pins)		return -ENOTSUP;

{	}

	struct pixart_data *data = CONTAINER_OF(cb, struct pixart_data, irq_gpio_cb);

	const struct device *dev = data->dev;	if (unlikely(!data->ready)) {

	paw3212_set_interrupt(dev, false);		LOG_INF("Device is not initialized yet");

	k_work_submit(&data->trigger_work);		return -EBUSY;

}	}



static void paw3212_work_callback(struct k_work *work)	switch ((uint32_t)attr) {

{	case PAW3212_ATTR_CPI:

	struct pixart_data *data = CONTAINER_OF(work, struct pixart_data, trigger_work);		err = update_cpi(dev, PAW3212_SVALUE_TO_CPI(*val));

	const struct device *dev = data->dev;		break;

	paw3212_report_data(dev);

	paw3212_set_interrupt(dev, true);	case PAW3212_ATTR_SLEEP_ENABLE:

}		err = toggle_sleep_modes(dev,

					 PAW3212_REG_OPERATION_MODE,

static int paw3212_init_irq(const struct device *dev)					 PAW3212_REG_CONFIGURATION,

{					 PAW3212_SVALUE_TO_BOOL(*val));

	int err;		break;

	struct pixart_data *data = dev->data;

	const struct pixart_config *config = dev->config;	case PAW3212_ATTR_SLEEP1_TIMEOUT:

		err = update_sleep_timeout(dev,

	if (!device_is_ready(config->irq_gpio.port)) {					   PAW3212_REG_SLEEP1,

		LOG_ERR("IRQ GPIO device not ready");					   PAW3212_SVALUE_TO_TIME(*val));

		return -ENODEV;		break;

	}

	case PAW3212_ATTR_SLEEP2_TIMEOUT:

	err = gpio_pin_configure_dt(&config->irq_gpio, GPIO_INPUT);		err = update_sleep_timeout(dev,

	if (err) {					   PAW3212_REG_SLEEP2,

		LOG_ERR("Cannot configure IRQ GPIO");					   PAW3212_SVALUE_TO_TIME(*val));

		return err;		break;

	}

	case PAW3212_ATTR_SLEEP3_TIMEOUT:

	gpio_init_callback(&data->irq_gpio_cb, paw3212_gpio_callback,		err = update_sleep_timeout(dev,

			   BIT(config->irq_gpio.pin));					   PAW3212_REG_SLEEP3,

					   PAW3212_SVALUE_TO_TIME(*val));

	err = gpio_add_callback(config->irq_gpio.port,		break;

				&data->irq_gpio_cb);

	case PAW3212_ATTR_SLEEP1_SAMPLE_TIME:

	if (err) {		err = update_sample_time(dev,

		LOG_ERR("Cannot add IRQ GPIO callback");					 PAW3212_REG_SLEEP1,

	}					 PAW3212_SVALUE_TO_TIME(*val));

		break;

	return err;

}	case PAW3212_ATTR_SLEEP2_SAMPLE_TIME:

		err = update_sample_time(dev,

static int paw3212_init(const struct device *dev)					 PAW3212_REG_SLEEP2,

{					 PAW3212_SVALUE_TO_TIME(*val));

	struct pixart_data *data = dev->data;		break;

	const struct pixart_config *config = dev->config;

	int err;	case PAW3212_ATTR_SLEEP3_SAMPLE_TIME:

		err = update_sample_time(dev,

	/* Assert that negative numbers are processed as expected */					 PAW3212_REG_SLEEP3,

	__ASSERT_NO_MSG(-1 == expand_s12(0xFFF));					 PAW3212_SVALUE_TO_TIME(*val));

		break;

	if (!spi_is_ready_dt(&config->spi)) {

		LOG_ERR("%s is not ready", config->spi.bus->name);	default:

		return -ENODEV;		LOG_ERR("Unknown attribute");

	}		return -ENOTSUP;

	}

	if (!device_is_ready(config->cs_gpio.port)) {

		LOG_ERR("SPI CS device not ready");	return err;

		return -ENODEV;}

	}

static const struct sensor_driver_api paw3212_driver_api = {

	err = gpio_pin_configure_dt(&config->cs_gpio, GPIO_OUTPUT_INACTIVE);	.sample_fetch = paw3212_sample_fetch,

	if (err) {	.channel_get  = paw3212_channel_get,

		LOG_ERR("Cannot configure SPI CS GPIO");	.trigger_set  = paw3212_trigger_set,

		return err;	.attr_set     = paw3212_attr_set,

	}};



	data->dev = dev;#define PMW3610_SPI_MODE (SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_MODE_CPOL | \

                        SPI_MODE_CPHA | SPI_TRANSFER_MSB)

	k_work_init(&data->trigger_work, paw3212_work_callback);

#define PAW3212_DEFINE(n)						       \

	err = paw3212_init_irq(dev);	static struct paw3212_data data##n;				       \

	if (err) {									       \

		return err;	static const struct paw3212_config config##n = {		       \

	}		.irq_gpio = GPIO_DT_SPEC_INST_GET(n, irq_gpios),	       \

		.bus = {						       \

	k_work_init_delayable(&data->init_work, paw3212_async_init);			    .bus = DEVICE_DT_GET(DT_INST_BUS(n)),		       \

				.config = {					       \

	k_work_schedule(&data->init_work,				.frequency = DT_INST_PROP(n, spi_max_frequency),  \

			K_MSEC(async_init_delay[data->async_init_step]));				.operation = SPI_WORD_SET(8) |		       \

					         SPI_TRANSFER_MSB |		       \

	return err;							 SPI_MODE_CPOL | SPI_MODE_CPHA,    \

}				.slave = DT_INST_REG_ADDR(n),		       \

			},						       \

static int paw3212_attr_set(const struct device *dev, enum sensor_channel chan,		},							       \

			    enum sensor_attribute attr,		.cs_gpio = SPI_CS_GPIOS_DT_SPEC_GET(DT_DRV_INST(n)),	       \

			    const struct sensor_value *val)	};								       \

{									       \

	struct pixart_data *data = dev->data;	DEVICE_DT_INST_DEFINE(n, paw3212_init, NULL, &data##n, &config##n,     \

	int err;			      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,	       \

			      &paw3212_driver_api);

	if (unlikely(chan != SENSOR_CHAN_ALL)) {

		return -ENOTSUP;DT_INST_FOREACH_STATUS_OKAY(PAW3212_DEFINE)

	}

ZMK_LISTENER(zmk_pmw3610_idle_sleeper, on_activity_state);

	if (unlikely(!data->ready)) {ZMK_SUBSCRIPTION(zmk_pmw3610_idle_sleeper, zmk_activity_state_changed);

		LOG_INF("Device is not initialized yet");
		return -EBUSY;
	}

	switch ((uint32_t)attr) {
	case PAW3212_ATTR_CPI:
		err = paw3212_set_cpi(dev, PAW3212_SVALUE_TO_CPI(*val));
		break;

	case PAW3212_ATTR_SLEEP_ENABLE:
		err = paw3212_toggle_sleep_modes(dev,
					 PAW3212_REG_OPERATION_MODE,
					 PAW3212_REG_CONFIGURATION,
					 PAW3212_SVALUE_TO_BOOL(*val));
		break;

	case PAW3212_ATTR_SLEEP1_TIMEOUT:
		err = paw3212_set_sleep_timeout(dev,
					   PAW3212_REG_SLEEP1,
					   PAW3212_SVALUE_TO_TIME(*val));
		break;

	case PAW3212_ATTR_SLEEP2_TIMEOUT:
		err = paw3212_set_sleep_timeout(dev,
					   PAW3212_REG_SLEEP2,
					   PAW3212_SVALUE_TO_TIME(*val));
		break;

	case PAW3212_ATTR_SLEEP3_TIMEOUT:
		err = paw3212_set_sleep_timeout(dev,
					   PAW3212_REG_SLEEP3,
					   PAW3212_SVALUE_TO_TIME(*val));
		break;

	case PAW3212_ATTR_SLEEP1_SAMPLE_TIME:
		err = paw3212_set_sample_time(dev,
					 PAW3212_REG_SLEEP1,
					 PAW3212_SVALUE_TO_TIME(*val));
		break;

	case PAW3212_ATTR_SLEEP2_SAMPLE_TIME:
		err = paw3212_set_sample_time(dev,
					 PAW3212_REG_SLEEP2,
					 PAW3212_SVALUE_TO_TIME(*val));
		break;

	case PAW3212_ATTR_SLEEP3_SAMPLE_TIME:
		err = paw3212_set_sample_time(dev,
					 PAW3212_REG_SLEEP3,
					 PAW3212_SVALUE_TO_TIME(*val));
		break;

	default:
		LOG_ERR("Unknown attribute");
		return -ENOTSUP;
	}

	return err;
}

static const struct sensor_driver_api paw3212_driver_api = {
	.attr_set = paw3212_attr_set,
};

#define PAW3212_SPI_MODE (SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_MODE_CPOL | \
			  SPI_MODE_CPHA | SPI_TRANSFER_MSB)

#define PAW3212_DEFINE(n)						       \
	static struct pixart_data data##n;				       \
									       \
	static const struct pixart_config config##n = {		       \
		.spi = SPI_DT_SPEC_INST_GET(n, PAW3212_SPI_MODE, 0),	       \
		.cs_gpio = SPI_CS_GPIOS_DT_SPEC_GET(DT_DRV_INST(n)),	       \
		.irq_gpio = GPIO_DT_SPEC_INST_GET(n, irq_gpios),	       \
		.evt_type = DT_PROP(DT_DRV_INST(n), evt_type),		       \
		.x_input_code = DT_PROP(DT_DRV_INST(n), x_input_code),	       \
		.y_input_code = DT_PROP(DT_DRV_INST(n), y_input_code),	       \
	};								       \
									       \
	DEVICE_DT_INST_DEFINE(n, paw3212_init, NULL, &data##n, &config##n,     \
			      POST_KERNEL, CONFIG_INPUT_PAW3212_INIT_PRIORITY, \
			      &paw3212_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PAW3212_DEFINE)

#define GET_PAW3212_DEV(node_id) DEVICE_DT_GET(node_id),

static const struct device *paw3212_devs[] = {
	DT_FOREACH_STATUS_OKAY(pixart_paw3212, GET_PAW3212_DEV)
};

static int on_activity_state(const zmk_event_t *eh)
{
	struct zmk_activity_state_changed *state_ev = as_zmk_activity_state_changed(eh);

	if (!state_ev) {
		LOG_WRN("NO EVENT, leaving early");
		return 0;
	}

	bool enable = state_ev->state == ZMK_ACTIVITY_ACTIVE ? 1 : 0;
	for (size_t i = 0; i < ARRAY_SIZE(paw3212_devs); i++) {
		paw3212_toggle_sleep_modes(paw3212_devs[i],
					   PAW3212_REG_OPERATION_MODE,
					   PAW3212_REG_CONFIGURATION,
					   !enable);
	}

	return 0;
}

ZMK_LISTENER(zmk_paw3212_idle_sleeper, on_activity_state);
ZMK_SUBSCRIPTION(zmk_paw3212_idle_sleeper, zmk_activity_state_changed);
