/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT pixart_pmw3610

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/input/input.h>
#include <zephyr/pm/device.h>
#include <zmk/keymap.h>
#include <zmk/events/activity_state_changed.h>
#include "paw3212.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(paw3212, CONFIG_PAW3212_LOG_LEVEL);

//////// Sensor initialization steps definition //////////
// init is done in non-blocking manner (i.e., async), a //
// delayable work is defined for this purpose           //
enum paw3212_init_step {
	ASYNC_INIT_STEP_POWER_UP,
	ASYNC_INIT_STEP_VERIFY_ID,
	ASYNC_INIT_STEP_CONFIGURE,

	ASYNC_INIT_STEP_COUNT
};

/* Timings (in ms) needed in between steps to allow each step finishes succussfully. */
// - Since MCU is not involved in the sensor init process, i is allowed to do other tasks.
//   Thus, k_sleep or delayed schedule can be used.
static const int32_t async_init_delay[ASYNC_INIT_STEP_COUNT] = {
	[ASYNC_INIT_STEP_POWER_UP]  = 10,
	[ASYNC_INIT_STEP_VERIFY_ID] = 0,
	[ASYNC_INIT_STEP_CONFIGURE] = 0,
};

static int paw3212_async_init_power_up(const struct device *dev);
static int paw3212_async_init_verify_id(const struct device *dev);
static int paw3212_async_init_configure(const struct device *dev);

static int (* const async_init_fn[ASYNC_INIT_STEP_COUNT])(const struct device *dev) = {
	[ASYNC_INIT_STEP_POWER_UP]  = paw3212_async_init_power_up,
	[ASYNC_INIT_STEP_VERIFY_ID] = paw3212_async_init_verify_id,
	[ASYNC_INIT_STEP_CONFIGURE] = paw3212_async_init_configure,
};

//////// Function definitions //////////

static int pmw3610_read(const struct device *dev, uint8_t addr, uint8_t *value, uint8_t len) {
	const struct pixart_config *cfg = dev->config;
	const struct spi_buf tx_buf = { .buf = &addr, .len = sizeof(addr) };
	const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };
	struct spi_buf rx_buf[] = {
		{ .buf = NULL, .len = sizeof(addr), },
		{ .buf = value, .len = len, },
	};
	const struct spi_buf_set rx = { .buffers = rx_buf, .count = ARRAY_SIZE(rx_buf) };
	return spi_transceive_dt(&cfg->spi, &tx, &rx);
}

static int pmw3610_read_reg(const struct device *dev, uint8_t addr, uint8_t *value) {
	return pmw3610_read(dev, addr, value, 1);
}

static int pmw3610_write_reg(const struct device *dev, uint8_t addr, uint8_t value) {
	const struct pixart_config *cfg = dev->config;
	uint8_t write_buf[] = {addr | SPI_WRITE_BIT, value};
	const struct spi_buf tx_buf = { .buf = write_buf, .len = sizeof(write_buf), };
	const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1, };
	return spi_write_dt(&cfg->spi, &tx);
}

static int pmw3610_write(const struct device *dev, uint8_t reg, uint8_t val) {
	pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_ENABLE);
	k_sleep(K_USEC(T_CLOCK_ON_DELAY_US));

    int err = pmw3610_write_reg(dev, reg, val);
    if (unlikely(err != 0)) {
        return err;
    }

    pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_DISABLE);
    return 0;
}

static int paw3212_set_cpi(const struct device *dev, uint32_t cpi)
{
	int err;

	if ((cpi > PAW3212_CPI_MAX) || (cpi < PAW3212_CPI_MIN)) {
		LOG_ERR("CPI %" PRIu32 " out of range", cpi);
		return -EINVAL;
	}

	uint8_t regval = cpi / PAW3212_CPI_STEP;

	LOG_DBG("Set CPI: %u (requested: %u, reg:0x%" PRIx8 ")",
		regval * PAW3212_CPI_STEP, cpi, regval);

	err = reg_write(dev, PAW3212_REG_WRITE_PROTECT, PAW3212_WPMAGIC);
	if (err) {
		LOG_ERR("Cannot disable write protect");
		return err;
	}

	err = reg_write(dev, PAW3212_REG_CPI_X, regval);
	if (err) {
		LOG_ERR("Failed to change x CPI");
		return err;
	}

	err = reg_write(dev, PAW3212_REG_CPI_Y, regval);
	if (err) {
		LOG_ERR("Failed to change y CPI");
		return err;
	}

	err = reg_write(dev, PAW3212_REG_WRITE_PROTECT, 0);
	if (err) {
		LOG_ERR("Cannot enable write protect");
	}

	return err;
}

static int pmw3610_set_axis(const struct device *dev, bool swap_xy, bool inv_x, bool inv_y) {
    LOG_INF("Setting axis swap_xy: %s inv_x: %s inv_y: %s",
            swap_xy ? "yes" : "no", inv_x ? "yes" : "no", inv_y ? "yes" : "no");

    uint8_t value;
    int err = pmw3610_read_reg(dev, PMW3610_REG_RES_STEP, &value);
    if (err) {
        LOG_ERR("Can't read res step %d", err);
        return err;
    }
    LOG_INF("Get res step register (reg value 0x%x)", value);

    // Convert axis to register value
    // Set prefered RES_STEP
    //   BIT 7: SWAP_XY
    //   BIT 6: INV_X
    //   BIT 5: INV_Y
#if IS_ENABLED(CONFIG_PMW3610_SWAP_XY)
    value |= (1 << 7);
#else
    if (swap_xy) { value |= (1 << 7); } else { value &= ~(1 << 7); }
#endif
#if IS_ENABLED(CONFIG_PMW3610_INVERT_X)
    value |= (1 << 6);
#else
    if (inv_x) { value |= (1 << 6); } else { value &= ~(1 << 6); }
#endif
#if IS_ENABLED(CONFIG_PMW3610_INVERT_Y)
    value |= (1 << 5);
#else
    if (inv_y) { value |= (1 << 5); } else { value &= ~(1 << 5); }
#endif
    LOG_INF("Setting RES_STEP to (reg value 0x%x)", value);

    /* set the axis control register */
    uint8_t addr[] = {0x7F, PMW3610_REG_RES_STEP, 0x7F};
    uint8_t data[] = {0xFF, value, 0x00};

	pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_ENABLE);
	k_sleep(K_USEC(T_CLOCK_ON_DELAY_US));

    /* Write data */
    for (size_t i = 0; i < sizeof(data); i++) {
        err = pmw3610_write_reg(dev, addr[i], data[i]);
        if (err) {
            LOG_ERR("Burst write failed on SPI write (data)");
            break;
        }
    }
    pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_DISABLE);

    if (err) {
        LOG_ERR("Failed to set axis");
        return err;
    }

    return 0;
}

/* Set sampling rate in each mode (in ms) */
static int pmw3610_set_sample_time(const struct device *dev, uint8_t reg_addr, uint32_t sample_time) {
    uint32_t maxtime = 2550;
    uint32_t mintime = 10;
    if ((sample_time > maxtime) || (sample_time < mintime)) {
        LOG_WRN("Sample time %u out of range [%u, %u]", sample_time, mintime, maxtime);
        return -EINVAL;
    }

    uint8_t value = sample_time / mintime;
    LOG_INF("Set sample time to %u ms (reg value: 0x%x)", sample_time, value);

    /* The sample time is (reg_value * mintime ) ms. 0x00 is rounded to 0x1 */
    int err = pmw3610_write(dev, reg_addr, value);
    if (err) {
        LOG_ERR("Failed to change sample time");
    }

    return err;
}

/* Set downshift time in ms. */
// NOTE: The unit of run-mode downshift is related to pos mode rate, which is hard coded to be 4 ms
// The pos-mode rate is configured in pmw3610_async_init_configure
static int pmw3610_set_downshift_time(const struct device *dev, uint8_t reg_addr, uint32_t time) {
    uint32_t maxtime;
    uint32_t mintime;

    switch (reg_addr) {
    case PMW3610_REG_RUN_DOWNSHIFT:
        /*
         * Run downshift time = PMW3610_REG_RUN_DOWNSHIFT
         *                      * 8 * pos-rate (fixed to 4ms)
         */
        maxtime = 8160; // 32 * 255;
        mintime = 32; // hard-coded in pmw3610_async_init_configure
        break;

    case PMW3610_REG_REST1_DOWNSHIFT:
        /*
         * Rest1 downshift time = PMW3610_REG_RUN_DOWNSHIFT
         *                        * 16 * Rest1_sample_period (default 40 ms)
         */
        maxtime = 255 * 16 * CONFIG_PMW3610_REST1_SAMPLE_TIME_MS;
        mintime = 16 * CONFIG_PMW3610_REST1_SAMPLE_TIME_MS;
        break;

    case PMW3610_REG_REST2_DOWNSHIFT:
        /*
         * Rest2 downshift time = PMW3610_REG_REST2_DOWNSHIFT
         *                        * 128 * Rest2 rate (default 100 ms)
         */
        maxtime = 255 * 128 * CONFIG_PMW3610_REST2_SAMPLE_TIME_MS;
        mintime = 128 * CONFIG_PMW3610_REST2_SAMPLE_TIME_MS;
        break;

    default:
        LOG_ERR("Not supported");
        return -ENOTSUP;
    }

    if ((time > maxtime) || (time < mintime)) {
        LOG_WRN("Downshift time %u out of range (%u - %u)", time, mintime, maxtime);
        return -EINVAL;
    }

    __ASSERT_NO_MSG((mintime > 0) && (maxtime / mintime <= UINT8_MAX));

    /* Convert time to register value */
    uint8_t value = time / mintime;

    LOG_INF("Set downshift time to %u ms (reg value 0x%x)", time, value);

    int err = pmw3610_write(dev, reg_addr, value);
    if (err) {
        LOG_ERR("Failed to change downshift time");
    }

    return err;
}

static int pmw3610_set_performance(const struct device *dev, bool enabled) {
    const struct pixart_config *config = dev->config;
    int err = 0;

    if (config->force_awake) {
        uint8_t value;
        err = pmw3610_read_reg(dev, PMW3610_REG_PERFORMANCE, &value);
        if (err) {
            LOG_ERR("Can't read ref-performance %d", err);
            return err;
        }
        LOG_INF("Get performance register (reg value 0x%x)", value);

        // Set prefered RUN RATE
        //   BIT 3:   VEL_RUNRATE    0x0: 8ms; 0x1 4ms;
        //   BIT 2:   POSHI_RUN_RATE 0x0: 8ms; 0x1 4ms;
        //   BIT 1-0: POSLO_RUN_RATE 0x0: 8ms; 0x1 4ms; 0x2 2ms; 0x4 Reserved
        uint8_t perf;
        if (config->force_awake_4ms_mode) {
            perf = 0x0d; // RUN RATE @ 4ms
        } else {
            // reset bit[3..0] to 0x0 (normal operation)
            perf = value & 0x0F; // RUN RATE @ 8ms
        }

        if (enabled) {
            perf |= 0xF0; // set bit[3..0] to 0xF (force awake)
        }
        if (perf != value) {
            err = pmw3610_write(dev, PMW3610_REG_PERFORMANCE, perf);
            if (err) {
                LOG_ERR("Can't write performance register %d", err);
                return err;
            }
            LOG_INF("Set performance register (reg value 0x%x)", perf);
        }
        LOG_INF("%s performance mode", enabled ? "enable" : "disable");
    }

    return err;
}

static int pmw3610_set_interrupt(const struct device *dev, const bool en) {
    const struct pixart_config *config = dev->config;
    int ret = gpio_pin_interrupt_configure_dt(&config->irq_gpio,
                                              en ? GPIO_INT_LEVEL_ACTIVE : GPIO_INT_DISABLE);
    if (ret < 0) {
        LOG_ERR("can't set interrupt");
    }
    return ret;
}

static int paw3212_async_init_power_up(const struct device *dev)
{
	return reg_write(dev, PAW3212_REG_CONFIGURATION,
			 PAW3212_CONFIG_CHIP_RESET);
}

static int pmw3610_async_init_clear_ob1(const struct device *dev) {
    return pmw3610_write(dev, PMW3610_REG_OBSERVATION, 0x00);
}

static int pmw3610_async_init_check_ob1(const struct device *dev) {
    uint8_t value;
    int err = pmw3610_read_reg(dev, PMW3610_REG_OBSERVATION, &value);
    if (err) {
        LOG_ERR("Can't do self-test");
        return err;
    }

    if ((value & 0x0F) != 0x0F) {
        LOG_ERR("Failed self-test (0x%x)", value);
        return -EINVAL;
    }

    uint8_t product_id = 0x01;
    err = pmw3610_read_reg(dev, PMW3610_REG_PRODUCT_ID, &product_id);
    if (err) {
        LOG_ERR("Cannot obtain product id");
        return err;
    }

    if (product_id != PMW3610_PRODUCT_ID) {
        LOG_ERR("Incorrect product id 0x%x (expecting 0x%x)!", product_id, PMW3610_PRODUCT_ID);
        return -EIO;
    }

    return 0;
}

static int paw3212_async_init_configure(const struct device *dev)
{
	int err;

	err = reg_write(dev, PAW3212_REG_WRITE_PROTECT, PAW3212_WPMAGIC);
	if (!err) {
		uint8_t mouse_option = 0;

		if (IS_ENABLED(CONFIG_PAW3212_ORIENTATION_90)) {
			mouse_option |= PAW3212_MOUSE_OPT_XY_SWAP |
					PAW3212_MOUSE_OPT_Y_INV;
		} else if (IS_ENABLED(CONFIG_PAW3212_ORIENTATION_180)) {
			mouse_option |= PAW3212_MOUSE_OPT_Y_INV |
					PAW3212_MOUSE_OPT_X_INV;
		} else if (IS_ENABLED(CONFIG_PAW3212_ORIENTATION_270)) {
			mouse_option |= PAW3212_MOUSE_OPT_XY_SWAP |
					PAW3212_MOUSE_OPT_X_INV;
		}

		if (IS_ENABLED(CONFIG_PAW3212_12_BIT_MODE)) {
			mouse_option |= PAW3212_MOUSE_OPT_12BITMODE;
		}

		err = reg_write(dev, PAW3212_REG_MOUSE_OPTION,
				mouse_option);
	}
	if (!err) {
		err = reg_write(dev, PAW3212_REG_WRITE_PROTECT, 0);
	}

    if (err) {
        LOG_ERR("Config the sensor failed");
        return err;
    }

	return 0;
}


static void paw3212_async_init(struct k_work *work)
{
	struct paw3212_data *data = CONTAINER_OF(work, struct paw3212_data,
						 init_work.work);
	const struct device *dev = data->dev;

	LOG_DBG("PAW3212 async init step %d", data->async_init_step);

	data->err = async_init_fn[data->async_init_step](dev);
	if (data->err) {
		LOG_ERR("PAW3212 initialization failed");
	} else {
		data->async_init_step++;

		if (data->async_init_step == ASYNC_INIT_STEP_COUNT) {
			data->ready = true;
			LOG_INF("PAW3212 initialized");
		} else {
			k_work_schedule(&data->init_work,
					K_MSEC(async_init_delay[
						data->async_init_step]));
		}
	}
}

static int pmw3610_report_data(const struct device *dev) {
    struct pixart_data *data = dev->data;
    const struct pixart_config *config = dev->config;
    uint8_t buf[PMW3610_BURST_SIZE];

    if (unlikely(!data->ready)) {
        LOG_WRN("Device is not initialized yet");
        return -EBUSY;
    }

    static int64_t dx = 0;
    static int64_t dy = 0;

#if CONFIG_PMW3610_REPORT_INTERVAL_MIN > 0
    static int64_t last_smp_time = 0;
    static int64_t last_rpt_time = 0;
    int64_t now = k_uptime_get();
#endif

	int err = pmw3610_read(dev, PMW3610_REG_MOTION_BURST, buf, PMW3610_BURST_SIZE);
    if (err) {
        return err;
    }
    // LOG_HEXDUMP_DBG(buf, PMW3610_BURST_SIZE, "buf");

// 12-bit two's complement value to int16_t
// adapted from https://stackoverflow.com/questions/70802306/convert-a-12-bit-signed-number-in-c
#define TOINT16(val, bits) (((struct { int16_t value : bits; }){val}).value)

    int16_t x = TOINT16((buf[PMW3610_X_L_POS] + ((buf[PMW3610_XY_H_POS] & 0xF0) << 4)), 12);
    int16_t y = TOINT16((buf[PMW3610_Y_L_POS] + ((buf[PMW3610_XY_H_POS] & 0x0F) << 8)), 12);
    LOG_DBG("x/y: %d/%d", x, y);

#ifdef CONFIG_PMW3610_SMART_ALGORITHM
    int16_t shutter = ((int16_t)(buf[PMW3610_SHUTTER_H_POS] & 0x01) << 8)
                    + buf[PMW3610_SHUTTER_L_POS];
    if (data->sw_smart_flag && shutter < 45) {
        pmw3610_write(dev, 0x32, 0x00);
        data->sw_smart_flag = false;
    }
    if (!data->sw_smart_flag && shutter > 45) {
        pmw3610_write(dev, 0x32, 0x80);
        data->sw_smart_flag = true;
    }
#endif

#if CONFIG_PMW3610_REPORT_INTERVAL_MIN > 0
    // purge accumulated delta, if last sampled had not been reported on last report tick
    if (now - last_smp_time >= CONFIG_PMW3610_REPORT_INTERVAL_MIN) {
        dx = 0;
        dy = 0;
    }
    last_smp_time = now;
#endif

    // accumulate delta until report in next iteration
    dx += x;
    dy += y;

#if CONFIG_PMW3610_REPORT_INTERVAL_MIN > 0
    // strict to report inerval
    if (now - last_rpt_time < CONFIG_PMW3610_REPORT_INTERVAL_MIN) {
        return 0;
    }
#endif

    // fetch report value
    int16_t rx = (int16_t)CLAMP(dx, INT16_MIN, INT16_MAX);
    int16_t ry = (int16_t)CLAMP(dy, INT16_MIN, INT16_MAX);
    bool have_x = rx != 0;
    bool have_y = ry != 0;

    if (have_x || have_y) {
#if CONFIG_PMW3610_REPORT_INTERVAL_MIN > 0
        last_rpt_time = now;
#endif
        dx = 0;
        dy = 0;
        if (have_x) {
            input_report(dev, config->evt_type, config->x_input_code, rx, !have_y, K_NO_WAIT);
        }
        if (have_y) {
            input_report(dev, config->evt_type, config->y_input_code, ry, true, K_NO_WAIT);
        }
    }

    return err;
}

static void pmw3610_gpio_callback(const struct device *gpiob, struct gpio_callback *cb,
                                  uint32_t pins) {
    struct pixart_data *data = CONTAINER_OF(cb, struct pixart_data, irq_gpio_cb);
    const struct device *dev = data->dev;
    pmw3610_set_interrupt(dev, false);
    k_work_submit(&data->trigger_work);
}

static void pmw3610_work_callback(struct k_work *work) {
    struct pixart_data *data = CONTAINER_OF(work, struct pixart_data, trigger_work);
    const struct device *dev = data->dev;
    pmw3610_report_data(dev);
    pmw3610_set_interrupt(dev, true);
}

static int paw3212_init_irq(const struct device *dev)
{
	int err;
	struct paw3212_data *data = dev->data;
	const struct paw3212_config *config = dev->config;

	if (!device_is_ready(config->irq_gpio.port)) {
		LOG_ERR("IRQ GPIO device not ready");
		return -ENODEV;
	}

	err = gpio_pin_configure_dt(&config->irq_gpio, GPIO_INPUT);
	if (err) {
		LOG_ERR("Cannot configure IRQ GPIO");
		return err;
	}

	gpio_init_callback(&data->irq_gpio_cb, irq_handler,
			   BIT(config->irq_gpio.pin));

	err = gpio_add_callback(config->irq_gpio.port,
				&data->irq_gpio_cb);

	if (err) {
		LOG_ERR("Cannot add IRQ GPIO callback");
	}

	return err;
}

static int paw3212_init(const struct device *dev)
{
	struct paw3212_data *data = dev->data;
	const struct paw3212_config *config = dev->config;
	int err;

	/* Assert that negative numbers are processed as expected */
	__ASSERT_NO_MSG(-1 == expand_s12(0xFFF));

	k_work_init(&data->trigger_handler_work, trigger_handler);
	data->dev = dev;

	if (!spi_is_ready_dt(&config->bus)) {
		return -ENODEV;
	}

	if (!device_is_ready(config->cs_gpio.port)) {
		LOG_ERR("SPI CS device not ready");
		return -ENODEV;
	}

	err = gpio_pin_configure_dt(&config->cs_gpio, GPIO_OUTPUT_INACTIVE);
	if (err) {
		LOG_ERR("Cannot configure SPI CS GPIO");
		return err;
	}

	err = paw3212_init_irq(dev);
	if (err) {
		return err;
	}

	k_work_init_delayable(&data->init_work, paw3212_async_init);

	k_work_schedule(&data->init_work,
			K_MSEC(async_init_delay[data->async_init_step]));

	return err;
}

static int paw3212_attr_set(const struct device *dev, enum sensor_channel chan,
			    enum sensor_attribute attr,
			    const struct sensor_value *val)
{
	struct paw3212_data *data = dev->data;
	int err;

	if (unlikely(chan != SENSOR_CHAN_ALL)) {
		return -ENOTSUP;
	}

	if (unlikely(!data->ready)) {
		LOG_INF("Device is not initialized yet");
		return -EBUSY;
	}

	switch ((uint32_t)attr) {
	case PAW3212_ATTR_CPI:
		err = update_cpi(dev, PAW3212_SVALUE_TO_CPI(*val));
		break;

	case PAW3212_ATTR_SLEEP_ENABLE:
		err = toggle_sleep_modes(dev,
					 PAW3212_REG_OPERATION_MODE,
					 PAW3212_REG_CONFIGURATION,
					 PAW3212_SVALUE_TO_BOOL(*val));
		break;

	case PAW3212_ATTR_SLEEP1_TIMEOUT:
		err = update_sleep_timeout(dev,
					   PAW3212_REG_SLEEP1,
					   PAW3212_SVALUE_TO_TIME(*val));
		break;

	case PAW3212_ATTR_SLEEP2_TIMEOUT:
		err = update_sleep_timeout(dev,
					   PAW3212_REG_SLEEP2,
					   PAW3212_SVALUE_TO_TIME(*val));
		break;

	case PAW3212_ATTR_SLEEP3_TIMEOUT:
		err = update_sleep_timeout(dev,
					   PAW3212_REG_SLEEP3,
					   PAW3212_SVALUE_TO_TIME(*val));
		break;

	case PAW3212_ATTR_SLEEP1_SAMPLE_TIME:
		err = update_sample_time(dev,
					 PAW3212_REG_SLEEP1,
					 PAW3212_SVALUE_TO_TIME(*val));
		break;

	case PAW3212_ATTR_SLEEP2_SAMPLE_TIME:
		err = update_sample_time(dev,
					 PAW3212_REG_SLEEP2,
					 PAW3212_SVALUE_TO_TIME(*val));
		break;

	case PAW3212_ATTR_SLEEP3_SAMPLE_TIME:
		err = update_sample_time(dev,
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
	.sample_fetch = paw3212_sample_fetch,
	.channel_get  = paw3212_channel_get,
	.trigger_set  = paw3212_trigger_set,
	.attr_set     = paw3212_attr_set,
};

#define PMW3610_SPI_MODE (SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_MODE_CPOL | \
                        SPI_MODE_CPHA | SPI_TRANSFER_MSB)

#define PAW3212_DEFINE(n)						       \
	static struct paw3212_data data##n;				       \
									       \
	static const struct paw3212_config config##n = {		       \
		.irq_gpio = GPIO_DT_SPEC_INST_GET(n, irq_gpios),	       \
		.bus = {						       \
			    .bus = DEVICE_DT_GET(DT_INST_BUS(n)),		       \
				.config = {					       \
				.frequency = DT_INST_PROP(n, spi_max_frequency),  \
				.operation = SPI_WORD_SET(8) |		       \
					         SPI_TRANSFER_MSB |		       \
							 SPI_MODE_CPOL | SPI_MODE_CPHA,    \
				.slave = DT_INST_REG_ADDR(n),		       \
			},						       \
		},							       \
		.cs_gpio = SPI_CS_GPIOS_DT_SPEC_GET(DT_DRV_INST(n)),	       \
	};								       \
									       \
	DEVICE_DT_INST_DEFINE(n, paw3212_init, NULL, &data##n, &config##n,     \
			      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,	       \
			      &paw3212_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PAW3212_DEFINE)

ZMK_LISTENER(zmk_pmw3610_idle_sleeper, on_activity_state);
ZMK_SUBSCRIPTION(zmk_pmw3610_idle_sleeper, zmk_activity_state_changed);
