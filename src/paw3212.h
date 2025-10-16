#pragma once

#include <zephyr/drivers/sensor.h>
#include "pixart.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Timings (in us) used in SPI communication. */
#define T_CLOCK_ON_DELAY_US 300

/* Sensor registers (addresses) */
#define PAW3212_REG_PRODUCT_ID		0x00
#define PAW3212_REG_REVISION_ID		0x01
#define PAW3212_REG_MOTION		0x02
#define PAW3212_REG_DELTA_X_LOW		0x03
#define PAW3212_REG_DELTA_Y_LOW		0x04
#define PAW3212_REG_OPERATION_MODE	0x05
#define PAW3212_REG_CONFIGURATION	0x06
#define PAW3212_REG_WRITE_PROTECT	0x09
#define PAW3212_REG_DELTA_XY_HIGH	0x12
#define PAW3212_REG_MOUSE_OPTION	0x19
#define PAW3212_REG_SLEEP1		0x0A
#define PAW3212_REG_SLEEP2		0x0B
#define PAW3212_REG_SLEEP3		0x0C
#define PAW3212_REG_CPI_X		0x0D
#define PAW3212_REG_CPI_Y		0x0E

/* CPI */
#define PAW3212_CPI_STEP		38u
#define PAW3212_CPI_MIN			(0x00 * PAW3212_CPI_STEP)
#define PAW3212_CPI_MAX			(0x3F * PAW3212_CPI_STEP)

/* Sleep */
#define PAW3212_SLP_ENH_POS		4u
#define PAW3212_SLP2_ENH_POS		3u
#define PAW3212_SLP3_ENH_POS		5u

#define PAW3212_ETM_POS			0u
#define PAW3212_ETM_SIZE		4u
#define PAW3212_FREQ_POS		(PAW3212_ETM_POS + PAW3212_ETM_SIZE)
#define PAW3212_FREQ_SIZE		4u

#define PAW3212_ETM_MIN			0u
#define PAW3212_ETM_MAX			BIT_MASK(PAW3212_ETM_SIZE)
#define PAW3212_ETM_MASK		(PAW3212_ETM_MAX << PAW3212_ETM_POS)

#define PAW3212_FREQ_MIN		0u
#define PAW3212_FREQ_MAX		BIT_MASK(PAW3212_FREQ_SIZE)
#define PAW3212_FREQ_MASK		(PAW3212_FREQ_MAX << PAW3212_FREQ_POS)

/* Motion status bits */
#define PAW3212_MOTION_STATUS_MOTION	BIT(7)
#define PAW3212_MOTION_STATUS_DYOVF	BIT(4)
#define PAW3212_MOTION_STATUS_DXOVF	BIT(3)

/* Configuration bits */
#define PAW3212_CONFIG_CHIP_RESET	BIT(7)

/* Mouse option bits */
#define PAW3212_MOUSE_OPT_XY_SWAP	BIT(5)
#define PAW3212_MOUSE_OPT_Y_INV		BIT(4)
#define PAW3212_MOUSE_OPT_X_INV		BIT(3)
#define PAW3212_MOUSE_OPT_12BITMODE	BIT(2)

/* Convert deltas to x and y */
#define PAW3212_DELTA_X(xy_high, x_low)	expand_s12((((xy_high) & 0xF0) << 4) | (x_low))
#define PAW3212_DELTA_Y(xy_high, y_low)	expand_s12((((xy_high) & 0x0F) << 8) | (y_low))

/* Sensor identification values */
#define PAW3212_PRODUCT_ID		0x30

/* Write protect magic */
#define PAW3212_WPMAGIC			0x5A

#define SPI_WRITE_BIT			BIT(7)

/* Helper macros used to convert sensor values. */
#define PAW3212_SVALUE_TO_CPI(svalue) ((uint32_t)(svalue).val1)
#define PAW3212_SVALUE_TO_TIME(svalue) ((uint32_t)(svalue).val1)
#define PAW3212_SVALUE_TO_BOOL(svalue) ((svalue).val1 != 0)

/** @brief Sensor specific attributes of PMW3610. */
enum pmw3612_attribute {

	/** Sensor CPI for both X and Y axes. */
	PAW3212_ATTR_CPI,

	/** Entering time from Run mode to REST1 mode [ms]. */
	PAW3212_ATTR_RUN_DOWNSHIFT_TIME,

	/** Entering time from REST1 mode to REST2 mode [ms]. */
	PAW3212_ATTR_SLEEP1_DOWNSHIFT_TIME,

	/** Entering time from REST2 mode to REST3 mode [ms]. */
	PAW3212_ATTR_SLEEP2_DOWNSHIFT_TIME,

	/** Sampling frequency time during REST1 mode [ms]. */
	PAW3212_ATTR_SLEEP1_SAMPLE_TIME,

	/** Sampling frequency time during REST2 mode [ms]. */
	PAW3212_ATTR_SLEEP2_SAMPLE_TIME,

	/** Sampling frequency time during REST3 mode [ms]. */
	PAW3212_ATTR_SLEEP3_SAMPLE_TIME,

};

#ifdef __cplusplus
}
#endif
