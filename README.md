# PAW3212 driver implementation for ZMK

This driver is a port of the Nordic PAW3212 sensor driver to ZMK, following the same structure and patterns as the PMW3610 ZMK driver.

## Overview

The PAW3212 is a low-power optical motion tracking sensor with the following features:
- CPI range: 0-2394 (in steps of 38)
- 8-bit or 12-bit motion reporting modes
- Configurable sleep modes for power saving
- Orientation configuration (90°, 180°, 270° rotation)

## Installation

Include this project in ZMK's west manifest in `config/west.yml`:

```yml
manifest:
  remotes:
    ...
    - name: your-remote
      url-base: https://github.com/your-username
    ...
  projects:
    ...
    - name: zmk-paw3212-driver
      remote: your-remote
      revision: main
    ...
  self:
    path: config
```

Update `board.overlay` adding the necessary bits (update the pins for your board accordingly):

```dts
&pinctrl {
    spi0_default: spi0_default {
        group1 {
            psels = <NRF_PSEL(SPIM_SCK, 0, 8)>,
                <NRF_PSEL(SPIM_MOSI, 0, 17)>,
                <NRF_PSEL(SPIM_MISO, 0, 17)>;
        };
    };

    spi0_sleep: spi0_sleep {
        group1 {
            psels = <NRF_PSEL(SPIM_SCK, 0, 8)>,
                <NRF_PSEL(SPIM_MOSI, 0, 17)>,
                <NRF_PSEL(SPIM_MISO, 0, 17)>;
            low-power-enable;
        };
    };
};

#include <zephyr/dt-bindings/input/input-event-codes.h>

&spi0 {
    status = "okay";
    compatible = "nordic,nrf-spim";
    pinctrl-0 = <&spi0_default>;
    pinctrl-1 = <&spi0_sleep>;
    pinctrl-names = "default", "sleep";
    cs-gpios = <&gpio0 20 GPIO_ACTIVE_LOW>;

    trackball: trackball@0 {
        status = "okay";
        compatible = "pixart,paw3212";
        reg = <0>;
        spi-max-frequency = <2000000>;
        irq-gpios = <&gpio0 6 (GPIO_ACTIVE_LOW | GPIO_PULL_UP)>;
        evt-type = <INPUT_EV_REL>;
        x-input-code = <INPUT_REL_X>;
        y-input-code = <INPUT_REL_Y>;
    };
};

/ {
  trackball_listener {
    compatible = "zmk,input-listener";
    device = <&trackball>;
  };
};
```

Enable the driver config in `<shield>.config` file:

```conf
CONFIG_SPI=y
CONFIG_INPUT=y
CONFIG_ZMK_POINTING=y
CONFIG_PAW3212=y
# CONFIG_PAW3212_ORIENTATION_90=y    # Optional: 90 degree rotation
# CONFIG_PAW3212_ORIENTATION_180=y   # Optional: 180 degree rotation
# CONFIG_PAW3212_ORIENTATION_270=y   # Optional: 270 degree rotation
# CONFIG_PAW3212_12_BIT_MODE=y       # Optional: Enable 12-bit mode (default is 8-bit)
# CONFIG_PAW3212_LOG_LEVEL_DBG=y     # Optional: Enable debug logging
```

## Features

- **Async initialization**: Non-blocking initialization process
- **ZMK activity integration**: Automatically enables/disables sleep modes based on ZMK activity state
- **Input subsystem**: Uses Zephyr's input API for seamless integration with ZMK
- **Power management**: Configurable sleep modes (Sleep1, Sleep2, Sleep3) for power saving
- **Orientation support**: Hardware-level orientation configuration
- **12-bit mode**: Optional high-precision reporting mode

## Configuration

The driver supports the following attributes that can be configured at runtime:

- `PAW3212_ATTR_CPI`: Set CPI (Counts Per Inch)
- `PAW3212_ATTR_SLEEP_ENABLE`: Enable/disable sleep modes
- `PAW3212_ATTR_SLEEP1_TIMEOUT`: Sleep1 timeout in ms
- `PAW3212_ATTR_SLEEP2_TIMEOUT`: Sleep2 timeout in ms
- `PAW3212_ATTR_SLEEP3_TIMEOUT`: Sleep3 timeout in ms
- `PAW3212_ATTR_SLEEP1_SAMPLE_TIME`: Sleep1 sample time in ms
- `PAW3212_ATTR_SLEEP2_SAMPLE_TIME`: Sleep2 sample time in ms
- `PAW3212_ATTR_SLEEP3_SAMPLE_TIME`: Sleep3 sample time in ms

## Differences from Nordic Driver

This ZMK port differs from the original Nordic driver in the following ways:

1. **Input API instead of Sensor API**: Uses Zephyr's `input_report()` instead of the sensor driver API
2. **ZMK integration**: Integrates with ZMK's activity state management for automatic power saving
3. **Manual CS control**: Uses manual chip select control via GPIO instead of SPI subsystem CS
4. **Device tree configuration**: Input codes and event types are configured via device tree

## Credits

Based on:
- Nordic Semiconductor's PAW3212 sensor driver (nRF Connect SDK)
- ZMK PMW3610 driver implementation patterns
- Zephyr RTOS SPI and GPIO APIs
