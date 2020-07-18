## HUB75E driver with Tang Nano FPGA and WiFi adaptor with M5ATOM

The driver part by Tang Nano for 64x64 LED panels with the HUB75E
interface is based on https://github.com/shinyblink/sled-fpga-hub75.

The WiFi part by M5ATOM is an esp-idf application. It receives RGB565
udp packet format of [LED master](https://github.com/spectrenoir06/LED_master)
and writes the frame data to FPGA with SPI.

The main SPI should assert CS, write 64x64(4096) RGB555 words and negate CS
with mode0. The HUB75E panel is divided to 2 64x32 planes and the SPI data
should be organized as

```
   word 0, word 2048, word 1, word 2049, ... , word 2047, word 4095
```

so as to get 2 words for both panels at the same time with one 32-bit data
from BRAM.

This driver does nothing to sync frame read/write from/to BRAM, the images
displayed on the top and bottom panels may be out of sync.

I use 16Mhz clock for the base clock to minimize the artifacts on my panel,
though there may be panels which can be ok with the higher clocks. SPI
requires the base clock which is at least x4 the SCLK, i.e. the maximal
SCLK is limited by 4Mhz for the 16Mhz base clock.

All sources are covered by ISC License, except src/hub75e.{sdc,cst} files
which are generated files by GOWIN IDE.

![simple schematic](https://github.com/kazkojima/hub75e-tang-nano/blob/master/other/hub75e-nano.png)
