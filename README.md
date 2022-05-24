# ASC 333 Led Marquee controller with Kinetis KL25

## TODO

- [x] Drive column lines via SPI0 (FreeRTOS)
- [x] Drive row lines via GPIO (FreeRTOS)
- [ ] Display fixed image
- [ ] Get image data via SPI

# Pin mapping

| Description      | MCU pin name | FRDM board pin name | ASC333 IC:pin | ASC333 wire color | Notes                                                                                                      |
|------------------|--------------|---------------------|---------------|-------------------|------------------------------------------------------------------------------------------------------------|
| Ground           |              |                     |               | black             |                                                                                                            |
| Row bit 0        | `PTA5`       | D5                  | U3:1 (P1.0)   | brown             |                                                                                                            |
| Row bit 1        | `PTC8`       | D6                  | U3:2 (P1.1)   | red               |                                                                                                            |
| Row bit 2        | `PTA13`      | D8                  | U3:3 (P1.2)   | orange            |                                                                                                            |
| Row bit 3, green | `PTD5`       | D9                  | U3:4 (P1.3)   | yellow            |                                                                                                            |
| Row bit 3, red   | `PTD0`       | D10                 | U3:5 (P1.4)   | green             |                                                                                                            |
| Column clock     | `SPI0_CLK`   | D13                 | U9:6          | blue              |                                                                                                            |
| Column strobe    | `PTD2`       | D11                 | U9:3          | purple            | Do not use SPI0 SS: it strobes every 8 bits. This one must strobe only after a full 85-bit string is sent. |
| Column data      | `SPI0_SOUT`  | D12                 | U8:5          | gray              |                                                                                                            |
