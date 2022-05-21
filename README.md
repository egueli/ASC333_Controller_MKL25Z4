# Pin mapping

| Description      | MCU pin name | FRDM board pin name | ASC333 IC:pin | ASC333 wire color | Notes                                                                                                      |
|------------------|--------------|---------------------|---------------|-------------------|------------------------------------------------------------------------------------------------------------|
| Ground           |              |                     |               | black             |                                                                                                            |
| Row bit 0        |              |                     | U3:1 (P1.0)   | brown             |                                                                                                            |
| Row bit 1        |              |                     | U3:2 (P1.1)   | red               |                                                                                                            |
| Row bit 2        |              |                     | U3:3 (P1.2)   | orange            |                                                                                                            |
| Row bit 3, green |              |                     | U3:4 (P1.3)   | yellow            |                                                                                                            |
| Row bit 3, red   |              |                     | U3:5 (P1.4)   | green             |                                                                                                            |
| Column clock     | `SPI0_CLK`   | D13                 | U9:6          | blue              |                                                                                                            |
| Column strobe    | `PTD2`       | D11                 | U9:3          | purple            | Do not use SPI0 SS: it strobes every 8 bits. This one must strobe only after a full 85-bit string is sent. |
| Column data      | `SPI0_SOUT`  | D12                 | U8:5          | gray              |                                                                                                            |
