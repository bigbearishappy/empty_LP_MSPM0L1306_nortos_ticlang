## Summary

Empty project using DriverLib.
This project control the bootmode of AM62 and provide the ADC value to soc via i2c.

## Peripherals & Pin Assignments

| Peripheral | Pin | Function |
| --- | --- | --- |
| SYSCTL |  |  |
| DEBUGSS | PA20 | Debug Clock |
| DEBUGSS | PA19 | Debug Data In Out |
| I2C | PA0 | I2C slave data |
| I2C | PA1 | I2C slave clk |
| BOOTMODE3 | PA3 | bootmode pin3 |
| BOOTMODE4 | PA4 | bootmode pin4 |
| BOOTMODE5 | PA9 | bootmode pin5 |
| BOOTMODE6 | PA10 | bootmode pin6 |
| BOOTMODE7 | PA11 | bootmode pin7 |
| BOOTMODE8 | PA15 | bootmode pin8 |
| BOOTMODE9 | PA16 | bootmode pin9 |
| SOC_EN| PA17 | enable pin for soc/BSL_invoke|
| BOOTMODE14 | PA18 | bootmode pin14|
| SWDIO | PA19 | SWDIO |
| SWCLK | PA20 | SWCLK|
| BOOT_SEL0 | PA21 | bootmode select pin0 |
| BOOTMODE11 | PA22 | bootmode pin11 |
| BOOTMODE10 | PA23 | bootmode pin10 |
| ADC1 | PA24 | adc channel1 |
| ADC0 | PA25 | adc channel0 |
| BOOT_SEL1 | PA26 | bootmode select pin1 |

## Usage
### Get firmware version
```
i2ctransfer -y 1 w1@0x13 0x00 r2
```
### example output(version 0.1)
```
0x00 0x01

```
### Enter BSL mode
```
i2ctransfer -y 1 w1@0x13 0x01 r2
```
### Get ADC value
```
i2ctransfer -y 1 w2@0x13 0x02 0x00 r2
i2ctransfer -y 1 w2@0x13 0x02 0x01 r2
```
### example output(ADC0:0x00cb ADC1:0x00cf)
```
0x00 0xcb
0x00 0xcf

```
### Get bootmode data
```
i2ctransfer -y 1 w1@0x13 0x03 r2
```
### example output(bootmode:0xb623)
```
0xb6 0x23

```
