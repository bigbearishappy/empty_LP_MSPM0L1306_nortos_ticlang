/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"
#include <stdio.h>
#include <string.h>

#define MAIN_VER 0
#define SUB_VER 1

typedef enum {
    /* Get the firmware version*/
    REG_VERSION = 0x00,
    /* Reboot to BSL mode for MCU firmware update*/
    REG_ENTER_BSL,
    /* Get ADC value of specific channel*/
    REG_GET_ADC,
    /* Update the bootmode pins default value for XSPI boot*/
    REG_UPDATE_XSPI_DATA,
    REG_MAX
}REG_ADDR;

/* ADC data channel 0*/
#define REG_ADC_CHN0 0x00
/* ADC data channel 1*/
#define REG_ADC_CHN1 0x01

struct bootmode_pins {
    GPIO_Regs* pin_port;
    uint32_t pin_num;
    uint32_t pin_iomux;
};
typedef struct bootmode_pins BootMode_Pins;

#define I2C_TX_MAX_PACKET_SIZE (16)
#define I2C_RX_MAX_PACKET_SIZE (16)

struct i2c_slave_data {
    uint8_t gTxPacket[I2C_TX_MAX_PACKET_SIZE];
    uint32_t gTxCount;

    uint8_t gRxPacket[I2C_RX_MAX_PACKET_SIZE];
    uint32_t gRxCount;
};
typedef struct i2c_slave_data I2C_Slave_Data;

uint16_t bootmode = 0x0000;

BootMode_Pins bootmode_buf[] = {
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0},
    {BOOTMODE_PORT, BOOTMODE_BOOTMODE3_PIN, BOOTMODE_BOOTMODE3_IOMUX},
    {BOOTMODE_PORT, BOOTMODE_BOOTMODE4_PIN, BOOTMODE_BOOTMODE4_IOMUX},
    {BOOTMODE_PORT, BOOTMODE_BOOTMODE5_PIN, BOOTMODE_BOOTMODE5_IOMUX},
    {BOOTMODE_PORT, BOOTMODE_BOOTMODE6_PIN, BOOTMODE_BOOTMODE6_IOMUX},
    {BOOTMODE_PORT, BOOTMODE_BOOTMODE7_PIN, BOOTMODE_BOOTMODE7_IOMUX},
    {BOOTMODE_PORT, BOOTMODE_BOOTMODE8_PIN, BOOTMODE_BOOTMODE8_IOMUX},
    {BOOTMODE_PORT, BOOTMODE_BOOTMODE9_PIN, BOOTMODE_BOOTMODE9_IOMUX},
    {BOOTMODE_PORT, BOOTMODE_BOOTMODE10_PIN, BOOTMODE_BOOTMODE10_IOMUX},
    {BOOTMODE_PORT, BOOTMODE_BOOTMODE11_PIN, BOOTMODE_BOOTMODE11_IOMUX},
    {0, 0, 0},
    {0, 0, 0},
    {BOOTMODE_PORT, BOOTMODE_BOOTMODE14_PIN, BOOTMODE_BOOTMODE14_IOMUX},
    {0, 0, 0},
};

I2C_Slave_Data i2c_slave;

int main(void)
{
    SYSCFG_DL_init();

    uint32_t bootsel0 = 0x00, bootsel1 = 0x00;
    uint16_t bootmode_tmp;

    /* Enable the timer for WWDT feeding */
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN); 
    DL_TimerG_startCounter(TIMER_0_INST);

    DL_GPIO_initDigitalInputFeatures(BOOT_SEL_BOOT_SEL0_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(BOOT_SEL_BOOT_SEL1_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    bootsel0 = !!DL_GPIO_readPins(BOOT_SEL_PORT, BOOT_SEL_BOOT_SEL0_PIN);
    bootsel1 = !!DL_GPIO_readPins(BOOT_SEL_PORT, BOOT_SEL_BOOT_SEL1_PIN);

    /*
    bootsel0:bootsel1(default 1:1)
    1:1 -> eMMC
    1:0 -> SD Card
    0:1 -> xSPI
    0:0 -> Ethernet
    */
    switch((bootsel1 << 4) + bootsel0)
    {
        case 0x11:  //1:1 -> eMMC
            bootmode = 0xb64b;
        break;
        case 0x10:  //1:0 -> SD Card
            bootmode = 0xb643;
        break;
        case 0x01:  //0:1 -> xSPI
            /* read the current xSPI boot data here if needed*/
            bootmode = 0xb673;
        break;
        case 0x00:  //0:0 -> Ethernet
            bootmode = 0xb623;
        break;
        default:
        break;
    }

    /* Config the GPIO for bootmode*/
    bootmode_tmp = bootmode;
    for (int i = 0; i < 16; ++i) {
        if(bootmode_buf[i].pin_port != 0) {
            if(bootmode_tmp & 0x0001) {
                DL_GPIO_setPins(bootmode_buf[i].pin_port, bootmode_buf[i].pin_num);
            } else {
                DL_GPIO_clearPins(bootmode_buf[i].pin_port, bootmode_buf[i].pin_num);
            }
        }
        bootmode_tmp = bootmode_tmp >> 1;
    }

    /* Enable the SOC chip after the bootmode pins is ready*/
    DL_GPIO_setPins(SOC_ENABLE_PORT, SOC_ENABLE_SOC_EN_PIN);

    /*Delay for the AM62 to detect the bootmode*/
    delay_cycles(16000000);

    /* Release the bootmode pins for linux kernel*/
    for (int i = 0; i < 16; ++i) {
        if(bootmode_buf[i].pin_port != 0) {
            DL_GPIO_enableHiZ(bootmode_buf[i].pin_iomux);
        }
    }

    /* Start ADC convert*/
    DL_ADC12_startConversion(ADC12_0_INST);

    /* Init the I2C slave interrupt*/
    i2c_slave.gTxCount = 0;
    i2c_slave.gRxCount = 0;
    DL_I2C_enableInterrupt(I2C_0_INST, DL_I2C_INTERRUPT_TARGET_TXFIFO_TRIGGER);
    NVIC_EnableIRQ(I2C0_INT_IRQn);

    while (1) {
    }
}

void I2C_cmd_Handler(uint8_t *rx_buf, uint32_t rx_len, uint8_t *tx_buf, uint32_t *tx_len)
{
    uint16_t adc_value = 0x0000;

    if(!rx_buf || !tx_buf) {
        return ;
    }

    switch (rx_buf[0]) {
        case REG_VERSION:
            tx_buf[0] = MAIN_VER;
            tx_buf[1] = SUB_VER;
            *tx_len = 2;
        break;
        case REG_ENTER_BSL:
            /* You can detect a i2c slave device with addr 0x48 after reboot */
            /* Force Reset calling BSL entry*/
            DL_SYSCTL_resetDevice(DL_SYSCTL_RESET_BOOTLOADER_ENTRY);
        break;
        case REG_GET_ADC:
            if(rx_len == 2) {
                if(rx_buf[1] == REG_ADC_CHN0)
                    adc_value = DL_ADC12_getMemResult(ADC12_0_INST, DL_ADC12_MEM_IDX_2);
                else if(rx_buf[1] == REG_ADC_CHN1)
                    adc_value = DL_ADC12_getMemResult(ADC12_0_INST, DL_ADC12_MEM_IDX_3);
                
                tx_buf[0] = (adc_value >> 8) & 0xff;
                tx_buf[1] = adc_value & 0xff;
                *tx_len = 2;
            }
        break;
        case REG_UPDATE_XSPI_DATA:
            if(rx_len == 1) {
                /* Get the current xSPI boot data*/
                tx_buf[0] = (bootmode >> 8) & 0xff;
                tx_buf[1] = bootmode & 0xff;
                *tx_len = 2;
            } else if(rx_len == 3) {
                /* Write new xSPI boot data*/
            }
        break;
        default:
        break;
    }
}

void I2C_0_INST_IRQHandler(void)
{
    static bool dataRx = false;

    switch (DL_I2C_getPendingInterrupt(I2C_0_INST)) {
        case DL_I2C_IIDX_TARGET_START:
            /* Initialize RX or TX after Start condition is received */
            i2c_slave.gTxCount = 0;
            i2c_slave.gRxCount = 0;
            dataRx   = false;
            /* Flush TX FIFO to refill it */
            DL_I2C_flushTargetTXFIFO(I2C_0_INST);
            break;
        case DL_I2C_IIDX_TARGET_RXFIFO_TRIGGER:
            /* Store received data in buffer */
            dataRx = true;
            while (DL_I2C_isTargetRXFIFOEmpty(I2C_0_INST) != true) {
                if (i2c_slave.gRxCount < I2C_RX_MAX_PACKET_SIZE) {
                    i2c_slave.gRxPacket[i2c_slave.gRxCount++] = DL_I2C_receiveTargetData(I2C_0_INST);
                } else {
                    /* Prevent overflow and just ignore data */
                    DL_I2C_receiveTargetData(I2C_0_INST);
                }
            }
            I2C_cmd_Handler((uint8_t *)&i2c_slave.gRxPacket, i2c_slave.gRxCount, (uint8_t *)&i2c_slave.gTxPacket, &i2c_slave.gTxCount);
            break;
        case DL_I2C_IIDX_TARGET_TXFIFO_TRIGGER:
            /* Fill TX FIFO if there are more bytes to send */
            if (i2c_slave.gTxCount < I2C_TX_MAX_PACKET_SIZE) {
                i2c_slave.gTxCount += DL_I2C_fillTargetTXFIFO(
                    I2C_0_INST, &i2c_slave.gTxPacket[i2c_slave.gTxCount], (I2C_TX_MAX_PACKET_SIZE - i2c_slave.gTxCount));
            } else {
                /*
                 * Fill FIFO with 0x00 if more data is requested than
                 * expected I2C_TX_MAX_PACKET_SIZE
                 */
                while (DL_I2C_transmitTargetDataCheck(I2C_0_INST, 0x00) != false)
                    ;
            }
            break;
        case DL_I2C_IIDX_TARGET_STOP:
            /* If data was received, echo to TX buffer */
            if (dataRx == true) {
                for (uint16_t i = 0;
                     (i < i2c_slave.gRxCount) && (i < I2C_TX_MAX_PACKET_SIZE); i++) {
                    i2c_slave.gTxPacket[i] = i2c_slave.gRxPacket[i];
                    DL_I2C_flushTargetTXFIFO(I2C_0_INST);
                }
                dataRx = false;
            }
            break;
        case DL_I2C_IIDX_TARGET_RX_DONE:
            /* Not used for this example */
        case DL_I2C_IIDX_TARGET_RXFIFO_FULL:
            /* Not used for this example */
        case DL_I2C_IIDX_TARGET_GENERAL_CALL:
            /* Not used for this example */
        case DL_I2C_IIDX_TARGET_EVENT1_DMA_DONE:
            /* Not used for this example */
        case DL_I2C_IIDX_TARGET_EVENT2_DMA_DONE:
            /* Not used for this example */
        default:
            break;
    }
}

void TIMER_0_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(TIMER_0_INST)) {
        case DL_TIMER_IIDX_ZERO:
            /* Restart WWDT timer */
            DL_WWDT_restart(WWDT0_INST);
            break;
        default:
            break;
    }
}

