/**
 * @file    main.c
 * @brief   SPI Master Demo
 * @details Shows Master loopback demo for SPI
 *          Read the printf() for instructions
 */

/******************************************************************************
 * Copyright (C) 2023 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *
 ******************************************************************************/

/***** Includes *****/
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "board.h"
#include "dma.h"
#include "mxc_device.h"
#include "mxc_delay.h"
#include "mxc_pins.h"
#include "nvic_table.h"
#include "icc.h"
#include "spi.h"
#include "uart.h"
#include "gpio.h"
#include "led.h"

/***** Pre-processors *****/
#define MASTERSYNC 1
#define MASTERASYNC 0
#define MASTERDMA 0

#if (!(MASTERSYNC || MASTERASYNC || MASTERDMA))
#error "You must set either MASTERSYNC or MASTERASYNC or MASTERDMA to 1."
#endif
#if ((MASTERSYNC && MASTERASYNC) || (MASTERASYNC && MASTERDMA) || (MASTERDMA && MASTERSYNC))
#error "You must select either MASTERSYNC or MASTERASYNC or MASTERDMA, not all 3."
#endif

/***** Definitions *****/
#define DATA_LEN 120000 // Words
#define DATA_VALUE 0xA5A5 // This is for master mode only...
#define VALUE 0xFFFF
#define SPI_SPEED 50000000 // Bit Rate

//#define AUTO_CYCLE

#ifdef BOARD_EVKIT_V1
#define SPI_INSTANCE_NUM 0
#define SPI MXC_SPI1
#define SPI_IRQ SPI1_IRQn
#else
#define SPI_INSTANCE_NUM 1
#define SPI MXC_SPI0
#define SPI_IRQ SPI0_IRQn
#endif

/***** Globals *****/
static int g_stream_buffer_size = 64;
static int g_dma_spi_tx = 0;
static int g_dma_spi_rx = 1;
static int g_dma_already_setup = 0;
static int g_sample_size = 2;
//uint16_t rx_data[DATA_LEN];
//uint16_t tx_data[DATA_LEN];
static uint8_t rx_data_8[DATA_LEN];
static uint8_t tx_data_8[6];
mxc_spi_req_t req;
mxc_spi_pins_t spi_pins;
//mxc_spi_regs_t* spi;
int ssel = 1;

volatile int SPI_FLAG;
volatile uint32_t DMA0_FLAG = 0;
volatile uint32_t DMA1_FLAG = 0;
volatile uint32_t GPIO_FLAG = 0;
//// ADI AD4696 register addresses

#define SPI_CONFIG_A    0x0000
#define SPI_CONFIG_B    0x0001

#define DEVICE_TYPE     0x0003
#define SCRATCH_PAD     0x000A
#define VENDOR_L        0x000C
#define VENDOR_H        0x000D

#define LOOP_MODE       0x000E
#define SPI_CONFIG_C    0x0010

#define SPI_STATUS      0x0011
#define STATUS          0x0014


#define SETUP           0x0020
#define REF_CTRL        0x0021
#define SEQ_CTRL        0x0022
#define AC_CTRL         0x0023
#define STD_SEQ_CONFIG  0x0024

#define GPIO_CTRL       0x0026
#define GP_MODE         0x0027
#define GPIO_STATE      0x0028
#define TEMP_CTRL       0x0029
#define CONFIG_IN0      0x0030

// the following two lines are slow to use to measure a high speed loop
#define DEBUG_PIN_1H MXC_GPIO_OutSet(MXC_GPIO1,MXC_GPIO_PIN_0);
#define DEBUG_PIN_1L MXC_GPIO_OutClr(MXC_GPIO1,MXC_GPIO_PIN_0);

#define DEBUG_PIN_H *((volatile uint32_t *) (0x4000901C)) = 0x1;
#define DEBUG_PIN_L *((volatile uint32_t *) (0x40009020)) = 0x1;

int AD4696_command_convert(uint8_t value,uint8_t len);

void print_GPIO(uint32_t port){
	printf("\n***** GPIO_%d REGISTERS *****\n",port);
	//format
	// 	printf("0x:%08X\n",     *(  (volatile uint32_t *) <address in hex>  )      );
	//
    if (port == MXC_GPIO_PORT_3) {
        printf(" GPIO3 is not supported by this function!\n");
        return;
    }
    uint32_t offset =0;
    switch (port) {
        case MXC_GPIO_PORT_0:
            offset = 0x0000;
            break;
        case MXC_GPIO_PORT_1:
            offset = 0x1000;
            break;
        case MXC_GPIO_PORT_2:
            offset = 0x78400;
            break;
        default:
            offset = 0x1000;
            break;
    }

	uint32_t gpio_base = 0x40008000 + offset;
	printf("00 - EN0:  %08X\n",*((volatile uint32_t *) (gpio_base + 0x00)));
	printf("0C - OUTEN:%08X\n",*((volatile uint32_t *) (gpio_base + 0x0C)));
	printf("18 - OUT:  %08X\n",*((volatile uint32_t *) (gpio_base + 0x18)));
	printf("24 - IN:   %08X\n",*((volatile uint32_t *) (gpio_base + 0x24)));
	printf("60 - PAD0: %08X\n",*((volatile uint32_t *) (gpio_base + 0x60)));
	printf("64 - PAD1: %08X\n",*((volatile uint32_t *) (gpio_base + 0x64)));
	printf("68 - EN1:  %08X\n",*((volatile uint32_t *) (gpio_base + 0x68)));
	printf("74 - EN2:  %08X\n",*((volatile uint32_t *) (gpio_base + 0x74)));
	printf("B0 - DS0:  %08X\n",*((volatile uint32_t *) (gpio_base + 0xB0)));
	printf("B4 - DS11: %08X\n",*((volatile uint32_t *) (gpio_base + 0xB4)));
    printf("B8 - PS:   %08X\n",*((volatile uint32_t *) (gpio_base + 0xB8)));
	printf("C0 - VSSL: %08X\n",*((volatile uint32_t *) (gpio_base + 0xC0)));
	printf("*************************\n");

}

void print_SPI(void){
	printf("\n***** SPI REGISTERS *****\n");
	//format
	// 	printf("0x:%08X\n",     *(  (volatile uint32_t *) <address in hex>  )      );
	//
	uint32_t spi_base = 0x400BE000;
	printf("04 - CTRL0:  %08X\n",*((volatile uint32_t *) (spi_base + 0x04)));
	printf("08 - CTRL1:  %08X\n",*((volatile uint32_t *) (spi_base + 0x08)));
	printf("0C - CTRL2:  %08X\n",*((volatile uint32_t *) (spi_base + 0x0C)));
	printf("10 - SSTIM:  %08X\n",*((volatile uint32_t *) (spi_base + 0x10)));
	printf("14 - CKCTR:  %08X\n",*((volatile uint32_t *) (spi_base + 0x14)));
	printf("1C - DMA:    %08X\n",*((volatile uint32_t *) (spi_base + 0x1C)));
	printf("20 - INTFL:  %08X\n",*((volatile uint32_t *) (spi_base + 0x20)));
	printf("24 - INTEN:  %08X\n",*((volatile uint32_t *) (spi_base + 0x24)));
	printf("28 - WKFL:   %08X\n",*((volatile uint32_t *) (spi_base + 0x28)));
    printf("2C - WKEN:   %08X\n",*((volatile uint32_t *) (spi_base + 0x2C)));
	printf("30 - STAT:   %08X\n",*((volatile uint32_t *) (spi_base + 0x30)));
	printf("*************************\n");

}

void print_DMA(void){
    printf("\n***** DMA REGISTERS *****\n");
    //format
    // 	printf("0x:%08X\n",     *(  (volatile uint32_t *) <address in hex>  )      );
    //
    uint32_t dma_base = (0x40028000+0x0100);
    printf("00 - CH0CTRL: %08X\n",*((volatile uint32_t *) (dma_base + 0x00)));
    printf("04 - CH0STAT: %08X\n",*((volatile uint32_t *) (dma_base + 0x04)));
    printf("08 - CH0SRC:  %08X\n",*((volatile uint32_t *) (dma_base + 0x08)));
    printf("0C - CH0DST:  %08X\n",*((volatile uint32_t *) (dma_base + 0x0C)));
    printf("10 - CH0CNT:  %08X\n\n",*((volatile uint32_t *) (dma_base + 0x10)));

    dma_base = (0x40028000+0x0120);
    printf("00 - CH1CTRL: %08X\n",*((volatile uint32_t *) (dma_base + 0x00)));
    printf("04 - CH1STAT: %08X\n",*((volatile uint32_t *) (dma_base + 0x04)));
    printf("08 - CH1SRC:  %08X\n",*((volatile uint32_t *) (dma_base + 0x08)));
    printf("0C - CH1DST:  %08X\n",*((volatile uint32_t *) (dma_base + 0x0C)));
    printf("10 - CH1CNT:  %08X\n\n",*((volatile uint32_t *) (dma_base + 0x10)));

    printf("*************************\n");

}

inline void DEBUG_TOGGLE_H2L(void){
    DEBUG_PIN_H;
    DEBUG_PIN_L;
}

inline void DEBUG_TOGGLE_L2H(void){
    DEBUG_PIN_L;
    DEBUG_PIN_H;
}

// configure AD4696 nRESET(p1.1), CNV(p1.6) and BUSY(p0.19). p1.0 is debug pin
const mxc_gpio_cfg_t ad4696_pins[] = {
    { MXC_GPIO1, MXC_GPIO_PIN_6, MXC_GPIO_FUNC_IN, MXC_GPIO_PAD_WEAK_PULL_UP, MXC_GPIO_VSSEL_VDDIO },  //CNV
    { MXC_GPIO1, MXC_GPIO_PIN_1, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO },  //nRESET
    { MXC_GPIO0, MXC_GPIO_PIN_19, MXC_GPIO_FUNC_IN, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO },  //BUSY
    { MXC_GPIO1, MXC_GPIO_PIN_0, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO },  //Debug
};

const unsigned int ad4696_num_gpios = (sizeof(ad4696_pins) / sizeof(mxc_gpio_cfg_t));

//TODO fix register descriptions
typedef struct {
    __IO uint8_t  spi_config_a;                  /**< <tt>\b 0x00:</tt> GPIO_REVA EN0 Register */
    __IO uint8_t  spi_config_b;              /**< <tt>\b 0x04:</tt> GPIO_REVA EN0_SET Register */
    __R  uint8_t  rsv_0x0002;              /**< <tt>\b 0x08:</tt> GPIO_REVA EN0_CLR Register */
    __I  uint8_t  device_type;                /**< <tt>\b 0x0C:</tt> GPIO_REVA OUTEN Register */
    __R  uint8_t  rsv_0x0004_0x0009[6];            /**< <tt>\b 0x10:</tt> GPIO_REVA OUTEN_SET Register */
    __IO uint8_t  scratch_pad;            /**< <tt>\b 0x14:</tt> GPIO_REVA OUTEN_CLR Register */
    __R  uint8_t  rsv_0x000b;                  /**< <tt>\b 0x18:</tt> GPIO_REVA OUT Register */
    __I  uint8_t  vendor_l;              /**< <tt>\b 0x1C:</tt> GPIO_REVA OUT_SET Register */
    __I  uint8_t  vendor_h;              /**< <tt>\b 0x20:</tt> GPIO_REVA OUT_CLR Register */
    __IO uint8_t  loop_mode;                   /**< <tt>\b 0x24:</tt> GPIO_REVA IN Register */
    __R  uint8_t  rsv_0x000f;              /**< <tt>\b 0x28:</tt> GPIO_REVA INTMODE Register */
    __IO uint8_t  spi_config_c;               /**< <tt>\b 0x2C:</tt> GPIO_REVA INTPOL Register */
    __I  uint8_t  spi_status;                 /**< <tt>\b 0x30:</tt> GPIO_REVA INEN Register */
    __R  uint8_t  rsv_0x0012;              /**< <tt>\b 0x08:</tt> GPIO_REVA EN0_CLR Register */
    __R  uint8_t  rsv_0x0013;              /**< <tt>\b 0x08:</tt> GPIO_REVA EN0_CLR Register */
    __I  uint8_t  status;                /**< <tt>\b 0x34:</tt> GPIO_REVA INTEN Register */
    __I  uint8_t  alert_status1;            /**< <tt>\b 0x38:</tt> GPIO_REVA INTEN_SET Register */
    __I  uint8_t  alert_status2;            /**< <tt>\b 0x38:</tt> GPIO_REVA INTEN_SET Register */
    __I  uint8_t  alert_status3;            /**< <tt>\b 0x38:</tt> GPIO_REVA INTEN_SET Register */
    __I  uint8_t  alert_status4;            /**< <tt>\b 0x38:</tt> GPIO_REVA INTEN_SET Register */
    __R  uint8_t  rsv_0x0019;
    __I  uint8_t  clamp_status1;            /**< <tt>\b 0x38:</tt> GPIO_REVA INTEN_SET Register */
    __I  uint8_t  clamp_status2;            /**< <tt>\b 0x38:</tt> GPIO_REVA INTEN_SET Register */
    __R  uint8_t  rsv_0x001c_0x001f[4];            /**< <tt>\b 0x10:</tt> GPIO_REVA OUTEN_SET Register */
    __IO uint8_t  setup;            /**< <tt>\b 0x3C:</tt> GPIO_REVA INTEN_CLR Register */
    __IO uint8_t  ref_ctrl;            /**< <tt>\b 0x3C:</tt> GPIO_REVA INTEN_CLR Register */
    __IO uint8_t  seq_ctrl;            /**< <tt>\b 0x3C:</tt> GPIO_REVA INTEN_CLR Register */
    __IO uint8_t  ac_ctrl;            /**< <tt>\b 0x3C:</tt> GPIO_REVA INTEN_CLR Register */
    __IO uint16_t std_seq_config;                /**< <tt>\b 0x40:</tt> GPIO_REVA INTFL Register */
    __IO uint8_t  gpio_ctrl;            /**< <tt>\b 0x3C:</tt> GPIO_REVA INTEN_CLR Register */
    __IO uint8_t  gp_mode;            /**< <tt>\b 0x3C:</tt> GPIO_REVA INTEN_CLR Register */
    __IO uint8_t  gpio_state;            /**< <tt>\b 0x3C:</tt> GPIO_REVA INTEN_CLR Register */
    __IO uint8_t  temp_ctrl;            /**< <tt>\b 0x3C:</tt> GPIO_REVA INTEN_CLR Register */
    __R  uint8_t  rsv_0x002a_0x002f[6];
    __IO uint8_t  config_in_n[16];            /**< <tt>\b 0x10:</tt> GPIO_REVA OUTEN_SET Register */
    __IO uint16_t upper_in_n[16];            /**< <tt>\b 0x10:</tt> GPIO_REVA OUTEN_SET Register */
    __IO uint16_t lower_in_n[16];            /**< <tt>\b 0x10:</tt> GPIO_REVA OUTEN_SET Register */
    __IO uint16_t hyst_in_n[16];            /**< <tt>\b 0x10:</tt> GPIO_REVA OUTEN_SET Register */
    __IO uint16_t offset_in_n[16];            /**< <tt>\b 0x10:</tt> GPIO_REVA OUTEN_SET Register */
    __IO uint16_t gain_in_n[16];            /**< <tt>\b 0x10:</tt> GPIO_REVA OUTEN_SET Register */
    __R  uint8_t  rsv_0x00e0_0x00ff[32];
    __IO uint8_t  as_slot_n[128];            /**< <tt>\b 0x10:</tt> GPIO_REVA OUTEN_SET Register */
} ad4696_regs_t;

ad4696_regs_t ad_4696_regs;
////
/***** Functions *****/
static void spi_dma_setup(uint32_t sample_count,uint32_t sample_size)
{
    MXC_DMA->inten = 0x0;
    //DEBUG_TOGGLE_H2L();
    //SPI->TX
    MXC_DMA->ch[g_dma_spi_tx].status = MXC_F_DMA_STATUS_CTZ_IF; // Clear CTZ status flag
    //MXC_DMA->ch[g_dma_spi_tx].dst = (uint32_t) rx_data; // Cast Pointer
    MXC_DMA->ch[g_dma_spi_tx].cnt = g_stream_buffer_size;
    MXC_DMA->ch[g_dma_spi_tx].src = (uint32_t)(tx_data_8);
    MXC_DMA->ch[g_dma_spi_tx].ctrl = ((0x1 << MXC_F_DMA_CTRL_CTZ_IE_POS)     +
                                      (0x0 << MXC_F_DMA_CTRL_DIS_IE_POS)     +
                                      (0x0 << MXC_F_DMA_CTRL_BURST_SIZE_POS) +
                                      (0x0 << MXC_F_DMA_CTRL_DSTINC_POS)     +
                                      (0x0 << MXC_F_DMA_CTRL_DSTWD_POS)      +
                                      (0x0 << MXC_F_DMA_CTRL_SRCINC_POS)     +
                                      (0x0 << MXC_F_DMA_CTRL_SRCWD_POS)      +
                                      (0x0 << MXC_F_DMA_CTRL_TO_CLKDIV_POS)  +
                                      (0x0 << MXC_F_DMA_CTRL_TO_WAIT_POS)    +
                                      (0x2F << MXC_F_DMA_CTRL_REQUEST_POS)   +  // To SPI0 -> Tx
                                      (0x0 << MXC_F_DMA_CTRL_PRI_POS)        +  // High Priority
                                      (0x0 << MXC_F_DMA_CTRL_RLDEN_POS)    //+  // Reload disabled
                                   //(0x1 << MXC_F_DMA_CTRL_EN_POS)           // Enable DMA channel
                                    );
    //SPI->RX
    MXC_DMA->ch[g_dma_spi_rx].status = MXC_F_DMA_STATUS_CTZ_IF; // Clear CTZ status flag
    MXC_DMA->ch[g_dma_spi_rx].dst = (uint32_t) (rx_data_8); // Cast Pointer
    MXC_DMA->ch[g_dma_spi_rx].cnt = g_stream_buffer_size;
    //MXC_DMA->ch[g_dma_spi_rx].src = (uint32_t)(rx_data + g_stream_buffer_size);
    MXC_DMA->ch[g_dma_spi_rx].ctrl = ((0x1 << MXC_F_DMA_CTRL_CTZ_IE_POS)     +
                                      (0x0 << MXC_F_DMA_CTRL_DIS_IE_POS)     +
                                      (0x0 << MXC_F_DMA_CTRL_BURST_SIZE_POS) +
                                      (0x1 << MXC_F_DMA_CTRL_DSTINC_POS)     +
                                      (0x0 << MXC_F_DMA_CTRL_DSTWD_POS)      +
                                      (0x0 << MXC_F_DMA_CTRL_SRCINC_POS)     +
                                      (0x0 << MXC_F_DMA_CTRL_SRCWD_POS)      +
                                      (0x0 << MXC_F_DMA_CTRL_TO_CLKDIV_POS)  +
                                      (0x1 << MXC_F_DMA_CTRL_TO_WAIT_POS)    +
                                      (0xF << MXC_F_DMA_CTRL_REQUEST_POS)    +  // To SPI0 -> Rx
                                      (0x0 << MXC_F_DMA_CTRL_PRI_POS)        +  // High Priority
                                      (0x0 << MXC_F_DMA_CTRL_RLDEN_POS)    //+  // Reload disabled
                                    //(0x1 << MXC_F_DMA_CTRL_EN_POS)           // Enable DMA channel
                                    );
    MXC_SPI0->ctrl0 &= ~(MXC_F_SPI_CTRL0_EN);
    MXC_SETFIELD(MXC_SPI0->ctrl1, MXC_F_SPI_CTRL1_TX_NUM_CHAR, (sample_size) << MXC_F_SPI_CTRL1_TX_NUM_CHAR_POS);
    MXC_SETFIELD(MXC_SPI0->ctrl1, MXC_F_SPI_CTRL1_RX_NUM_CHAR, (sample_size*sample_count) << MXC_F_SPI_CTRL1_RX_NUM_CHAR_POS);
    MXC_SPI0->dma   |= (MXC_F_SPI_DMA_TX_FLUSH | MXC_F_SPI_DMA_RX_FLUSH);
        // QSPIn port is enabled
    //MXC_SPI0->ctrl0 |= (MXC_F_SPI_CTRL0_EN);

        // Clear master done flag
    MXC_SPI0->intfl = MXC_F_SPI_INTFL_MST_DONE;
    MXC_SETFIELD (MXC_SPI0->dma, MXC_F_SPI_DMA_TX_THD_VAL, (sample_size) << MXC_F_SPI_DMA_TX_THD_VAL_POS);
    MXC_SETFIELD (MXC_SPI0->dma, MXC_F_SPI_DMA_RX_THD_VAL, (0) << MXC_F_SPI_DMA_RX_THD_VAL_POS);
    MXC_SPI0->dma |= (MXC_F_SPI_DMA_TX_FIFO_EN);
    MXC_SPI0->dma |= (MXC_F_SPI_DMA_RX_FIFO_EN);
    MXC_SPI0->dma |= (MXC_F_SPI_DMA_DMA_TX_EN);
    MXC_SPI0->dma |= (MXC_F_SPI_DMA_DMA_RX_EN);
    MXC_SPI0->ctrl0 |= (MXC_F_SPI_CTRL0_EN);
    
    
    MXC_DMA->inten = 0x1; // ch0 interrupt enabled
    g_dma_already_setup = 1;
    //DEBUG_TOGGLE_H2L();
    //print_DMA();
    return;
}

static void spi_dma_transmit(uint8_t* src_ptr, uint8_t* dst_ptr, uint32_t sample_count, uint32_t sample_size) 
{ 
//DEBUG_TOGGLE_H2L();
//DEBUG_TOGGLE_H2L();
    
    MXC_DMA->ch[g_dma_spi_tx].src = (uint32_t)(src_ptr);
    MXC_DMA->ch[g_dma_spi_rx].cnt = sample_size*sample_count;
    MXC_DMA->ch[g_dma_spi_rx].dst = (uint32_t)(dst_ptr);
    MXC_DMA->ch[g_dma_spi_rx].ctrl += (0x1 << MXC_F_DMA_CTRL_EN_POS);
    for (uint32_t i=0; i < sample_count; i++) {
        while((MXC_DMA->ch[g_dma_spi_tx].status & MXC_F_DMA_STATUS_STATUS) &&
              (MXC_DMA->ch[g_dma_spi_rx].status & MXC_F_DMA_STATUS_STATUS))
            {
                ;
            }
    //DEBUG_TOGGLE_H2L();
            if (MXC_DMA->ch[g_dma_spi_tx].status & MXC_F_DMA_STATUS_CTZ_IF)
            {
                MXC_DMA->ch[g_dma_spi_tx].status = MXC_F_DMA_STATUS_CTZ_IF;
            }
            if (MXC_DMA->ch[g_dma_spi_rx].status & MXC_F_DMA_STATUS_CTZ_IF)
            {
                MXC_DMA->ch[g_dma_spi_rx].status = MXC_F_DMA_STATUS_CTZ_IF;
            }
    //DEBUG_TOGGLE_H2L();
            MXC_DMA->ch[g_dma_spi_tx].cnt = sample_size;
            //MXC_DMA->ch[g_dma_spi_tx].src = (uint32_t)src_ptr;
    
//            MXC_DMA->ch[g_dma_spi_rx].cnt = sample_size;
//           MXC_DMA->ch[g_dma_spi_rx].dst = (uint32_t)(dst_ptr + (sample_size)*i);
    //DEBUG_TOGGLE_H2L();
            //printf("DMA dst reg =%08X, %08X\n", (MXC_DMA->ch[g_dma_spi_rx].dst),dst_ptr);
            // Enable DMA channel
       //     MXC_DMA->ch[g_dma_spi_rx].ctrl += (0x1 << MXC_F_DMA_CTRL_EN_POS);
            MXC_DMA->ch[g_dma_spi_tx].ctrl += (0x1 << MXC_F_DMA_CTRL_EN_POS);
            //print_DMA();
            //MXC_Delay(1);
            //asm("NOP");
            //asm("NOP");
            // Start DMA
            MXC_SPI0->ctrl0 |= MXC_F_SPI_CTRL0_START;
    //DEBUG_TOGGLE_H2L();
            //while((MXC_DMA->ch[g_dma_spi_tx].status & MXC_F_DMA_STATUS_STATUS));
    //DEBUG_TOGGLE_H2L();
    }
    return;
}

static void spi_dma_transmit_Int(uint8_t* src_ptr, uint8_t* dst_ptr, uint32_t sample_count, uint32_t sample_size) 
{ 
//DEBUG_TOGGLE_H2L();
//DEBUG_TOGGLE_H2L();
    
    MXC_DMA->ch[g_dma_spi_tx].src = (uint32_t)(src_ptr);
    MXC_DMA->ch[g_dma_spi_rx].cnt = sample_size*sample_count;
    MXC_DMA->ch[g_dma_spi_rx].dst = (uint32_t)(dst_ptr);
    MXC_DMA->ch[g_dma_spi_rx].ctrl += (0x1 << MXC_F_DMA_CTRL_EN_POS);

    MXC_DMA->ch[g_dma_spi_tx].cnt = sample_size;
    MXC_DMA->ch[g_dma_spi_tx].ctrl += (0x1 << MXC_F_DMA_CTRL_EN_POS);
    MXC_SPI0->ctrl0 |= MXC_F_SPI_CTRL0_START;
    return;
}

static void spi_transmit(void* datain, unsigned int count)
{
    unsigned int            offset;
    unsigned int            fifo;
    volatile uint8_t* u8ptrin = (volatile uint8_t*) datain;
    unsigned int             start = 0;

    // HW requires disabling/re-enabling SPI block at end of each transaction (when SS is inactive).
    SPI->ctrl0 &= ~(MXC_F_SPI_CTRL0_EN);

    // Setup the slave select
    //MXC_SETFIELD(spi->ctrl0, MXC_F_SPI_CTRL0_SS_ACTIVE, ((1 << ssel) << MXC_F_SPI_CTRL0_SS_ACTIVE_POS));

    // number of RX Char is 0xffff
    SPI->ctrl1 &= ~(MXC_F_SPI_CTRL1_RX_NUM_CHAR);

    //DMA RX FIFO disabled
    SPI->dma &= ~(MXC_F_SPI_DMA_RX_FIFO_EN);

    // set number of char to be transmit
    MXC_SETFIELD(SPI->ctrl1, MXC_F_SPI_CTRL1_TX_NUM_CHAR, count << MXC_F_SPI_CTRL1_TX_NUM_CHAR_POS);
    // DMA TX fifo enable
    SPI->dma |= MXC_F_SPI_DMA_TX_FIFO_EN;

    /* Clear TX and RX FIFO in DMA
        TX: Set this bit to clear the TX FIFO and all TX FIFO flags in the QSPIn_INT_FL register.
            Note: The TX FIFO should be disabled (QSPIn_DMA.tx_fifo_en = 0) prior to setting this field.
            Note: Setting this field to 0 has no effect.
        RX: Clear the RX FIFO and any pending RX FIFO flags in QSPIn_INTFL.
            This should be done when the RX FIFO is inactive.
    */
    SPI->dma   |= (MXC_F_SPI_DMA_TX_FLUSH | MXC_F_SPI_DMA_RX_FLUSH);
    // QSPIn port is enabled
    SPI->ctrl0 |= (MXC_F_SPI_CTRL0_EN);

    // Clear master done flag
    SPI->intfl = MXC_F_SPI_INTFL_MST_DONE;

    /* Loop until all data is transmitted */
    offset = 0;

    do {
        fifo = (count > 8) ? 8 : count;
        count -= fifo;

        while (fifo > 0) {
            /* Send data */
            SPI->fifo8[0] = u8ptrin[offset];
            offset++;
            fifo--;
        }

        /*
            Master Start Data Transmission
                Set this field to 1 to start a SPI master mode transaction.
                0: No master mode transaction active.
                1: Master initiates a data transmission. Ensure that all pending transactions are
                complete before setting this field to 1.
                Note: This field is only used when the QSPIn is configured for Master Mode
                (QSPIn_CTRL0.master = 1).
        */
        if (start == 0) {
            SPI->ctrl0 |= MXC_F_SPI_CTRL0_START;
            start = 1;
        }

        /* Wait for data transmitting complete and then De-asserts nSS I/O */
        // Deassert slave select at the end of the transaction
        SPI->ctrl0 &= ~MXC_F_SPI_CTRL0_SS_CTRL;
    }
    while (count);

    while (!(SPI->intfl & MXC_F_SPI_INTFL_MST_DONE)) {
        // wait until done
    }
    return;
}

void GPIO0_IRQHandler(void) 
{
    DEBUG_TOGGLE_H2L();
    //MXC_GPIO_Handler(MXC_GPIO_PORT_0);
    if (GPIO_FLAG <30020) {
        AD4696_command_convert(0x00,0);
    //    DEBUG_TOGGLE_H2L();
    }
    else {
        AD4696_command_convert(0xA0,0);
        NVIC_DisableIRQ(MXC_GPIO_GET_IRQ(MXC_GPIO_GET_IDX(MXC_GPIO0)));
    }

    uint32_t stat;
    mxc_gpio_regs_t *gpio = MXC_GPIO_GET_GPIO(0);
    stat = MXC_GPIO_GetFlags(gpio);
    //DEBUG_TOGGLE_H2L();
    MXC_GPIO_ClearFlags(gpio, stat);
    //DEBUG_TOGGLE_H2L();
    //MXC_GPIO_Handler(MXC_GPIO_PORT_0);
    DEBUG_TOGGLE_H2L();
    GPIO_FLAG ++;
}

void SPI_IRQHandler(void)
{
    MXC_SPI_AsyncHandler(SPI);
}

void DMA0_IRQHandler(void)
{
    //TODO: not working
    //MXC_DMA_Handler();
    MXC_DMA->inten = 0x0;
    //DEBUG_TOGGLE_H2L();
    //printf("DMA0_flag %d\n",DMA0_FLAG);
    MXC_DMA->ch[g_dma_spi_tx].status = 0xFF;
    // while((MXC_DMA->ch[g_dma_spi_tx].status & MXC_F_DMA_STATUS_STATUS) &&
    //       (MXC_DMA->ch[g_dma_spi_rx].status & MXC_F_DMA_STATUS_STATUS))
    while(MXC_DMA->ch[g_dma_spi_tx].status & MXC_F_DMA_STATUS_STATUS)
    {
            ;
    }
    //DEBUG_TOGGLE_H2L();
    if (MXC_DMA->ch[g_dma_spi_tx].status & MXC_F_DMA_STATUS_CTZ_IF)
    {
        MXC_DMA->ch[g_dma_spi_tx].status = MXC_F_DMA_STATUS_CTZ_IF;
    }
    // if (MXC_DMA->ch[g_dma_spi_rx].status & MXC_F_DMA_STATUS_CTZ_IF)
    // {
    //     MXC_DMA->ch[g_dma_spi_rx].status = MXC_F_DMA_STATUS_CTZ_IF;
    // }
    //DEBUG_TOGGLE_H2L();
    MXC_DMA->ch[g_dma_spi_tx].cnt = g_sample_size;
    MXC_DMA->ch[g_dma_spi_tx].ctrl += (0x1 << MXC_F_DMA_CTRL_EN_POS);
    MXC_SPI0->ctrl0 |= MXC_F_SPI_CTRL0_START;
    DMA0_FLAG++;
    if (DMA0_FLAG < 1000)
        MXC_DMA->inten = 0x1;
        //DEBUG_TOGGLE_H2L();
}

void DMA1_IRQHandler(void)
{
    // TODO: may not be needed
    MXC_DMA_Handler();
    MXC_DMA->ch[g_dma_spi_tx].status = 0xFF;
    DEBUG_TOGGLE_H2L();
    // uint32_t stat;
    // // mxc_dma_regs_t *dma = MXC_DMA_GET_IDX(MXC_DMA);
    // stat = MXC_DMA_ChannelGetFlags(1);
    // //DEBUG_TOGGLE_H2L();
    // MXC_DMA_ChannelClearFlags(1,stat);
    //MXC_DMA_Handler();
    DMA1_FLAG++;
}

void SPI_Callback(mxc_spi_req_t *req, int error)
{
    SPI_FLAG = error;
}

int ad4696_pins_Init(void)
{
    int retval = E_NO_ERROR;
    unsigned int i;

    /* setup 3 GPIOs for the adc communications */
    for (i = 0; i < ad4696_num_gpios; i++) {
        if (MXC_GPIO_Config(&ad4696_pins[i]) != E_NO_ERROR) {
            retval = E_UNKNOWN;
        }
    }
    return retval;
}

int MAX78000_SPI_Config(void) {
    
    int retVal;
    // mxc_spi_req_t req;
    // mxc_spi_pins_t spi_pins;
    
    spi_pins.clock  = TRUE;
    spi_pins.miso   = TRUE;
    spi_pins.mosi   = TRUE;
    spi_pins.sdio2  = FALSE;
    spi_pins.sdio3  = FALSE;
    spi_pins.ss0    = TRUE;
    spi_pins.ss1    = TRUE;
    spi_pins.ss2    = FALSE;
    spi_pins.vddioh = FALSE;


    req.ssIdx = 1;
    req.ssDeassert = 1;
    req.txCnt = 0;
    req.rxCnt = 0;
    req.completeCB = (spi_complete_cb_t)SPI_Callback;
    SPI_FLAG = 1;

#if MASTERSYNC
    printf("Performing blocking (synchronous) transactions...\n");
#endif
#if MASTERASYNC
    printf("Performing non-blocking (asynchronous) transactions...\n");
    MXC_NVIC_SetVector(SPI_IRQ, SPI_IRQHandler);
#endif
#if MASTERDMA
    printf("Performing transactions with DMA...\n");
    MXC_DMA_ReleaseChannel(0);
    MXC_DMA_ReleaseChannel(1);

    NVIC_EnableIRQ(DMA0_IRQn);
    NVIC_EnableIRQ(DMA1_IRQn);
#endif

   retVal = MXC_SPI_Init(SPI, 1, 0, 1, 0, SPI_SPEED, spi_pins);
    if (retVal != E_NO_ERROR) {
        printf("\nSPI INITIALIZATION ERROR\n");
        return retVal;
    }
   retVal = MXC_SPI_SetDataSize(SPI, 8);

    if (retVal != E_NO_ERROR) {
        printf("\nSPI SET DATASIZE ERROR: %d\n", retVal);
        return retVal;
    }

    retVal = MXC_SPI_SetWidth(SPI, SPI_WIDTH_STANDARD);

    if (retVal != E_NO_ERROR) {
        printf("\nSPI SET WIDTH ERROR: %d\n", retVal);
        return retVal;
    }
    // AD4696 operates in SPI mode 3
    retVal = MXC_SPI_SetMode(SPI, SPI_MODE_3);
    if (retVal != E_NO_ERROR) {
        printf("\nSPI SET WIDTH ERROR: %d\n", retVal);
        return retVal;
    }
    return 0;
}

void AD4696_RESET_HOLD(void) {
    MXC_GPIO_OutClr(MXC_GPIO1, MXC_GPIO_PIN_1);
}

void AD4696_GO(void){
    MXC_GPIO_OutSet(MXC_GPIO1, MXC_GPIO_PIN_1);
}

int AD4696_READ(uint16_t addr) {
    
    int retVal;
    //mxc_spi_req_t req;

    tx_data_8[0] = ((addr + 0x8000) & 0xFF00 )>>8;//0x80;
    tx_data_8[1] = (addr & 0x00FF )>>0;//0x0C;
    tx_data_8[2] = 0x00;
    req.spi = SPI;
    //req.txData = (uint8_t *)tx_data;
    //req.rxData = (uint8_t *)rx_data;
    req.txData = (uint8_t *)tx_data_8;
    req.rxData = (uint8_t *)rx_data_8;
    req.txLen = 3;
    req.rxLen = 3;

#if MASTERSYNC
        retVal = MXC_SPI_MasterTransaction(&req);
#endif

#if MASTERASYNC
        NVIC_EnableIRQ(SPI_IRQ);
        MXC_SPI_MasterTransactionAsync(&req);

        while (SPI_FLAG == 1) {}

#endif

#if MASTERDMA
    MXC_DMA_ReleaseChannel(0);
    MXC_DMA_ReleaseChannel(1);

    // NVIC_EnableIRQ(DMA0_IRQn);
    // NVIC_EnableIRQ(DMA1_IRQn);
    retVal = MXC_SPI_MasterTransactionDMA(&req);

    while (DMA_FLAG == 0) {}

    DMA_FLAG = 0;
#endif
     return retVal;
}

int AD4696_WRITE(uint16_t addr, uint8_t value) {
    
    int retVal;
    //mxc_spi_req_t req;

    tx_data_8[0] = ((addr + 0x0000) & 0xFF00 )>>8;//0x80;
    tx_data_8[1] = (addr & 0x00FF )>>0;//0x0C;
    tx_data_8[2] = value;
    req.spi = SPI;
    //req.txData = (uint8_t *)tx_data;
    //req.rxData = (uint8_t *)rx_data;
    req.txData = (uint8_t *)tx_data_8;
    req.rxData = (uint8_t *)rx_data_8;
    req.txLen = 3;
    req.rxLen = 3;

    //print_GPIO(0);
#if MASTERSYNC
        retVal = MXC_SPI_MasterTransaction(&req);
#endif

#if MASTERASYNC
        NVIC_EnableIRQ(SPI_IRQ);
        MXC_SPI_MasterTransactionAsync(&req);

        while (SPI_FLAG == 1) {}

#endif

#if MASTERDMA
    MXC_DMA_ReleaseChannel(0);
    MXC_DMA_ReleaseChannel(1);

    // NVIC_EnableIRQ(DMA0_IRQn);
    // NVIC_EnableIRQ(DMA1_IRQn);
    retVal = MXC_SPI_MasterTransactionDMA(&req);

    while (DMA_FLAG == 0) {}

    DMA_FLAG = 0;
#endif
    return retVal;
}

int AD4696_READ_all(ad4696_regs_t *value)
{
    value->spi_config_a = 0x43;
    value->spi_config_b = 0xeb;
    value->spi_config_c = 0xf3;
    value->gain_in_n[3] = 0xbeef;
    value->std_seq_config = 0xdeed;
    value->gpio_ctrl = 0x99;
    printf("here it is: %02x\n", value->spi_config_a);
    printf("here it is: %02x\n", value->spi_config_c);
    printf("here it is: %04x\n", value->gain_in_n[3]);
    value->spi_config_c = 0x33;
    printf("here it is: %02x\n", value->spi_config_a);
    printf("here it is: %02x\n", value->spi_config_c);
    printf("here it is (.): %02x\n", &(value->spi_config_c));
    printf("here it is: %04x\n", value->gain_in_n[3]);
    //printf("here is value: %08x\n", value[1]);

    printf("here is the value from read1 function %04x\n", *((uint8_t*)(value)+0x26));
    printf("here is the config b addr from read1 function %04x\n", &(value->spi_config_b));
    printf("here is the size of value from read1 function %04x\n",sizeof((value[0])));
    printf("here is the value addr from read1 function %04x\n", (uint8_t*)((value)));
    printf("here is the base  addr from read1 function %04x\n", &(value->spi_config_a));
    printf("here is the diff addr from read1 function %04x\n", (uint8_t*)((value))-&(value->spi_config_a));
    return 0;
}

int AD4696_READ_Loop(uint16_t init_addr, uint8_t NumReg) {
    
    AD4696_WRITE(LOOP_MODE,NumReg);
    int retVal;
    //mxc_spi_req_t req;

    tx_data_8[0] = ((init_addr + 0x8000) & 0xFF00 )>>8;//0x80;
    tx_data_8[1] = (init_addr & 0x00FF )>>0;//0x0C;
    for (int i = 2; i < (2 + NumReg ); i++) {
        tx_data_8[i] = 0x00;
    }

    req.spi = SPI;
    req.txData = (uint8_t *)tx_data_8;
    req.rxData = (uint8_t *)rx_data_8;
    req.txLen = 2 + NumReg;
    req.rxLen = 2 + NumReg;
#if MASTERSYNC
        retVal = MXC_SPI_MasterTransaction(&req);
#endif

#if MASTERASYNC
        NVIC_EnableIRQ(SPI_IRQ);
        MXC_SPI_MasterTransactionAsync(&req);

        while (SPI_FLAG == 1) {}

#endif

#if MASTERDMA
    MXC_DMA_ReleaseChannel(0);
    MXC_DMA_ReleaseChannel(1);

    // NVIC_EnableIRQ(DMA0_IRQn);
    // NVIC_EnableIRQ(DMA1_IRQn);
    retVal = MXC_SPI_MasterTransactionDMA(&req);

    while (DMA_FLAG == 0) {}

    DMA_FLAG = 0;
#endif

    AD4696_WRITE(LOOP_MODE,0);
    return retVal;

}

int AD4696_command_convert(uint8_t value,uint8_t len) {

    int retVal;
    //mxc_spi_req_t req;

    tx_data_8[0] = value>>1;
    tx_data_8[1] = 0x00;
    tx_data_8[2] = 0x00;


    req.spi = SPI;
    //req.txData = (uint8_t *)tx_data;
    //req.rxData = (uint8_t *)rx_data;
    //req.txData = (uint8_t *)tx_data_8;
    req.rxData = (uint8_t *)rx_data_8;
    req.txData = (uint8_t *)tx_data_8;
    req.txLen = len+2;
    req.rxLen = len+2;
#if MASTERSYNC
         spi_transmit((uint8_t*)tx_data_8,2+len);  
         retVal = 0; 
        //retVal= MXC_SPI_MasterTransaction(&req);
#endif

#if MASTERASYNC
        NVIC_EnableIRQ(SPI_IRQ);
        MXC_SPI_MasterTransactionAsync(&req);

        while (SPI_FLAG == 1) {}

#endif

#if MASTERDMA
    MXC_DMA_ReleaseChannel(0);
    MXC_DMA_ReleaseChannel(1);

    // NVIC_EnableIRQ(DMA0_IRQn);
    // NVIC_EnableIRQ(DMA1_IRQn);
    //while ((MXC_GPIO_InGet(MXC_GPIO0, MXC_GPIO_PIN_19)) == 0);
    //while ((MXC_GPIO_InGet(MXC_GPIO0, MXC_GPIO_PIN_19)) == (1<<19));
    //while ((MXC_GPIO_InGet(MXC_GPIO0, MXC_GPIO_PIN_19)) == 0);
    retVal = MXC_SPI_MasterTransactionDMA(&req);

    while (DMA_FLAG == 0) {}

    DMA_FLAG = 0;
#endif
    //MXC_Delay(160);
    return retVal;
}

int AD4696_command_convert_dma(uint8_t value, uint32_t sample_count, uint32_t sample_size) {

    int retVal = 0;
    //mxc_spi_req_t req;

    tx_data_8[0] = value>>1;
    tx_data_8[1] = 0x00;
    tx_data_8[2] = 0x00;

    //spi_dma_transmit(tx_data_8, rx_data_8, sample_count, sample_size);
    spi_dma_transmit_Int(tx_data_8, rx_data_8, sample_count, sample_size);
 
    return retVal;
}

void AD4696_busy_config(void) {
    // mxc_gpio_cfg_t req;
    // req.mask = MXC_GPIO_PIN_19;
    //uint32_t* dummy=0;
    // the following Register call back could be very slow to use
#if 0
    MXC_GPIO_RegisterCallback(&ad4696_pins[2],
                             (mxc_gpio_callback_fn) GPIO_IRQHandler,
                             (void*)&ad4696_pins[2]);
#endif
    MXC_GPIO_IntConfig(&ad4696_pins[2], MXC_GPIO_INT_FALLING);
    MXC_GPIO_EnableInt(MXC_GPIO0, MXC_GPIO_PIN_19);
    //NVIC_EnableIRQ(MXC_GPIO_GET_IRQ(MXC_GPIO_GET_IDX(MXC_GPIO0)));
}

void print_received_buffer(uint32_t nr_sample, uint32_t SAMPLE_SIZE_IN_BYTES) {
    for(uint32_t j = 0; j< ((nr_sample)* SAMPLE_SIZE_IN_BYTES); j=j+SAMPLE_SIZE_IN_BYTES) {
        if (SAMPLE_SIZE_IN_BYTES == 2)
            printf("%02X %02X\t",rx_data_8[j],rx_data_8[j+1]);
        if (SAMPLE_SIZE_IN_BYTES == 3)
            printf("%02X %02X %02X\t",rx_data_8[j],rx_data_8[j+1],rx_data_8[j+2]);
        else
            continue;
    }
    printf("\n");
}

///////////////////////////////////////////* main function *////////////////////////////////////////////////
int main(void)
{
    //int retVal;
    //uint16_t temp;
    //mxc_spi_req_t req;
    //mxc_spi_pins_t spi_pins;
    //ad4696_regs_t ad_4696_regs;
    uint32_t SAMPLE_SIZE_IN_BYTES = 2;
    MXC_ICC_Enable(MXC_ICC0);

    /* Set system clock to 100 MHz */
    MXC_SYS_Clock_Select(MXC_SYS_CLOCK_IPO);
    SystemCoreClockUpdate();
    printf("\n**************************** SPI MASTER with AD4696 *************************\n");
    printf("[x] Compiled: %s, %s\n",__DATE__,__TIME__);
    //print_GPIO(0);
    printf("set up RESET and CNV pins\n");
    ad4696_pins_Init();
    //print_GPIO(0);
#ifdef BOARD_EVKIT_V1
    printf("evkit is not supported\n");
    return 0;
#else
    printf("MAX78000 feather board connection to AD4696.\n\n");
#endif
    MXC_DMA_Init();
    AD4696_GO();
    MAX78000_SPI_Config();
    print_SPI();
    AD4696_busy_config();
    
    AD4696_READ(VENDOR_H);
    //printf("read value: %02x\n",rx_data_8[2]);
    AD4696_READ(VENDOR_L);
    AD4696_READ(DEVICE_TYPE);
    AD4696_WRITE(SCRATCH_PAD, 0x73);
    AD4696_READ(SCRATCH_PAD);
    // AD4696_READ(VENDOR_H);
    // AD4696_READ(VENDOR_L);
    // AD4696_READ(DEVICE_TYPE);
    // AD4696_READ(SCRATCH_PAD);
    // AD4696_READ(SPI_STATUS);
    // AD4696_READ(STATUS);
    // AD4696_READ_Loop(LOOP_MODE,15);
    // AD4696_READ(STATUS);
    // AD4696_READ_all(&ad_4696_regs);
    // printf("print &ad_4696_regs %08x\n", sizeof(ad_4696_regs.config_in_n[3]));
    // printf("in the main %04x\n", ad_4696_regs.spi_config_c);
    // printf("in the main(->) %04x\n", (uint8_t*)(&(ad_4696_regs.as_slot_n[127]))-&ad_4696_regs.spi_config_a);
    // //AD4696_READ1(&ad_4696_regs.std_seq_config);
    AD4696_WRITE(CONFIG_IN0, 0); //no oversampling
    AD4696_READ(SPI_STATUS);
    AD4696_READ(STATUS);
    AD4696_WRITE(GPIO_CTRL, 0x1); //GPIO output
    AD4696_WRITE(GP_MODE, 0x2);    // GPIO busy
    #ifdef AUTO_CYCLE
    AD4696_WRITE(AC_CTRL, (0x0<<1)+1); // 0x0<<1 is the fastest
    #endif
    AD4696_WRITE(SETUP, 0x14); // conversion starts
    //MXC_Delay(12);
    #ifdef AUTO_CYCLE
    NVIC_EnableIRQ(MXC_GPIO_GET_IRQ(MXC_GPIO_GET_IDX(MXC_GPIO0)));
    LED_On(LED_GREEN);
    while(GPIO_FLAG < 30021);
    LED_Off(LED_GREEN);
    #endif
    //while(1);
    #ifndef AUTO_CYCLE
    uint32_t nr_sample = 1000; //should be less than or equal to (DATA_LEN/SAMPLE_SIZE_IN_BYTES)
    spi_dma_setup(nr_sample, SAMPLE_SIZE_IN_BYTES);
    LED_On(LED_GREEN);
    
    NVIC_EnableIRQ(MXC_DMA_CH_GET_IRQ(0));
    //NVIC_EnableIRQ(MXC_DMA_CH_GET_IRQ(1));
    AD4696_command_convert_dma(0x00, nr_sample ,SAMPLE_SIZE_IN_BYTES);
    //print_received_buffer(nr_sample, SAMPLE_SIZE_IN_BYTES);
    //uint32_t local_count =0;
     //MXC_Delay(1000);
    while(DMA0_FLAG<nr_sample)
    {
        //local_count ++;
    }
    NVIC_DisableIRQ(MXC_DMA_CH_GET_IRQ(0));
    //print_received_buffer(nr_sample, SAMPLE_SIZE_IN_BYTES);
    //MXC_Delay(1000);
    //printf("DMA0_flag %d and %d\n",DMA0_FLAG,local_count);
    
    spi_dma_setup(1, SAMPLE_SIZE_IN_BYTES);
    NVIC_EnableIRQ(MXC_DMA_CH_GET_IRQ(0));
    AD4696_command_convert_dma(0xA0,  1, SAMPLE_SIZE_IN_BYTES);
    NVIC_DisableIRQ(MXC_DMA_CH_GET_IRQ(0));
    //NVIC_DisableIRQ(MXC_DMA_CH_GET_IRQ(1));
    
    LED_Off(LED_GREEN);
    #endif
    AD4696_READ(SPI_STATUS);
    AD4696_READ(STATUS);
    printf("Status register %08x\n",rx_data_8[2]);
    AD4696_READ(SCRATCH_PAD);

    AD4696_RESET_HOLD();
    //MXC_GPIO_OutClr(MXC_GPIO1, MXC_GPIO_PIN_1);
    printf("GPIO_flag %d\n",GPIO_FLAG);
    printf("DMA0_flag %d\n",DMA0_FLAG);
    printf("DMA1_flag %d\n",DMA1_FLAG);
    printf("\nExample Complete.\n");
    return E_NO_ERROR;
}

#if 0
int AD4696_READ1(ad4696_regs_t *regs) {

    
    int retVal, addr;
    //mxc_spi_req_t req;
    printf("here is the value from read1 function %04x\n",*value);
    printf("here is the size of value from read1 function %04x\n",sizeof(*value));
    printf("here is the value addr from read1 function %04x\n", (uint8_t*)(&(value)));
    printf("here is the base  addr from read1 function %04x\n", &(ad_4696_regs.spi_config_a));
    printf("here is the diff addr from read1 function %04x\n", (uint8_t*)((value))-&(ad_4696_regs.spi_config_a));
    return 0;
    tx_data_8[0] = ((addr + 0x8000) & 0xFF00 )>>8;//0x80;
    tx_data_8[1] = (addr & 0x00FF )>>0;//0x0C;
    tx_data_8[2] = 0x00;
    req.spi = SPI;
    //req.txData = (uint8_t *)tx_data;
    //req.rxData = (uint8_t *)rx_data;
    req.txData = (uint8_t *)tx_data_8;
    req.rxData = (uint8_t *)rx_data_8;
    req.txLen = 3;
    req.rxLen = 3;
#if 0
    // req.ssIdx = 1;
    // req.ssDeassert = 1;
    // req.txCnt = 0;
    // req.rxCnt = 0;
    // req.completeCB = (spi_complete_cb_t)SPI_Callback;
    // SPI_FLAG = 1;

    //retVal = MXC_SPI_SetDataSize(SPI, 16);
    // retVal = MXC_SPI_SetDataSize(SPI, 8);

    // if (retVal != E_NO_ERROR) {
    //     printf("\nSPI SET DATASIZE ERROR: %d\n", retVal);
    //     return retVal;
    // }

    // retVal = MXC_SPI_SetWidth(SPI, SPI_WIDTH_STANDARD);

    // if (retVal != E_NO_ERROR) {
    //     printf("\nSPI SET WIDTH ERROR: %d\n", retVal);
    //     return retVal;
    // }
    //print_GPIO(0);
#endif
#if MASTERSYNC
        MXC_SPI_MasterTransaction(&req);
#endif

#if MASTERASYNC
        NVIC_EnableIRQ(SPI_IRQ);
        MXC_SPI_MasterTransactionAsync(&req);

        while (SPI_FLAG == 1) {}

#endif

#if MASTERDMA
    MXC_DMA_ReleaseChannel(0);
    MXC_DMA_ReleaseChannel(1);

    // NVIC_EnableIRQ(DMA0_IRQn);
    // NVIC_EnableIRQ(DMA1_IRQn);
    retVal = MXC_SPI_MasterTransactionDMA(&req);

    while (DMA_FLAG == 0) {}

    DMA_FLAG = 0;
#endif
     return retVal;
}
#endif

#if 0
int main1(void)
{
    int i, j, retVal;
    uint16_t temp;
    mxc_spi_req_t req;
    mxc_spi_pins_t spi_pins;

    printf("\n**************************** SPI MASTER TEST *************************\n");
    printf("This example configures SPI to send data between the MISO and MOSI\n");
#ifdef BOARD_EVKIT_V1
    printf("pins (pins 14 and 15 on header J4). Connect these two pins together.\n");
    printf("Additionally, ensure that jumper JP22 is set to \"CAM\".\n\n");
#else
    printf("pins (pins 12 and 13 on header J8). Connect these two pins together.\n\n");
#endif

    printf("Multiple word sizes (2 through 16 bits) are demonstrated.\n\n");

    spi_pins.clock = TRUE;
    spi_pins.miso = TRUE;
    spi_pins.mosi = TRUE;
    spi_pins.sdio2 = FALSE;
    spi_pins.sdio3 = FALSE;
    spi_pins.ss0 = TRUE;
    spi_pins.ss1 = TRUE;
    spi_pins.ss2 = FALSE;

    spi_pins.vddioh = FALSE;

#if MASTERSYNC
    printf("Performing blocking (synchronous) transactions...\n");
#endif
#if MASTERASYNC
    printf("Performing non-blocking (asynchronous) transactions...\n");
    MXC_NVIC_SetVector(SPI_IRQ, SPI_IRQHandler);
#endif
#if MASTERDMA
    printf("Performing transactions with DMA...\n");
#endif

    for (i = 2; i < 17; i++) {
        // Sending out 2 to 16 bits

        // The hardware doesn't support 9-bit wide characters at high speeds.
        if (i == 9) {
            printf("Hardware does not support 9-bit wide characters.\n");
            continue;
        }

        for (j = 0; j < DATA_LEN; j++) {
            tx_data[j] = DATA_VALUE;
        }

        // Configure the peripheral
        retVal = MXC_SPI_Init(SPI, 1, 0, 1, 0, SPI_SPEED, spi_pins);
        if (retVal != E_NO_ERROR) {
            printf("\nSPI INITIALIZATION ERROR\n");
            return retVal;
        }

        memset(rx_data, 0x0, DATA_LEN * sizeof(uint16_t));

        //SPI Request
        req.spi = SPI;
        req.txData = (uint8_t *)tx_data;
        req.rxData = (uint8_t *)rx_data;
        req.txLen = DATA_LEN;
        req.rxLen = DATA_LEN;
        req.ssIdx = 1;
        req.ssDeassert = 1;
        req.txCnt = 0;
        req.rxCnt = 0;
        req.completeCB = (spi_complete_cb_t)SPI_Callback;
        SPI_FLAG = 1;

        retVal = MXC_SPI_SetDataSize(SPI, i);

        if (retVal != E_NO_ERROR) {
            printf("\nSPI SET DATASIZE ERROR: %d\n", retVal);
            return retVal;
        }

        retVal = MXC_SPI_SetWidth(SPI, SPI_WIDTH_STANDARD);

        if (retVal != E_NO_ERROR) {
            printf("\nSPI SET WIDTH ERROR: %d\n", retVal);
            return retVal;
        }

#if MASTERSYNC
        MXC_SPI_MasterTransaction(&req);
#endif

#if MASTERASYNC
        NVIC_EnableIRQ(SPI_IRQ);
        MXC_SPI_MasterTransactionAsync(&req);

        while (SPI_FLAG == 1) {}

#endif

#if MASTERDMA
        MXC_DMA_ReleaseChannel(0);
        MXC_DMA_ReleaseChannel(1);

        NVIC_EnableIRQ(DMA0_IRQn);
        NVIC_EnableIRQ(DMA1_IRQn);
        MXC_SPI_MasterTransactionDMA(&req);

        while (DMA_FLAG == 0) {}

        DMA_FLAG = 0;


        MXC_SPI_MasterTransactionDMA(&req);

        while (DMA_FLAG == 0) {}

        DMA_FLAG = 0;
#endif

        uint8_t bits = MXC_SPI_GetDataSize(SPI);

        for (j = 0; j < DATA_LEN; j++) {
            if (bits <= 8) {
                if (j < (DATA_LEN / 2)) {
                    temp = VALUE >> (16 - bits);
                    temp = (temp << 8) | temp;
                    temp &= DATA_VALUE;
                    tx_data[j] = temp;
                } else if (j == (DATA_LEN / 2) && DATA_LEN % 2 == 1) {
                    temp = VALUE >> (16 - bits);
                    temp &= DATA_VALUE;
                    tx_data[j] = temp;
                } else {
                    tx_data[j] = 0x0000;
                }
            } else {
                temp = VALUE >> (16 - bits);
                temp &= DATA_VALUE;
                tx_data[j] = temp;
            }
        }

        Compare Sent data vs Received data
        Printf needs the Uart turned on since they share the same pins
        if (memcmp(rx_data, tx_data, sizeof(tx_data)) != 0) {
            printf("\n-->%2d Bits Transaction Failed\n", i);
            return E_COMM_ERR;
        } else {
            printf("-->%2d Bits Transaction Successful\n", i);
        }

        retVal = MXC_SPI_Shutdown(SPI);

        if (retVal != E_NO_ERROR) {
            printf("\n-->SPI SHUTDOWN ERROR: %d\n", retVal);
            return retVal;
        }
    }

        // Configure the peripheral
        retVal = MXC_SPI_Init(SPI, 1, 0, 1, 0, SPI_SPEED, spi_pins);
        if (retVal != E_NO_ERROR) {
            printf("\nSPI INITIALIZATION ERROR\n");
            return retVal;
        }

        tx_data[0] = 0xabcd;
        tx_data[1] = 0x1234;
        tx_data[2] = 0x6789;
        req.spi = SPI;
        req.txData = (uint8_t *)tx_data;
        req.rxData = (uint8_t *)rx_data;
        req.txLen = 3;
        req.rxLen = 3;
        req.ssIdx = 1;
        req.ssDeassert = 1;
        req.txCnt = 0;
        req.rxCnt = 0;
        req.completeCB = (spi_complete_cb_t)SPI_Callback;
        SPI_FLAG = 1;

        retVal = MXC_SPI_SetDataSize(SPI, 16);

        if (retVal != E_NO_ERROR) {
            printf("\nSPI SET DATASIZE ERROR: %d\n", retVal);
            return retVal;
        }

        retVal = MXC_SPI_SetWidth(SPI, SPI_WIDTH_STANDARD);

        if (retVal != E_NO_ERROR) {
            printf("\nSPI SET WIDTH ERROR: %d\n", retVal);
            return retVal;
        }
        MXC_DMA_ReleaseChannel(0);
        MXC_DMA_ReleaseChannel(1);

        NVIC_EnableIRQ(DMA0_IRQn);
        NVIC_EnableIRQ(DMA1_IRQn);
        MXC_SPI_MasterTransactionDMA(&req);

        while (DMA_FLAG == 0) {}

        DMA_FLAG = 0;

        tx_data[0] = 0xdead;
        tx_data[1] = 0xbeef;
        tx_data[2] = 0x1354;
        req.txData = (uint8_t *)tx_data;
        req.rxData = (uint8_t *)rx_data;
        req.txLen = 3;
        req.rxLen = 3;

        MXC_DMA_ReleaseChannel(0);
        MXC_DMA_ReleaseChannel(1);

        // NVIC_EnableIRQ(DMA0_IRQn);
        // NVIC_EnableIRQ(DMA1_IRQn);
        MXC_SPI_MasterTransactionDMA(&req);

        while (DMA_FLAG == 0) {}

        DMA_FLAG = 0;



    printf("\nExample Complete.\n");
    return E_NO_ERROR;
}

#endif

#if 0
    for (uint32_t i=0;i<sample_count;i++) {

        while (MXC_DMA->ch[g_dma_spi_rx].status & MXC_F_DMA_STATUS_STATUS)
        {
                ;
        }
        if (MXC_DMA->ch[g_dma_spi_rx].status & MXC_F_DMA_STATUS_CTZ_IF)
        {
            MXC_DMA->ch[g_dma_spi_rx].status = MXC_F_DMA_STATUS_CTZ_IF;
        }
        MXC_DMA->ch[g_dma_spi_rx].cnt = sample_size;
        MXC_DMA->ch[g_dma_spi_rx].dst = (uint32_t)(dst_ptr + (sample_size)*i);
        MXC_DMA->ch[g_dma_spi_rx].ctrl += (0x1 << MXC_F_DMA_CTRL_EN_POS);
        
        while((MXC_DMA->ch[g_dma_spi_tx].status & MXC_F_DMA_STATUS_STATUS))
        {
            ;
        }
        if (MXC_DMA->ch[g_dma_spi_tx].status & MXC_F_DMA_STATUS_CTZ_IF)
        {
            MXC_DMA->ch[g_dma_spi_tx].status = MXC_F_DMA_STATUS_CTZ_IF;
        }
        
        MXC_DMA->ch[g_dma_spi_tx].cnt = sample_size;
        //MXC_DMA->ch[g_dma_spi_tx].src = (uint32_t)src_ptr;

  
        MXC_DMA->ch[g_dma_spi_tx].ctrl += (0x1 << MXC_F_DMA_CTRL_EN_POS);
        MXC_SPI0->ctrl0 |= MXC_F_SPI_CTRL0_START;

    
    //DEBUG_TOGGLE_H2L();
  

    
    //DEBUG_TOGGLE_H2L();
            //printf("DMA dst reg =%08X, %08X\n", (MXC_DMA->ch[g_dma_spi_rx].dst),dst_ptr);
            // Enable DMA channel
     
            //print_DMA();
            //MXC_Delay(1);
            //asm("NOP");
            //asm("NOP");
            // Start DMA
            
    //DEBUG_TOGGLE_H2L();
            //while((MXC_DMA->ch[g_dma_spi_tx].status & MXC_F_DMA_STATUS_STATUS));
    //DEBUG_TOGGLE_H2L();
    }
#endif


//printf("Read value %02x %02x\n",rx_data_8[0],rx_data_8[1]);
    //print_DMA();
    //while(1);
    
    // AD4696_command_convert_dma(0x00,2);
    // //    printf("Read value %02x %02x\n",rx_data_8[0],rx_data_8[1]);
    // AD4696_command_convert_dma(0x00,2);
    // //    printf("Read value %02x %02x\n",rx_data_8[0],rx_data_8[1]);
    // AD4696_command_convert_dma(0x00,2);
    // //    printf("Read value %02x %02x\n",rx_data_8[0],rx_data_8[1]);
    // AD4696_command_convert_dma(0x00,2);
    // //    printf("Read value %02x %02x\n",rx_data_8[0],rx_data_8[1]);
    // AD4696_command_convert_dma(0x00,2);
    // //    printf("Read value %02x %02x\n",rx_data_8[0],rx_data_8[1]);
    // AD4696_command_convert_dma(0x00,2);
    // //    printf("Read value %02x %02x\n",rx_data_8[0],rx_data_8[1]);
    // AD4696_command_convert_dma(0x00,2);
    // //    printf("Read value %02x %02x\n",rx_data_8[0],rx_data_8[1]);
    // AD4696_command_convert_dma(0x00,2);
    // //    printf("Read value %02x %02x\n",rx_data_8[0],rx_data_8[1]);
    // AD4696_command_convert_dma(0xA0,2);
    // //    printf("Read value %02x %02x\n",rx_data_8[0],rx_data_8[1]);
    
    
// this is the function before changes
// static void spi_dma_transmit(uint8_t* src_ptr, uint8_t* dst_ptr, uint32_t sample_count, uint32_t sample_size) 
// { 
// //DEBUG_TOGGLE_H2L();
// //DEBUG_TOGGLE_H2L();
//     MXC_DMA->ch[g_dma_spi_tx].src = (uint32_t)src_ptr;
//     for (uint32_t i=0;i<sample_count;i++) {
//         while((MXC_DMA->ch[g_dma_spi_tx].status & MXC_F_DMA_STATUS_STATUS) &
//               (MXC_DMA->ch[g_dma_spi_rx].status & MXC_F_DMA_STATUS_STATUS))
//             {
//                 ;
//             }
//     //DEBUG_TOGGLE_H2L();
//             if (MXC_DMA->ch[g_dma_spi_tx].status & MXC_F_DMA_STATUS_CTZ_IF)
//             {
//                 MXC_DMA->ch[g_dma_spi_tx].status = MXC_F_DMA_STATUS_CTZ_IF;
//             }
//             if (MXC_DMA->ch[g_dma_spi_rx].status & MXC_F_DMA_STATUS_CTZ_IF)
//             {
//                 MXC_DMA->ch[g_dma_spi_rx].status = MXC_F_DMA_STATUS_CTZ_IF;
//             }
//     //DEBUG_TOGGLE_H2L();
//             MXC_DMA->ch[g_dma_spi_tx].cnt = sample_size;
//             //MXC_DMA->ch[g_dma_spi_tx].src = (uint32_t)src_ptr;
    
//             MXC_DMA->ch[g_dma_spi_rx].cnt = sample_size;
//             MXC_DMA->ch[g_dma_spi_rx].dst = (uint32_t)(dst_ptr + (sample_size)*i);
//     //DEBUG_TOGGLE_H2L();
//             //printf("DMA dst reg =%08X, %08X\n", (MXC_DMA->ch[g_dma_spi_rx].dst),dst_ptr);
//             // Enable DMA channel
//             MXC_DMA->ch[g_dma_spi_rx].ctrl += (0x1 << MXC_F_DMA_CTRL_EN_POS);
//             MXC_DMA->ch[g_dma_spi_tx].ctrl += (0x1 << MXC_F_DMA_CTRL_EN_POS);
//             //print_DMA();
//             //MXC_Delay(1);
//             //asm("NOP");
//             //asm("NOP");
//             // Start DMA
//             MXC_SPI0->ctrl0 |= MXC_F_SPI_CTRL0_START;
//     //DEBUG_TOGGLE_H2L();
//             //while((MXC_DMA->ch[g_dma_spi_tx].status & MXC_F_DMA_STATUS_STATUS));
//     //DEBUG_TOGGLE_H2L();
//     }
//     return;
// }