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
#include "spi.h"
#include "uart.h"
#include "gpio.h"

/***** Preprocessors *****/
#define MASTERSYNC 0
#define MASTERASYNC 0
#define MASTERDMA 1

#if (!(MASTERSYNC || MASTERASYNC || MASTERDMA))
#error "You must set either MASTERSYNC or MASTERASYNC or MASTERDMA to 1."
#endif
#if ((MASTERSYNC && MASTERASYNC) || (MASTERASYNC && MASTERDMA) || (MASTERDMA && MASTERSYNC))
#error "You must select either MASTERSYNC or MASTERASYNC or MASTERDMA, not all 3."
#endif

/***** Definitions *****/
#define DATA_LEN 1000 // Words
#define DATA_VALUE 0xA5A5 // This is for master mode only...
#define VALUE 0xFFFF
#define SPI_SPEED 100000 // Bit Rate

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
uint16_t rx_data[DATA_LEN];
uint16_t tx_data[DATA_LEN];
uint8_t rx_data_8[DATA_LEN];
uint8_t tx_data_8[DATA_LEN];
mxc_spi_req_t req;
mxc_spi_pins_t spi_pins;

volatile int SPI_FLAG;
volatile uint8_t DMA_FLAG = 0;
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
    __IO uint8_t  config_in_n[16];            /**< <tt>\b 0x10:</tt> GPIO_REVA OUTEN_SET Register */
    __IO uint16_t upper_in_n[16];            /**< <tt>\b 0x10:</tt> GPIO_REVA OUTEN_SET Register */
    __IO uint16_t lower_in_n[16];            /**< <tt>\b 0x10:</tt> GPIO_REVA OUTEN_SET Register */
    __IO uint16_t hyst_in_n[16];            /**< <tt>\b 0x10:</tt> GPIO_REVA OUTEN_SET Register */
    __IO uint16_t offset_in_n[16];            /**< <tt>\b 0x10:</tt> GPIO_REVA OUTEN_SET Register */
    __IO uint16_t gain_in_n[16];            /**< <tt>\b 0x10:</tt> GPIO_REVA OUTEN_SET Register */
    __IO uint8_t  as_slot_n[128];            /**< <tt>\b 0x10:</tt> GPIO_REVA OUTEN_SET Register */
} ad4696_regs_t;

////
/***** Functions *****/
void print_GPIO(void){
	printf("\n***** GPIO REGISTERS *****\n");
	//format
	// 	printf("0x:%08X\n",     *(  (volatile uint32_t *) <address in hex>  )      );
	//
	uint32_t gpio_base = 0x40008000;
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


void SPI_IRQHandler(void)
{
    MXC_SPI_AsyncHandler(SPI);
}

void DMA0_IRQHandler(void)
{
    MXC_DMA_Handler();
}

void DMA1_IRQHandler(void)
{
    MXC_DMA_Handler();
    DMA_FLAG = 1;
}

void SPI_Callback(mxc_spi_req_t *req, int error)
{
    SPI_FLAG = error;
}
// configure AD4696 nRESET (p1.1), CNV(p1.6) and GPIO (p0.19)
const mxc_gpio_cfg_t debug_pin[] = {
    { MXC_GPIO1, MXC_GPIO_PIN_6, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO }, //CNV
    { MXC_GPIO1, MXC_GPIO_PIN_1, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO },  //nRESET
    //TODO: p0.19
};
const unsigned int num_debugs = (sizeof(debug_pin) / sizeof(mxc_gpio_cfg_t));

int debug_Init(void)
{
    int retval = E_NO_ERROR;
    unsigned int i;

    /* setup 2 GPIOs for the debug */
    for (i = 0; i < num_debugs; i++) {
        if (MXC_GPIO_Config(&debug_pin[i]) != E_NO_ERROR) {
            retval = E_UNKNOWN;
        }
    }

    return retval;
}

int MAX78000_SPI_Config(void) {
    
    int retVal;
    // mxc_spi_req_t req;
    // mxc_spi_pins_t spi_pins;
    
    spi_pins.clock = TRUE;
    spi_pins.miso = TRUE;
    spi_pins.mosi = TRUE;
    spi_pins.sdio2 = FALSE;
    spi_pins.sdio3 = FALSE;
    spi_pins.ss0 = TRUE;
    spi_pins.ss1 = TRUE;
    spi_pins.ss2 = FALSE;
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

    return 0;
}


void AD4696_RESET(void) {
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
    //print_GPIO();
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
    //print_GPIO();
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

int AD4696_READ_all(ad4696_regs_t *value)
{
    value->spi_config_a = 0x43;
    value->spi_config_c = 0xf3;
    value->gain_in_n[3] = 0xbeef;
    printf("here it is: %02x\n", value->spi_config_a);
    printf("here it is: %02x\n", value->spi_config_c);
    printf("here it is: %04x\n", value->gain_in_n[3]);
    value->spi_config_c = 0x33;
       printf("here it is: %02x\n", value->spi_config_a);
    printf("here it is: %02x\n", value->spi_config_c);
    printf("here it is: %04x\n", value->gain_in_n[3]);
    //printf("here is value: %08x\n", value[1]);
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

    AD4696_WRITE(LOOP_MODE,0);
    return retVal;

}


int main(void)
{
    //int retVal;
    //uint16_t temp;
    //mxc_spi_req_t req;
    //mxc_spi_pins_t spi_pins;
    ad4696_regs_t ad_4696_regs;

    printf("\n**************************** SPI MASTER with AD4696 *************************\n");
    printf("[x] Compiled: %s, %s\n",__DATE__,__TIME__);
    //print_GPIO();
    printf("set up RESET and CNV pins\n");
    debug_Init();
    //print_GPIO();
#ifdef BOARD_EVKIT_V1
    printf("not supported\n");
    return 0;
#else
    printf("MAX78000 feather board connection to AD4696.\n\n");
#endif

/*
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
    printf("releasing nRESET\n");
    //MXC_GPIO_OutSet(MXC_GPIO1, MXC_GPIO_PIN_1);
    AD4696_GO();
    // Configure the peripheral
    retVal = MXC_SPI_Init(SPI, 1, 0, 1, 0, SPI_SPEED, spi_pins);
    if (retVal != E_NO_ERROR) {
        printf("\nSPI INITIALIZATION ERROR\n");
        return retVal;
    }
*/
    AD4696_GO();
    MAX78000_SPI_Config();
    AD4696_READ(VENDOR_H);
    //printf("read value: %02x\n",rx_data_8[2]);
    AD4696_READ(VENDOR_L);
    AD4696_READ(DEVICE_TYPE);
    AD4696_WRITE(SCRATCH_PAD, 0x73);
    AD4696_READ(SCRATCH_PAD);
    AD4696_READ(VENDOR_H);
    AD4696_READ(VENDOR_L);
    AD4696_READ(DEVICE_TYPE);
    AD4696_READ(SCRATCH_PAD);
    AD4696_READ(SPI_STATUS);
    AD4696_READ(STATUS);
    AD4696_READ_Loop(LOOP_MODE,15);
    AD4696_READ(STATUS);
    AD4696_READ_all(&ad_4696_regs);

#if 0
//     //tx_data[0] = 0xabcd;
//     //tx_data[1] = 0x1234;
//     //tx_data[2] = 0x6789;
//     tx_data_8[0] = (VENDOR_H & 0xFF00 )>>8;//0x80;
//     tx_data_8[1] = (VENDOR_H & 0x00FF )>>0;//0x0C;
//     tx_data_8[2] = 0x00;
//     req.spi = SPI;
//     //req.txData = (uint8_t *)tx_data;
//     //req.rxData = (uint8_t *)rx_data;
//     req.txData = (uint8_t *)tx_data_8;
//     req.rxData = (uint8_t *)rx_data_8;
//     req.txLen = 3;
//     req.rxLen = 3;
//     req.ssIdx = 1;
//     req.ssDeassert = 1;
//     req.txCnt = 0;
//     req.rxCnt = 0;
//     req.completeCB = (spi_complete_cb_t)SPI_Callback;
//     SPI_FLAG = 1;

//     //retVal = MXC_SPI_SetDataSize(SPI, 16);
//     retVal = MXC_SPI_SetDataSize(SPI, 8);

//     if (retVal != E_NO_ERROR) {
//         printf("\nSPI SET DATASIZE ERROR: %d\n", retVal);
//         return retVal;
//     }

//     retVal = MXC_SPI_SetWidth(SPI, SPI_WIDTH_STANDARD);

//     if (retVal != E_NO_ERROR) {
//         printf("\nSPI SET WIDTH ERROR: %d\n", retVal);
//         return retVal;
//     }
//     print_GPIO();
// #if MASTERSYNC
//         MXC_SPI_MasterTransaction(&req);
// #endif

// #if MASTERASYNC
//         NVIC_EnableIRQ(SPI_IRQ);
//         MXC_SPI_MasterTransactionAsync(&req);

//         while (SPI_FLAG == 1) {}

// #endif

// #if MASTERDMA
//     MXC_DMA_ReleaseChannel(0);
//     MXC_DMA_ReleaseChannel(1);

//     NVIC_EnableIRQ(DMA0_IRQn);
//     NVIC_EnableIRQ(DMA1_IRQn);
//     MXC_SPI_MasterTransactionDMA(&req);

//     while (DMA_FLAG == 0) {}

//     DMA_FLAG = 0;
// #endif

    // tx_data[0] = 0xdead;
    // tx_data[1] = 0xbeef;
    // tx_data[2] = 0x1354;
    tx_data_8[0] = 0x80;
    tx_data_8[1] = 0x0D;
    tx_data_8[2] = 0x00;
    // req.txData = (uint8_t *)tx_data;
    // req.rxData = (uint8_t *)rx_data;
    req.txData = (uint8_t *)tx_data_8;
    req.rxData = (uint8_t *)rx_data_8;
    req.txLen = 3;
    req.rxLen = 3;

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
    MXC_SPI_MasterTransactionDMA(&req);

    while (DMA_FLAG == 0) {}

    DMA_FLAG = 0;
#endif
#endif
    AD4696_RESET();
    //MXC_GPIO_OutClr(MXC_GPIO1, MXC_GPIO_PIN_1);
    printf("\nExample Complete.\n");
    return E_NO_ERROR;
}


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

