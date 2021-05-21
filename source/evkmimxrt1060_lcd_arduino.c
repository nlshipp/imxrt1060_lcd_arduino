/*
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @file    evkmimxrt1060_lcd_arduino.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MIMXRT1062.h"
#include "fsl_debug_console.h"

extern volatile uint32_t g_systickCounter;

/* The PIN status */
volatile bool g_pinSet = false;

#define EXAMPLE_SEMC_START_ADDRESS (0x80000000U)

/*******************************************************************************
 * Code
 ******************************************************************************/
extern status_t BOARD_InitSEMC(void);
extern void SysTick_DelayTicks(uint32_t n);
void loop();

/*
 * Arduino pins D0-D7
 */

int pins[] =
{
		23,
		22,
		11,
		24,
		9,
		10,
		18,
		19
};

#define digitalPinToBitMask(x) (1 << pins[x])

/*
ARDUINO UNO pins
const int LCD_DI = 2;
const int LCD_YSCL = 3;
const int LCD_XSCL = 4;
const int LCD_D0 = 5;
const int LCD_FR = 6;
const int LCD_D1 = 8;
const int LCD_D2 = 11;
const int LCD_D3 = 12;
*/
/* NXP 1060 pins */
const int LCD_DI = 0;
const int LCD_YSCL = 1;
const int LCD_XSCL = 2;
const int LCD_D0 = 3;
const int LCD_FR = 4;
const int LCD_D1 = 5;
const int LCD_D2 = 6;
const int LCD_D3 = 7;

uint32_t cpuFreq;

volatile uint32_t * PORT;

unsigned int XSCL, YSCL;
uint32_t FR;

uint32_t BIT_DI;
uint32_t BIT_YSCL;
uint32_t BIT_XSCL;
uint32_t BIT_D0;
uint32_t BIT_D1;
uint32_t BIT_D2;
uint32_t BIT_D3;
uint32_t BIT_FR;
uint32_t BIT_ALL;

int main(void) {

	/* Init board hardware. */
    BOARD_ConfigMPU();
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
	/* Init FSL debug console. */
    BOARD_InitDebugConsole();

	PRINTF("LCD test program\n");
#if 0
    CLOCK_InitSysPfd(kCLOCK_Pfd2, 29);
    /* Set semc clock to 163.86 MHz */
    CLOCK_SetMux(kCLOCK_SemcMux, 1);
    CLOCK_SetDiv(kCLOCK_SemcDiv, 1);

    if (BOARD_InitSEMC() != kStatus_Success)
    {
        PRINTF("\r\n SEMC SDRAM Init Failed\r\n");
    }
#endif
    // get CPU freq and divide by 3 to get 0.5us resolution
    cpuFreq = CLOCK_GetFreq(kCLOCK_CpuClk) / 6;

    /* Force the counter to be placed into memory. */
    volatile static int i = 0, pin = 0 ;

    uint32_t *ptr = (uint32_t *)EXAMPLE_SEMC_START_ADDRESS;

    XSCL = 0;
    YSCL = 0;
    FR = 0;

    // NXP 1060
    PORT = (uint32_t *)GPIO1_BASE;
    BIT_DI = digitalPinToBitMask(LCD_DI);
    BIT_YSCL = digitalPinToBitMask(LCD_YSCL);
    BIT_XSCL = digitalPinToBitMask(LCD_XSCL);
    BIT_D0 = digitalPinToBitMask(LCD_D0);
    BIT_D1 = digitalPinToBitMask(LCD_D1);
    BIT_D2 = digitalPinToBitMask(LCD_D2);
    BIT_D3 = digitalPinToBitMask(LCD_D3);
    BIT_FR = digitalPinToBitMask(LCD_FR);

    BIT_ALL = BIT_DI | BIT_YSCL | BIT_XSCL | BIT_D0 | BIT_FR;

    /* Set systick reload value to generate 1ms interrupt */
    if (SysTick_Config(SystemCoreClock / 1000U))
    {
        PRINTF("\r\n SysTick Config Failed\r\n");
    	while (1);
    }

    loop();
#if 0
    i = 1;
    pin = 0;
    while (1)
    {
        /* Delay 10 ms */
//        SysTick_DelayTicks(10U);
        if (g_pinSet)
        {
            GPIO_PinWrite(GPIO1, pins[pin], 0U);
            g_pinSet = false;
        }
        else
        {
            GPIO_PinWrite(GPIO1, pins[pin], 1U);
            g_pinSet = true;
        }
//        *ptr++ = i++;
    }
#endif

    return 0 ;
}


/*
 * At a 60Hz display rate, Trefresh= 16,666 uS
 * 128 lines = 130uS per line
 * 256 pixels/line = 0.5uS
 */

void loop() {
    register uint32_t portVal;
    register uint32_t temp;

    while (1) {

        for (YSCL = 1; YSCL < 102; YSCL ++) {
            portVal = *PORT;
            portVal &= ~(BIT_FR | BIT_XSCL | BIT_YSCL | BIT_D0 | BIT_D1 | BIT_D2 | BIT_D3 | BIT_DI);
            portVal |= FR;
        	// foreach line clock out a 2x2 pattern. First 16 bits aren't visible.
            for (XSCL = 0; XSCL < 256; XSCL ++) {
                temp = portVal;
                if (((XSCL & 0x02) == (YSCL & 0x02)) && ((XSCL >= 16) && (XSCL < 256)) && (YSCL < (XSCL - 16)))
                    temp |= BIT_D0 | BIT_D1 | BIT_D2 | BIT_D3;

                temp |= BIT_XSCL;
                *PORT = temp;
                SDK_DelayAtLeastUs(1U, cpuFreq);

                // Data latched on falling edge of XSCL
                temp &= ~BIT_XSCL;
                *PORT = temp;
                SDK_DelayAtLeastUs(1U, cpuFreq);
            }

            // at end of row, advance the row clock (YSCL)
            portVal = *PORT;
            portVal &= ~(BIT_DI | BIT_YSCL);
            if (YSCL <= 1) {
                portVal |= BIT_DI;
            }
            *PORT = portVal;
            SDK_DelayAtLeastUs(1U, cpuFreq);

            // data latched on rising edge of YSCL
            portVal |= BIT_YSCL;
            *PORT = portVal;
            SDK_DelayAtLeastUs(1U, cpuFreq);

            FR ^= BIT_FR;
        }
    }
}
