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
#include "fsl_flexio.h"

/*******************************************************************************
 * Definitions for FlexIO
 ******************************************************************************/
#define DEMO_TIME_DELAY_FOR_DUTY_CYCLE_UPDATE (2000000U)
#define DEMO_FLEXIO_BASEADDR FLEXIO3
#define DEMO_FLEXIO_OUTPUTPIN (0U) /* Select FXIO3_D0 as PWM output */
#define DEMO_FLEXIO_TIMER_CH (0U)  /* Flexio timer0 used */

/* Select USB1 PLL (480 MHz) as flexio clock source */
#define FLEXIO_CLOCK_SELECT (3U)
/* Clock pre divider for flexio clock source */
#define FLEXIO_CLOCK_PRE_DIVIDER (4U)
/* Clock divider for flexio clock source */
#define FLEXIO_CLOCK_DIVIDER (7U)
#define DEMO_FLEXIO_CLOCK_FREQUENCY \
    (CLOCK_GetFreq(kCLOCK_Usb1PllClk) / (FLEXIO_CLOCK_PRE_DIVIDER + 1U) / (FLEXIO_CLOCK_DIVIDER + 1U))
/* FLEXIO output PWM frequency */
#define DEMO_FLEXIO_FREQUENCY (48000U)
#define FLEXIO_MAX_FREQUENCY (DEMO_FLEXIO_CLOCK_FREQUENCY / 2U)
#define FLEXIO_MIN_FREQUENCY (DEMO_FLEXIO_CLOCK_FREQUENCY / 256U)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief Configures the timer as a 8-bits PWM mode to generate the PWM waveform
 *
 * @param freq_Hz PWM frequency in hertz, range is [FLEXIO_MIN_FREQUENCY, FLEXIO_MAX_FREQUENCY]
 * @param duty Specified duty in unit of %, with a range of [1, 99]
 */
static void flexio_pwm_init(uint32_t freq_Hz, uint32_t duty);

/*!
 * @brief Enables the timer by setting TIMOD to 8-bits PWM and start generating the PWM
 */
static void flexio_pwm_start(void);


extern volatile uint32_t g_systickCounter;

extern const unsigned char font8x8_basic[128][8];

/* The PIN status */
volatile bool g_pinSet = false;


#define EXAMPLE_SEMC_START_ADDRESS (0x80000000U)

/*******************************************************************************
 * Code
 ******************************************************************************/
extern status_t BOARD_InitSEMC(void);
extern void SysTick_DelayTicks(uint32_t n);
void loop();
void text();
void checker();
void black();

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

    flexio_config_t fxioUserConfig;

	/* Init board hardware. */
    BOARD_ConfigMPU();
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
	/* Init FSL debug console. */
    BOARD_InitDebugConsole();

    /* Clock setting for Flexio */
    CLOCK_SetMux(kCLOCK_Flexio2Mux, FLEXIO_CLOCK_SELECT);
    CLOCK_SetDiv(kCLOCK_Flexio2PreDiv, FLEXIO_CLOCK_PRE_DIVIDER);
    CLOCK_SetDiv(kCLOCK_Flexio2Div, FLEXIO_CLOCK_DIVIDER);

    /* Init flexio, use default configure
     * Disable doze and fast access mode
     * Enable in debug mode
     */
    FLEXIO_GetDefaultConfig(&fxioUserConfig);
    FLEXIO_Init(DEMO_FLEXIO_BASEADDR, &fxioUserConfig);

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
    // get CPU freq and divide by 3 to get 0.25us resolution
    cpuFreq = CLOCK_GetFreq(kCLOCK_CpuClk) / 3;
    /* Force the counter to be placed into memory. */
//    volatile static int i = 0, pin = 0 ;

//    uint32_t *ptr = (uint32_t *)EXAMPLE_SEMC_START_ADDRESS;

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

    /* Set systick reload value to generate 1ms interrupt */
    if (SysTick_Config(SystemCoreClock / 1000U))
    {
        PRINTF("\r\n SysTick Config Failed\r\n");
    	while (1);
    }

    flexio_pwm_init(DEMO_FLEXIO_FREQUENCY, 50);
    flexio_pwm_start();

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

    loop();

    return 0 ;
}


/*
 * At a 60Hz display rate, Trefresh= 16,666 uS
 * 128 lines = 130uS per line
 * 256 pixels/line = 0.5uS
 *
 * SDK_DelayAtLeastUs(1, cpuFreq) should delay 0.25us if cpuFreq is multiplied by 4
 */
void loop() {
//	black();
//  checker();
	text();
}

void black() {
    register uint32_t portVal;
    register uint32_t temp;

    BIT_ALL = BIT_FR | BIT_XSCL | BIT_YSCL | BIT_D0 | BIT_D1 | BIT_D2 | BIT_D3 | BIT_DI;

    portVal = *PORT;
    portVal &= ~(BIT_ALL);
    portVal |= FR;

	// clock out all 1's for TL and BR partitions. First 16 bits aren't visible.
    for (XSCL = 0; XSCL < 256; XSCL ++) {
        temp = portVal;
        if ((XSCL >= 16) && (XSCL < 256))
            temp |= BIT_D0 | BIT_D3;

        temp |= BIT_XSCL;
        *PORT = temp;
        SDK_DelayAtLeastUs(1U, cpuFreq);

        // Data latched on falling edge of XSCL
        temp &= ~BIT_XSCL;
        *PORT = temp;
        SDK_DelayAtLeastUs(1U, cpuFreq);
    }

    while (1) {
        for (YSCL = 1; YSCL < 102; YSCL ++) {
            portVal = *PORT;
            portVal &= ~(BIT_ALL);
            portVal |= FR;
            *PORT = portVal;

            SDK_DelayAtLeastUs(100U, cpuFreq);

            // at end of row, advance the row clock (YSCL)
            portVal = *PORT;
            portVal &= ~(BIT_DI | BIT_YSCL);
            if (YSCL <= 4) {
                portVal |= BIT_DI;
            }
            *PORT = portVal;
            SDK_DelayAtLeastUs(1U, cpuFreq);

            // data latched on rising edge of YSCL
            portVal |= BIT_YSCL;
            *PORT = portVal;
            SDK_DelayAtLeastUs(1U, cpuFreq);
        }  // for (YSCL)
        FR ^= BIT_FR;
    }  // while (1)
}


void checker() {
    register uint32_t portVal;
    register uint32_t temp;

    BIT_ALL = BIT_FR | BIT_XSCL | BIT_YSCL | BIT_D0 | BIT_D1 | BIT_D2 | BIT_D3 | BIT_DI;

    while (1) {
        for (YSCL = 1; YSCL < 102; YSCL ++) {
            portVal = *PORT;
            portVal &= ~(BIT_ALL);
            portVal |= FR;

        	// foreach line clock out a 2x2 pattern. First 16 bits aren't visible.
            for (XSCL = 0; XSCL < 256; XSCL ++) {
                temp = portVal;
                if (((XSCL & 0x02) == (YSCL & 0x02)) && ((XSCL >= 16) && (XSCL < 256)) && (YSCL <= (XSCL - 15)))
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
        }  // for (YSCL)
        FR ^= BIT_FR;
    }  // while (1)
}

const char testString[] = "\001This is a test.\002 ";
const char loremIpsum[] = "\002Lorem ipsum dolor sit amet, consectetur adipiscing elit. Quisque congue dolor augue, ac accumsan nunc luctus sit amet. Aenean vulputate id sapien sed eleifend. Curabitur et accumsan tortor. Vivamus finibus magna ac sapien congue, vitae ullamcorper urna sollicitudin. Pellentesque semper ornare tincidunt. Quisque id efficitur ipsum. Nulla facilisi. Maecenas vitae tellus a odio tempor semper eu eget lectus. Mauris hendrerit cursus laoreet. Integer vel ligula orci. Nam accumsan nisl porta mi lobortis, at accumsan lorem luctus. Sed ac iaculis orci. Phasellus semper tincidunt enim vel volutpat. Mauris pharetra, lectus sit amet euismod viverra, est nisl maximus nisl, in pulvinar nulla ligula quis dolor. Nam efficitur dui sem, quis dapibus lectus semper a. "
		"In convallis tincidunt vehicula. Donec sit amet ligula consequat, egestas odio et, posuere nisl. Vestibulum sollicitudin ante quis posuere auctor. Nunc risus orci, lobortis id eros in, tristique elementum justo. Duis ut lacus vitae orci laoreet luctus. Curabitur eu lectus eu odio sagittis commodo. Curabitur rutrum faucibus accumsan. Nunc felis orci, consectetur sit amet ipsum sed, congue elementum felis. "
		"Vivamus egestas pharetra nisi, eget semper dui commodo in. Duis vel enim placerat erat lobortis semper. Etiam non odio sit amet ex convallis tempor ac vitae neque. Ut hendrerit augue in cursus tempor. Integer viverra tellus eget nisl accumsan pharetra. Praesent lacinia congue magna ut volutpat. Suspendisse nec lobortis tellus, eu imperdiet erat. Phasellus euismod lectus ut sem gravida, eu vestibulum velit condimentum. Quisque sodales eu turpis ultrices mattis. Ut laoreet at felis vel cursus. "
		"Aenean commodo sollicitudin mauris, quis auctor leo molestie ut. Proin ut ultrices tellus. Nunc diam augue, volutpat non elementum a, facilisis a nisl. Quisque euismod vulputate est, a semper quam luctus vel. Aenean at tellus felis. Fusce nec quam non sem laoreet tempor. Aliquam viverra sagittis risus. Pellentesque vehicula est et neque luctus aliquet. Aliquam non sem metus. Vestibulum a nulla quis odio venenatis sagittis. Vivamus ac quam dapibus sapien feugiat tincidunt. Morbi faucibus, nulla consectetur mollis imperdiet, felis justo laoreet enim, id tincidunt purus felis et tellus. Suspendisse aliquet neque nec leo faucibus pretium. Sed eu leo non nunc tristique iaculis eget vitae justo. Praesent pretium ligula ut lectus mattis accumsan. "
		"Nam efficitur, tortor quis ornare lacinia, enim massa fermentum eros, nec pretium sem tortor nec risus. Duis quam lectus, eleifend placerat lacus malesuada, mattis ultrices ligula. Integer molestie metus vitae rutrum lacinia. Aliquam tempor enim odio, at vehicula lectus dapibus ac. Fusce at lacus ligula. Quisque consectetur elementum enim, at gravida est sollicitudin nec. Interdum et malesuada fames ac ante ipsum primis in faucibus. Nam et pharetra lacus. Phasellus semper nec erat vitae eleifend. Lorem ipsum dolor sit amet, consectetur adipiscing elit. Suspendisse lacinia eu dolor ut hendrerit. Proin quis finibus purus. Donec rhoncus, augue quis vulputate rutrum, dolor odio tincidunt felis, vel feugiat ex enim faucibus ligula. Morbi fringilla odio turpis, a imperdiet nibh suscipit sed. Donec arcu mauris, rutrum a lorem ac, mattis suscipit felis. Sed non maximus velit, sed ultrices orci.";
const int testLen = sizeof(testString);
const int loremLen = sizeof(loremIpsum);

char testChar(unsigned int x, unsigned y)
{
	if (y * 60 + x < loremLen)
		return loremIpsum[y * 60 + x];
	else
		return 0;
}

void text() {
    register uint32_t portVal;
    register uint32_t temp;
    int refresh = 0;
    BIT_ALL = BIT_FR | BIT_XSCL | BIT_YSCL | BIT_D0 | BIT_D1 | BIT_D2 | BIT_D3 | BIT_DI;

    while (1) {

        for (YSCL = 0; YSCL < 101; YSCL ++) {
            portVal = *PORT;
            portVal &= ~(BIT_ALL);
            portVal |= FR;
        	// Output a test string.
            for (XSCL = 0; XSCL < 256; XSCL ++) {
                temp = portVal;
                if (XSCL >= 16)
                {
                	register unsigned int xmask = (1 << XSCL % 8);

                	if (font8x8_basic[(int)testChar((XSCL - 16) / 8, YSCL / 8)][YSCL % 8] & xmask)
						temp |= BIT_D0;

                	if (font8x8_basic[(int)testChar((XSCL - 16) / 8 + 30, YSCL / 8)][YSCL % 8] & xmask)
						temp |= BIT_D1;

                	if (font8x8_basic[(int)testChar((XSCL - 16) / 8, (YSCL + 100) / 8)][(YSCL + 100) % 8] & xmask)
						temp |= BIT_D2;

                	if (font8x8_basic[(int)testChar((XSCL - 16) / 8 + 30, (YSCL + 100) / 8)][(YSCL + 100) % 8] & xmask)
						temp |= BIT_D3;
                }

                temp |= BIT_XSCL;
                *PORT = temp;
//                SDK_DelayAtLeastUs(1U, cpuFreq);
                // do some work to delay a bit
            	if (font8x8_basic[(int)testChar((XSCL - 16) / 8, YSCL / 8)][YSCL % 8] & (1 << (XSCL % 8)))
					temp |= BIT_D0;

            	if (font8x8_basic[(int)testChar((XSCL - 16) / 8 + 30, YSCL / 8)][YSCL % 8] & (1 << (XSCL % 8)))
					temp |= BIT_D1;
            	// end delay

                // Data latched on falling edge of XSCL
                temp &= ~BIT_XSCL;
                *PORT = temp;
                if (XSCL == 255)
                	SDK_DelayAtLeastUs(1U, cpuFreq);
            }

            // at end of row, advance the row clock (YSCL)
            portVal = *PORT;
            portVal &= ~(BIT_DI | BIT_YSCL);
            if (YSCL < 1) {
                portVal |= BIT_DI;
            }
            *PORT = portVal;
            SDK_DelayAtLeastUs(1U, cpuFreq);

            // data latched on rising edge of YSCL
            portVal |= BIT_YSCL;
            *PORT = portVal;
            SDK_DelayAtLeastUs(1U, cpuFreq);

            // toggle refresh signal
            refresh ++;
            if (refresh == 33)
            {
                FR ^= BIT_FR;
                refresh = 0;
            }

        } // for (YSCL)

    } // while (1)
}

// Constant: font8x8_basic
// Contains an 8x8 font map for unicode points U+0000 - U+007F (basic latin)
const unsigned char font8x8_basic[128][8] = {
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+0000 (nul)
    { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff},   // U+0001
    { 0xff, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0xff},   // U+0002
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+0003
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+0004
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+0005
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+0006
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+0007
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+0008
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+0009
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+000A
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+000B
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+000C
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+000D
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+000E
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+000F
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+0010
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+0011
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+0012
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+0013
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+0014
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+0015
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+0016
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+0017
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+0018
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+0019
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+001A
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+001B
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+001C
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+001D
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+001E
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+001F
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+0020 (space)
    { 0x18, 0x3C, 0x3C, 0x18, 0x18, 0x00, 0x18, 0x00},   // U+0021 (!)
    { 0x36, 0x36, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+0022 (")
    { 0x36, 0x36, 0x7F, 0x36, 0x7F, 0x36, 0x36, 0x00},   // U+0023 (#)
    { 0x0C, 0x3E, 0x03, 0x1E, 0x30, 0x1F, 0x0C, 0x00},   // U+0024 ($)
    { 0x00, 0x63, 0x33, 0x18, 0x0C, 0x66, 0x63, 0x00},   // U+0025 (%)
    { 0x1C, 0x36, 0x1C, 0x6E, 0x3B, 0x33, 0x6E, 0x00},   // U+0026 (&)
    { 0x06, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+0027 (')
    { 0x18, 0x0C, 0x06, 0x06, 0x06, 0x0C, 0x18, 0x00},   // U+0028 (()
    { 0x06, 0x0C, 0x18, 0x18, 0x18, 0x0C, 0x06, 0x00},   // U+0029 ())
    { 0x00, 0x66, 0x3C, 0xFF, 0x3C, 0x66, 0x00, 0x00},   // U+002A (*)
    { 0x00, 0x0C, 0x0C, 0x3F, 0x0C, 0x0C, 0x00, 0x00},   // U+002B (+)
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C, 0x06},   // U+002C (,)
    { 0x00, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x00, 0x00},   // U+002D (-)
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C, 0x00},   // U+002E (.)
    { 0x60, 0x30, 0x18, 0x0C, 0x06, 0x03, 0x01, 0x00},   // U+002F (/)
    { 0x3E, 0x63, 0x73, 0x7B, 0x6F, 0x67, 0x3E, 0x00},   // U+0030 (0)
    { 0x0C, 0x0E, 0x0C, 0x0C, 0x0C, 0x0C, 0x3F, 0x00},   // U+0031 (1)
    { 0x1E, 0x33, 0x30, 0x1C, 0x06, 0x33, 0x3F, 0x00},   // U+0032 (2)
    { 0x1E, 0x33, 0x30, 0x1C, 0x30, 0x33, 0x1E, 0x00},   // U+0033 (3)
    { 0x38, 0x3C, 0x36, 0x33, 0x7F, 0x30, 0x78, 0x00},   // U+0034 (4)
    { 0x3F, 0x03, 0x1F, 0x30, 0x30, 0x33, 0x1E, 0x00},   // U+0035 (5)
    { 0x1C, 0x06, 0x03, 0x1F, 0x33, 0x33, 0x1E, 0x00},   // U+0036 (6)
    { 0x3F, 0x33, 0x30, 0x18, 0x0C, 0x0C, 0x0C, 0x00},   // U+0037 (7)
    { 0x1E, 0x33, 0x33, 0x1E, 0x33, 0x33, 0x1E, 0x00},   // U+0038 (8)
    { 0x1E, 0x33, 0x33, 0x3E, 0x30, 0x18, 0x0E, 0x00},   // U+0039 (9)
    { 0x00, 0x0C, 0x0C, 0x00, 0x00, 0x0C, 0x0C, 0x00},   // U+003A (:)
    { 0x00, 0x0C, 0x0C, 0x00, 0x00, 0x0C, 0x0C, 0x06},   // U+003B (;)
    { 0x18, 0x0C, 0x06, 0x03, 0x06, 0x0C, 0x18, 0x00},   // U+003C (<)
    { 0x00, 0x00, 0x3F, 0x00, 0x00, 0x3F, 0x00, 0x00},   // U+003D (=)
    { 0x06, 0x0C, 0x18, 0x30, 0x18, 0x0C, 0x06, 0x00},   // U+003E (>)
    { 0x1E, 0x33, 0x30, 0x18, 0x0C, 0x00, 0x0C, 0x00},   // U+003F (?)
    { 0x3E, 0x63, 0x7B, 0x7B, 0x7B, 0x03, 0x1E, 0x00},   // U+0040 (@)
    { 0x0C, 0x1E, 0x33, 0x33, 0x3F, 0x33, 0x33, 0x00},   // U+0041 (A)
    { 0x3F, 0x66, 0x66, 0x3E, 0x66, 0x66, 0x3F, 0x00},   // U+0042 (B)
    { 0x3C, 0x66, 0x03, 0x03, 0x03, 0x66, 0x3C, 0x00},   // U+0043 (C)
    { 0x1F, 0x36, 0x66, 0x66, 0x66, 0x36, 0x1F, 0x00},   // U+0044 (D)
    { 0x7F, 0x46, 0x16, 0x1E, 0x16, 0x46, 0x7F, 0x00},   // U+0045 (E)
    { 0x7F, 0x46, 0x16, 0x1E, 0x16, 0x06, 0x0F, 0x00},   // U+0046 (F)
    { 0x3C, 0x66, 0x03, 0x03, 0x73, 0x66, 0x7C, 0x00},   // U+0047 (G)
    { 0x33, 0x33, 0x33, 0x3F, 0x33, 0x33, 0x33, 0x00},   // U+0048 (H)
    { 0x1E, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x1E, 0x00},   // U+0049 (I)
    { 0x78, 0x30, 0x30, 0x30, 0x33, 0x33, 0x1E, 0x00},   // U+004A (J)
    { 0x67, 0x66, 0x36, 0x1E, 0x36, 0x66, 0x67, 0x00},   // U+004B (K)
    { 0x0F, 0x06, 0x06, 0x06, 0x46, 0x66, 0x7F, 0x00},   // U+004C (L)
    { 0x63, 0x77, 0x7F, 0x7F, 0x6B, 0x63, 0x63, 0x00},   // U+004D (M)
    { 0x63, 0x67, 0x6F, 0x7B, 0x73, 0x63, 0x63, 0x00},   // U+004E (N)
    { 0x1C, 0x36, 0x63, 0x63, 0x63, 0x36, 0x1C, 0x00},   // U+004F (O)
    { 0x3F, 0x66, 0x66, 0x3E, 0x06, 0x06, 0x0F, 0x00},   // U+0050 (P)
    { 0x1E, 0x33, 0x33, 0x33, 0x3B, 0x1E, 0x38, 0x00},   // U+0051 (Q)
    { 0x3F, 0x66, 0x66, 0x3E, 0x36, 0x66, 0x67, 0x00},   // U+0052 (R)
    { 0x1E, 0x33, 0x07, 0x0E, 0x38, 0x33, 0x1E, 0x00},   // U+0053 (S)
    { 0x3F, 0x2D, 0x0C, 0x0C, 0x0C, 0x0C, 0x1E, 0x00},   // U+0054 (T)
    { 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x3F, 0x00},   // U+0055 (U)
    { 0x33, 0x33, 0x33, 0x33, 0x33, 0x1E, 0x0C, 0x00},   // U+0056 (V)
    { 0x63, 0x63, 0x63, 0x6B, 0x7F, 0x77, 0x63, 0x00},   // U+0057 (W)
    { 0x63, 0x63, 0x36, 0x1C, 0x1C, 0x36, 0x63, 0x00},   // U+0058 (X)
    { 0x33, 0x33, 0x33, 0x1E, 0x0C, 0x0C, 0x1E, 0x00},   // U+0059 (Y)
    { 0x7F, 0x63, 0x31, 0x18, 0x4C, 0x66, 0x7F, 0x00},   // U+005A (Z)
    { 0x1E, 0x06, 0x06, 0x06, 0x06, 0x06, 0x1E, 0x00},   // U+005B ([)
    { 0x03, 0x06, 0x0C, 0x18, 0x30, 0x60, 0x40, 0x00},   // U+005C (\)
    { 0x1E, 0x18, 0x18, 0x18, 0x18, 0x18, 0x1E, 0x00},   // U+005D (])
    { 0x08, 0x1C, 0x36, 0x63, 0x00, 0x00, 0x00, 0x00},   // U+005E (^)
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF},   // U+005F (_)
    { 0x0C, 0x0C, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+0060 (`)
    { 0x00, 0x00, 0x1E, 0x30, 0x3E, 0x33, 0x6E, 0x00},   // U+0061 (a)
    { 0x07, 0x06, 0x06, 0x3E, 0x66, 0x66, 0x3B, 0x00},   // U+0062 (b)
    { 0x00, 0x00, 0x1E, 0x33, 0x03, 0x33, 0x1E, 0x00},   // U+0063 (c)
    { 0x38, 0x30, 0x30, 0x3e, 0x33, 0x33, 0x6E, 0x00},   // U+0064 (d)
    { 0x00, 0x00, 0x1E, 0x33, 0x3f, 0x03, 0x1E, 0x00},   // U+0065 (e)
    { 0x1C, 0x36, 0x06, 0x0f, 0x06, 0x06, 0x0F, 0x00},   // U+0066 (f)
    { 0x00, 0x00, 0x6E, 0x33, 0x33, 0x3E, 0x30, 0x1F},   // U+0067 (g)
    { 0x07, 0x06, 0x36, 0x6E, 0x66, 0x66, 0x67, 0x00},   // U+0068 (h)
    { 0x0C, 0x00, 0x0E, 0x0C, 0x0C, 0x0C, 0x1E, 0x00},   // U+0069 (i)
    { 0x30, 0x00, 0x30, 0x30, 0x30, 0x33, 0x33, 0x1E},   // U+006A (j)
    { 0x07, 0x06, 0x66, 0x36, 0x1E, 0x36, 0x67, 0x00},   // U+006B (k)
    { 0x0E, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x1E, 0x00},   // U+006C (l)
    { 0x00, 0x00, 0x33, 0x7F, 0x7F, 0x6B, 0x63, 0x00},   // U+006D (m)
    { 0x00, 0x00, 0x1F, 0x33, 0x33, 0x33, 0x33, 0x00},   // U+006E (n)
    { 0x00, 0x00, 0x1E, 0x33, 0x33, 0x33, 0x1E, 0x00},   // U+006F (o)
    { 0x00, 0x00, 0x3B, 0x66, 0x66, 0x3E, 0x06, 0x0F},   // U+0070 (p)
    { 0x00, 0x00, 0x6E, 0x33, 0x33, 0x3E, 0x30, 0x78},   // U+0071 (q)
    { 0x00, 0x00, 0x3B, 0x6E, 0x66, 0x06, 0x0F, 0x00},   // U+0072 (r)
    { 0x00, 0x00, 0x3E, 0x03, 0x1E, 0x30, 0x1F, 0x00},   // U+0073 (s)
    { 0x08, 0x0C, 0x3E, 0x0C, 0x0C, 0x2C, 0x18, 0x00},   // U+0074 (t)
    { 0x00, 0x00, 0x33, 0x33, 0x33, 0x33, 0x6E, 0x00},   // U+0075 (u)
    { 0x00, 0x00, 0x33, 0x33, 0x33, 0x1E, 0x0C, 0x00},   // U+0076 (v)
    { 0x00, 0x00, 0x63, 0x6B, 0x7F, 0x7F, 0x36, 0x00},   // U+0077 (w)
    { 0x00, 0x00, 0x63, 0x36, 0x1C, 0x36, 0x63, 0x00},   // U+0078 (x)
    { 0x00, 0x00, 0x33, 0x33, 0x33, 0x3E, 0x30, 0x1F},   // U+0079 (y)
    { 0x00, 0x00, 0x3F, 0x19, 0x0C, 0x26, 0x3F, 0x00},   // U+007A (z)
    { 0x38, 0x0C, 0x0C, 0x07, 0x0C, 0x0C, 0x38, 0x00},   // U+007B ({)
    { 0x18, 0x18, 0x18, 0x00, 0x18, 0x18, 0x18, 0x00},   // U+007C (|)
    { 0x07, 0x0C, 0x0C, 0x38, 0x0C, 0x0C, 0x07, 0x00},   // U+007D (})
    { 0x6E, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+007E (~)
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}    // U+007F
};

/*******************************************************************************
 * Code
 ******************************************************************************/
static void flexio_pwm_init(uint32_t freq_Hz, uint32_t duty)
{
    assert((freq_Hz < FLEXIO_MAX_FREQUENCY) && (freq_Hz > FLEXIO_MIN_FREQUENCY));

    uint32_t lowerValue = 0; /* Number of clock cycles in high logic state in one period */
    uint32_t upperValue = 0; /* Number of clock cycles in low logic state in one period */
    uint32_t sum        = 0; /* Number of clock cycles in one period */
    flexio_timer_config_t fxioTimerConfig;

    /* Check parameter */
    if ((duty > 99) || (duty == 0))
    {
        duty = 50;
    }

    /* Configure the timer DEMO_FLEXIO_TIMER_CH for generating PWM */
    fxioTimerConfig.triggerSelect   = FLEXIO_TIMER_TRIGGER_SEL_SHIFTnSTAT(0U);
    fxioTimerConfig.triggerSource   = kFLEXIO_TimerTriggerSourceInternal;
    fxioTimerConfig.triggerPolarity = kFLEXIO_TimerTriggerPolarityActiveLow;
    fxioTimerConfig.pinConfig       = kFLEXIO_PinConfigOutput;
    fxioTimerConfig.pinPolarity     = kFLEXIO_PinActiveHigh;
    fxioTimerConfig.pinSelect       = DEMO_FLEXIO_OUTPUTPIN; /* Set pwm output */
    fxioTimerConfig.timerMode       = kFLEXIO_TimerModeDisabled;
    fxioTimerConfig.timerOutput     = kFLEXIO_TimerOutputOneNotAffectedByReset;
    fxioTimerConfig.timerDecrement  = kFLEXIO_TimerDecSrcOnFlexIOClockShiftTimerOutput;
    fxioTimerConfig.timerDisable    = kFLEXIO_TimerDisableNever;
    fxioTimerConfig.timerEnable     = kFLEXIO_TimerEnabledAlways;
    fxioTimerConfig.timerReset      = kFLEXIO_TimerResetNever;
    fxioTimerConfig.timerStart      = kFLEXIO_TimerStartBitDisabled;
    fxioTimerConfig.timerStop       = kFLEXIO_TimerStopBitDisabled;

    /* Calculate timer lower and upper values of TIMCMP */
    /* Calculate the nearest integer value for sum, using formula round(x) = (2 * floor(x) + 1) / 2 */
    /* sum = DEMO_FLEXIO_CLOCK_FREQUENCY / freq_H */
    sum = (DEMO_FLEXIO_CLOCK_FREQUENCY * 2 / freq_Hz + 1) / 2;
    /* Calculate the nearest integer value for lowerValue, the high period of the pwm output */
    /* lowerValue = sum * duty / 100 */
    lowerValue = (sum * duty / 50 + 1) / 2;
    /* Calculate upper value, the low period of the pwm output */
    upperValue                   = sum - lowerValue;
    fxioTimerConfig.timerCompare = ((upperValue - 1) << 8U) | (lowerValue - 1);

    FLEXIO_SetTimerConfig(DEMO_FLEXIO_BASEADDR, DEMO_FLEXIO_TIMER_CH, &fxioTimerConfig);
}

static void flexio_pwm_start(void)
{
    /* Set Timer mode to kFLEXIO_TimerModeDual8BitPWM to start timer */
    DEMO_FLEXIO_BASEADDR->TIMCTL[DEMO_FLEXIO_TIMER_CH] |= FLEXIO_TIMCTL_TIMOD(kFLEXIO_TimerModeDual8BitPWM);
}

