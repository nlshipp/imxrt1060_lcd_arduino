/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

#ifndef _PIN_MUX_H_
#define _PIN_MUX_H_

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/

/*! @brief Direction type  */
typedef enum _pin_mux_direction
{
  kPIN_MUX_DirectionInput = 0U,         /* Input direction */
  kPIN_MUX_DirectionOutput = 1U,        /* Output direction */
  kPIN_MUX_DirectionInputOrOutput = 2U  /* Input or output direction */
} pin_mux_direction_t;

/*!
 * @addtogroup pin_mux
 * @{
 */

/***********************************************************************************************************************
 * API
 **********************************************************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Calls initialization functions.
 *
 */
void BOARD_InitBootPins(void);

/* GPIO_AD_B1_07 (coord K10), CSI_HSYNC/J35[16]/J22[1]/UART_RX */
#define BOARD_INITPINS_CSI_HSYNC_GPIO                                      GPIO1   /*!< GPIO device name: GPIO1 */
#define BOARD_INITPINS_CSI_HSYNC_PORT                                      GPIO1   /*!< PORT device name: GPIO1 */
#define BOARD_INITPINS_CSI_HSYNC_PIN                                         23U   /*!< GPIO1 pin index: 23 */

/* GPIO_AD_B1_06 (coord J12), CSI_VSYNC/J35[18]/J22[2]/UART_TX */
#define BOARD_INITPINS_CSI_VSYNC_GPIO                                      GPIO1   /*!< GPIO device name: GPIO1 */
#define BOARD_INITPINS_CSI_VSYNC_PORT                                      GPIO1   /*!< PORT device name: GPIO1 */
#define BOARD_INITPINS_CSI_VSYNC_PIN                                         22U   /*!< GPIO1 pin index: 22 */

/* GPIO_AD_B1_08 (coord H13), AUD_INT/CSI_D9//J35[13]/J22[4] */
#define BOARD_INITPINS_CSI_D9_GPIO                                         GPIO1   /*!< GPIO device name: GPIO1 */
#define BOARD_INITPINS_CSI_D9_PORT                                         GPIO1   /*!< PORT device name: GPIO1 */
#define BOARD_INITPINS_CSI_D9_PIN                                            24U   /*!< GPIO1 pin index: 24 */

/* GPIO_AD_B1_02 (coord L11), SPDIF_OUT/J22[7] */
#define BOARD_INITPINS_SPDIF_OUT_GPIO                                      GPIO1   /*!< GPIO device name: GPIO1 */
#define BOARD_INITPINS_SPDIF_OUT_PORT                                      GPIO1   /*!< PORT device name: GPIO1 */
#define BOARD_INITPINS_SPDIF_OUT_PIN                                         18U   /*!< GPIO1 pin index: 18 */

/* GPIO_AD_B1_03 (coord M12), SPDIF_IN/J22[8] */
#define BOARD_INITPINS_SPDIF_IN_GPIO                                       GPIO1   /*!< GPIO device name: GPIO1 */
#define BOARD_INITPINS_SPDIF_IN_PORT                                       GPIO1   /*!< PORT device name: GPIO1 */
#define BOARD_INITPINS_SPDIF_IN_PIN                                          19U   /*!< GPIO1 pin index: 19 */

/* GPIO_AD_B0_09 (coord F14), JTAG_TDI/J21[5]/ENET_RST/J22[5] */
#define BOARD_INITPINS_TDI_GPIO                                            GPIO1   /*!< GPIO device name: GPIO1 */
#define BOARD_INITPINS_TDI_PORT                                            GPIO1   /*!< PORT device name: GPIO1 */
#define BOARD_INITPINS_TDI_PIN                                                9U   /*!< GPIO1 pin index: 9 */

/* GPIO_AD_B0_10 (coord G13), JTAG_TDO/J21[13]/INT1_COMBO/ENET_INT/J22[6]/U32[11] */
#define BOARD_INITPINS_INT1_COMBO_GPIO                                     GPIO1   /*!< GPIO device name: GPIO1 */
#define BOARD_INITPINS_INT1_COMBO_PORT                                     GPIO1   /*!< PORT device name: GPIO1 */
#define BOARD_INITPINS_INT1_COMBO_PIN                                        10U   /*!< GPIO1 pin index: 10 */

/* GPIO_AD_B0_11 (coord G10), JTAG_nTRST/J21[3]/INT2_COMBO/LCD_TOUCH_INT/J22[3]/U32[9] */
#define BOARD_INITPINS_INT2_COMBO_GPIO                                     GPIO1   /*!< GPIO device name: GPIO1 */
#define BOARD_INITPINS_INT2_COMBO_PORT                                     GPIO1   /*!< PORT device name: GPIO1 */
#define BOARD_INITPINS_INT2_COMBO_PIN                                        11U   /*!< GPIO1 pin index: 11 */

/* GPIO_AD_B1_00 (coord J11), I2C1_SCL/CSI_I2C_SCL/J35[20]/J23[6]/U13[17]/U32[4] */
#define BOARD_INITPINS_CSI_I2C_SCL_PERIPHERAL                            FLEXIO3   /*!< Device name: FLEXIO3 */
#define BOARD_INITPINS_CSI_I2C_SCL_SIGNAL                                     IO   /*!< FLEXIO3 signal: IO */
#define BOARD_INITPINS_CSI_I2C_SCL_CHANNEL                                    0U   /*!< FLEXIO3 IO channel: 00 */


/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitPins(void);

/* GPIO_AD_B0_12 (coord K14), UART1_TXD */
#define BOARD_INITDEBUG_UART_UART1_TXD_PERIPHERAL                        LPUART1   /*!< Device name: LPUART1 */
#define BOARD_INITDEBUG_UART_UART1_TXD_SIGNAL                                 TX   /*!< LPUART1 signal: TX */

/* GPIO_AD_B0_13 (coord L14), UART1_RXD */
#define BOARD_INITDEBUG_UART_UART1_RXD_PERIPHERAL                        LPUART1   /*!< Device name: LPUART1 */
#define BOARD_INITDEBUG_UART_UART1_RXD_SIGNAL                                 RX   /*!< LPUART1 signal: RX */


/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitDEBUG_UART(void);

/* GPIO_EMC_09 (coord C2), SEMC_A0 */
#define BOARD_INITSDRAM_SEMC_A0_PERIPHERAL                                  SEMC   /*!< Device name: SEMC */
#define BOARD_INITSDRAM_SEMC_A0_SIGNAL                                      ADDR   /*!< SEMC signal: ADDR */
#define BOARD_INITSDRAM_SEMC_A0_CHANNEL                                       0U   /*!< SEMC ADDR channel: 00 */

/* GPIO_EMC_10 (coord G1), SEMC_A1 */
#define BOARD_INITSDRAM_SEMC_A1_PERIPHERAL                                  SEMC   /*!< Device name: SEMC */
#define BOARD_INITSDRAM_SEMC_A1_SIGNAL                                      ADDR   /*!< SEMC signal: ADDR */
#define BOARD_INITSDRAM_SEMC_A1_CHANNEL                                       1U   /*!< SEMC ADDR channel: 01 */

/* GPIO_EMC_11 (coord G3), SEMC_A2 */
#define BOARD_INITSDRAM_SEMC_A2_PERIPHERAL                                  SEMC   /*!< Device name: SEMC */
#define BOARD_INITSDRAM_SEMC_A2_SIGNAL                                      ADDR   /*!< SEMC signal: ADDR */
#define BOARD_INITSDRAM_SEMC_A2_CHANNEL                                       2U   /*!< SEMC ADDR channel: 02 */

/* GPIO_EMC_12 (coord H1), SEMC_A3 */
#define BOARD_INITSDRAM_SEMC_A3_PERIPHERAL                                  SEMC   /*!< Device name: SEMC */
#define BOARD_INITSDRAM_SEMC_A3_SIGNAL                                      ADDR   /*!< SEMC signal: ADDR */
#define BOARD_INITSDRAM_SEMC_A3_CHANNEL                                       3U   /*!< SEMC ADDR channel: 03 */

/* GPIO_EMC_13 (coord A6), SEMC_A4 */
#define BOARD_INITSDRAM_SEMC_A4_PERIPHERAL                                  SEMC   /*!< Device name: SEMC */
#define BOARD_INITSDRAM_SEMC_A4_SIGNAL                                      ADDR   /*!< SEMC signal: ADDR */
#define BOARD_INITSDRAM_SEMC_A4_CHANNEL                                       4U   /*!< SEMC ADDR channel: 04 */

/* GPIO_EMC_14 (coord B6), SEMC_A5 */
#define BOARD_INITSDRAM_SEMC_A5_PERIPHERAL                                  SEMC   /*!< Device name: SEMC */
#define BOARD_INITSDRAM_SEMC_A5_SIGNAL                                      ADDR   /*!< SEMC signal: ADDR */
#define BOARD_INITSDRAM_SEMC_A5_CHANNEL                                       5U   /*!< SEMC ADDR channel: 05 */

/* GPIO_EMC_15 (coord B1), SEMC_A6 */
#define BOARD_INITSDRAM_SEMC_A6_PERIPHERAL                                  SEMC   /*!< Device name: SEMC */
#define BOARD_INITSDRAM_SEMC_A6_SIGNAL                                      ADDR   /*!< SEMC signal: ADDR */
#define BOARD_INITSDRAM_SEMC_A6_CHANNEL                                       6U   /*!< SEMC ADDR channel: 06 */

/* GPIO_EMC_16 (coord A5), SEMC_A7 */
#define BOARD_INITSDRAM_SEMC_A7_PERIPHERAL                                  SEMC   /*!< Device name: SEMC */
#define BOARD_INITSDRAM_SEMC_A7_SIGNAL                                      ADDR   /*!< SEMC signal: ADDR */
#define BOARD_INITSDRAM_SEMC_A7_CHANNEL                                       7U   /*!< SEMC ADDR channel: 07 */

/* GPIO_EMC_17 (coord A4), SEMC_A8 */
#define BOARD_INITSDRAM_SEMC_A8_PERIPHERAL                                  SEMC   /*!< Device name: SEMC */
#define BOARD_INITSDRAM_SEMC_A8_SIGNAL                                      ADDR   /*!< SEMC signal: ADDR */
#define BOARD_INITSDRAM_SEMC_A8_CHANNEL                                       8U   /*!< SEMC ADDR channel: 08 */

/* GPIO_EMC_18 (coord B2), SEMC_A9 */
#define BOARD_INITSDRAM_SEMC_A9_PERIPHERAL                                  SEMC   /*!< Device name: SEMC */
#define BOARD_INITSDRAM_SEMC_A9_SIGNAL                                      ADDR   /*!< SEMC signal: ADDR */
#define BOARD_INITSDRAM_SEMC_A9_CHANNEL                                       9U   /*!< SEMC ADDR channel: 09 */

/* GPIO_EMC_23 (coord G2), SEMC_A10 */
#define BOARD_INITSDRAM_SEMC_A10_PERIPHERAL                                 SEMC   /*!< Device name: SEMC */
#define BOARD_INITSDRAM_SEMC_A10_SIGNAL                                     ADDR   /*!< SEMC signal: ADDR */
#define BOARD_INITSDRAM_SEMC_A10_CHANNEL                                     10U   /*!< SEMC ADDR channel: 10 */

/* GPIO_EMC_19 (coord B4), SEMC_A11 */
#define BOARD_INITSDRAM_SEMC_A11_PERIPHERAL                                 SEMC   /*!< Device name: SEMC */
#define BOARD_INITSDRAM_SEMC_A11_SIGNAL                                     ADDR   /*!< SEMC signal: ADDR */
#define BOARD_INITSDRAM_SEMC_A11_CHANNEL                                     11U   /*!< SEMC ADDR channel: 11 */

/* GPIO_EMC_20 (coord A3), SEMC_A12 */
#define BOARD_INITSDRAM_SEMC_A12_PERIPHERAL                                 SEMC   /*!< Device name: SEMC */
#define BOARD_INITSDRAM_SEMC_A12_SIGNAL                                     ADDR   /*!< SEMC signal: ADDR */
#define BOARD_INITSDRAM_SEMC_A12_CHANNEL                                     12U   /*!< SEMC ADDR channel: 12 */

/* GPIO_EMC_21 (coord C1), SEMC_BA0 */
#define BOARD_INITSDRAM_SEMC_BA0_PERIPHERAL                                 SEMC   /*!< Device name: SEMC */
#define BOARD_INITSDRAM_SEMC_BA0_SIGNAL                                       BA   /*!< SEMC signal: BA */
#define BOARD_INITSDRAM_SEMC_BA0_CHANNEL                                      0U   /*!< SEMC BA channel: 0 */

/* GPIO_EMC_22 (coord F1), SEMC_BA1 */
#define BOARD_INITSDRAM_SEMC_BA1_PERIPHERAL                                 SEMC   /*!< Device name: SEMC */
#define BOARD_INITSDRAM_SEMC_BA1_SIGNAL                                       BA   /*!< SEMC signal: BA */
#define BOARD_INITSDRAM_SEMC_BA1_CHANNEL                                      1U   /*!< SEMC BA channel: 1 */

/* GPIO_EMC_24 (coord D3), SEMC_CAS */
#define BOARD_INITSDRAM_SEMC_CAS_PERIPHERAL                                 SEMC   /*!< Device name: SEMC */
#define BOARD_INITSDRAM_SEMC_CAS_SIGNAL                                 semc_cas   /*!< SEMC signal: semc_cas */

/* GPIO_EMC_27 (coord A2), SEMC_CKE */
#define BOARD_INITSDRAM_SEMC_CKE_PERIPHERAL                                 SEMC   /*!< Device name: SEMC */
#define BOARD_INITSDRAM_SEMC_CKE_SIGNAL                                 semc_cke   /*!< SEMC signal: semc_cke */

/* GPIO_EMC_26 (coord B3), SEMC_CLK */
#define BOARD_INITSDRAM_SEMC_CLK_PERIPHERAL                                 SEMC   /*!< Device name: SEMC */
#define BOARD_INITSDRAM_SEMC_CLK_SIGNAL                                 semc_clk   /*!< SEMC signal: semc_clk */

/* GPIO_EMC_00 (coord E3), SEMC_D0 */
#define BOARD_INITSDRAM_SEMC_D0_PERIPHERAL                                  SEMC   /*!< Device name: SEMC */
#define BOARD_INITSDRAM_SEMC_D0_SIGNAL                                      DATA   /*!< SEMC signal: DATA */
#define BOARD_INITSDRAM_SEMC_D0_CHANNEL                                       0U   /*!< SEMC DATA channel: 00 */

/* GPIO_EMC_01 (coord F3), SEMC_D1 */
#define BOARD_INITSDRAM_SEMC_D1_PERIPHERAL                                  SEMC   /*!< Device name: SEMC */
#define BOARD_INITSDRAM_SEMC_D1_SIGNAL                                      DATA   /*!< SEMC signal: DATA */
#define BOARD_INITSDRAM_SEMC_D1_CHANNEL                                       1U   /*!< SEMC DATA channel: 01 */

/* GPIO_EMC_02 (coord F4), SEMC_D2 */
#define BOARD_INITSDRAM_SEMC_D2_PERIPHERAL                                  SEMC   /*!< Device name: SEMC */
#define BOARD_INITSDRAM_SEMC_D2_SIGNAL                                      DATA   /*!< SEMC signal: DATA */
#define BOARD_INITSDRAM_SEMC_D2_CHANNEL                                       2U   /*!< SEMC DATA channel: 02 */

/* GPIO_EMC_03 (coord G4), SEMC_D3 */
#define BOARD_INITSDRAM_SEMC_D3_PERIPHERAL                                  SEMC   /*!< Device name: SEMC */
#define BOARD_INITSDRAM_SEMC_D3_SIGNAL                                      DATA   /*!< SEMC signal: DATA */
#define BOARD_INITSDRAM_SEMC_D3_CHANNEL                                       3U   /*!< SEMC DATA channel: 03 */

/* GPIO_EMC_04 (coord F2), SEMC_D4 */
#define BOARD_INITSDRAM_SEMC_D4_PERIPHERAL                                  SEMC   /*!< Device name: SEMC */
#define BOARD_INITSDRAM_SEMC_D4_SIGNAL                                      DATA   /*!< SEMC signal: DATA */
#define BOARD_INITSDRAM_SEMC_D4_CHANNEL                                       4U   /*!< SEMC DATA channel: 04 */

/* GPIO_EMC_05 (coord G5), SEMC_D5 */
#define BOARD_INITSDRAM_SEMC_D5_PERIPHERAL                                  SEMC   /*!< Device name: SEMC */
#define BOARD_INITSDRAM_SEMC_D5_SIGNAL                                      DATA   /*!< SEMC signal: DATA */
#define BOARD_INITSDRAM_SEMC_D5_CHANNEL                                       5U   /*!< SEMC DATA channel: 05 */

/* GPIO_EMC_06 (coord H5), SEMC_D6 */
#define BOARD_INITSDRAM_SEMC_D6_PERIPHERAL                                  SEMC   /*!< Device name: SEMC */
#define BOARD_INITSDRAM_SEMC_D6_SIGNAL                                      DATA   /*!< SEMC signal: DATA */
#define BOARD_INITSDRAM_SEMC_D6_CHANNEL                                       6U   /*!< SEMC DATA channel: 06 */

/* GPIO_EMC_07 (coord H4), SEMC_D7 */
#define BOARD_INITSDRAM_SEMC_D7_PERIPHERAL                                  SEMC   /*!< Device name: SEMC */
#define BOARD_INITSDRAM_SEMC_D7_SIGNAL                                      DATA   /*!< SEMC signal: DATA */
#define BOARD_INITSDRAM_SEMC_D7_CHANNEL                                       7U   /*!< SEMC DATA channel: 07 */

/* GPIO_EMC_30 (coord C6), SEMC_D8 */
#define BOARD_INITSDRAM_SEMC_D8_PERIPHERAL                                  SEMC   /*!< Device name: SEMC */
#define BOARD_INITSDRAM_SEMC_D8_SIGNAL                                      DATA   /*!< SEMC signal: DATA */
#define BOARD_INITSDRAM_SEMC_D8_CHANNEL                                       8U   /*!< SEMC DATA channel: 08 */

/* GPIO_EMC_31 (coord C5), SEMC_D9 */
#define BOARD_INITSDRAM_SEMC_D9_PERIPHERAL                                  SEMC   /*!< Device name: SEMC */
#define BOARD_INITSDRAM_SEMC_D9_SIGNAL                                      DATA   /*!< SEMC signal: DATA */
#define BOARD_INITSDRAM_SEMC_D9_CHANNEL                                       9U   /*!< SEMC DATA channel: 09 */

/* GPIO_EMC_32 (coord D5), SEMC_D10 */
#define BOARD_INITSDRAM_SEMC_D10_PERIPHERAL                                 SEMC   /*!< Device name: SEMC */
#define BOARD_INITSDRAM_SEMC_D10_SIGNAL                                     DATA   /*!< SEMC signal: DATA */
#define BOARD_INITSDRAM_SEMC_D10_CHANNEL                                     10U   /*!< SEMC DATA channel: 10 */

/* GPIO_EMC_33 (coord C4), SEMC_D11 */
#define BOARD_INITSDRAM_SEMC_D11_PERIPHERAL                                 SEMC   /*!< Device name: SEMC */
#define BOARD_INITSDRAM_SEMC_D11_SIGNAL                                     DATA   /*!< SEMC signal: DATA */
#define BOARD_INITSDRAM_SEMC_D11_CHANNEL                                     11U   /*!< SEMC DATA channel: 11 */

/* GPIO_EMC_34 (coord D4), SEMC_D12 */
#define BOARD_INITSDRAM_SEMC_D12_PERIPHERAL                                 SEMC   /*!< Device name: SEMC */
#define BOARD_INITSDRAM_SEMC_D12_SIGNAL                                     DATA   /*!< SEMC signal: DATA */
#define BOARD_INITSDRAM_SEMC_D12_CHANNEL                                     12U   /*!< SEMC DATA channel: 12 */

/* GPIO_EMC_35 (coord E5), SEMC_D13 */
#define BOARD_INITSDRAM_SEMC_D13_PERIPHERAL                                 SEMC   /*!< Device name: SEMC */
#define BOARD_INITSDRAM_SEMC_D13_SIGNAL                                     DATA   /*!< SEMC signal: DATA */
#define BOARD_INITSDRAM_SEMC_D13_CHANNEL                                     13U   /*!< SEMC DATA channel: 13 */

/* GPIO_EMC_36 (coord C3), SEMC_D14 */
#define BOARD_INITSDRAM_SEMC_D14_PERIPHERAL                                 SEMC   /*!< Device name: SEMC */
#define BOARD_INITSDRAM_SEMC_D14_SIGNAL                                     DATA   /*!< SEMC signal: DATA */
#define BOARD_INITSDRAM_SEMC_D14_CHANNEL                                     14U   /*!< SEMC DATA channel: 14 */

/* GPIO_EMC_37 (coord E4), SEMC_D15 */
#define BOARD_INITSDRAM_SEMC_D15_PERIPHERAL                                 SEMC   /*!< Device name: SEMC */
#define BOARD_INITSDRAM_SEMC_D15_SIGNAL                                     DATA   /*!< SEMC signal: DATA */
#define BOARD_INITSDRAM_SEMC_D15_CHANNEL                                     15U   /*!< SEMC DATA channel: 15 */

/* GPIO_EMC_08 (coord H3), SEMC_DM0 */
#define BOARD_INITSDRAM_SEMC_DM0_PERIPHERAL                                 SEMC   /*!< Device name: SEMC */
#define BOARD_INITSDRAM_SEMC_DM0_SIGNAL                                       DM   /*!< SEMC signal: DM */
#define BOARD_INITSDRAM_SEMC_DM0_CHANNEL                                      0U   /*!< SEMC DM channel: 0 */

/* GPIO_EMC_38 (coord D6), SEMC_DM1 */
#define BOARD_INITSDRAM_SEMC_DM1_PERIPHERAL                                 SEMC   /*!< Device name: SEMC */
#define BOARD_INITSDRAM_SEMC_DM1_SIGNAL                                       DM   /*!< SEMC signal: DM */
#define BOARD_INITSDRAM_SEMC_DM1_CHANNEL                                      1U   /*!< SEMC DM channel: 1 */

/* GPIO_EMC_25 (coord D2), SEMC_RAS */
#define BOARD_INITSDRAM_SEMC_RAS_PERIPHERAL                                 SEMC   /*!< Device name: SEMC */
#define BOARD_INITSDRAM_SEMC_RAS_SIGNAL                                 semc_ras   /*!< SEMC signal: semc_ras */

/* GPIO_EMC_28 (coord D1), SEMC_WE */
#define BOARD_INITSDRAM_SEMC_WE_PERIPHERAL                                  SEMC   /*!< Device name: SEMC */
#define BOARD_INITSDRAM_SEMC_WE_SIGNAL                                   semc_we   /*!< SEMC signal: semc_we */

/* GPIO_EMC_39 (coord B7), SEMC_DQS */
#define BOARD_INITSDRAM_SEMC_DQS_PERIPHERAL                                 SEMC   /*!< Device name: SEMC */
#define BOARD_INITSDRAM_SEMC_DQS_SIGNAL                                 semc_dqs   /*!< SEMC signal: semc_dqs */

/* GPIO_EMC_29 (coord E1), SEMC_CS0 */
#define BOARD_INITSDRAM_SEMC_CS0_PERIPHERAL                                 SEMC   /*!< Device name: SEMC */
#define BOARD_INITSDRAM_SEMC_CS0_SIGNAL                                       CS   /*!< SEMC signal: CS */
#define BOARD_INITSDRAM_SEMC_CS0_CHANNEL                                      0U   /*!< SEMC CS channel: 0 */


/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitSDRAM(void);

/* GPIO_EMC_40 (coord A7), ENET_MDC */
#define BOARD_INITENET_ENET_MDC_PERIPHERAL                                  ENET   /*!< Device name: ENET */
#define BOARD_INITENET_ENET_MDC_SIGNAL                                  enet_mdc   /*!< ENET signal: enet_mdc */

/* GPIO_EMC_41 (coord C7), ENET_MDIO */
#define BOARD_INITENET_ENET_MDIO_PERIPHERAL                                 ENET   /*!< Device name: ENET */
#define BOARD_INITENET_ENET_MDIO_SIGNAL                                enet_mdio   /*!< ENET signal: enet_mdio */

/* GPIO_B1_10 (coord B13), ENET_TX_CLK */
#define BOARD_INITENET_ENET_TX_CLK_PERIPHERAL                               ENET   /*!< Device name: ENET */
#define BOARD_INITENET_ENET_TX_CLK_SIGNAL                           enet_ref_clk   /*!< ENET signal: enet_ref_clk */

/* GPIO_B1_04 (coord E12), ENET_RXD0 */
#define BOARD_INITENET_ENET_RXD0_PERIPHERAL                                 ENET   /*!< Device name: ENET */
#define BOARD_INITENET_ENET_RXD0_SIGNAL                             enet_rx_data   /*!< ENET signal: enet_rx_data */
#define BOARD_INITENET_ENET_RXD0_CHANNEL                                      0U   /*!< ENET enet_rx_data channel: 0 */

/* GPIO_B1_05 (coord D12), ENET_RXD1 */
#define BOARD_INITENET_ENET_RXD1_PERIPHERAL                                 ENET   /*!< Device name: ENET */
#define BOARD_INITENET_ENET_RXD1_SIGNAL                             enet_rx_data   /*!< ENET signal: enet_rx_data */
#define BOARD_INITENET_ENET_RXD1_CHANNEL                                      1U   /*!< ENET enet_rx_data channel: 1 */

/* GPIO_B1_06 (coord C12), ENET_CRS_DV */
#define BOARD_INITENET_ENET_CRS_DV_PERIPHERAL                               ENET   /*!< Device name: ENET */
#define BOARD_INITENET_ENET_CRS_DV_SIGNAL                             enet_rx_en   /*!< ENET signal: enet_rx_en */

/* GPIO_B1_11 (coord C13), ENET_RXER */
#define BOARD_INITENET_ENET_RXER_PERIPHERAL                                 ENET   /*!< Device name: ENET */
#define BOARD_INITENET_ENET_RXER_SIGNAL                               enet_rx_er   /*!< ENET signal: enet_rx_er */

/* GPIO_B1_07 (coord B12), ENET_TXD0 */
#define BOARD_INITENET_ENET_TXD0_PERIPHERAL                                 ENET   /*!< Device name: ENET */
#define BOARD_INITENET_ENET_TXD0_SIGNAL                             enet_tx_data   /*!< ENET signal: enet_tx_data */
#define BOARD_INITENET_ENET_TXD0_CHANNEL                                      0U   /*!< ENET enet_tx_data channel: 0 */

/* GPIO_B1_08 (coord A12), ENET_TXD1 */
#define BOARD_INITENET_ENET_TXD1_PERIPHERAL                                 ENET   /*!< Device name: ENET */
#define BOARD_INITENET_ENET_TXD1_SIGNAL                             enet_tx_data   /*!< ENET signal: enet_tx_data */
#define BOARD_INITENET_ENET_TXD1_CHANNEL                                      1U   /*!< ENET enet_tx_data channel: 1 */

/* GPIO_B1_09 (coord A13), ENET_TXEN */
#define BOARD_INITENET_ENET_TXEN_PERIPHERAL                                 ENET   /*!< Device name: ENET */
#define BOARD_INITENET_ENET_TXEN_SIGNAL                               enet_tx_en   /*!< ENET signal: enet_tx_en */


/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitENET(void);

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */
#endif /* _PIN_MUX_H_ */

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
