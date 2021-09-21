/******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port_Cfg.h
 *
 * Description: Pre-Compile Configuration Header file for TM4C123GH6PM Microcontroller - Port Driver
 *
 * Author: Moaaz Mansour
 ******************************************************************************/

#ifndef PORT_CFG_H
#define PORT_CFG_H

/*
 * Module Version 1.0.0
 */
#define PORT_CFG_SW_MAJOR_VERSION (1U)
#define PORT_CFG_SW_MINOR_VERSION (0U)
#define PORT_CFG_SW_PATCH_VERSION (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_CFG_AR_RELEASE_MAJOR_VERSION (4U)
#define PORT_CFG_AR_RELEASE_MINOR_VERSION (0U)
#define PORT_CFG_AR_RELEASE_PATCH_VERSION (3U)

/* Default configuration for Port_PinConfigType*/
#define Port_PinDefaultCfg(port_num, pin_num) port_num, pin_num, PORT_PIN_OUT, PORT_PIN_MODE_DIO, PORT_PIN_LEVEL_LOW, FALSE, FALSE, OFF

/* Specific Configurtaions */
#define Port_PinCfg1(port_num, pin_num) port_num, pin_num, PORT_PIN_OUT, PORT_PIN_MODE_DIO, PORT_PIN_LEVEL_HIGH, FALSE, FALSE, OFF
#define Port_PinCfg2(port_num, pin_num) port_num, pin_num, PORT_PIN_IN, PORT_PIN_MODE_DIO, PORT_PIN_LEVEL_LOW, FALSE, FALSE, PULL_UP

/* Combined configuration used with Port_Init API */
#define Port_A_Cfg Port_PinDefaultCfg(0, 0), Port_PinDefaultCfg(0, 1), Port_PinDefaultCfg(0, 2), Port_PinDefaultCfg(0, 3), Port_PinDefaultCfg(0, 4), Port_PinDefaultCfg(0, 5), Port_PinDefaultCfg(0, 6), Port_PinDefaultCfg(0, 7)
#define Port_B_Cfg Port_PinDefaultCfg(1, 8), Port_PinDefaultCfg(1, 9), Port_PinDefaultCfg(1, 10), Port_PinDefaultCfg(1, 11), Port_PinDefaultCfg(1, 12), Port_PinDefaultCfg(1, 13), Port_PinDefaultCfg(1, 14), Port_PinDefaultCfg(1, 15)
#define Port_C_Cfg Port_PinDefaultCfg(2, 16), Port_PinDefaultCfg(2, 17), Port_PinDefaultCfg(2, 18), Port_PinDefaultCfg(2, 19), Port_PinDefaultCfg(2, 20), Port_PinDefaultCfg(2, 21), Port_PinDefaultCfg(2, 22), Port_PinDefaultCfg(2, 23)
#define Port_D_Cfg Port_PinDefaultCfg(3, 24), Port_PinDefaultCfg(3, 25), Port_PinDefaultCfg(3, 26), Port_PinDefaultCfg(3, 27), Port_PinDefaultCfg(3, 28), Port_PinDefaultCfg(3, 29), Port_PinDefaultCfg(3, 30), Port_PinDefaultCfg(3, 31)
#define Port_E_Cfg Port_PinDefaultCfg(4, 32), Port_PinDefaultCfg(4, 33), Port_PinDefaultCfg(4, 34), Port_PinDefaultCfg(4, 35), Port_PinDefaultCfg(4, 36), Port_PinDefaultCfg(4, 37)
#define Port_F_Cfg Port_PinDefaultCfg(5, 38), Port_PinCfg1(5, 39), Port_PinDefaultCfg(5, 40), Port_PinDefaultCfg(5, 41), Port_PinCfg2(5, 42)

// /* Pre-compile option for Development Error Detect */
// #define PORT_DEV_ERROR_DETECT (STD_ON)

// /* Pre-compile option for Version Info API */
// #define PORT_VERSION_INFO_API (STD_OFF)

// /* Number of the configured Dio Channels */
// #define PORT_CONFIGURED_CHANNLES (2U)
#endif /* DIO_CFG_H */
