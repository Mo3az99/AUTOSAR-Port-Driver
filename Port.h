/******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.h
 *
 * Description: Header file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Moaaz Mansour
 ******************************************************************************/

#ifndef PORT_H
#define PORT_H

/* Id for the company in the AUTOSAR
 * for example Mohamed Tarek's ID = 1000 :) */
#define PORT_VENDOR_ID (1000U)

/* PORT Module Id */
#define PORT_MODULE_ID (120U)

/* PORT Instance Id */
#define PORT_INSTANCE_ID (0U)

/*
 * Module Version 1.0.0
 */
#define PORT_SW_MAJOR_VERSION (1U)
#define PORT_SW_MINOR_VERSION (0U)
#define PORT_SW_PATCH_VERSION (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_AR_RELEASE_MAJOR_VERSION (4U)
#define PORT_AR_RELEASE_MINOR_VERSION (0U)
#define PORT_AR_RELEASE_PATCH_VERSION (3U)

/* Pre-compile option for Version Info API */
#define PORT_GET_VERSION_INFO_SID (STD_ON)

/* Pre-compile option for Set Pin Direction API*/
#define PORT_SET_PIN_DIRECTION_API (STD_ON)

/* Pre-compile option for Set Pin Direction API*/
#define PORT_SET_PIN_MODE_API (STD_ON)
/*
 * Macros for PORT Status
 */
#define PORT_INITIALIZED (1U)
#define PORT_NOT_INITIALIZED (0U)

/*
 * Macros for PIN CHANGABLE Status
 */
#define PIN_CHANGABLE (1U)
#define PIN_UNCHANGABLE (0U)

/* Standard AUTOSAR types */
#include "Std_Types.h"

/* AUTOSAR checking between Std Types and PORT Modules */
#if ((STD_TYPES_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION) || (STD_TYPES_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION) || (STD_TYPES_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
#error "The AR version of Std_Types.h does not match the expected version"
#endif

/* PORT Pre-Compile Configuration Header file */
#include "PORT_Cfg.h"

/* AUTOSAR Version checking between PORT_Cfg.h and PORT.h files */
#if ((PORT_CFG_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION) || (PORT_CFG_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION) || (PORT_CFG_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
#error "The AR version of PORT_Cfg.h does not match the expected version"
#endif

/* Software Version checking between PORT_Cfg.h and PORT.h files */
#if ((PORT_CFG_SW_MAJOR_VERSION != PORT_SW_MAJOR_VERSION) || (PORT_CFG_SW_MINOR_VERSION != PORT_SW_MINOR_VERSION) || (PORT_CFG_SW_PATCH_VERSION != PORT_SW_PATCH_VERSION))
#error "The SW version of PORT_Cfg.h does not match the expected version"
#endif

/* Non AUTOSAR files */
#include "Common_Macros.h"
#include "Port_Helper_Functions.h"
#include "Det.h"

/******************************************************************************
 *                      API Service Id Macros                                 *
 ******************************************************************************/

/* Service ID for Port Init */
#define PORT_INIT_SID (uint8)0x00

/* Service ID for Port SetPinDirection */
#define PORT_SETPINDIRECTION_SID (uint8)0x01

/* Service ID for Port RefreshPortDirection */
#define PORT_REFRESHPORTDIRECTION_SID (uint8)0x02

/* Service ID for Port GetVersionInfo */
#define PORT_GETVERSIONINFO_SID (uint8)0x03

/* Service ID for Port SetPinMode */
#define PORT_SETPINMODE_SID (uint8)0x04

/*******************************************************************************
 *                      DET Error Codes                                        *
 *******************************************************************************/
/* DET code to report Invalid Port Pin ID requested */
#define PORT_E_PARAM_PIN (uint8)0x0A

/* DET code to report Port Pin not configured as changeable */
#define PORT_E_DIRECTION_UNCHANGEABLE (uint8)0x0B

/* DET code to report API Port_Init service called with wrong parameter. */
#define PORT_E_PARAM_CONFIG (uint8)0x0C

/* DET code to report API Port_SetPinMode service called when mode is unchangeable. */
#define PORT_E_PARAM_INVALID_MODE (uint8)0x0D

/* DET code to report API Port_SetPinMode service called when mode is unchangeable. */
#define PORT_E_MODE_UNCHANGEABLE (uint8)0x0E

/* DET code to report API service called without module initializationp */
#define PORT_E_UNINIT (uint8)0x0F

/* DET code to report APIs called with a Null Pointer */
#define PORT_E_PARAM_POINTER (uint8)0x10

/*******************************************************************************
 *                              Module Definitions                             *
 *******************************************************************************/

/* GPIO Registers base addresses */
#define GPIO_PORTA_BASE_ADDRESS 0x40004000
#define GPIO_PORTB_BASE_ADDRESS 0x40005000
#define GPIO_PORTC_BASE_ADDRESS 0x40006000
#define GPIO_PORTD_BASE_ADDRESS 0x40007000
#define GPIO_PORTE_BASE_ADDRESS 0x40024000
#define GPIO_PORTF_BASE_ADDRESS 0x40025000

/* GPIO Registers offset addresses */
#define PORT_DATA_REG_OFFSET 0x3FC
#define PORT_DIR_REG_OFFSET 0x400
#define PORT_ALT_FUNC_REG_OFFSET 0x420
#define PORT_PULL_UP_REG_OFFSET 0x510
#define PORT_PULL_DOWN_REG_OFFSET 0x514
#define PORT_DIGITAL_ENABLE_REG_OFFSET 0x51C
#define PORT_LOCK_REG_OFFSET 0x520
#define PORT_COMMIT_REG_OFFSET 0x524
#define PORT_ANALOG_MODE_SEL_REG_OFFSET 0x528
#define PORT_CTL_REG_OFFSET 0x52C

#define PORTS_PINS 43

/*******************************************************************************
 *                              Module Data Types                              *
 *******************************************************************************/

/* Description: Enum to hold PIN mode */

typedef enum
{
    PORT_PIN_MODE_ADC,
    PORT_PIN_MODE_CAN,
    PORT_PIN_MODE_DIO,
    PORT_PIN_MODE_DIO_GPT,
    PORT_PIN_MODE_DIO_WDG,
    PORT_PIN_MODE_GPT,
    PORT_PIN_MODE_WDG,
    PORT_PIN_MODE_FLEXRAY,
    PORT_PIN_MODE_ICU,
    PORT_PIN_MODE_LIN,
    PORT_PIN_MODE_MEM,
    PORT_PIN_MODE_PWM,
    PORT_PIN_MODE_SPI

} Port_Pin_mode;
typedef uint16 Port_PinModeType;

/* Description: Enum to hold PIN Direction */

typedef enum
{
    PORT_PIN_IN,
    PORT_PIN_OUT
} Port_PinDirectionType;

/* Description: Enum to hold PIN level */

typedef enum
{
    PORT_PIN_LEVEL_LOW,
    PORT_PIN_LEVEL_HIGH

} Port_Pin_level_Init_Value;

/* Description: Enum to hold internal resistor type for PIN */
typedef enum
{
    OFF,
    PULL_UP,
    PULL_DOWN
} Port_InternalResistor;

/* Description: Structure to configure each individual PIN:
 *	1. the pin mode of the pin eg(adc,spi,...)
 *	2. the direction of pin --> INPUT or OUTPUT
 *  3. the initial value of pin --> high or low
 *  4. the changeability of the pin during runtime  
 */
typedef struct
{
    uint8 port_num;
    uint8 pin_num;
    Port_PinDirectionType direction;
    Port_Pin_mode pin_mode;
    Port_Pin_level_Init_Value initial_value;
    boolean Pin_direction_changeable;
    boolean Pin_mode_changeable;
    Port_InternalResistor resistor;

    // Port_PinDirection direction;
    // uint8 initial_value;

    // Port_PinModeType pin_mode;
} Port_PinConfigType;

typedef struct
{
    Port_PinConfigType pins[PORTS_PINS];
} Port_ConfigType;

//typedef uint8 Port_PinType;

// global variable to the Port_Configuration
extern const Port_ConfigType Port_Configuration;

typedef uint8 Port_PinType;

/*******************************************************************************
 *                      Function Prototypes                                    *
 *******************************************************************************/

/************************************************************************************
* Description: Function to Setup the pin configuration:
*              - Setup the pin as Digital GPIO pin
*              - Setup the direction of the GPIO pin
*              - Setup the internal resistor for i/p pin
************************************************************************************/

// Description: Initializes the Port Driver module.
void Port_Init(const Port_ConfigType *ConfigPtr);

/* Description: Sets the port pin direction */
void Port_SetPinDirection(Port_PinType Pin, Port_PinDirectionType Direction);

// Description: Refreshes port direction.
void Port_RefreshPortDirection(void);

// Description: Returns the version information of this module.
void Port_GetVersionInfo(Std_VersionInfoType *versioninfo);

// Description: Sets the port pin mode.
void Port_SetPinMode(Port_PinType Pin, Port_PinModeType Mode);


STATIC void Port_SetupGpioPin(const Port_PinConfigType *ConfigPtr);


#endif /* PORT_H */
