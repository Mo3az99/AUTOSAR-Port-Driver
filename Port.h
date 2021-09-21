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

#include "Common_Macros.h"
#include "Std_Types.h"
#include "Det.h"

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
    Port_Pin_Mode_DIO,
    Port_Pin_Mode_ADC,
    Port_Pin_Mode_CAN,
    Port_Pin_Mode_SPI,
    Port_Pin_Mode_PWM,
    Port_Pin_Mode_LIN,
    Port_Pin_Mode_WDG,
    Port_Pin_Mode_DIO_GPT,
    Port_Pin_Mode_DIO_WDG,
    Port_Pin_Mode_ICU,
    Port_Pin_Mode_MUM,
    Port_Pin_Mode_XRAY

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
void Port_SetupGpioPin(const Port_PinConfigType *ConfigPtr);
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

extern const Port_ConfigType Port_Configuration;


#endif /* PORT_H */
