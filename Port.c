/******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.c
 *
 * Description: Source file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Moaaz Mansour
 ******************************************************************************/

#include "Port.h"
#include "tm4c123gh6pm_registers.h"
   
#if (PORT_DEV_ERROR_DETECT == STD_ON)

#include "Det.h"
/* AUTOSAR Version checking between Det and Dio Modules */
#if ((PORT_AR_MAJOR_VERSION != PORTAR_RELEASE_MAJOR_VERSION) || (PORT_AR_MINOR_VERSION != PORTAR_RELEASE_MINOR_VERSION) || (PORT_AR_PATCH_VERSION != PORTAR_RELEASE_PATCH_VERSION))
#error "The AR version of port.h does not match the expected version"
#endif

#endif

STATIC boolean Port_Status = PORT_NOT_INITIALIZED;

/************************************************************************************
* Service Name: Port_Init
* Sync/Async: Synchronous
* Reentrancy: Non Reentrant
* Parameters (in): ConfigPtr - Pointer to configuration set.
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Initializes the Port Driver module.
************************************************************************************/
void Port_Init(const Port_ConfigType *ConfigPtr)
{

#if (PORT_DEV_ERROR_DETECT == STD_ON)
    /* check if the input configuration pointer is not a NULL_PTR */
    if (NULL_PTR == ConfigPtr)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_INIT_SID,
                        PORT_E_PARAM_CONFIG);
    }
    else
#endif
        Port_Status = PORT_INITIALIZED;

    for (uint8 i = 0; i < PORTS_PINS; i++)
    {
        Port_SetupGpioPin(&ConfigPtr->pins[i]);
    }
}

/************************************************************************************
* Service Name: Port_SetPinDirection
* Sync/Async: Synchronous
* Reentrancy: Reentrant
* Parameters (in): Pin - Port Pin ID number.
                   Direction - Port Pin Direction
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Sets the port pin direction
************************************************************************************/
#if (PORT_SET_PIN_DIRECTION_API == STD_ON)
void Port_SetPinDirection(Port_PinType Pin, Port_PinDirectionType Direction)
{

    if (Port_Status == PORT_NOT_INITIALIZED)
    {

#if (PORT_DEV_ERROR_DETECT == STD_ON)
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
                        PORT_SETPINDIRECTION_SID, PORT_E_UNINIT);
#endif
    }
    else
    {
        if ((Pin < 0) || (Pin > PORTS_PINS))
        {

#if (PORT_DEV_ERROR_DETECT == STD_ON)
            Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
                            PORT_SETPINDIRECTION_SID, PORT_E_PARAM_PIN);
#endif
        }
        else
        {
            if (Port_Configuration.pins[Pin].Pin_mode_changeable == PIN_CHANGABLE)
            {
                volatile uint32 portAddress = PortAddressFromPinId(Pin);
                Pin = PinNumFromPinID(Pin);
                if (Direction == PORT_PIN_OUT)
                {
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)portAddress + PORT_DIR_REG_OFFSET), Pin); /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
                }
                else if (Direction == PORT_PIN_IN)
                {
                    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)portAddress + PORT_DIR_REG_OFFSET), Pin); /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
                }
            }
            else
            {
#if (PORT_DEV_ERROR_DETECT == STD_ON)
                Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
                                PORT_SETPINDIRECTION_SID, PORT_E_DIRECTION_UNCHANGEABLE);
#endif
            }
        }
    }
}
#else
#endif

/************************************************************************************
* Service Name: Port_RefreshPortDirection
* Sync/Async: Synchronous
* Reentrancy: Non Reentrant
* Parameters (in): None
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Refreshes port direction.
************************************************************************************/
void Port_RefreshPortDirection(void)
{
    if (Port_Status == PORT_NOT_INITIALIZED)
    {
#if (PORT_DEV_ERROR_DETECT == STD_ON)
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
                        PORT_REFRESHPORTDIRECTION_SID, PORT_E_UNINIT);
#endif
    }
    else
    {
        const volatile Port_PinConfigType *pin;
        for (uint8 i = 0; i < PORTS_PINS; i++)
        {
            pin = &(Port_Configuration.pins[i]);

            if (pin->Pin_direction_changeable != TRUE)
            {
                //const volatile Port_PinConfigType *pin = &Port_Config[Pin];
                volatile uint32 portAddress = PortAddressFromPinId(i);
                // volatile uint32 *portAddress = PortAddressFromPinId(pin);
                if (pin->direction == PORT_PIN_OUT)
                {
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)portAddress + PORT_DIR_REG_OFFSET), pin->pin_num); /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
                }
                else if (pin->direction == PORT_PIN_IN)
                {
                    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)portAddress + PORT_DIR_REG_OFFSET), pin->pin_num); /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
                }
            }
        }
    }
}

/************************************************************************************
* Service Name: Port_GetVersionInfo
* Sync/Async: Synchronous
* Reentrancy: Non Reentrant
* Parameters (in): None
* Parameters (inout): None
* Parameters (out): versioninfo - Pointer to where to store the version information of this module.
* Return value: None
* Description: Returns the version information of this module.
************************************************************************************/

#if (PORT_VERSION_INFO_API == STD_ON)
void PORT_GetVersionInfo(Std_VersionInfoType *versioninfo)
{
    if (Port_Status == PORT_NOT_INITIALIZED)
    {
#if (PORT_DEV_ERROR_DETECT == STD_ON)
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
                        PORT_GET_VERSION_INFO_SID, PORT_E_UNINIT);
#endif
    }
    else
    {
#if (PORT_DEV_ERROR_DETECT == STD_ON)
        /* Check if input pointer is not Null pointer */
        if (NULL_PTR == versioninfo)
        {
            /* Report to DET  */
            Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
                            PORT_GET_VERSION_INFO_SID, PORT_E_PARAM_POINTER);
        }
        else
#endif /* (PORT_DEV_ERROR_DETECT == STD_ON) */
        {
            /* Copy the vendor Id */
            versioninfo->vendorID = (uint16)PORT_VENDOR_ID;
            /* Copy the module Id */
            versioninfo->moduleID = (uint16)PORT_MODULE_ID;
            /* Copy Software Major Version */
            versioninfo->sw_major_version = (uint8)PORT_SW_MAJOR_VERSION;
            /* Copy Software Minor Version */
            versioninfo->sw_minor_version = (uint8)PORT_SW_MINOR_VERSION;
            /* Copy Software Patch Version */
            versioninfo->sw_patch_version = (uint8)PORT_SW_PATCH_VERSION;
        }
    }
}
#endif

/************************************************************************************
* Service Name: Port_SetPinMode
* Sync/Async: Synchronous
* Reentrancy: Reentrant
* Parameters (in): Pin - Port Pin ID number.
*                  Mode - New Port Pin mode to be set on port pin.
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Sets the port pin mode.
************************************************************************************/
#if (PORT_SET_PIN_MODE_API == STD_ON)

void Port_SetPinMode(Port_PinType Pin, Port_PinModeType Mode)
{ /* Check if the Driver is initialized before using this function */
    if (Port_Status == PORT_NOT_INITIALIZED)
    {
#if (PORT_DEV_ERROR_DETECT == STD_ON)
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
                        PORT_SETPINMODE_SID, PORT_E_UNINIT);
#endif
    }
    else
    {

        if ((Pin < 0) || (Pin > PORTS_PINS))
        {
#if (PORT_DEV_ERROR_DETECT == STD_ON)
            Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
                            PORT_SETPINMODE_SID, PORT_E_PARAM_PIN);
#endif
        }
        else
        {
            if (Mode < 0 || Mode > 12)
            {
#if (PORT_DEV_ERROR_DETECT == STD_ON)
                Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
                                PORT_SETPINMODE_SID, PORT_E_PARAM_INVALID_MODE);
#endif
            }
            else
            {
                const volatile Port_PinConfigType *pincfg = &(Port_Configuration.pins[Pin]);
                // volatile uint32 *portAddress = PortAddressFromPinId(pin);
                if (pincfg->Pin_mode_changeable == TRUE)
                {
                    volatile uint32 portAddress = PortAddressFromPinId(Pin);

                    if (Mode == PORT_PIN_MODE_DIO_GPT || PORT_PIN_MODE_DIO_WDG)
                    {
                        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)portAddress + PORT_ALT_FUNC_REG_OFFSET), pincfg->pin_num); /* Set the corresponding bit in Alternate function register to allow changes on this pin */
                    }
                    else
                    {
                        SET_BIT(*(volatile uint32 *)((volatile uint8 *)portAddress + PORT_ALT_FUNC_REG_OFFSET), pincfg->pin_num); /* Set the corresponding bit in Alternate function register to allow changes on this pin */
                    }

                    if (Mode == PORT_PIN_MODE_ADC)
                    {
                        SET_BIT(*(volatile uint32 *)((volatile uint8 *)portAddress + PORT_ANALOG_MODE_SEL_REG_OFFSET), pincfg->pin_num); /* Set the corresponding bit in Alternate function register to allow changes on this pin */
                    }
                    else
                    {
                        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)portAddress + PORT_ANALOG_MODE_SEL_REG_OFFSET), pincfg->pin_num); /* Set the corresponding bit in Alternate function register to allow changes on this pin */
                        SET_BIT(*(volatile uint32 *)((volatile uint8 *)portAddress + PORT_DIGITAL_ENABLE_REG_OFFSET), pincfg->pin_num);    /* Set the corresponding bit in Alternate function register to allow changes on this pin */
                    }
                    *(volatile uint32 *)((volatile uint8 *)portAddress + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (pincfg->pin_num * 4)); /* Set the corresponding bit in Alternate function register to allow changes on this pin */
                    *(volatile uint32 *)((volatile uint8 *)portAddress + PORT_CTL_REG_OFFSET) |= Mode << (pincfg->pin_num * 4);          /* Set the corresponding bit in Alternate function register to allow changes on this pin */
                }
                else
                {

#if (PORT_DEV_ERROR_DETECT == STD_ON)
                    Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
                                    PORT_SETPINMODE_SID, PORT_E_MODE_UNCHANGEABLE);
#endif
                }
            }
        }
    }
}
#endif



/************************************************************************************
* Service Name: Port_SetupGpioPin
* Sync/Async: Synchronous
* Reentrancy: reentrant
* Parameters (in): ConfigPtr - Pointer to post-build configuration data
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Function to Setup the pin configuration:
*              - Setup the pin as Digital GPIO pin
*              - Setup the direction of the GPIO pin
*              - Provide initial value for o/p pin
*              - Setup the internal resistor for i/p pin
************************************************************************************/

STATIC void Port_SetupGpioPin(const Port_PinConfigType *ConfigPtr)
{
    volatile uint32 *PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */
    volatile uint32 delay = 0;

    switch (ConfigPtr->port_num)
    {
    case 0:
        PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
        break;
    case 1:
        PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
        break;
    case 2:
        PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
        break;
    case 3:
        PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
        break;
    case 4:
        PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
        break;
    case 5:
        PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
        break;
    }

    /* Enable clock for PORT and allow time for clock to start*/
    SYSCTL_REGCGC2_REG |= (1 << ConfigPtr->port_num);
    delay = SYSCTL_REGCGC2_REG;

    if (((ConfigPtr->port_num == 3) && (ConfigPtr->pin_num == 7)) || ((ConfigPtr->port_num == 5) && (ConfigPtr->pin_num == 0))) /* PD7 or PF0 */
    {
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_LOCK_REG_OFFSET) = 0x4C4F434B;                   /* Unlock the GPIOCR register */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_COMMIT_REG_OFFSET), ConfigPtr->pin_num); /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
    }
    else if ((ConfigPtr->port_num == 2) && (ConfigPtr->pin_num <= 3)) /* PC0 to PC3 */
    {
        /* Do Nothing ...  this is the JTAG pins */
        return;
    }
    else
    {
        /* Do Nothing ... No need to unlock the commit register for this pin */
    }

    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET), ConfigPtr->pin_num);   /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET), ConfigPtr->pin_num);          /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (ConfigPtr->pin_num * 4)); /* Clear the PMCx bits for this pin */

    if (ConfigPtr->direction == PORT_PIN_OUT)
    {
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET), ConfigPtr->pin_num); /* Set the corresponding bit in the GPIODIR register to configure it as output pin */

        if (ConfigPtr->initial_value == STD_HIGH)
        {
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET), ConfigPtr->pin_num); /* Set the corresponding bit in the GPIODATA register to provide initial value 1 */
        }
        else
        {
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET), ConfigPtr->pin_num); /* Clear the corresponding bit in the GPIODATA register to provide initial value 0 */
        }
    }
    else if (ConfigPtr->direction == PORT_PIN_IN)
    {
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET), ConfigPtr->pin_num); /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */

        if (ConfigPtr->resistor == PULL_UP)
        {
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET), ConfigPtr->pin_num); /* Set the corresponding bit in the GPIOPUR register to enable the internal pull up pin */
        }
        else if (ConfigPtr->resistor == PULL_DOWN)
        {
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET), ConfigPtr->pin_num); /* Set the corresponding bit in the GPIOPDR register to enable the internal pull down pin */
        }
        else
        {
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET), ConfigPtr->pin_num);   /* Clear the corresponding bit in the GPIOPUR register to disable the internal pull up pin */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET), ConfigPtr->pin_num); /* Clear the corresponding bit in the GPIOPDR register to disable the internal pull down pin */
        }
    }
    else
    {
        /* Do Nothing */
    }

    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET), ConfigPtr->pin_num); /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
}
