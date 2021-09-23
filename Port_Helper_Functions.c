/*****************************  HELPER FUNCTIONS  *******************************/

#include "Port.h"
#include "tm4c123gh6pm_registers.h"

STATIC uint32 PortAddressFromPinId(Port_PinType pin);
STATIC Port_PinType PinNumFromPinID(Port_PinType pinId);


STATIC uint32 PortAddressFromPinId(Port_PinType pin)
{
    uint32 portAddress;
    if (pin < 8)
    {
        portAddress = GPIO_PORTA_BASE_ADDRESS;
    }
    else if (pin < 16)
    {
        portAddress = GPIO_PORTB_BASE_ADDRESS;
    }
    else if (pin < 24)
    {
        portAddress = GPIO_PORTC_BASE_ADDRESS;
    }
    else if (pin < 32)
    {
        portAddress = GPIO_PORTD_BASE_ADDRESS;
    }
    else if (pin < 38)
    {
        portAddress = GPIO_PORTE_BASE_ADDRESS;
    }
    else if (pin < 43)
    {
        portAddress = GPIO_PORTF_BASE_ADDRESS;
    }
    else
    {
        /* Do Nothing */
    }
    return portAddress;
}
STATIC Port_PinType PinNumFromPinID(Port_PinType pinId)
{
    Port_PinType pinNum;
    if (pinId < 32)
    {
        pinNum = pinId % 8;
    }
    else if (pinId < 38)
    {
        pinNum = (pinId - 32);
    }
    else if (pinId < 43)
    {
        pinNum = (pinId - 38);
    }
    else
    {
        /* Do Nothing */
    }
    return pinNum;
}

/**********************************************************************************/

