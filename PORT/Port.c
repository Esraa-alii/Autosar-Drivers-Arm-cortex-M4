/******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.c
 *
 * Description: Source file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Esraa Ali
 ******************************************************************************/

#include "Port.h"
#include "Port_Regs.h"
#include "tm4c123gh6pm_registers.h"

#if (PORT_DEV_ERROR_DETECT == STD_ON)
#include "Det.h"

/*check if the det module have the same version of port or not*/
#if (((DET_AR_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION) || (DET_AR_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION) || (DET_AR_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION)))
#error "The AR version of Det.h does not match the expected version"
#endif

#endif

/*initialize port status flag to be not initialize*/
STATIC uint8 Port_Status = PORT_NOT_INITIALIZED;
/*Hold the pointer of port_configtype*/
STATIC const Port_ConfigPin *Port_Pins = NULL_PTR;

/*Non Autosar functions*/

/*Describtion: function used to calculate the port base address based on the pin id */
STATIC volatile uint32 *Get_Base_Address(Port_PinType pinId)
{
    /* Get the port id by dividing(int division) pin id byy the total number of pins in one port*/
    uint8 portId = pinId / NUM_OF_PINS_SINGLE_PORT;

    switch (portId)
    {
    case 0:
        return (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
                                                           /*no need for break as the return key word will exit the function*/
    case 1:
        return (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
    case 2:
        return (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
    case 3:
        return (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
    case 4:
        return (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
    case 5:
        return (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
    default:                                               /*if port id not match with any availabe ids for the MCU return null pointer*/
        return NULL_PTR;
    }
}


/*Autosar functions*/

/*********************************************************************************
Service name:        Port_Init
Service ID[hex]:     0x00
Sync/Async:          Synchronous
Reentrancy:          Non Reentrant
Parameters (in):     ConfigPtr
Parameters (inout):  None
Parameters (out):    None
Return value:        None
Description:         Initializes the Port Driver module.
***************************************************************************************/

void Port_Init(const Port_ConfigType *ConfigPtr)
{

#if (PORT_DEV_ERROR_DETECT == STD_ON)
    /*if the input ptr is a null ptr -> report Det error*/
    if (ConfigPtr == NULL_PTR)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_INIT_SID,
                        PORT_E_PARAM_CONFIG);
    }
    else
#endif
    {
        /*initialize port status flag*/
        Port_Status = PORT_INITIALIZED;
        /*Address of the first pin structure */
        Port_Pins = ConfigPtr->Port_Pins;

        /*looping on all pins*/
        for (Port_PinType Pin = ZERO; Pin < PORT_CONFIGURED_PINS; Pin++)
        {
            /*point to the req base address of the register*/
            volatile uint32 *Port_GPIO_Ptr = NULL_PTR;
            volatile uint32 delay = 0;

            /*For port number*/
            /* Get the base address of the required pin */
            Port_GPIO_Ptr = Get_Base_Address(Port_Pins[Pin].port_num);
            /* Enable clock for PORT and allow time for clock to start*/
            SYSCTL_REGCGC2_REG |= (1 << Port_Pins[Pin].pin_num);
            delay = SYSCTL_REGCGC2_REG;

            /* if the pin is PD7 or PF0 so its need to lock*/
            if (((Port_Pins[Pin].port_num == 3) && (Port_Pins[Pin].pin_num == 7)) || ((Port_Pins[Pin].port_num == 5) && (Port_Pins[Pin].pin_num == 0))) /* PD7 or PF0 */
            {
                *(volatile uint32 *)((volatile uint8 *)Port_GPIO_Ptr + PORT_LOCK_REG_OFFSET) = 0x4C4F434B;                       /* Unlock the GPIOCR register */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_GPIO_Ptr + PORT_COMMIT_REG_OFFSET), Port_Pins[Pin].pin_num); /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
            }
            else if ((Port_Pins[Pin].port_num == 2) && (Port_Pins[Pin].pin_num <= 3)) /* PC0 to PC3 */
            {
                /* Do Nothing ...  this is the JTAG pins (for safety) */
            }
            else
            {
                /* Do Nothing ... No need to unlock the commit register for this pin */
            }

            /*For Pin direction*/
            /******************************************case of output pin *****************************************/
            if (Port_Pins[Pin].direction == OUTPUT)
            {
                /*clear bit to act as output pin */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_GPIO_Ptr + PORT_DIR_REG_OFFSET), Port_Pins[Pin].pin_num); /* Set the corresponding bit in the GPIODIR register to configure it as output pin */

                if (Port_Pins[Pin].initial_value == STD_HIGH)
                {
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_GPIO_Ptr + PORT_DATA_REG_OFFSET), Port_Pins[Pin].pin_num); /* Set the corresponding bit in the GPIODATA register to provide initial value 1 */
                }
                else
                {
                    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_GPIO_Ptr + PORT_DATA_REG_OFFSET), Port_Pins[Pin].pin_num); /* Clear the corresponding bit in the GPIODATA register to provide initial value 0 */
                }
            }
            /******************************************case of input pin *****************************************/
            else if (Port_Pins[Pin].direction == INPUT)
            {
                /*clear bit to act as input pin */
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_GPIO_Ptr + PORT_DIR_REG_OFFSET), Port_Pins[Pin].pin_num); /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
                /* check if pull up resistor or pull down or dont include internal resistor*/
                /*case of pull up */
                if (Port_Pins[Pin].resistor == PULL_UP)
                {
                    /*set corresponding bit to enanle internal pull up resistor*/
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_GPIO_Ptr + PORT_PULL_UP_REG_OFFSET), Port_Pins[Pin].pin_num); /* Set the corresponding bit in the GPIOPUR register to enable the internal pull up pin */
                }
                /*case of pull down */
                else if (Port_Pins[Pin].resistor == PULL_DOWN)
                {
                    /*set corresponding bit to enanle internal pull down resistor*/
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_GPIO_Ptr + PORT_PULL_DOWN_REG_OFFSET), Port_Pins[Pin].pin_num); /* Set the corresponding bit in the GPIOPDR register to enable the internal pull down pin */
                }
                /*case of no internal resistor*/
                else
                {
                    /*clear the 2 corresponding bits to disable internal pull up resistor and disable internal pull down resistor */
                    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_GPIO_Ptr + PORT_PULL_UP_REG_OFFSET), Port_Pins[Pin].pin_num);   /* Clear the corresponding bit in the GPIOPUR register to disable the internal pull up pin */
                    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_GPIO_Ptr + PORT_PULL_DOWN_REG_OFFSET), Port_Pins[Pin].pin_num); /* Clear the corresponding bit in the GPIOPDR register to disable the internal pull down pin */
                }
            }
            else
            {
                /* Do Nothing */
            }

            /*For pin mode*/
            if (Port_Pins[Pin].mode == GPIO_MODE)
            {
                /*disable analog functionality on this pin */
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_GPIO_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET), Port_Pins[Pin].pin_num);

                /* Disable Alternative function for this pin*/
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_GPIO_Ptr + PORT_ALT_FUNC_REG_OFFSET), Port_Pins[Pin].pin_num);

                /* Clear the PMCx bits for this pin */
                *(volatile uint32 *)((volatile uint8 *)Port_GPIO_Ptr + PORT_CTL_REG_OFFSET) &= ~(MASK << (Port_Pins[Pin].pin_num * LOCATIONS_VALUE));

                /* enable digital functionality on this pin */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_GPIO_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET), Port_Pins[Pin].pin_num);
            }
            else if ((Port_Pins[Pin].mode == ADC_MODE) || (Port_Pins[Pin].mode == ANALOG_COMPARATOR_MODE) || (Port_Pins[Pin].mode == ANALOG_MODE))
            {
                /* enable analog functionality on this pin */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_GPIO_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET), Port_Pins[Pin].pin_num);

                /* Disable Alternative function for this pin */
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_GPIO_Ptr + PORT_ALT_FUNC_REG_OFFSET), Port_Pins[Pin].pin_num);

                /* Clear the PMCx bits for this pin */
                *(volatile uint32 *)((volatile uint8 *)Port_GPIO_Ptr + PORT_CTL_REG_OFFSET) &= ~(MASK << (Port_Pins[Pin].pin_num * LOCATIONS_VALUE));

                /* enable digital functionality on this pin */
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_GPIO_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET), Port_Pins[Pin].pin_num);
            }
            else
            {
                /* disable analog functionality on this pin */
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_GPIO_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET), Port_Pins[Pin].pin_num);

                /* Enable Alternative function for this pin */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_GPIO_Ptr + PORT_ALT_FUNC_REG_OFFSET), Port_Pins[Pin].pin_num);

                /* Set the PMCx bits for this pin */
                *(volatile uint32 *)((volatile uint8 *)Port_GPIO_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32)Port_Pins[Pin].mode << (Port_Pins[Pin].pin_num * LOCATIONS_VALUE));

                /*enable digital functionality on this pin */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_GPIO_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET), Port_Pins[Pin].pin_num);
            }
        }
    }
}

#if (PORT_PIN_DIRECTION_CHANGEABLE_API == STD_ON)
/*********************************************************************************
Service name:        Port_SetPinDirection
Service ID[hex]:     0x01
Sync/Async:          Synchronous
Reentrancy:          Reentrant
Parameters (in):     Pin,Direction
Parameters (inout):  None
Parameters (out):    None
Return value:        None
Description:         Sets the port pin direction
***************************************************************************************/
void Port_SetPinDirection(Port_PinType Pin, Port_PinDirectionType Direction)
{
#if (PORT_PIN_DIRECTION_CHANGEABLE_API == STD_ON)
    /*check if the port is not initialized -> report det error*/
    if (Port_Status == PORT_NOT_INITIALIZED)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_DIRECTION_SID, PORT_E_UNINIT);
    }
    else
    {
        /*Do nothing .... */
    }

    /*check if the pin number is greater than or equal configured pins -> report det error*/
    if (Pin >= PORT_CONFIGURED_PINS)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_DIRECTION_SID, PORT_E_PARAM_PIN);
    }
    else
    {
        /*Do nothing .... */
    }

    /*check if the pin direction is not changable  -> report det error*/
    if (Port_Pins[Pin].direction_changable == STD_OFF)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_DIRECTION_SID, PORT_E_DIRECTION_UNCHANGEABLE);
    }
    else
    {
        /*Do nothing .... */
    }

#endif

    /* point to the required Port Registers base address */
    volatile uint32 *Port_GPIO_Ptr = NULL_PTR;
    /* Get the base address of the required pin */
    Port_GPIO_Ptr = Get_Base_Address(Port_Pins[Pin].port_num);
    if (!((Port_Pins[Pin].port_num == PORTC_ID) && (Pin <= PIN3_ID))) /* PC0 to PC3 */
    {

        if (Direction == OUTPUT)
        {
            /* Set the corresponding bit to configure it as output pin */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_GPIO_Ptr + PORT_DIR_REG_OFFSET), Port_Pins[Pin].pin_num);
        }
        else if (Direction == INPUT)
        {
            /* Clear the corresponding bit to configure it as input pin */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_GPIO_Ptr + PORT_DIR_REG_OFFSET), Port_Pins[Pin].pin_num);
        }
        else
        {
            /* Do Nothing */
        }
    }
    else
    {
        /* Do Nothing */
    }
}
#endif

/*********************************************************************************
Service name:        Port_RefreshPortDirection
Service ID[hex]:     0x02
Sync/Async:          Synchronous
Reentrancy:          Non Reentrant
Parameters (in):     None
Parameters (inout):  None
Parameters (out):    None
Return value:        None
Description:         Refreshes port direction.
***************************************************************************************/
void Port_RefreshPortDirection(void)
{

    /* point to the required Port Registers base address */
    volatile uint32 *PortGpio_Ptr = NULL_PTR;

    /*Variable used as an Index for the Port_Pin structures*/
    volatile uint8 Pin = 0;

    /*Det Error */
#if (PORT_DEV_ERROR_DETECT == STD_ON)
    /* check if the input configuration pointer is  a NULL_PTR -> report det error */
    if (Port_Status == PORT_NOT_INITIALIZED)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_DIRECTION_SID, PORT_E_UNINIT);
    }
    else
    {
        /* Do Nothing */
    }

#endif
    for (Port_PinType index = ZERO; index < PORT_CONFIGURED_PINS; index++)
    {
        PortGpio_Ptr = Get_Base_Address(Port_Pins[Pin].port_num);
        if (!((Port_Pins[Pin].port_num == PORTC_ID) && (Port_Pins[Pin].pin_num <= PIN3_ID))) /* PC0 to PC3 */
        {
            if (Port_Pins[Pin].direction_changable == NON_CHANGEABLE_DIRECTION)
            {
                if (Port_Pins[Pin].direction == OUTPUT)
                {
                    /* Set the corresponding bit to configure it as output pin */
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET), Port_Pins[Pin].pin_num);
                }
                else if (Port_Pins[Pin].direction == INPUT)
                {
                    /* Clear the corresponding bit to configure it as input pin */
                    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET), Port_Pins[Pin].pin_num);
                }
                else
                {
                    /* Do Nothing */
                }
            }
        }
        else
        {
            /* Do Nothing ... JTAG Pins */
        }
    }
}

/*********************************************************************************
Service name:        Port_GetVersionInfo
Service ID[hex]:     0x03
Sync/Async:          Synchronous
Reentrancy:          Non Reentrant
Parameters (in):     None
Parameters (inout):  None
Parameters (out):    versioninfo
Return value:        None
Description:        Returns the version information of this module.

***************************************************************************************/
#if (PORT_VERSION_INFO_API == STD_ON)
void Port_GetVersionInfo(Std_VersionInfoType *versioninfo)
{
#if (PORT_DEV_ERROR_DETECT == STD_ON)
    /* Check if input pointer is a Null pointer -> REPORT DET ERROE */
    if (NULL_PTR == versioninfo)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_GET_VERSION_INFO_SID, PORT_E_PARAM_POINTER);
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
#endif

/*********************************************************************************
Service name:        Port_SetPinMode
Service ID[hex]:     0x04
Sync/Async:          Synchronous
Reentrancy:          Reentrant
Parameters (in):     Pin,Mode
Parameters (inout):  None
Parameters (out):    None
Return value:        None
Description:        Sets the port pin mode
***************************************************************************************/
#if (PORT_PIN_MODE_CHANGEABLE_API == STD_ON)

void Port_SetPinMode(Port_PinType Pin, Port_PinModeType Mode)
{
#if (PORT_DEV_ERROR_DETECT == STD_ON)
    /* Check if the Driver is initialized before using this function */
    if (Port_Status == PORT_NOT_INITIALIZED)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE_SID, PORT_E_UNINIT);
    }
    else
    {
        /* Do Nothing */
    }

    /* check if incorrect Port Pin ID passed */
    if (Pin >= PORT_CONFIGURED_PINS)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE_SID, PORT_E_PARAM_PIN);
    }
    else
    {
        /* Do Nothing */
    }

    /* check if the API called when the mode is unchangeable */
    if (Port_Pins[Pin].mode_changable == NON_CHANGEABLE_MODE)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE_SID, PORT_E_MODE_UNCHANGEABLE);
    }
    else
    {
        /* Do Nothing */
    }
    if (!(Mode <= GROUP_12_ID || Mode == GROUP_14_ID))
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE_SID, PORT_E_MODE_UNCHANGEABLE);
    }
    else
    {
        /* Do Nothing */
    }
#endif
    /* point to the required Port Registers base address */
    volatile uint32 *Port_GPIO_Ptr = NULL_PTR;
    Port_GPIO_Ptr = Get_Base_Address(Port_Pins[Pin].port_num);
    /*Condition for JTAG pins to protection from changeing their mode */
    if (!((Port_Pins[Pin].port_num == PORTC_ID) && (Port_Pins[Pin].pin_num <= PIN3_ID))) /* PC0 to PC3 */
    {

        if (Mode == GPIO_MODE)
        {

            /* Clear the corresponding bit to disable analog functionality on this pin */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_GPIO_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET), Port_Pins[Pin].pin_num);

            /* Disable Alternative function for this pin*/
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_GPIO_Ptr + PORT_ALT_FUNC_REG_OFFSET), Port_Pins[Pin].pin_num);

            /* Clear the PMCx bits for this pin */
            *(volatile uint32 *)((volatile uint8 *)Port_GPIO_Ptr + PORT_CTL_REG_OFFSET) &= ~(MASK << (Port_Pins[Pin].pin_num * LOCATIONS_VALUE));

            /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_GPIO_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET), Port_Pins[Pin].pin_num);
        }

        else if ((Mode == ADC_MODE) || (Mode == ANALOG_COMPARATOR_MODE) || (Mode == ANALOG_MODE))
        {

            /* Set the corresponding bit enable analog functionality on this pin */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_GPIO_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET), Port_Pins[Pin].pin_num);

            /* Disable Alternative function for this pin  */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_GPIO_Ptr + PORT_ALT_FUNC_REG_OFFSET), Port_Pins[Pin].pin_num);

            /* Clear the PMCx bits for this pin */
            *(volatile uint32 *)((volatile uint8 *)Port_GPIO_Ptr + PORT_CTL_REG_OFFSET) &= ~(MASK << (Port_Pins[Pin].pin_num * LOCATIONS_VALUE));

            /* enable digital functionality on this pin */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_GPIO_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET), Port_Pins[Pin].pin_num);
        }

        else
        {

            /* Clear the corresponding bit to disable analog functionality on this pin */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_GPIO_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET), Port_Pins[Pin].pin_num);

            /* Enable Alternative function for this pin */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_GPIO_Ptr + PORT_ALT_FUNC_REG_OFFSET), Port_Pins[Pin].pin_num);

            /* Set the PMCx bits for this pin */
            *(volatile uint32 *)((volatile uint8 *)Port_GPIO_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32)Port_Pins[Pin].mode << (Port_Pins[Pin].pin_num * LOCATIONS_VALUE));

            /* enable digital functionality on this pin */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_GPIO_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET), Port_Pins[Pin].pin_num);
        }
    }
    else
    {
        /* Do Nothing ... No need to unlock the commit register for this pin */
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
void Port_SetupGpioPin(const Port_ConfigPin *ConfigPtr)
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
    }
    else
    {
        /* Do Nothing ... No need to unlock the commit register for this pin */
    }

    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET), ConfigPtr->pin_num);   /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET), ConfigPtr->pin_num);          /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (ConfigPtr->pin_num * 4)); /* Clear the PMCx bits for this pin */

    if (ConfigPtr->direction == OUTPUT)
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
    else if (ConfigPtr->direction == INPUT)
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
