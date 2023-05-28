 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.h
 *
 * Description: Header file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Esraa Ali
 ******************************************************************************/

/*Garding*/
#ifndef PORT_H
#define PORT_H
/********************************** INCLUDES ***********************************/
#include "Common_Macros.h" /*Non Autosar module*/

/*Port registers header file*/
#include "Port_Regs.h"

 /* Port Pre-Compile Configuration Header file */
#include "Port_cfg.h"

/* Standard AUTOSAR types */
#include "Std_Types.h"

/* Id for the company in the AUTOSAR
 * for example Esraa Ali's ID = 1000  */
#define PORT_VENDOR_ID    (1000U)

/* PORT Module Id = 124*/
#define PORT_MODULE_ID    (124U)

/* PORT Instance Id */
#define PORT_INSTANCE_ID  (0U)

/*
 * Module Version 1.0.0
 */
#define PORT_SW_MAJOR_VERSION           (1U)
#define PORT_SW_MINOR_VERSION           (0U)
#define PORT_SW_PATCH_VERSION           (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_AR_RELEASE_MAJOR_VERSION   (4U)
#define PORT_AR_RELEASE_MINOR_VERSION   (0U)
#define PORT_AR_RELEASE_PATCH_VERSION   (3U)

/*
 * Macros for PORT Status
 */
#define PORT_INITIALIZED                (1U)
#define PORT_NOT_INITIALIZED            (0U)


/* AUTOSAR checking between Std Types and PORT Modules */
#if ((STD_TYPES_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (STD_TYPES_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (STD_TYPES_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Std_Types.h does not match the expected version"
#endif
   
   /* AUTOSAR Version checking between PORT_Cfg.h and PORT.h files */
#if ((PORT_CFG_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (PORT_CFG_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (PORT_CFG_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of PORT_Cfg.h does not match the expected version"
#endif

/* Software Version checking between PORT_Cfg.h and PORT.h files */
#if ((PORT_CFG_SW_MAJOR_VERSION != PORT_SW_MAJOR_VERSION)\
 ||  (PORT_CFG_SW_MINOR_VERSION != PORT_SW_MINOR_VERSION)\
 ||  (PORT_CFG_SW_PATCH_VERSION != PORT_SW_PATCH_VERSION))
  #error "The SW version of PORT_Cfg.h does not match the expected version"
#endif

 /*
 * Macros for Port Status
 */
#define PORT_INITIALIZED                (1U)
#define PORT_NOT_INITIALIZED            (0U)

/* Number of Pins in the Single Port (MCU Specific) */
#define NUM_OF_PINS_SINGLE_PORT         (8U)

#define ZERO                            (0U)
   
 
/******************************************************************************
 *                      API Service Id Macros                                 *
 ******************************************************************************/
#define PORT_INIT_SID                   (uint8)0x00
#define PORT_SET_PIN_DIRECTION_SID      (uint8)0x01
#define PORT_REFRESH_PORT_DIRECTION_SID (uint8)0x02
#define PORT_GET_VERSION_INFO_SID       (uint8)0x03
#define PORT_SET_PIN_MODE_SID           (uint8)0x04
   
/*******************************************************************************
 *                      DET Error Codes                                        *
 *******************************************************************************/
#define PORT_E_PARAM_PIN              0x0A /*Invalid Port Pin ID requested*/
#define PORT_E_DIRECTION_UNCHANGEABLE 0x0B /*Port Pin not configured as changeable*/
#define PORT_E_PARAM_CONFIG           0x0C /*API Port_Init service called with wrong parameter.*/   
/*API Port_SetPinMode service called when mode is unchangeable*/
#define PORT_E_PARAM_INVALID_MODE     0x0D
#define PORT_E_MODE_UNCHANGEABLE      0x0E
#define PORT_E_UNINIT                 0x0F /*API service called without module initialization*/
#define PORT_E_PARAM_POINTER          0X10 /*APIs called with a Null Pointer*/



/*******************************************************************************
 *                              Module Data Types                              *
 *******************************************************************************/

 /*Description: Data type for the symbolic name of a port pin.*/
typedef uint8 Port_PinType;

/*Type definition for port_pinModeType used by the port APIs*/
typedef uint8 Port_PinModeType;

/*Type definition for port_pinlevelType used by the port APIs*/
typedef uint8 Port_PinLevelType;

/*Type definition for Port_PortType used by the port APIs*/
typedef uint8 Port_PortType;

/************************************** ENUMS ***************************************/

/* Description: Enum to hold PIN direction */
typedef enum
{
    INPUT=0,OUTPUT
}Port_PinDirection;

/* Description: Enum to hold internal resistor type for PIN */
typedef enum
{
    OFF=0,PULL_UP,PULL_DOWN
}Port_InternalResistor;


/*Description: Possible directions of a port pin.*/
typedef enum
{
  PORT_PIN_IN=0,PORT_PIN_OUT
}Port_PinDirectionType;

/*Description: if the pin mode can be changable in run time or not.*/
typedef enum 
{
  NON_CHANGEABLE_MODE=0,CHANGEABLE_MODE
}Port_PinChangeableMode;

/*Description: if the pin direction can be changable in run time or not.*/
typedef enum
{
  NON_CHANGEABLE_DIRECTION=0,CHANGEABLE_DIRECTION
}Port_PinChangeableDirection;

/************************************** STRUCT ***************************************/

/* Description: Structure to configure each individual PIN:
 *	1. the PORT Which the pin belongs to. 0, 1, 2, 3, 4 or 5
 *	2. the number of the pin in the PORT.
 *      3. the direction of pin --> INPUT or OUTPUT
 *      4. the internal resistor --> Disable, Pull up or Pull down
 */

typedef struct 
{
    uint8 port_num;                
    uint8 pin_num;                  
    Port_PinDirection direction;                       /*INPUT or OUTPUT*/
    Port_InternalResistor resistor;                    /*case of input pin only*/
    uint8 initial_value;                               /*case of output pin only*/
    Port_PinModeType mode;                             /*e.g DIO,ADC,SPI,... */
    Port_PinChangeableDirection direction_changable;   /*Allow direction changes during runtime (STD_ON/STD_OFF) */
    Port_PinChangeableMode mode_changable;             /*Allow mode changes during runtime (STD_ON/STD_OFF) */
    
}Port_ConfigPin;

/*Structure required for initializing the port driver */
typedef struct 
{
	Port_ConfigPin Port_Pins[PORT_CONFIGURED_PINS];
}Port_ConfigType;


/*****************************************************************************************
 *                                External Variables                                    *
 *****************************************************************************************/
/*Extern Pos-build structures to be used by port.c and other modules*/
extern const Port_ConfigType port_config;

/*******************************************************************************
 *                      Function Prototypes                                    *
 *******************************************************************************/

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
*              - Setup the internal resistor for i/p pin
************************************************************************************/
void Port_SetupGpioPin(const Port_ConfigPin *ConfigPtr );


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
void Port_Init( const Port_ConfigType* ConfigPtr );

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
void Port_SetPinDirection( Port_PinType Pin, Port_PinDirectionType Direction );
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
void Port_RefreshPortDirection( void );

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
void Port_GetVersionInfo( Std_VersionInfoType* versioninfo );

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

void Port_SetPinMode( Port_PinType Pin, Port_PinModeType Mode );



#endif /* PORT_H */
