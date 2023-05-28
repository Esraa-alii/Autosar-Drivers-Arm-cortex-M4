 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port_PBcfg.c
 *
 * Description: Post Build Configuration Source file for TM4C123GH6PM Microcontroller - port Driver
 *
 * Author: Esraa Ali
 ******************************************************************************/

#ifndef PORT_CFG_H
#define PORT_CFG_H

#include "Port.h"

/*
 * Module Version 1.0.0
 */
#define PORT_PBCFG_SW_MAJOR_VERSION              (1U)
#define PORT_PBCFG_SW_MINOR_VERSION              (0U)
#define PORT_PBCFG_SW_PATCH_VERSION              (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_PBCFG_AR_RELEASE_MAJOR_VERSION     (4U)
#define PORT_PBCFG_AR_RELEASE_MINOR_VERSION     (0U)
#define PORT_PBCFG_AR_RELEASE_PATCH_VERSION     (3U)

/* AUTOSAR Version checking between PORT_PBcfg.c and PORT.h files */
#if ((PORT_PBCFG_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (PORT_PBCFG_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (PORT_PBCFG_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of PBcfg.c does not match the expected version"
#endif
  
   
/* Software Version checking between PORT_PBcfg.c and PORT.h files */
#if ((PORT_PBCFG_SW_MAJOR_VERSION != PORT_SW_MAJOR_VERSION)\
 ||  (PORT_PBCFG_SW_MINOR_VERSION != PORT_SW_MINOR_VERSION)\
 ||  (PORT_PBCFG_SW_PATCH_VERSION != PORT_SW_PATCH_VERSION))
  #error "The SW version of PBcfg.c does not match the expected version"

#endif   
   
/*Post Build structure t be used in port_init function
   Members : 1. Port number         (A=0,B=1,...)
             2. Pin number          (0,1,2,......)
             3. Direction           (input,output)
             4. Mode                (GPIO,UART,..)
             5. Internal Resistor   (case of pull up)
             6. Initial Value       (case of output)
             7. Direction changable (yes or not during run time)
             8. Mode changable      (yes or not during run time)
*/

   const Port_ConfigType port_config={
 /*********************************************************UART PINS*******************************************************/
   PORTA_ID , PIN0_ID , INPUT  , U0RX_MODE , OFF     , STD_HIGH , NON_CHANGEABLE_DIRECTION , NON_CHANGEABLE_MODE,
   PORTA_ID , PIN1_ID , INPUT  , U0TX_MODE , OFF     , STD_HIGH , NON_CHANGEABLE_DIRECTION , NON_CHANGEABLE_MODE,
 /*********************************************************************************************************************/
   PORTA_ID , PIN2_ID , INPUT  , GPIO_MODE , PULL_UP , STD_HIGH , NON_CHANGEABLE_DIRECTION , NON_CHANGEABLE_MODE,
   PORTA_ID , PIN3_ID , INPUT  , GPIO_MODE , PULL_UP , STD_HIGH , NON_CHANGEABLE_DIRECTION , NON_CHANGEABLE_MODE,
   PORTA_ID , PIN4_ID , INPUT  , GPIO_MODE , PULL_UP , STD_HIGH , NON_CHANGEABLE_DIRECTION , NON_CHANGEABLE_MODE,
   PORTA_ID , PIN5_ID , INPUT  , GPIO_MODE , PULL_UP , STD_HIGH , NON_CHANGEABLE_DIRECTION , NON_CHANGEABLE_MODE,
   PORTA_ID , PIN6_ID , INPUT  , GPIO_MODE , PULL_UP , STD_HIGH , NON_CHANGEABLE_DIRECTION , NON_CHANGEABLE_MODE,
   PORTA_ID , PIN7_ID , INPUT  , GPIO_MODE , PULL_UP , STD_HIGH , NON_CHANGEABLE_DIRECTION , NON_CHANGEABLE_MODE,

   PORTB_ID , PIN0_ID , INPUT  , GPIO_MODE , PULL_UP , STD_HIGH , NON_CHANGEABLE_DIRECTION , NON_CHANGEABLE_MODE,
   PORTB_ID , PIN1_ID , INPUT  , GPIO_MODE , PULL_UP , STD_HIGH , NON_CHANGEABLE_DIRECTION , NON_CHANGEABLE_MODE,
   PORTB_ID , PIN2_ID , INPUT  , GPIO_MODE , PULL_UP , STD_HIGH , NON_CHANGEABLE_DIRECTION , NON_CHANGEABLE_MODE,
   PORTB_ID , PIN3_ID , INPUT  , GPIO_MODE , PULL_UP , STD_HIGH , NON_CHANGEABLE_DIRECTION , NON_CHANGEABLE_MODE,
   PORTB_ID , PIN4_ID , INPUT  , GPIO_MODE , PULL_UP , STD_HIGH , NON_CHANGEABLE_DIRECTION , NON_CHANGEABLE_MODE,
   PORTB_ID , PIN5_ID , INPUT  , GPIO_MODE , PULL_UP , STD_HIGH , NON_CHANGEABLE_DIRECTION , NON_CHANGEABLE_MODE,
   PORTB_ID , PIN6_ID , INPUT  , GPIO_MODE , PULL_UP , STD_HIGH , NON_CHANGEABLE_DIRECTION , NON_CHANGEABLE_MODE,
   PORTB_ID , PIN7_ID , INPUT  , GPIO_MODE , PULL_UP , STD_HIGH , NON_CHANGEABLE_DIRECTION , NON_CHANGEABLE_MODE,
   
/*********************************************************JTAC PINS*******************************************************/
   PORTC_ID , PIN0_ID , INPUT  , TCK_SWCLK_MODE , PULL_UP , STD_HIGH , NON_CHANGEABLE_DIRECTION , NON_CHANGEABLE_MODE,
   PORTC_ID , PIN1_ID , INPUT  , TMS_SWDI0_MODE , PULL_UP , STD_HIGH , NON_CHANGEABLE_DIRECTION , NON_CHANGEABLE_MODE,
   PORTC_ID , PIN2_ID , INPUT  , TD1_MODE       , PULL_UP , STD_HIGH , NON_CHANGEABLE_DIRECTION , NON_CHANGEABLE_MODE,
   PORTC_ID , PIN3_ID , INPUT  , TD0_SW0_MODE   , PULL_UP , STD_HIGH , NON_CHANGEABLE_DIRECTION , NON_CHANGEABLE_MODE,
   /*********************************************************************************************************************/
   PORTC_ID , PIN4_ID , INPUT  , GPIO_MODE , PULL_UP , STD_HIGH , NON_CHANGEABLE_DIRECTION , NON_CHANGEABLE_MODE,
   PORTC_ID , PIN5_ID , INPUT  , GPIO_MODE , PULL_UP , STD_HIGH , NON_CHANGEABLE_DIRECTION , NON_CHANGEABLE_MODE,
   PORTC_ID , PIN6_ID , INPUT  , GPIO_MODE , PULL_UP , STD_HIGH , NON_CHANGEABLE_DIRECTION , NON_CHANGEABLE_MODE,
   PORTC_ID , PIN7_ID , INPUT  , GPIO_MODE , PULL_UP , STD_HIGH , NON_CHANGEABLE_DIRECTION , NON_CHANGEABLE_MODE,

   PORTD_ID , PIN0_ID , INPUT  , GPIO_MODE , PULL_UP , STD_HIGH , NON_CHANGEABLE_DIRECTION , NON_CHANGEABLE_MODE,
   PORTD_ID , PIN1_ID , INPUT  , GPIO_MODE , PULL_UP , STD_HIGH , NON_CHANGEABLE_DIRECTION , NON_CHANGEABLE_MODE,
   PORTD_ID , PIN2_ID , INPUT  , GPIO_MODE , PULL_UP , STD_HIGH , NON_CHANGEABLE_DIRECTION , NON_CHANGEABLE_MODE,
   PORTD_ID , PIN3_ID , INPUT  , GPIO_MODE , PULL_UP , STD_HIGH , NON_CHANGEABLE_DIRECTION , NON_CHANGEABLE_MODE,
   PORTD_ID , PIN4_ID , INPUT  , GPIO_MODE , PULL_UP , STD_HIGH , NON_CHANGEABLE_DIRECTION , NON_CHANGEABLE_MODE,
   PORTD_ID , PIN5_ID , INPUT  , GPIO_MODE , PULL_UP , STD_HIGH , NON_CHANGEABLE_DIRECTION , NON_CHANGEABLE_MODE,
   PORTD_ID , PIN6_ID , INPUT  , GPIO_MODE , PULL_UP , STD_HIGH , NON_CHANGEABLE_DIRECTION , NON_CHANGEABLE_MODE,
   PORTD_ID , PIN7_ID , INPUT  , GPIO_MODE , PULL_UP , STD_HIGH , NON_CHANGEABLE_DIRECTION , NON_CHANGEABLE_MODE,
   
   PORTE_ID , PIN0_ID , INPUT  , GPIO_MODE , PULL_UP , STD_HIGH , NON_CHANGEABLE_DIRECTION , NON_CHANGEABLE_MODE,
   PORTE_ID , PIN1_ID , INPUT  , GPIO_MODE , PULL_UP , STD_HIGH , NON_CHANGEABLE_DIRECTION , NON_CHANGEABLE_MODE,
   PORTE_ID , PIN2_ID , INPUT  , GPIO_MODE , PULL_UP , STD_HIGH , NON_CHANGEABLE_DIRECTION , NON_CHANGEABLE_MODE,
   PORTE_ID , PIN3_ID , INPUT  , GPIO_MODE , PULL_UP , STD_HIGH , NON_CHANGEABLE_DIRECTION , NON_CHANGEABLE_MODE,
   PORTE_ID , PIN4_ID , INPUT  , GPIO_MODE , PULL_UP , STD_HIGH , NON_CHANGEABLE_DIRECTION , NON_CHANGEABLE_MODE,
   PORTE_ID , PIN5_ID , INPUT  , GPIO_MODE , PULL_UP , STD_HIGH , NON_CHANGEABLE_DIRECTION , NON_CHANGEABLE_MODE,
   PORTE_ID , PIN6_ID , INPUT  , GPIO_MODE , PULL_UP , STD_HIGH , NON_CHANGEABLE_DIRECTION , NON_CHANGEABLE_MODE,
   PORTE_ID , PIN7_ID , INPUT  , GPIO_MODE , PULL_UP , STD_HIGH , NON_CHANGEABLE_DIRECTION , NON_CHANGEABLE_MODE,
   
   PORTF_ID , PIN0_ID , INPUT  , GPIO_MODE , PULL_UP , STD_HIGH , NON_CHANGEABLE_DIRECTION , NON_CHANGEABLE_MODE,
   PORTF_ID , PIN1_ID , OUTPUT , GPIO_MODE , PULL_UP , STD_HIGH , NON_CHANGEABLE_DIRECTION , NON_CHANGEABLE_MODE, /*LED */
   PORTF_ID , PIN2_ID , INPUT  , GPIO_MODE , PULL_UP , STD_HIGH , NON_CHANGEABLE_DIRECTION , NON_CHANGEABLE_MODE,
   PORTF_ID , PIN3_ID , INPUT  , GPIO_MODE , PULL_UP , STD_HIGH , NON_CHANGEABLE_DIRECTION , NON_CHANGEABLE_MODE,
   PORTF_ID , PIN4_ID , INPUT  , GPIO_MODE , PULL_UP , STD_HIGH , NON_CHANGEABLE_DIRECTION , NON_CHANGEABLE_MODE,
   PORTF_ID , PIN5_ID , INPUT  , GPIO_MODE , PULL_UP , STD_HIGH , NON_CHANGEABLE_DIRECTION , NON_CHANGEABLE_MODE,
   PORTF_ID , PIN6_ID , INPUT  , GPIO_MODE , PULL_UP , STD_HIGH , NON_CHANGEABLE_DIRECTION , NON_CHANGEABLE_MODE,
   PORTF_ID , PIN7_ID , INPUT  , GPIO_MODE , PULL_UP , STD_HIGH , NON_CHANGEABLE_DIRECTION , NON_CHANGEABLE_MODE,
   
   };


#endif /* PORT_CFG_H_ */
