 /******************************************************************************
 *
 * Module: PORT
 *
 * File Name: PORT_Cfg.h
 *
 * Description: Pre-Compile Configuration Header file for TM4C123GH6PM Microcontroller - PORT Driver
 *
 * Author: Esraa Ali 
 ******************************************************************************/

/* Garding */
#ifndef PORT_CFG_H
#define PORT_CFG_H

/*
 * Module Version 1.0.0
 */
#define PORT_CFG_SW_MAJOR_VERSION              (1U)
#define PORT_CFG_SW_MINOR_VERSION              (0U)
#define PORT_CFG_SW_PATCH_VERSION              (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_CFG_AR_RELEASE_MAJOR_VERSION     (4U)
#define PORT_CFG_AR_RELEASE_MINOR_VERSION     (0U)
#define PORT_CFG_AR_RELEASE_PATCH_VERSION     (3U)


/* Pre-compile option for Development Error Detect */
#define PORT_DEV_ERROR_DETECT                (STD_ON)

/* Pre-compile option for Version Info API */
#define PORT_VERSION_INFO_API                (STD_ON)
   
/* Number of Configured Pins In the Mcu (MCU DEPENDENT) */
#define PORT_ALL_PINS_NUMBER                  (48U)

/* pre-compile option for PORT SET PIN DIRECTION API */
#define PORT_SET_PIN_DIRECTION_API            (STD_ON)
   
/* Pre-compile option for Pin Direction Changeable API */
#define PORT_PIN_DIRECTION_CHANGEABLE_API       (STD_ON)

/* Pre-compile option for Pin Mode Changeable API */
#define PORT_PIN_MODE_CHANGEABLE_API            (STD_ON)


#define PORT_NUMS
#define PORT_CONFIGURED_PINS                  (43U)

/************************Tiva-c ports*************************************/
#define PORTA_ID   (0U)
#define PORTB_ID   (1U)
#define PORTC_ID   (2U)
#define PORTD_ID   (3U)
#define PORTE_ID   (4U)
#define PORTF_ID   (5U)

/************************Tiva-c Pins*************************************/
#define PIN0_ID    (0U)
#define PIN1_ID    (1U)
#define PIN2_ID    (2U)
#define PIN3_ID    (3U)
#define PIN4_ID    (4U)
#define PIN5_ID    (5U)
#define PIN6_ID    (6U)
#define PIN7_ID    (7U)


/***********************************Digital Function modes****************************************/
#define GROUP_1_ID 	(1U)
#define U0RX_MODE      (Port_PinModeType)(1U)
#define U0TX_MODE      (Port_PinModeType)(1U)
#define U1RX_MODE      (Port_PinModeType)(1U)
#define U1TX_MODE      (Port_PinModeType)(1U)
#define U4RX_MODE      (Port_PinModeType)(1U)
#define U4TX_MODE      (Port_PinModeType)(1U)
#define U3RX_MODE      (Port_PinModeType)(1U)
#define U3TX_MODE      (Port_PinModeType)(1U)
#define SSI3CLK_MODE   (Port_PinModeType)(1U)
#define SSI3FSS_MODE   (Port_PinModeType)(1U)
#define SSI3RX_MODE    (Port_PinModeType)(1U)
#define SSI3TX_MODE    (Port_PinModeType)(1U)
#define U6RX_MODE      (Port_PinModeType)(1U)
#define U6TX_M0DE      (Port_PinModeType)(1U)
#define U2RX_MODE      (Port_PinModeType)(1U)
#define U2TX_M0DE      (Port_PinModeType)(1U)
#define U7RX_MODE      (Port_PinModeType)(1U)
#define U7TX_M0DE      (Port_PinModeType)(1U)
#define U5RX_MODE      (Port_PinModeType)(1U)
#define U5TX_M0DE      (Port_PinModeType)(1U)
#define U1RTS_MODE     (Port_PinModeType)(1U)
#define U1CTS_MODE     (Port_PinModeType)(1U)
#define TCK_SWCLK_MODE (Port_PinModeType)(1U)
#define TMS_SWDI0_MODE (Port_PinModeType)(1U)
#define TD1_MODE       (Port_PinModeType)(1U)
#define TD0_SW0_MODE   (Port_PinModeType)(1U)

/**************************************************************************************************/

#define GROUP_2_ID 	(2U)   
#define SSI0CLK_MODE   (Port_PinModeType)(2U)
#define SSI0FSS_MODE   (Port_PinModeType)(2U)
#define SSI0RX_MODE    (Port_PinModeType)(2U)
#define SSI0TX_MODE    (Port_PinModeType)(2U)
#define SSI2CLK_MODE   (Port_PinModeType)(2U)
#define SSI2FSS_MODE   (Port_PinModeType)(2U)
#define SSI2RX_MODE    (Port_PinModeType)(2U)
#define SSI2TX_MODE    (Port_PinModeType)(2U)
#define U1RX_MODE      (Port_PinModeType)(2U)
#define U1TX_MODE      (Port_PinModeType)(2U)
#define SSI1CLK_MODE   (Port_PinModeType)(2U)
#define SSI1FSS_MODE   (Port_PinModeType)(2U)
#define SSI1RX_MODE    (Port_PinModeType)(2U)
#define SSI1TX_MODE    (Port_PinModeType)(2U)

/**************************************************************************************************/

#define GROUP_3_ID     (3U)
#define I2C1SCL_MODE   (Port_PinModeType)(3U)
#define I2C1SDA_MODE   (Port_PinModeType)(3U)
#define I2C0SCL_MODE   (Port_PinModeType)(3U)
#define I2C0SDA_MODE   (Port_PinModeType)(3U)
#define I2C3SCL_MODE   (Port_PinModeType)(3U)
#define I2C3SDA_MODE   (Port_PinModeType)(3U)
#define I2C2SCL_MODE   (Port_PinModeType)(3U)
#define I2C2SDA_MODE   (Port_PinModeType)(3U)
#define CAN0RX_MODE    (Port_PinModeType)(3U)
#define CAN0TX_MODE    (Port_PinModeType)(3U)

/**************************************************************************************************/

#define GROUP_4_ID     (4U)
#define M0PWM2_MODE    (Port_PinModeType)(4U)
#define M0PWM3_MODE    (Port_PinModeType)(4U)
#define M0PWM0_MODE    (Port_PinModeType)(4U)
#define M0PWM1_MODE    (Port_PinModeType)(4U)
#define M0PWM6_MODE    (Port_PinModeType)(4U)
#define M0PWM7_MODE    (Port_PinModeType)(4U)
#define M0FAULT0_MODE  (Port_PinModeType)(4U)
#define M0PWM4_MODE    (Port_PinModeType)(4U)
#define M0PWM5_MODE    (Port_PinModeType)(4U)

/**************************************************************************************************/

#define GROUP_5_ID       (5U)
#define M1PWM0_MODE      (Port_PinModeType)(5U)
#define M1PWM1_MODE      (Port_PinModeType)(5U)
#define M1PWM2_MODE      (Port_PinModeType)(5U)
#define M1PWM3_MODE      (Port_PinModeType)(5U)
#define M1PWM4_MODE      (Port_PinModeType)(5U)
#define M1PWM5_MODE      (Port_PinModeType)(5U)
#define M1PWM6_MODE      (Port_PinModeType)(5U)
#define M1PWM7_MODE      (Port_PinModeType)(5U)
#define M1FAULT0_MODE    (Port_PinModeType)(5U)

/**************************************************************************************************/

#define GROUP_6_ID       (6U)
#define IDX1_MODE        (Port_PinModeType)(6U)
#define PHA1_MODE        (Port_PinModeType)(6U)
#define PHB1_MODE        (Port_PinModeType)(6U)
#define IDX0_MODE        (Port_PinModeType)(6U)
#define PHA0_MODE        (Port_PinModeType)(6U)
#define PHB0_MODE        (Port_PinModeType)(6U)

/**************************************************************************************************/

#define GROUP_7_ID           (7U)
#define T2CCP0_MODE         (Port_PinModeType)(7U)
#define T2CCP1_MODE         (Port_PinModeType)(7U)
#define T3CCP0_MODE         (Port_PinModeType)(7U)
#define T3CCP1_MODE         (Port_PinModeType)(7U)
#define T1CCP0_MODE         (Port_PinModeType)(7U)
#define T1CCP1_MODE         (Port_PinModeType)(7U)
#define T0CCP0_MODE         (Port_PinModeType)(7U)
#define T0CCP1_MODE         (Port_PinModeType)(7U)
#define T4CCP0_MODE         (Port_PinModeType)(7U)
#define T4CCP1_MODE         (Port_PinModeType)(7U)
#define T5CCP0_MODE         (Port_PinModeType)(7U)
#define T5CCP1_MODE         (Port_PinModeType)(7U)
#define WT0CCP0_MODE        (Port_PinModeType)(7U)
#define WT0CCP1_MODE        (Port_PinModeType)(7U)
#define WT1CCP0_MODE        (Port_PinModeType)(7U)
#define WT1CCP1_MODE        (Port_PinModeType)(7U)
#define WT2CCP0_MODE        (Port_PinModeType)(7U)
#define WT2CCP1_MODE        (Port_PinModeType)(7U)
#define WT3CCP0_MODE        (Port_PinModeType)(7U)
#define WT3CCP1_MODE        (Port_PinModeType)(7U)
#define WT4CCP0_MODE        (Port_PinModeType)(7U)
#define WT4CCP1_MODE        (Port_PinModeType)(7U)
#define WT5CCP0_MODE        (Port_PinModeType)(7U)
#define WT5CCP1_MODE        (Port_PinModeType)(7U)

/**************************************************************************************************/

#define GROUP_8_ID          (8U)
#define CAN1RX_MODE         (Port_PinModeType)(8U)
#define CAN1TX_MODE         (Port_PinModeType)(8U)
#define CAN0RX_MODE         (Port_PinModeType)(8U)
#define CAN0TX_MODE         (Port_PinModeType)(8U)
#define U1TRS_MODE          (Port_PinModeType)(8U)
#define U1CTS_MODE          (Port_PinModeType)(8U)
#define USB0EPEN_MODE       (Port_PinModeType)(8U)
#define USB0PFLT_MODE       (Port_PinModeType)(8U)
#define NMT_MODE            (Port_PinModeType)(8U)

/**************************************************************************************************/

#define GROUP_9_ID          (9U)
#define c0o_MODE            (Port_PinModeType)(9U)
#define c1o_MODE            (Port_PinModeType)(9U)

/**************************************************************************************************/   
   
#define GROUP_11_ID         (11U)
#define C1_NEGATIVE_MODE    (Port_PinModeType)(11U)
#define C1_POSITIVE_MODE    (Port_PinModeType)(11U)
#define C0_POSITIVE_MODE    (Port_PinModeType)(11U)
#define C0_NEGATIVE_MODE    (Port_PinModeType)(11U)

/**************************************************************************************************/

#define GROUP_12_ID         (12U)
#define USB0DM_MODE         (Port_PinModeType)(12U)
#define USB0DP_MODE         (Port_PinModeType)(12U)
#define USB0ID_MODE         (Port_PinModeType)(12U)
#define USB0VBUS_MODE       (Port_PinModeType)(12U)

/**************************************************************************************************/

#define GROUP_14_ID          (14U)
#define TRD1_MODE            (Port_PinModeType)(14U)
#define TRD0_MODE            (Port_PinModeType)(14U)
#define TRclk_MODE           (Port_PinModeType)(14U)


/*************************************other modes************************************************/

#define GPIO_MODE 		 (Port_PinModeType)(0U)
#define ADC_MODE 		 (Port_PinModeType)(10U)
#define ANALOG_COMPARATOR_MODE   (Port_PinModeType)(11U)
#define ANALOG_MODE 		 (Port_PinModeType)(12U)

#endif /* DIO_CFG_H */
