/*******************************************************************************
* File Name: main.c
*
* Version: 1.0
*
* Description:
*  This example project demonstrates how to configure the CAN Component in
*  the Full CAN mode.
*  The CAN component is configured to receive the following messages from the
*  remote node:
*   Message 1 - Status of Switch 1.
*   Message 2 - ADC data.
*
*  The component is also configured to transmit data to control the pulse width
*  of the PWM in the remote node. The transmitted data (pulse width value)
*  increments at a switch press.
*  Both transmitted and received data are displayed on a 2x16 LCD.
*
*  This is only one part of the CAN example project. Use this along with
*  CAN_Basic_Example for complete demonstration.
*
* Hardware Dependency:
*  CY8CKIT-001
*  CY8CKIT-017
*
********************************************************************************
* Copyright 2015, Cypress Semiconductor Corporation. All rights reserved.
* This software is owned by Cypress Semiconductor Corporation and is protected
* by and subject to worldwide patent and copyright laws and treaties.
* Therefore, you may use this software only as provided in the license agreement
* accompanying the software package from which you obtained this software.
* CYPRESS AND ITS SUPPLIERS MAKE NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
* WITH REGARD TO THIS SOFTWARE, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT,
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
*******************************************************************************/
#include <project.h>
#include <stdio.h>

#define PWM_PULSE_WIDTH_STEP        (10u)
#define SWITCH_PRESSED              (0u)
#define PWM_MESSAGE_ID              (0x1AAu)
#define PWM_MESSAGE_IDE             (0u)    /* Standard message */
#define PWM_MESSAGE_IRQ             (0u)    /* No transmit IRQ */
#define PWM_MESSAGE_RTR             (0u)    /* No RTR */
#define CAN_RX_MAILBOX_0_SHIFT      (1u)
#define CAN_RX_MAILBOX_1_SHIFT      (2u)
#define DATA_SIZE                   (6u)
#define ONE_BYTE_OFFSET             (8u)

/* Function prototypes */
CY_ISR_PROTO(ISR_CAN);

/* Global variables used to store configuration and data for BASIC CAN mailbox */
CAN_DATA_BYTES_MSG dataPWM;
CAN_TX_MSG messagePWM;

/* Global variable used to store PWM pulse width value */
uint8 pulseWidthValue = 0u;

/* Global variable used to store receive message mailbox number */
volatile uint8 receiveMailboxNumber = 0xFFu;


/*******************************************************************************
* Function Name: main
********************************************************************************
*
* Summary:
*  main() performs the following functions:
*  1: Initializes a structure for the Basic CAN mailbox to send messages.
*  2: Starts the CAN and LCD components.
*  3: When received Message 1, sends the PWM pulse width and displays
*     received switch status and value of PWM pulse width on an LCD; 
*     When received Message 2, display received ADC data on an LCD.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
int main()
{
    char8 txData[DATA_SIZE];
    uint16 adcData;
    
    /* BASIC CAN mailbox configuration */
    messagePWM.dlc = CAN_TX_DLC_MAX_VALUE;
    messagePWM.id  = PWM_MESSAGE_ID;
    messagePWM.ide = PWM_MESSAGE_IDE;
    messagePWM.irq = PWM_MESSAGE_IRQ;
    messagePWM.msg = &dataPWM;
    messagePWM.rtr = PWM_MESSAGE_RTR;

    LCD_Start();
    
    /* Display value of ADC output on LCD */
    LCD_Position(0u, 0u);
    LCD_PrintString("ADC");

    /* Display state of switch on LCD */
    LCD_Position(1u, 0u);
    LCD_PrintString("SW");

    /* Display state of PWM pulse width on LCD */
    LCD_Position(0u, 10u);
    LCD_PrintString("PWM");

    CAN_Start();

    /* Set CAN interrupt handler to local routine */
    CyIntSetVector(CAN_ISR_NUMBER, ISR_CAN);    

    CyGlobalIntEnable;

    for(;;)
    {
        if (receiveMailboxNumber == CAN_RX_MAILBOX_switchStatus)
        {
            LCD_Position(1u, 3u);
            if (CAN_RX_DATA_BYTE1(CAN_RX_MAILBOX_switchStatus) == SWITCH_PRESSED)
            {
                /* Display received switch status on LCD */
                LCD_PrintString("pressed ");

                /* Increase the PWM pulse width */
                pulseWidthValue += PWM_PULSE_WIDTH_STEP;

                /* Send message with the new PWM pulse width */
                dataPWM.byte[0u] = pulseWidthValue;
                CAN_SendMsg(&messagePWM);

                /* Display value of PWM pulse width on LCD */
                LCD_Position(0u, 14u);
                LCD_PrintInt8(pulseWidthValue);
            }
            else
            {
                /* Display received switch status on LCD */
                LCD_PrintString("released");
            }
            receiveMailboxNumber = 0xFFu;
        }
        
        if (receiveMailboxNumber == CAN_RX_MAILBOX_ADCdata)
        {
            adcData = ((uint16) ((uint16) CAN_RX_DATA_BYTE1(CAN_RX_MAILBOX_ADCdata) << ONE_BYTE_OFFSET)) | 
            CAN_RX_DATA_BYTE2(CAN_RX_MAILBOX_ADCdata);
            
            /* Display received ADC data on LCD */
            sprintf(txData, "%u.%.3u", (adcData / 1000u), (adcData % 1000u));
            txData[DATA_SIZE - 1u] = (char8) '\0';
            
            LCD_Position(0u, 4u);
            LCD_PrintString(txData);
            receiveMailboxNumber = 0xFFu;
        }
    }
}


/*******************************************************************************
* Function Name: ISR_CAN
********************************************************************************
*
* Summary:
*  This ISR is executed at a Receive Message event and set receiveMailboxNumber
*  global variable with receive message mailbox number.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
CY_ISR(ISR_CAN)
{
    /* Clear Receive Message flag */
    CAN_INT_SR_REG.byte[1u] = CAN_RX_MESSAGE_MASK;

    /* Switch Status message received */
    if ((CY_GET_REG16((reg16 *) &CAN_BUF_SR_REG.byte[0u]) & CAN_RX_MAILBOX_0_SHIFT) != 0u)
    {        
        receiveMailboxNumber = CAN_RX_MAILBOX_switchStatus;

        /* Acknowledges receipt of new message */
        CAN_RX_ACK_MESSAGE(CAN_RX_MAILBOX_switchStatus);
    }

    /* ADC data message received */
    if ((CY_GET_REG16((reg16 *) &CAN_BUF_SR_REG.byte[0u]) & CAN_RX_MAILBOX_1_SHIFT) != 0u)
    {
        receiveMailboxNumber = CAN_RX_MAILBOX_ADCdata;

        /* Acknowledges receipt of new message */
        CAN_RX_ACK_MESSAGE(CAN_RX_MAILBOX_ADCdata);
    }
}

/* [] END OF FILE */
