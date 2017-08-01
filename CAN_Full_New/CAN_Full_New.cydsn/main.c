/*******************************************************************************
* File Name: main.c
*
* Version: 1.0
*
* Description:
*  This test project demonstrates how to configure the CAN Component in
*  the Full CAN mode on a CY8CKIT-50 board with a CY8CKIT-017 CAN card connected to 
*  port E.

*  The CAN component is configured to send one transmitted message and then 
*  receive messages on an interrupt from up to ten mailboxess (6 extruders and 4 steppers).
*
*  The IDs of the transmit and receive mailboxes are defined in the component.
*   
*  
*  Based on the PS0C 4.1 CAN full example.
*
*
*
* Hardware Dependency:
*  CY8CKIT-050
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

/* user defines */
#define NUM_EXTRUDERS   (6u)
#define NUM_STEPPERS    (4u)

/* Function prototypes */
CY_ISR_PROTO(ISR_CAN);
static void ProcessRX(const char *pString);

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

    
    /* BASIC CAN mailbox configuration */
    messagePWM.dlc = CAN_TX_DLC_MAX_VALUE;
    messagePWM.id  = PWM_MESSAGE_ID;
    messagePWM.ide = PWM_MESSAGE_IDE;
    messagePWM.irq = PWM_MESSAGE_IRQ;
    messagePWM.msg = &dataPWM;
    messagePWM.rtr = PWM_MESSAGE_RTR;

    LCD_Start();
    
    /* Display value of RX Buffer on LCD */
    LCD_Position(0u, 0u);
    LCD_PrintString("RX");

    
    receiveMailboxNumber = 0xFFu;
    CAN_Start();

    /* Set CAN interrupt handler to local routine */
    CyIntSetVector(CAN_ISR_NUMBER, ISR_CAN);    

    CyGlobalIntEnable;
        CAN_TX_DATA_BYTE1(CAN_TX_MAILBOX_txex1) = 0x01;
        CAN_TX_DATA_BYTE2(CAN_TX_MAILBOX_txex1) = 0x02;
        CAN_TX_DATA_BYTE3(CAN_TX_MAILBOX_txex1) = 0x03;
        CAN_TX_DATA_BYTE4(CAN_TX_MAILBOX_txex1) = 0x04;
        CAN_TX_DATA_BYTE5(CAN_TX_MAILBOX_txex1) = 0xDE;
        CAN_TX_DATA_BYTE6(CAN_TX_MAILBOX_txex1) = 0xAD;
        CAN_TX_DATA_BYTE7(CAN_TX_MAILBOX_txex1) = 0xBE;
        CAN_TX_DATA_BYTE8(CAN_TX_MAILBOX_txex1) = 0xEF;
        CAN_SendMsgtxex1();
    for(;;)
    {


 
        switch(receiveMailboxNumber)
        {
            case(CAN_RX_MAILBOX_recvex1):
                ProcessRX("recvex1   ");
                break;
            case(CAN_RX_MAILBOX_recvex2):
                ProcessRX("recvex2   ");
                break;
             case(CAN_RX_MAILBOX_recvex3):
                ProcessRX("recvex3   ");
                break;
            case(CAN_RX_MAILBOX_recvex4):
                ProcessRX("recvex4   ");
                break;
            case(CAN_RX_MAILBOX_recvex5):
                ProcessRX("recvex5   ");
                break;
            case(CAN_RX_MAILBOX_recvex6):
                ProcessRX("recvex6   ");
                break;
            case(CAN_RX_MAILBOX_recvstepx):
                ProcessRX("recvstepx ");
                break;
            case(CAN_RX_MAILBOX_recvstepy1):
                ProcessRX("recvstepy1 ");
                break;
            case(CAN_RX_MAILBOX_recvstepy2):
                ProcessRX("recvstepy2 ");
                break;
            case(CAN_RX_MAILBOX_recvstepz):
                ProcessRX("recvstepz  ");
                break;
            default:
                break;
        }
 /*       receiveMailboxNumber = 0xFFu;*/
        
        CyDelay(100u);
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
    uint8 offset;
    uint16 mailMask = CAN_RX_MAILBOX_0_SHIFT;
    uint16 regValue = 0;
    
    /* Clear Receive Message flag */
    CAN_INT_SR_REG.byte[1u] = CAN_RX_MESSAGE_MASK;

    regValue = (uint16) (CY_GET_REG16((reg16 *) &CAN_BUF_SR_REG.byte[0u]));
    
    /* extruder messages, tie goes to low order  */
    for(offset = 0; offset < (uint8) NUM_EXTRUDERS; ++offset)
    {
        if(regValue & mailMask )
        {
            receiveMailboxNumber = (uint8) (CAN_RX_MAILBOX_recvex1 + offset);
            /* Acknowledges receipt of new message */
            CAN_RX_ACK_MESSAGE(CAN_RX_MAILBOX_recvex1 + offset);
            break;
        }
        mailMask <<= 1;
    }
  
    if( 0xFFu == receiveMailboxNumber) /* no mailbox acknowledged yet */
    {
        mailMask = (1u << NUM_EXTRUDERS);
        /* stepper messages, tie goes to low order  */
        for(offset = 0; offset < (uint8) NUM_STEPPERS; ++offset)
        {
            if(regValue & mailMask )
            {
                receiveMailboxNumber = (uint8) (CAN_RX_MAILBOX_recvstepx + offset);
                /* Acknowledges receipt of new message */
                CAN_RX_ACK_MESSAGE(CAN_RX_MAILBOX_recvstepx + offset);
                break;
            }
            mailMask <<= 1;
        }
    }
}
/*******************************************************************************
* Function Name: ProcessRX
********************************************************************************
*
* Summary:
*  ProcessRX() performs the following functions:
*  1: Prints the const string parameter to the LCD Display
*  2: resets the receive mailbox to 0xff

*
* Parameters:
*  char8 const string[]  string to print to the LCD
*
* Return:
*  None.
*
*******************************************************************************/
static void ProcessRX(char8 const string[])
{
    LCD_Position(0u, 3u);
    LCD_PrintString(string); 
    receiveMailboxNumber = 0xFFu;          
}
/* [] END OF FILE */
