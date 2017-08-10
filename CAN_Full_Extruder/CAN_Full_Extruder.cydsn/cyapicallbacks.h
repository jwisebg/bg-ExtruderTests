/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#ifndef CYAPICALLBACKS_H
#define CYAPICALLBACKS_H
    

    /*Define your macro callbacks here */
    /*For more information, refer to the Writing Code topic in the PSoC Creator Help.*/
#define CAN_MSG_TX_ISR_CALLBACK
void CAN_MsgTXIsr_Callback(void);
#define CAN_RECEIVE_MSG_0_CALLBACK
void CAN_ReceiveMsg_0_Callback(void);  
    
#endif /* CYAPICALLBACKS_H */   
/* [] */
