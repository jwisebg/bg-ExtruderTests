/*******************************************************************************
* File Name: main.c
*
* Version: 0.3
*
* Description:
* This project provides the firmware to drive the BioBot Extruder card
* in response to received CAN bus messages from a CAN Bus Master.
* 
* In Version 0.3 the Extruder supports a Full CAN interface with an assigned
* receive mailbox and an assigned transmit mailbox with identical Ids.
*
* The project utilizes CAN hardware support provided by the Cypress 
* CY8C5868AXI-LPO35 SoC and receive and transmit interrupts are utilized.
* 
* The particular Extruder card is addressed by its receive mailbox Id,
* the first octet of data identifies the message type for processing.
*
* This implementation provides stub functions for processing, EEPROM
* support for identified configuration data, and RAM allocations for 
* identified "live" data. 
*
* Includes optional diagnostic message support for development and manufacturing test. 
*
* Next Steps: 
*
*   Create and integrate messages as needed
*
*   Fan:
*     Save/Recall Configuration
*     Sleep modes?
*     Connect to temperature/speed control
*     Diagnostics and status
*     Debug and test
*
*   Thermistors
*     ADC Range setup
*     ADC offset corrections 
*     Lookup table, counts to thermistor indicated temperature
*     Integrate with temperature control
*     Save/Recall Configuration?
*     Diagnostics and status
*     Debug and test
*
*   Thermocouples
*     ADC Range setup
*     ADC offset corrections 
*     DAC range setup
*     DAC reference value generation
*     Convert thermocouple reading to temperature
*     Integrate with temperature control
*     Save/Recall Configuration?
*     Diagnostics and status
*     Debug and test     
*
*   Peltier Drive Initialization/Algorithm
*
*   Temperature Control Algorithms
*
*   Alarm and Status processing
*
*   Live data logging
*
*   Manufacturing/Diagnostic functions including bootloader, serialization
*
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
/********************************************************************************
* Portions Copyright 2017, BioBot Corporation. All rights reserved.
*******************************************************************************/
#include <project.h>
#include <stdio.h>
#include <stdbool.h>

/**** defines and typedefs ****/
#define DATA_SIZE              (6u)
#define KIT50_VERSION         

#define EXTRUDER_NODE_ID     0x006 
#define RX_MAILBOX_NUM      CAN_RX_MAILBOX_0 
#define TX_MAILBOX_NUM      CAN_TX_MAILBOX_0 
#define RX_MAX_MAILBOX      (15u)
#define TX_MAX_MAILBOX      (7u)
#define MAX_TXQ_SIZE        (8)
#define MAX_RXQ_SIZE        (8)

typedef struct sCANQMessage {
    uint8 u8FunctionID;
    uint8 mailbox;
    uint8 data[8];
 }CANQMessage_t;

#define RX_RESPONSE_NONE       (0)
#define RX_Q_OK                (1) /* same as TX_Q_OK */
#define TX_Q_OK                (1)

#define RX_FUNCTION_ID_INVALID (-1)
#define RX_MAILBOX_INVALID     (-2)
#define RX_Q_FULL              (-3)  /* same as TX_Q_FULL */
#define TX_Q_FULL              (-3)  /* keep these two the same for convenience */


#define SIMULATION_PROCESSING_STEP 2
#define CONTROL_PROCESSING_STEP 3
#define MAX_PROCESSING_STEP    4

/* Message types - Function Identifier */
#define SET_NODE_ID                 0x00 
#define GET_EXTRUDER_STATUS         0x01
#define SET_EXTRUDER_SHUTDOWN       0x02
#define GET_EXTRUDER_SHUTDOWN       0x03
#define SET_EXTRUDER_REPORTING      0x04
#define GET_EXTRUDER_REPORTING      0x05


#define SET_TC_TEMP1_SETPOINT       0x10  
#define SET_TC_TEMP1                0x11  /* diagnostic, for simulating temperature changes */
#define SET_TC_TEMP1_VREF           0x12
#define SET_TC_TEMP1_HIGH_ALARM     0x13
#define SET_TC_TEMP1_LOW_ALARM      0x14
#define SET_TC_TEMP1_ALARM_HYS      0x15
#define GET_TC_TEMP1                0x16
#define GET_TC_TEMP1_VREF           0x17

#define SET_TC_TEMP2_SETPOINT       0x18  
#define SET_TC_TEMP2                0x19  /* diagnostic, for simulating temperature changes */
#define SET_TC_TEMP2_VREF           0x1a
#define SET_TC_TEMP2_HIGH_ALARM     0x1b
#define SET_TC_TEMP2_LOW_ALARM      0x1c
#define SET_TC_TEMP2_ALARM_HYS      0x1d
#define GET_TC_TEMP2                0x1e
#define GET_TC_TEMP2_VREF           0x1f

#define SET_TH_TEMP1_SETPOINT       0x20  
#define SET_TH_TEMP1                0x21  /* diagnostic, for simulating temperature changes */
#define SET_TH_TEMP1_HIGH_ALARM     0x22
#define SET_TH_TEMP1_LOW_ALARM      0x23
#define SET_TH_TEMP1_ALARM_HYS      0x24
#define GET_TH_TEMP1                0x25

#define SET_TH_TEMP2_SETPOINT       0x26  
#define SET_TH_TEMP2                0x27  /* diagnostic, for simulating temperature changes */
#define SET_TH_TEMP2_HIGH_ALARM     0x28
#define SET_TH_TEMP2_LOW_ALARM      0x29
#define SET_TH_TEMP2_ALARM_HYS      0x2a
#define GET_TH_TEMP2                0x2b


#define SET_PELTIER_DIRECTION         0x30
#define SET_PELTIER_ON                0x31
#define SET_PELTIER_ON_TIME           0x32
#define GET_PELTIER_STATUS            0x33
#define GET_PELTIER_P_SENSE_DIAG      0x34
#define GET_PELTIER_N_SENSE_DIAG      0x35

/* diagnostic, for pass through motor control */
#define SET_PELTIER_P_INHIBIT         0x36
#define SET_PELTIER_P_IN              0x37
#define SET_PELTIER_P_SENSE_DIAG      0x38
#define SET_PELTIER_N_INHIBIT         0x39
#define SET_PELTIER_N_IN              0x3a
#define SET_PELTIER_N_SENSE_DIAG      0x3b
/* end diagnostic for motor control */

/* fan related */
#define SET_FAN_ENABLE              0x40
#define GET_FAN_ENABLE              0x41

#define SET_FAN_PWM                 0x42 /* diagnostic */
#define GET_FAN_PWM                 0x43 



/* control related */
#define SET_CONTROL1_KP             0x50
#define SET_CONTROL1_KI             0x51
#define SET_CONTROL1_KD             0x52
#define SET_CONTROL1_M_MAX          0x53
#define GET_CONTROL1_DELTAP         0x54
#define GET_CONTROL1_DELTAI         0x55
#define GET_CONTROL1_DELTAD         0x56
#define GET_CONTROL1_DELTAM         0x57
#define GET_CONTROL1_ERROR          0x58
#define GET_CONTROL1_PREV_ERROR     0x59
#define GET_CONTROL1_M              0x5a

#define SET_CONTROL2_KP             0x60
#define SET_CONTROL2_KI             0x61
#define SET_CONTROL2_KD             0x62
#define SET_CONTROL2_M_MAX          0x63
#define GET_CONTROL2_DELTAP         0x64
#define GET_CONTROL2_DELTAI         0x65
#define GET_CONTROL2_DELTAD         0x66
#define GET_CONTROL2_DELTAM         0x67
#define GET_CONTROL2_ERROR          0x68
#define GET_CONTROL2_PREV_ERROR     0x69
#define GET_CONTROL2_M              0x6a





 

/* EEPROM register defines */
#define NODE_ID_REG1            0x0000
#define NODE_ID_REG2            0x0001
#define TC_TEMP1_SETPOINT       0x0000
#define TC_TEMP1_HIGH_ALARM     0x0004
#define TC_TEMP1_LOW_ALARM      0x0008
#define TC_TEMP1_ALARM_HYS      0x000C
#define TC_TEMP2_SETPOINT       0x0010
#define TC_TEMP2_HIGH_ALARM     0x0014
#define TC_TEMP2_LOW_ALARM      0x0018
#define TC_TEMP2_ALARM_HYS      0x001C
#define TH_TEMP1_SETPOINT       0x0020
#define TH_TEMP1_HIGH_ALARM     0x0024
#define TH_TEMP1_LOW_ALARM      0x0028
#define TH_TEMP1_ALARM_HYS      0x002C
#define TH_TEMP2_SETPOINT       0x0030
#define TH_TEMP2_HIGH_ALARM     0x0034
#define TH_TEMP2_LOW_ALARM      0x0038
#define TH_TEMP2_ALARM_HYS      0x003C

typedef struct sController {
    float32    KP;
    float32    KI;
    float32    KD;
    float32    Error[2];
    float32    DMinusOne;
    float32    DeltaP;
    float32    DeltaI;
    float32    DeltaD;
    float32    DeltaM;
    float32    Alpha;
    float32    Beta;
    float32    M;
    bool       inWindup;
}Controller_t;

typedef union uConvF32U32{
    
    uint8   byte[4];
    uint32  u32val;
    float32 f32Val;
    
}uConvF32U32_t;
    

typedef struct sThermocouple {
    int32  setpoint;  
    int32  lastValue;  /* Celsius */
    uint8  lastVRef;
    int32  highAlarm;
    int32  lowAlarm;
    int32  alarmHysteresis;
    bool   inAlarm;
}Thermocouple_t;

typedef struct sThermistor {
    int32  setpoint;  
    int32  lastValue;  /* Celsius */
    int32  highAlarm;
    int32  lowAlarm;
    int32  alarmHysteresis;
    bool   inAlarm;
}Thermistor_t;



#define PELTIER_ON_BIT        0x80
#define PELTIER_DIR_BIT       0x40
#define PELTIER_ALM_BIT       0x20
#define PELTIER_P_INHIBIT_BIT 0x08
#define PELTIER_P_IN_BIT      0x04
#define PELTIER_N_INHIBIT_BIT 0x02
#define PELTIER_N_IN_BIT      0x01


typedef struct sExtPeltier {
    uint8  peltierStatus;
    int16  peltierFaultCurrentP;
    int16  peltierFaultCurrentN;
    uint16 peltierOnTime;
}ExtruderPeltier_t;

typedef struct sExtFan {
    bool   fanEnable;
    uint8  fanDutyCycle; /* count 0-255, later, percentage */
    uint16 fanPeriod;
    bool   fanOn;
}ExtruderFan_t;

/**** Function prototypes ****/


/* Global variable used to store ISR flag */
volatile uint8 isrFlag = 0u;
volatile uint32 nodeID = EXTRUDER_NODE_ID;
volatile CAN_RX_CFG CYCODE MB0_Fly_Config = { 0u, 0x28u, 0x1FFFF9u, EXTRUDER_NODE_ID << CAN_SET_TX_ID_STANDARD_MSG_SHIFT};

/**** private function prototypes ****/
static int8 incStart(bool bTxQ, uint8 mailbox);
static int8 incEnd(bool bTxQ, uint8 mailbox);
static int8 TransmitQItem(uint8 qIndex);
static int8 processIncomingMessage(uint8 rxmailbox);

static int8 SendU8(uint8 u8FunctionID,uint8 txmailbox, uint8 value);
static int8 SendU16(uint8 u8FunctionID,uint8 txmailbox, uint16 value);
static int8 SendU32(uint8 u8FunctionID,uint8 txmailbox, uint32 value);
static int8 SendU56(uint8 u8FunctionID,uint8 txmailbox, uint64 value);

static uint8  ReceiveU8(uint8 rxMailbox);
static uint16 ReceiveU16(uint8 rxMailbox);
static uint32 ReceiveU32(uint8 rxmailbox);
static uint64 ReceiveU56(uint8 rxmailbox);
static uint32 ReceiveU32ToEEPROM(uint8 rxmailbox, uint16 address);
static float32 ReceiveF32ToEEPROM(uint8 rxMailbox, uint16 address);

static void InitAMuxs(void);
static void InitADCs(void);
static void InitVDACs(void);
static void InitFanPWM(void);

static void InitControllers(void);

/**** private variables ****/

static uint8 processingStep;
static int8 i8ResponseSent = 0;

static volatile uint8 txQStart = 0;
static volatile uint8 txQEnd = 0;
static volatile CANQMessage_t txQ[MAX_TXQ_SIZE] = {};

static volatile uint8 rxQStart = 0;
static volatile uint8 rxQEnd = 0;
static volatile CANQMessage_t rxQ[MAX_RXQ_SIZE] = {};

static Thermocouple_t tc1 = {};  
static Thermocouple_t tc2 = {}; 
static Thermistor_t   th1 = {}; 
static Thermistor_t   th2 = {}; 

static ExtruderPeltier_t ePeltier = {};
static ExtruderFan_t   eFan = {};

static Controller_t ctrl1 = {};
static Controller_t ctrl2 = {};
 

/*******************************************************************************
* Function Name: main
********************************************************************************
*
* Summary:
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

    processingStep = 0;
    
    CAN_Start();
    EEPROM_Start();
#ifndef KIT50_VERSION    
    InitAMuxs();
    InitADCs();
    InitVDACs();
    InitFanPWM();
#endif
    InitControllers();
    
    
 #ifdef KIT50_VERSION       
    LCD_Start();
    
    /* Display value of RX Buffer on LCD */
    LCD_Position(0u, 0u);
    LCD_PrintString("EX ");
 #endif   
    //Initializing EEPROM parameters.
    //TODO: JW  nodeID = (EEPROM_ReadByte(NODE_ID_REG2) << 8) + EEPROM_ReadByte(NODE_ID_REG1);
     MB0_Fly_Config.rxacr = nodeID << CAN_SET_TX_ID_STANDARD_MSG_SHIFT;
     CAN_RxBufConfig((const CAN_RX_CFG *)(&MB0_Fly_Config));
 
    
   
    
    CY_SET_REG32(CAN_TX_ID_PTR(CAN_TX_MAILBOX_0), (uint32)nodeID << CAN_SET_TX_ID_STANDARD_MSG_SHIFT);
  /*  CY_SET_REG32(CAN_RX_ID_PTR(CAN_RX_MAILBOX_0), (uint32)nodeID << CAN_SET_TX_ID_STANDARD_MSG_SHIFT); */



  //  CyIntSetVector(CAN_ISR_NUMBER, CAN_ISR);
    CyGlobalIntEnable;
 
#ifndef KIT50_VERSION
    can_shdn_Write(0);
    
    tec_hb_n_inh_Write(1);
    tec_hb_n_in_Write(1);
    
    tec_hb_p_inh_Write(1);
    tec_hb_p_in_Write(0); 
#endif  

    for(;;)
    {
        /*process messages for all steps  */    
        
        if(0 != isrFlag) /* received a message that needs processing? */
        {
  
            i8ResponseSent = processIncomingMessage(CAN_RX_MAILBOX_0);
            if( TX_Q_FULL != i8ResponseSent) /* process again if can't send response now */
            {
                if(0 == incStart(false,CAN_RX_MAILBOX_0)) /* adjusted queue empty? */
                {
                    /* Clear the isrFlag */
                    isrFlag = 0u;
               
                }
            }
        }
        
        
        if(CONTROL_PROCESSING_STEP == processingStep)
        {
            /* obtain measurements, blocking mode, for now  */
#ifndef KIT50_VERSION            
            /* select Analog Mux location tc_temp1 */
            ADC_tc_StartConvert( );
            if(ADC_tc_IsEndConversion(ADC_tc_WAIT_FOR_RESULT)) /* 1 indicates done */
            {
                tc1.lastValue = (int32) ADC_tc_GetResult16(); /*TODO: JW  raw value for now */
            }
     
            /* select Analog Mux location tc_temp2 */
            ADC_tc_StartConvert( );
            if(ADC_tc_IsEndConversion(ADC_tc_WAIT_FOR_RESULT)) /* 1 indicates done */
            {
                tc2.lastValue = (int32) ADC_tc_GetResult16(); /*TODO: JW  raw value for now */
            }
  
            
            
            /* select Analog Mux location th_temp1 */
            ADC_th_StartConvert( );
            if(ADC_th_IsEndConversion(ADC_th_WAIT_FOR_RESULT)) /* 1 indicates done */
            {
                th1.lastValue = (int32) ADC_th_GetResult16(); /*TODO: JW  raw value for now */
            }
     
            /* select Analog Mux location th_temp2 */
            ADC_th_StartConvert( );
            if(ADC_th_IsEndConversion(ADC_th_WAIT_FOR_RESULT)) /* 1 indicates done */
            {
                th2.lastValue = (int32) ADC_th_GetResult16(); /*TODO: JW  raw value for now */
            }
            
#endif         
            
            /* run control algorithms and set outputs */
            /* send response with status*/
         
        }
         
      
        /* Sends the simulated live value via CAN */

  //        SendU56((uint8) 0x01, CAN_TX_MAILBOX_0,(uint64) 0x0002010201020102); /* 7 bytes data plus functionID */
 
  
        
        if( txQStart != txQEnd )
        {
            TransmitQItem(CAN_TX_MAILBOX_0);      
        }
        
        ++processingStep;
        if(processingStep >= MAX_PROCESSING_STEP)
        {
            processingStep = 0;
        }
        CyDelay(100u);
    } /*end of forever loop */
}
/*******************************************************************************
* Function Name:  CAN_ReceiveMsg_0_Callback
********************************************************************************
*
* Summary:
*   
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void CAN_ReceiveMsg_0_Callback()
{
    /* check for queue full condition */
    
    uint8 tempIdx = rxQEnd + 1;
    tempIdx = (tempIdx >= MAX_RXQ_SIZE)? 0 : tempIdx;
    if(tempIdx != rxQStart) /* receive if not full */
    {

        /* Set the isrFlag */
        isrFlag = 1u;  
        rxQ[rxQEnd].u8FunctionID = CAN_RX_DATA_BYTE1(CAN_RX_MAILBOX_0);
        rxQ[rxQEnd].mailbox = (uint8) CAN_RX_MAILBOX_0;
        rxQ[rxQEnd].data[0] = CAN_RX_DATA_BYTE2(CAN_RX_MAILBOX_0);
        rxQ[rxQEnd].data[1] = CAN_RX_DATA_BYTE3(CAN_RX_MAILBOX_0);
        rxQ[rxQEnd].data[2] = CAN_RX_DATA_BYTE4(CAN_RX_MAILBOX_0);
        rxQ[rxQEnd].data[3] = CAN_RX_DATA_BYTE5(CAN_RX_MAILBOX_0);
        rxQ[rxQEnd].data[4] = CAN_RX_DATA_BYTE6(CAN_RX_MAILBOX_0);
        rxQ[rxQEnd].data[5] = CAN_RX_DATA_BYTE7(CAN_RX_MAILBOX_0);
        rxQ[rxQEnd].data[6] = CAN_RX_DATA_BYTE8(CAN_RX_MAILBOX_0);
        incEnd(false, CAN_RX_MAILBOX_0); /* try to increment rxQEnd */
    }
 

}

/*******************************************************************************
* Function Name:  
********************************************************************************
*
* Summary:
*   
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/

/*******************************************************************************
* Function Name: incStart 
********************************************************************************
*
* Summary: adjusts queue indices after queue item has been used
*   
*
* Parameters:
*  None.
*
* Return:
*  int8 0  queue empty xxQStart== xxQEnd , else 1
*
*******************************************************************************/
static int8 incStart(bool bTxQ, uint8 mailbox)
{

    uint8 tempIdx = 0;

    /*TODO: JW add support for  more than one mailbox */
    if(bTxQ)
    {
        tempIdx = txQStart + 1;
        txQStart = (tempIdx >= MAX_TXQ_SIZE)? 0 : tempIdx;
        return( (txQStart == txQEnd)? 0 :1 );
    }
    else
    {
        tempIdx = rxQStart + 1;
        rxQStart = (tempIdx >= MAX_RXQ_SIZE)? 0 : tempIdx;
        return( (rxQStart == rxQEnd)? 0 :1 );
    }
       
}
/*******************************************************************************
* Function Name: incEnd 
********************************************************************************
*
* Summary: attempts to adjust queue indices for next queue location
*   
*
* Parameters:
*  None.
*
* Return:
*  int8 TX_Q_OK, RX_Q_OK or TX_Q_FULL,RX_Q_FULL  not able queue message
*
*******************************************************************************/
static int8 incEnd(bool bTxQ, uint8 mailbox)
{
    uint8 tempIdx = 0;
    int8 i8RetVal = TX_Q_FULL;

    /*TODO: JW add support for  more than one mailbox */
    if(bTxQ)
    {
        tempIdx = txQEnd + 1;
        tempIdx = (tempIdx >= MAX_TXQ_SIZE)? 0 : tempIdx;
        if(tempIdx != txQStart) /* not an overwrite, room left */
        {
            txQEnd = tempIdx;
            i8RetVal = TX_Q_OK;
        }
 
    }
    else
    {
        i8RetVal = RX_Q_FULL;
        tempIdx = rxQEnd + 1;
        tempIdx = (tempIdx >= MAX_RXQ_SIZE)? 0 : tempIdx;
        if(tempIdx != rxQStart) /* not an overwrite, room left */
        {
            rxQEnd = tempIdx;
            i8RetVal = RX_Q_OK;
        }
    }
   
    return(i8RetVal);
}

/*******************************************************************************
* Function Name:  CAN_MsgTXIsr_Callback
********************************************************************************
*
* Summary:
*   
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void CAN_MsgTXIsr_Callback()
{
    if( txQStart != txQEnd )
    {
        TransmitQItem(CAN_TX_MAILBOX_0);      
    }
}
/*******************************************************************************
* Function Name: TransmitQItem 
********************************************************************************
*
* Summary: assumes mailbox is ready for another message to send
*   
*
* Parameters:
*  unit8 qIndex
*
* Return:
*  int8 1 if 
*
*******************************************************************************/
static int8 TransmitQItem(uint8 mailbox)  
{
    /* TODO: JW support multiple mailboxes */
    
    uint8 txMailbox = txQ[txQStart].mailbox;
    CAN_TX_DATA_BYTE1(txMailbox) = txQ[txQStart].u8FunctionID;
    CAN_TX_DATA_BYTE2(txMailbox) = txQ[txQStart].data[0];
    CAN_TX_DATA_BYTE3(txMailbox) = txQ[txQStart].data[1];
    CAN_TX_DATA_BYTE4(txMailbox) = txQ[txQStart].data[2];
    CAN_TX_DATA_BYTE5(txMailbox) = txQ[txQStart].data[3];
    CAN_TX_DATA_BYTE6(txMailbox) = txQ[txQStart].data[4];
    CAN_TX_DATA_BYTE7(txMailbox) = txQ[txQStart].data[5];
    CAN_TX_DATA_BYTE8(txMailbox) = txQ[txQStart].data[6];
    CAN_SendMsg0();
    incStart(true, txMailbox); /* increment start to next position */
 
    
    return((txQStart == txQEnd)? 0: TX_Q_OK);
    
}

/*******************************************************************************
* Function Name: processIncomingMessage 
********************************************************************************
*
* Summary: develops functionID from first byte of data, interprets accordingly
*   
*          may send response message
*
* Parameters:
*  uint8 rxmailbox - mailbox 0-15
*
* Return:
*  int8  i8Response  
*         0  valid function - no response
*         TX_Q_OK  valid function - sent response
*         RX_FUNCTION_ID_INVALID  not a valid function
*         RX_MAILBOX_INVALID  not a valid mailbox 
*         TX_Q_FULL  not able to send response, tx queue full
*
*******************************************************************************/
static int8 processIncomingMessage(uint8 rxMailbox)
{
        int functionID = 0;
        int8 i8Response = (int8) RX_RESPONSE_NONE;
        uint8 byte1 = 0;
        uint8 byte2 = 0;
        uint8 byte3 = 0;
        uint8 byte4 = 0;

 

 #ifdef KIT50_VERSION   
        char8 lcdString[20]; 
        
        LCD_Position(0u, 3u);
        sprintf(lcdString, "%x   ",(int) rxQ[rxQStart].u8FunctionID);
        LCD_PrintString((const char8 *)lcdString);
 #endif           
            /* switch on message type */
            switch((int) rxQ[rxQStart].u8FunctionID)
            {
                
                case SET_NODE_ID:
                    nodeID = (((uint32)rxQ[rxQStart].data[0]) << 8) + ((uint32)rxQ[rxQStart].data[1]);

                    MB0_Fly_Config.rxacr = nodeID << CAN_SET_TX_ID_STANDARD_MSG_SHIFT;
                    CAN_RxBufConfig((const CAN_RX_CFG *)(&MB0_Fly_Config));
                    CY_SET_REG32(CAN_TX_ID_PTR(CAN_TX_MAILBOX_0), (uint32)nodeID << CAN_SET_TX_ID_STANDARD_MSG_SHIFT);
                    CY_SET_REG32(CAN_RX_ID_PTR(rxMailbox), (uint32)nodeID << CAN_SET_TX_ID_STANDARD_MSG_SHIFT);

                    nodeID = CY_GET_REG32(CAN_RX_ID_PTR(rxMailbox)) >> CAN_SET_TX_ID_STANDARD_MSG_SHIFT;

                    EEPROM_UpdateTemperature();
                    EEPROM_WriteByte(nodeID >> 8, NODE_ID_REG2);
                    EEPROM_WriteByte(nodeID, NODE_ID_REG1);

                    nodeID = (EEPROM_ReadByte(NODE_ID_REG2) << 8) + EEPROM_ReadByte(NODE_ID_REG1);
                    
                    i8Response = TX_Q_FULL;
    
                    uint8 tempIdx = txQEnd + 1;
                    tempIdx = (tempIdx >= MAX_TXQ_SIZE)? 0 : tempIdx;
                    if(tempIdx != txQStart) /* transmit if not full */
                    {
                        i8Response = TX_Q_OK;
                        txQ[txQEnd].u8FunctionID = 0x00;
                        txQ[txQEnd].mailbox = (uint8) CAN_TX_MAILBOX_0;
                        txQ[txQEnd].data[0] = (uint8)(nodeID >> 16);
                        txQ[txQEnd].data[1] = (uint8)(nodeID >> 8);
                        txQ[txQEnd].data[2] = (uint8)nodeID;
                        txQ[txQEnd].data[3] = 0x00;
                        txQ[txQEnd].data[4] = 0x33;
                        txQ[txQEnd].data[5] = (uint8)(nodeID >> 8);
                        txQ[txQEnd].data[6] = (uint8)nodeID;
                        incEnd(true, CAN_TX_MAILBOX_0); /* try to increment txQEnd */
                    }
                    break;
                    
                case SET_TC_TEMP1_SETPOINT:
                    tc1.setpoint = (int32) ReceiveU32ToEEPROM(rxMailbox, TC_TEMP1_SETPOINT);
                    break;
                case SET_TC_TEMP1:
                    /* diagnostic */
                    tc1.lastValue = (int32) ReceiveU32(rxMailbox);
                    break;   
                case SET_TC_TEMP1_VREF:
                    tc1.lastVRef = (uint8) ReceiveU8(rxMailbox);
                    break;
                case SET_TC_TEMP1_HIGH_ALARM:
                    tc1.highAlarm = (int32) ReceiveU32ToEEPROM(rxMailbox, TC_TEMP1_HIGH_ALARM);
                    break;
                case SET_TC_TEMP1_LOW_ALARM:
                    tc1.lowAlarm = (int32) ReceiveU32ToEEPROM(rxMailbox, TC_TEMP1_LOW_ALARM);
                    break;
                case SET_TC_TEMP1_ALARM_HYS:
                    tc1.alarmHysteresis = (int32) ReceiveU32ToEEPROM(rxMailbox, TC_TEMP1_ALARM_HYS);
                    break;
                    
                case GET_TC_TEMP1: /*TODO: JW send two responses as a test, just one later */
                    i8Response = SendU32((uint8) GET_TC_TEMP1,(uint8) CAN_TX_MAILBOX_0, tc1.lastValue);
                    if(TX_Q_OK == i8Response)
                    {
                        i8Response = SendU32((uint8)GET_TC_TEMP1_VREF,(uint8) CAN_TX_MAILBOX_0, tc1.lastVRef);
                    }
                    break;
                    
                case GET_TC_TEMP1_VREF:
                    i8Response = SendU8((uint8)GET_TC_TEMP1_VREF,(uint8) CAN_TX_MAILBOX_0, tc1.lastVRef);
                    break;
                    
                case SET_TC_TEMP2_SETPOINT:
                    tc2.setpoint = (int32) ReceiveU32ToEEPROM(rxMailbox, TC_TEMP2_SETPOINT);
                    break;
                case SET_TC_TEMP2:
                    /* diagnostic */
                    tc2.lastValue = (int32) ReceiveU32(rxMailbox);
                    break;   
                case SET_TC_TEMP2_VREF:
                    tc2.lastVRef = (uint8) ReceiveU8(rxMailbox);
                    break;
                case SET_TC_TEMP2_HIGH_ALARM:
                    tc2.highAlarm = (int32) ReceiveU32ToEEPROM(rxMailbox, TC_TEMP2_HIGH_ALARM);
                    break;
                case SET_TC_TEMP2_LOW_ALARM:
                    tc2.lowAlarm = (int32) ReceiveU32ToEEPROM(rxMailbox, TC_TEMP2_LOW_ALARM);
                    break;
                case SET_TC_TEMP2_ALARM_HYS:
                    tc2.alarmHysteresis = (int32) ReceiveU32ToEEPROM(rxMailbox, TC_TEMP2_ALARM_HYS);
                    break;
                case GET_TC_TEMP2:
                    i8Response = SendU32((uint8) GET_TC_TEMP2,(uint8) CAN_TX_MAILBOX_0, tc2.lastValue);
                    break;
                case GET_TC_TEMP2_VREF:
                    i8Response = SendU8((uint8)GET_TC_TEMP2_VREF,(uint8) CAN_TX_MAILBOX_0, tc2.lastVRef);
                    break;
                    
                case SET_TH_TEMP1_SETPOINT:
                    th1.setpoint = (int32) ReceiveU32ToEEPROM(rxMailbox, TH_TEMP1_SETPOINT);
                    break;
                case SET_TH_TEMP1:
                    /* diagnostic */
                    th1.lastValue = (int32) ReceiveU32(rxMailbox);
                    break;   
                case SET_TH_TEMP1_HIGH_ALARM:
                    th1.highAlarm = (int32) ReceiveU32ToEEPROM(rxMailbox, TH_TEMP1_HIGH_ALARM);
                    break;
                case SET_TH_TEMP1_LOW_ALARM:
                    th1.lowAlarm = (int32) ReceiveU32ToEEPROM(rxMailbox, TH_TEMP1_LOW_ALARM);
                    break;
                case SET_TH_TEMP1_ALARM_HYS:
                    th1.alarmHysteresis = (int32) ReceiveU32ToEEPROM(rxMailbox, TC_TEMP1_ALARM_HYS);
                    break;
                case GET_TH_TEMP1:
                    i8Response = SendU32((uint8) GET_TH_TEMP1,(uint8) CAN_TX_MAILBOX_0, th1.lastValue);
                    break;

                 case SET_TH_TEMP2_SETPOINT:
                    th2.setpoint = (int32) ReceiveU32ToEEPROM(rxMailbox, TH_TEMP2_SETPOINT);
                    break;
                case SET_TH_TEMP2:
                    /* diagnostic */
                    th2.lastValue = (int32) ReceiveU32(rxMailbox);
                    break;   
                case SET_TH_TEMP2_HIGH_ALARM:
                    th2.highAlarm = (int32) ReceiveU32ToEEPROM(rxMailbox, TH_TEMP2_HIGH_ALARM);
                    break;
                case SET_TH_TEMP2_LOW_ALARM:
                    th2.lowAlarm = (int32) ReceiveU32ToEEPROM(rxMailbox, TH_TEMP2_LOW_ALARM);
                    break;
                case SET_TH_TEMP2_ALARM_HYS:
                    th2.alarmHysteresis = (int32) ReceiveU32ToEEPROM(rxMailbox, TH_TEMP2_ALARM_HYS);
                    break;
                case GET_TH_TEMP2:
                    i8Response = SendU32((uint8) GET_TH_TEMP2,(uint8) CAN_TX_MAILBOX_0, th2.lastValue);
                    break;



                case SET_PELTIER_ON:
                    if(0 != ReceiveU8(rxMailbox))
                    {
                        ePeltier.peltierStatus |= (uint8) PELTIER_ON_BIT;
                        if(ePeltier.peltierStatus & (uint8) PELTIER_DIR_BIT) /* heat */
                        {
                            ePeltier.peltierStatus |= (uint8) PELTIER_P_IN_BIT;
                            ePeltier.peltierStatus &= ~((uint8) PELTIER_N_IN_BIT);
#ifndef KIT50_VERSION  
                            tec_hb_p_in_Write(1);
                            tec_hb_n_in_Write(0); 
#endif
                        }
                        else /* cool */
                        {
                            ePeltier.peltierStatus &= ~((uint8) PELTIER_P_IN_BIT);                                                       
                            ePeltier.peltierStatus |= (uint8) PELTIER_N_IN_BIT;
#ifndef KIT50_VERSION  
                            tec_hb_p_in_Write(0);
                            tec_hb_n_in_Write(1);
#endif                            
                        }

                        ePeltier.peltierStatus &= ~((uint8) PELTIER_P_INHIBIT_BIT + (uint8) PELTIER_N_INHIBIT_BIT);
#ifndef KIT50_VERSION                         
                        tec_hb_p_inh_Write(0);
                        tec_hb_n_inh_Write(0);
#endif
                    }
                    else
                    {
                        ePeltier.peltierStatus &= ~((uint8) PELTIER_ON_BIT);
                        ePeltier.peltierStatus &= ~((uint8) PELTIER_P_IN_BIT + (uint8) PELTIER_N_IN_BIT);
                        ePeltier.peltierStatus |= ((uint8) PELTIER_P_INHIBIT_BIT + (uint8) PELTIER_N_INHIBIT_BIT);
#ifndef KIT50_VERSION                         
                        tec_hb_p_in_Write(0);
                        tec_hb_n_in_Write(0);
                        tec_hb_p_inh_Write(1);
                        tec_hb_n_inh_Write(1);
#endif
                    }
                    break;
                    
                case SET_PELTIER_DIRECTION:
                    break;
                    
                case SET_PELTIER_ON_TIME:
                    ePeltier.peltierOnTime = ReceiveU16(rxMailbox);
                    /* check if peltier is off, turn  peltier on */
                    /* peltier time of zero will be checked at end of for loop and adjusted */
                    break;

                case GET_PELTIER_STATUS:
                    i8Response = SendU8((uint8) GET_PELTIER_STATUS,(uint8) CAN_TX_MAILBOX_0, ePeltier.peltierStatus);
                    break;
                    
                case GET_PELTIER_P_SENSE_DIAG:
                    i8Response = SendU16((uint8) GET_PELTIER_P_SENSE_DIAG,(uint8) CAN_TX_MAILBOX_0, (uint16) ePeltier.peltierFaultCurrentP);
                    break;
                    
                case GET_PELTIER_N_SENSE_DIAG:
                    i8Response = SendU16((uint8) GET_PELTIER_N_SENSE_DIAG,(uint8) CAN_TX_MAILBOX_0, (uint16) ePeltier.peltierFaultCurrentN);
                    break;
                    

                /* diagnostic, for pass through motor control */
                case SET_PELTIER_P_INHIBIT:
                    if(0 != ReceiveU8(rxMailbox))
                    {
                        ePeltier.peltierStatus |= (uint8) PELTIER_P_INHIBIT_BIT;
#ifndef KIT50_VERSION
                        tec_hb_p_inh_Write(1);
#endif                        
                    }
                    else
                    {
                        ePeltier.peltierStatus &= ~((uint8) PELTIER_P_INHIBIT_BIT);
#ifndef KIT50_VERSION
                        tec_hb_p_inh_Write(0);
#endif    
                    }
                    break;
                case SET_PELTIER_P_IN:
                    if(0 != ReceiveU8(rxMailbox))
                    {
                        ePeltier.peltierStatus |= (uint8) PELTIER_P_IN_BIT;
#ifndef KIT50_VERSION                        
                       tec_hb_p_in_Write(1);
#endif
                    }
                    else
                    {
                        ePeltier.peltierStatus &= ~((uint8) PELTIER_P_IN_BIT);
 //                       tec_hb_p_in_Write(0);
                    }
                    break;
                case SET_PELTIER_P_SENSE_DIAG:
                    ePeltier.peltierFaultCurrentP = (int16) ReceiveU16(rxMailbox);
                    break;
                case SET_PELTIER_N_INHIBIT:
                    if(0 != ReceiveU8(rxMailbox))
                    {
                        ePeltier.peltierStatus |= (uint8) PELTIER_N_INHIBIT_BIT;
#ifndef KIT50_VERSION                        
                        tec_hb_n_inh_Write(1);
#endif    
                    }
                    else
                    {
                        ePeltier.peltierStatus &= ~((uint8) PELTIER_N_INHIBIT_BIT);
#ifndef KIT50_VERSION
                        tec_hb_n_inh_Write(0);
#endif
                    }
                    break;
                case SET_PELTIER_N_IN:
                    if(0 != ReceiveU8(rxMailbox))
                    {
                        ePeltier.peltierStatus |= (uint8) PELTIER_N_IN_BIT;
#ifndef KIT50_VERSION                        
                        tec_hb_n_in_Write(1);
#endif    
                    }
                    else
                    {
                        ePeltier.peltierStatus &= ~((uint8) PELTIER_N_IN_BIT);
#ifndef KIT50_VERSION                        
                        tec_hb_n_in_Write(0);
#endif
                    }
                    break;
                case SET_PELTIER_N_SENSE_DIAG:
                    ePeltier.peltierFaultCurrentN = (int16) ReceiveU16(rxMailbox);
                    break;
                
                /* end diagnostic for motor control */  


                /* fan related */
                case SET_FAN_ENABLE:
                    eFan.fanEnable = (0 == ReceiveU8(rxMailbox))? false : true;
#ifndef KIT50_VERSION
                    if(eFan.fanEnable)
                    {
                        fan_Write(1);
                        PWM_fan_Start();
                    }
                    else
                    {
                        PWM_fan_Stop();
                        fan_Write(0);
                    }
#endif                    
                    break;
                    
                case GET_FAN_ENABLE:
                    i8Response = SendU8((uint8) GET_FAN_ENABLE,(uint8) CAN_TX_MAILBOX_0, (uint8) ((eFan.fanEnable)? 1:0) );
                    break;

                case SET_FAN_PWM:
                    eFan.fanDutyCycle = ReceiveU8(rxMailbox); /* later, adapt to control output percentage */
#ifndef KIT50_VERSION                    
                    PWM_fan_WriteCounter(eFan.fanDutyCycle);
#endif                    
                    break;
                    
                case GET_FAN_PWM:
                    i8Response = SendU8((uint8) GET_FAN_PWM,(uint8) CAN_TX_MAILBOX_0, eFan.fanDutyCycle );
                    break;

                /* incremental(velocity form) control related */
                case SET_CONTROL1_KP:
                    break;
                case SET_CONTROL1_KI:
                    break;
                case SET_CONTROL1_KD:
                    break;
                case SET_CONTROL1_M_MAX:
                    break;
                /* diagnostic */
                case GET_CONTROL1_DELTAP:
                    break;
                case GET_CONTROL1_DELTAI:
                    break;
                case GET_CONTROL1_DELTAD:
                    break;
                case GET_CONTROL1_DELTAM:
                    break;
                case GET_CONTROL1_ERROR:
                    break;
                case GET_CONTROL1_PREV_ERROR:
                    break;
                case GET_CONTROL1_M:
                    break;

                case SET_CONTROL2_KP:
                    break;
                case SET_CONTROL2_KI:
                    break;
                case SET_CONTROL2_KD:
                    break;
                case SET_CONTROL2_M_MAX:
                    break;
                /* diagnostic */
                case GET_CONTROL2_DELTAP:
                    break;
                case GET_CONTROL2_DELTAI:
                    break;
                case GET_CONTROL2_DELTAD:
                    break;
                case GET_CONTROL2_DELTAM:
                    break;
                case GET_CONTROL2_ERROR:
                    break;
                case GET_CONTROL2_PREV_ERROR:
                    break;
                case GET_CONTROL2_M:
                    break;
                
                
                /* overall */
                case SET_EXTRUDER_SHUTDOWN:
                    break;
                case GET_EXTRUDER_SHUTDOWN:
                    break;
                

                case SET_EXTRUDER_REPORTING:
                    break;
                case GET_EXTRUDER_REPORTING:
                    break;
                default:
                    i8Response = (int8)RX_FUNCTION_ID_INVALID;
                    break;
            } /* end switch */
            
    return i8Response;
}
/*******************************************************************************
* Function Name:  SendU8
********************************************************************************
*
* Summary:
*   
*
* Parameters:
*  uint8 u8FunctionID, 8 bit version of functionID /message type
*  uint8 txmailbox
*  uint8 value - the value to send, can hold other data types such as int8
*
* Return:
*  int8       TX_Q_OK or TX_Q_FULL  not able to send response, tx queue full
*
*******************************************************************************/
static int8 SendU8(uint8 u8FunctionID,uint8 txMailbox, uint8 value)
{
    int8 i8RetVal = TX_Q_FULL;
    
    uint8 tempIdx = txQEnd + 1;
    tempIdx = (tempIdx >= MAX_TXQ_SIZE)? 0 : tempIdx;
    if(tempIdx != txQStart) /* transmit if not full */
    {
        i8RetVal = TX_Q_OK;

        txQ[txQEnd].u8FunctionID = (uint8) u8FunctionID;
        txQ[txQEnd].mailbox = txMailbox;
        txQ[txQEnd].data[0] = value;
        txQ[txQEnd].data[1] = 0x00;
        txQ[txQEnd].data[2] = 0x00;
        txQ[txQEnd].data[3] = 0x00;
        txQ[txQEnd].data[4] = 0x00;
        txQ[txQEnd].data[5] = 0x00;
        txQ[txQEnd].data[6] = 0x00;
        incEnd(true, CAN_TX_MAILBOX_0); /* try to increment txQEnd */
    }
    return(i8RetVal);

                  
}
/*******************************************************************************
* Function Name:  SendU16
********************************************************************************
*
* Summary:
*   
*
* Parameters:
*  uint8 u8FunctionID, 8 bit version of functionID /message type
*  uint8 txmailbox
*  uint16 value - the value to send, can hold other data types such as int16
*
* Return:
*  int8       TX_Q_OK or TX_Q_FULL  not able to send response, tx queue full
*
*******************************************************************************/
static int8 SendU16(uint8 u8FunctionID,uint8 txMailbox, uint16 value)
{
    int8 i8RetVal = TX_Q_FULL;
    
    uint8 tempIdx = txQEnd + 1;
    tempIdx = (tempIdx >= MAX_TXQ_SIZE)? 0 : tempIdx;
    if(tempIdx != txQStart) /* transmit if not full */
    {
        i8RetVal = TX_Q_OK;
        txQ[txQEnd].u8FunctionID = (uint8) u8FunctionID;
        txQ[txQEnd].mailbox = txMailbox;
        txQ[txQEnd].data[0] = (uint8) (value >> 8);
        txQ[txQEnd].data[1] = (uint8) value;
        txQ[txQEnd].data[2] = 0x00;
        txQ[txQEnd].data[3] = 0x00;
        txQ[txQEnd].data[4] = 0x00;
        txQ[txQEnd].data[5] = 0x00;
        txQ[txQEnd].data[6] = 0x00;
        incEnd(true, CAN_TX_MAILBOX_0); /* try to increment txQEnd */
    }
    return(i8RetVal);                
}

/*******************************************************************************
* Function Name:  SendU32
********************************************************************************
*
* Summary:
*   
*
* Parameters:
*  uint8 u8FunctionID, 8 bit version of functionID /message type
*  uint8 txmailbox
*  uint32 value - the value to send, can hold other data types such as int32
*
* Return:
*  int8       TX_Q_OK or TX_Q_FULL  not able to send response, tx queue full
*
*******************************************************************************/
static int8 SendU32(uint8 u8FunctionID,uint8 txMailbox, uint32 value)
{
    int8 i8RetVal = TX_Q_FULL;
    
    uint8 tempIdx = txQEnd + 1;
    tempIdx = (tempIdx >= MAX_TXQ_SIZE)? 0 : tempIdx;
    if(tempIdx != txQStart) /* transmit if not full */
    {
       i8RetVal = TX_Q_OK;
       txQ[txQEnd].u8FunctionID = (uint8) u8FunctionID;
       txQ[txQEnd].mailbox = txMailbox;
       txQ[txQEnd].data[0] = (uint8)(value >> 24);
       txQ[txQEnd].data[1] = (uint8)(value >> 16);
       txQ[txQEnd].data[2] = (uint8)(value >> 8);
       txQ[txQEnd].data[3] = (uint8)value;
       txQ[txQEnd].data[4] = 0x00;
       txQ[txQEnd].data[5] = 0x00;
       txQ[txQEnd].data[6] = 0x00;
       incEnd(true, CAN_TX_MAILBOX_0); /* try to increment txQEnd */
    }
    return(i8RetVal);    
                  
}
/*******************************************************************************
* Function Name:  SendU56
********************************************************************************
*
* Summary: can only hold 7 bytes of data plus functionID so 56 bits 
*   
*
* Parameters:
*  uint8 u8FunctionID, 8 bit version of functionID /message type
*  uint8 txmailbox
*  uint64 value - the value to send, can hold other data types minus top 8 bits
*
* Return:
*  int8       TX_Q_OK or TX_Q_FULL  not able to send response, tx queue full
*
*******************************************************************************/
static int8 SendU56(uint8 u8FunctionID,uint8 txMailbox, uint64 value)
{
    int8 i8RetVal = TX_Q_FULL;
    
    uint8 tempIdx = txQEnd + 1;
    tempIdx = (tempIdx >= MAX_TXQ_SIZE)? 0 : tempIdx;
    if(tempIdx != txQStart) /* transmit if not full */
    {
        i8RetVal = TX_Q_OK;
        txQ[txQEnd].u8FunctionID = (uint8) u8FunctionID;
        txQ[txQEnd].mailbox = txMailbox;
        txQ[txQEnd].data[0] = (uint8)(value >> 48);
        txQ[txQEnd].data[1] = (uint8)(value >> 40);
        txQ[txQEnd].data[2] = (uint8)(value >> 32);
        txQ[txQEnd].data[3] = (uint8)(value >> 24);
        txQ[txQEnd].data[4] = (uint8)(value >> 16);
        txQ[txQEnd].data[5] = (uint8)(value >> 8);
        txQ[txQEnd].data[6] = (uint8)value;
        incEnd(true, CAN_TX_MAILBOX_0); /* try to increment txQEnd */
    }
    return(i8RetVal);                
}
/*******************************************************************************
* Function Name:  ReceiveU8
********************************************************************************
*
* Summary: assumes received uint8 in mailbox starting at CAN_RX_DATA_BYTE2
*           
*   
*         Note: this method can also facilitate int32 transfers
* Parameters:
*  uint8 mailbox number
*
* Return:
*  uint8 received value
*
*******************************************************************************/
static uint8 ReceiveU8(uint8 rxMailbox)
{
    /* TODO: JW handle multiple mailboxes */
    uint8 u8RetVal = (uint8)rxQ[rxQStart].data[0];

    
    
    return (u8RetVal);
}
/*******************************************************************************
* Function Name:  ReceiveU16
********************************************************************************
*
* Summary: assumes received uint8 in mailbox starting at CAN_RX_DATA_BYTE2
*           
*   
*         Note: this method can also facilitate int32 transfers
* Parameters:
*  uint8 mailbox number
*
* Return:
*  uint16 received value
*
*******************************************************************************/
static uint16 ReceiveU16(uint8 rxMailbox)
{
   
    /* TODO: JW handle multiple mailboxes  */
    uint16 u16RetVal = ((uint16)rxQ[rxQStart].data[0] << 8) + ((uint16)rxQ[rxQStart].data[1]);
 
    return (u16RetVal);
}
/*******************************************************************************
* Function Name:  ReceiveU32
********************************************************************************
*
* Summary: assumes received uint32 in mailbox starting at CAN_RX_DATA_BYTE2
*           
*   
*         Note: this method can also facilitate int32 transfers
* Parameters:
*  uint8 mailbox number
*
* Return:
*  uint32 received value
*
*******************************************************************************/
static uint32 ReceiveU32(uint8 rxMailbox)
{
    uint32 u32RetVal= 0;
    /* TODO: JW handle multiple mailboxes */
 
    u32RetVal = ( ((uint32)rxQ[rxQStart].data[0]) << 24 ) + ( ((uint32)rxQ[rxQStart].data[1]) << 16);
    u32RetVal += ( ((uint32)rxQ[rxQStart].data[2]) << 8 ) + ((uint32)rxQ[rxQStart].data[3]);
    

    
    return u32RetVal;
}
/*******************************************************************************
* Function Name:  ReceiveU32ToEEPROM
********************************************************************************
*
* Summary: assumes received uint32 in mailbox starting at CAN_RX_DATA_BYTE2
*          writes 4 received bytes in reverse order at EEPROM offset of address 
*   
*         Note: this method can also facilitate int32 transfers
* Parameters:
*  uint8 mailbox number
*  uint16 address (offset in EEPROM for byte4)
*
* Return:
*  uint32 received value
*
*******************************************************************************/
static uint32 ReceiveU32ToEEPROM(uint8 rxMailbox, uint16 address)
{
    uint32 u32RetVal= 0;
 
    /* TODO: JW handle multiple mailboxes */  
    
    u32RetVal =  ( ((uint32)rxQ[rxQStart].data[0]) << 24 ) + ( ((uint32)rxQ[rxQStart].data[1]) << 16);
    u32RetVal += ( ((uint32)rxQ[rxQStart].data[2]) << 8 ) + ((uint32)rxQ[rxQStart].data[3]);
    
    
    EEPROM_UpdateTemperature();
    EEPROM_WriteByte(rxQ[rxQStart].data[0], address+4); //TODO: optimize, since row write anyway
    EEPROM_WriteByte(rxQ[rxQStart].data[1], address+2);
    EEPROM_WriteByte(rxQ[rxQStart].data[2], address+1);
    EEPROM_WriteByte(rxQ[rxQStart].data[3], address);
    
 
    
    return u32RetVal;
}
/*******************************************************************************
* Function Name:  ReceiveF32ToEEPROM
********************************************************************************
*
* Summary: assumes received float32 in mailbox starting at CAN_RX_DATA_BYTE2
*          writes 4 received bytes in reverse order at EEPROM offset of address 
*   
*         Note: this method can also facilitate int32 transfers
* Parameters:
*  uint8 mailbox number
*  uint16 address (offset in EEPROM for byte4)
*
* Return:
*  float32 received value
*
*******************************************************************************/
static float32 ReceiveF32ToEEPROM(uint8 rxMailbox, uint16 address)
{
    float32 f32RetVal= 0.0;
    uConvF32U32_t f32Conv;
 
    /* TODO: JW handle multiple mailboxes */  
    f32Conv.byte[3] =  rxQ[rxQStart].data[0];
    f32Conv.byte[2] =  rxQ[rxQStart].data[1];
    f32Conv.byte[1] =  rxQ[rxQStart].data[2];
    f32Conv.byte[0] =  rxQ[rxQStart].data[3];
       
    f32RetVal = f32Conv.f32Val;
    
    
    EEPROM_UpdateTemperature();
    EEPROM_WriteByte(rxQ[rxQStart].data[0], address+4); //TODO: optimize, since row write anyway
    EEPROM_WriteByte(rxQ[rxQStart].data[1], address+2);
    EEPROM_WriteByte(rxQ[rxQStart].data[2], address+1);
    EEPROM_WriteByte(rxQ[rxQStart].data[3], address);
    
 
    
    return f32RetVal;
}
/*******************************************************************************
* Function Name:  ReceiveU56
********************************************************************************
*
* Summary: assumes received uint32 in mailbox starting at CAN_RX_DATA_BYTE2
*           
*   
*         Note: this method can also facilitate int32 transfers
* Parameters:
*  uint8 mailbox number
*
* Return:
*  uint32 received value
*
*******************************************************************************/
static uint64 ReceiveU56(uint8 rxMailbox)
{
    uint64 u64RetVal= 0;
    /* TODO: JW handle multiple mailboxes, transition to circular queue */

    u64RetVal =  ( ((uint64)rxQ[rxQStart].data[0]) << 48 ) + ( ((uint64)rxQ[rxQStart].data[1]) << 40);
    u64RetVal += ( ((uint64)rxQ[rxQStart].data[2]) << 32 );
    u64RetVal += ( ((uint64)rxQ[rxQStart].data[3]) << 24 ) + ( ((uint64)rxQ[rxQStart].data[4]) << 16);
    u64RetVal += ( ((uint64)rxQ[rxQStart].data[5]) << 8 ) + ((uint64)rxQ[rxQStart].data[6]);
    

    
    return u64RetVal;
}

/* board support functions */
/*******************************************************************************
* Function Name:  InitAMuxs
********************************************************************************
*
* Summary:
*   
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
static void InitAMuxs(void)
{
#ifndef KIT50_VERSION    
   AMux_th_Init();
   AMux_tc_Init(); 
#endif
}

/*******************************************************************************
* Function Name:  InitADCs
********************************************************************************
*
* Summary:
*   
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
static void InitADCs(void)
{
#ifndef KIT50_VERSION
    ADC_th_Init();
    ADC_th_Enable();
    
    ADC_tc_Init();
    ADC_tc_Enable();    

#endif
    /*TODO: JW Add range and offset setup, save/restore config */
}
/*******************************************************************************
* Function Name:  InitVDACs
********************************************************************************
*
* Summary:
*   
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
static void InitVDACs(void)
{
#ifndef KIT50_VERSION
    VDAC_tc_vref1_Init();
    VDAC_tc_vref1_Enable();
    

 /* VDAC_tc_vref1_SetSpeed(uint8 speed);
    VDAC_tc_vref1_SetRange(uint8 range); */
    
    /* retrieve last stored value */
    VDAC_tc_vref1_SetValue((uint8) tc1.lastVRef);
    VDAC_tc_vref1_Start();

    
    VDAC_tc_vref2_Init();
    VDAC_tc_vref2_Enable(); 
    

 /* VDAC_tc_vref1_SetSpeed(uint8 speed);
    VDAC_tc_vref1_SetRange(uint8 range); */
    
    /* retrieve last stored value */
    VDAC_tc_vref2_SetValue((uint8) tc2.lastVRef);
    VDAC_tc_vref2_Start();    
#endif   
    /*TODO: JW Add speed, range setup, save/restore config */
}
/*******************************************************************************
* Function Name:  InitFanPWM
********************************************************************************
*
* Summary:
*   
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
static void InitFanPWM(void)
{
#ifndef KIT50_VERSION
    PWM_fan_Init();
#endif    
  
}
/*******************************************************************************
* Function Name:  InitControllers
********************************************************************************
*
* Summary:
*   
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
static void InitControllers(void)
{
   
}
/*******************************************************************************
* Function Name:  
********************************************************************************
*
* Summary:
*   
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
/*******************************************************************************
* Function Name:  
********************************************************************************
*
* Summary:
*   
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
/*******************************************************************************
* Function Name:  
********************************************************************************
*
* Summary:
*   
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
/*******************************************************************************
* Function Name:  
********************************************************************************
*
* Summary:
*   
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
/*******************************************************************************
* Function Name:  
********************************************************************************
*
* Summary:
*   
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
/*******************************************************************************
* Function Name:  
********************************************************************************
*
* Summary:
*   
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
/*******************************************************************************
* Function Name:  
********************************************************************************
*
* Summary:
*   
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
/*******************************************************************************
* Function Name:  
********************************************************************************
*
* Summary:
*   
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
/*******************************************************************************
* Function Name:  
********************************************************************************
*
* Summary:
*   
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
/*******************************************************************************
* Function Name:  
********************************************************************************
*
* Summary:
*   
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
/*******************************************************************************
* Function Name:  
********************************************************************************
*
* Summary:
*   
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
/*******************************************************************************
* Function Name:  
********************************************************************************
*
* Summary:
*   
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
/*******************************************************************************
* Function Name:  
********************************************************************************
*
* Summary:
*   
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
/*******************************************************************************
* Function Name:  
********************************************************************************
*
* Summary:
*   
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
/*******************************************************************************
* Function Name:  
********************************************************************************
*
* Summary:
*   
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
/*******************************************************************************
* Function Name:  
********************************************************************************
*
* Summary:
*   
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/

/* [] END OF FILE */
