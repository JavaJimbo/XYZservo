/***********************************************************************************************************
 * PROJECT:     XYZ Servo
 * FileName:    main.c
 * 
 * 
 * 7-4-18:      Adapted from PIC32MX795 Test. I-JOG command using SPEED CONTROL works swell.
 *              Also SET_POSITION_CONTROL works nicely.
 ************************************************************************************************************/
#ifndef MAIN_C
#define MAIN_C

#define MAXPOTS 1

#define true TRUE
#define false FALSE

#define SERVO_ID 0x01

#define uint8_t unsigned char
#define uint16_t unsigned short
#define  uint32_t unsigned long
#define int16_t short


#define CMD_EEPROM_WRITE 0x01
#define CMD_EEPROM_READ  0x02
#define CMD_RAM_WRITE    0x03
#define CMD_RAM_READ     0x04
#define CMD_I_JOG        0x05
#define CMD_S_JOG        0x06
#define CMD_STAT         0x07
#define CMD_ROLLBACK     0x08
#define CMD_REBOOT       0x09

#define SET_POSITION_CONTROL 0
#define SET_SPEED_CONTROL 1
#define SET_TORQUE_OFF 2
#define SET_POSITION_CONTROL_SERVO_ON 3


/** INCLUDES *******************************************************/
// #include <XC.h>
#include <plib.h>


#include "HardwareProfile.h"
#include "Delay.h"

#include "GenericTypeDefs.h"
#include "Compiler.h"

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

/** CONFIGURATION **************************************************/
#pragma config UPLLEN   = ON        // USB PLL Enabled
#pragma config FPLLMUL  = MUL_20        // PLL Multiplier $$$$
#pragma config UPLLIDIV = DIV_2         // USB PLL Input Divider
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider
#pragma config FPLLODIV = DIV_1         // PLL Output Divider
#pragma config FPBDIV   = DIV_1         // Peripheral Clock divisor
#pragma config FWDTEN   = OFF           // Watchdog Timer
#pragma config WDTPS    = PS1           // Watchdog Timer Postscale
#pragma config FCKSM    = CSDCMD        // Clock Switching & Fail Safe Clock Monitor
#pragma config OSCIOFNC = OFF           // CLKO Enable
#pragma config POSCMOD  = HS            // Primary Oscillator
#pragma config IESO     = OFF           // Internal/External Switch-over
#pragma config FSOSCEN  = OFF           // Secondary Oscillator Enable (KLO was off)
#pragma config FNOSC    = PRIPLL        // Oscillator Selection
#pragma config CP       = OFF           // Code Protect
#pragma config BWP      = OFF           // Boot Flash Write Protect
#pragma config PWP      = OFF           // Program Flash Write Protect
#pragma config ICESEL   = ICS_PGx1      // ICE/ICD Comm Channel Select
#pragma config DEBUG    = ON


/** PRIVATE PROTOTYPES *********************************************/
#define HOSTuart UART2
#define HOSTbits U2STAbits
#define HOST_VECTOR _UART_2_VECTOR


#define XBEEbits U4STAbits
#define XBEE_VECTOR _UART_4_VECTOR

#define DMXuart UART5
#define DMXbits U5STAbits
#define DMX_VECTOR _UART_5_VECTOR
#define MAXBUFFER 128
#define XYZuart UART4

unsigned char HOSTRxBuffer[MAXBUFFER];
unsigned char HOSTRxBufferFull = false;

unsigned short XBEERxLength = 0;
unsigned char XBEERxBuffer[MAXBUFFER];
unsigned short XBEETxLength = 0;
unsigned char XBEETxBuffer[MAXBUFFER];

unsigned short ADresult[MAXPOTS];
void ConfigAd(void);
void UserInit(void);
static void InitializeSystem(void) ;
unsigned char controlCommand = 0;
uint8_t id = SERVO_ID;

void sendRequest(uint8_t cmd,  const uint8_t * data1, uint8_t data1Size);
void sendIJog(uint16_t goal, uint8_t type, uint8_t playtime);
void setSpeed(int16_t speed);
void setPosition(uint16_t position, uint8_t playtime);

int main(void) {
    short i = 0;    
    unsigned char clockwise = true;
    short position = 0x0000;    
    uint8_t playtime = 0x20;
    InitializeSystem();

    printf("\r Testing XYZ SET POSITION");
    
    while(1)
    {
        DelayMs(1);               
        if (HOSTRxBufferFull)
        {
            HOSTRxBufferFull = false;
            if (clockwise) position = position + 100;
            else position = position - 100;            
            if (position > 1023)
            {
                clockwise = false;
                position = position - 200;
            }
            else if (position < 0) 
            {
                clockwise = true;
                position = position + 200;
            }
            setPosition(position, playtime);                                     
            printf("\rPosition: %d, playtime: %d", position, playtime);
        }
        
        if (XBEERxLength)
        {
            DelayMs(200);
            printf("\rRX: ");
            for (i = 0; i < XBEERxLength; i++)            
                printf("%02X, ", XBEERxBuffer[i]);
            XBEERxLength = 0;
        }        
    }    
}//end main

static void InitializeSystem(void) {
    AD1PCFG = 0xFFFF;
    SYSTEMConfigPerformance(60000000);

    UserInit();

}//end InitializeSystem

void ConfigAd(void) {

    mPORTBSetPinsAnalogIn(BIT_0); // $$$$

    // ---- configure and enable the ADC ----

    // ensure the ADC is off before setting the configuration
    CloseADC10();

    // define setup parameters for OpenADC10
    //                 Turn module on | ouput in integer | trigger mode auto | enable autosample
#define PARAM1  ADC_MODULE_ON | ADC_FORMAT_INTG | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON

    // A ref external    | disable offset test    | enable scan mode | perform  samples | use dual buffers | use only mux A
#define PARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_ON | ADC_SAMPLES_PER_INT_1 | ADC_ALT_BUF_ON | ADC_ALT_INPUT_OFF

    //                   use ADC internal clock | set sample time
    // #define PARAM3  ADC_CONV_CLK_INTERNAL_RC | ADC_SAMPLE_TIME_31
#define PARAM3  ADC_CONV_CLK_PB | ADC_SAMPLE_TIME_31 | ADC_CONV_CLK_32Tcy

    //  set AN2 (A2 on Olimex 220 board) input to analog
    // #define PARAM4    ENABLE_AN0_ANA | ENABLE_AN1_ANA| ENABLE_AN2_ANA | ENABLE_AN3_ANA
#define PARAM4    ENABLE_AN3_ANA

    // USE AN2
#define PARAM5 SKIP_SCAN_AN0 | SKIP_SCAN_AN1 | SKIP_SCAN_AN2 |\
SKIP_SCAN_AN4 | SKIP_SCAN_AN5 | SKIP_SCAN_AN6 | SKIP_SCAN_AN7 |\
SKIP_SCAN_AN8 | SKIP_SCAN_AN9 | SKIP_SCAN_AN10 | SKIP_SCAN_AN11 |\
SKIP_SCAN_AN12 | SKIP_SCAN_AN13 | SKIP_SCAN_AN14 | SKIP_SCAN_AN15

    // set negative reference to Vref for Mux A
    SetChanADC10(ADC_CH0_NEG_SAMPLEA_NVREF);

    // open the ADC
    OpenADC10(PARAM1, PARAM2, PARAM3, PARAM4, PARAM5);

    ConfigIntADC10(ADC_INT_PRI_2 | ADC_INT_SUB_PRI_2 | ADC_INT_ON);

    // clear the interrupt flag
    mAD1ClearIntFlag();

    // Enable the ADC
    EnableADC10();
}


void __ISR(_ADC_VECTOR, ipl6) ADHandler(void) {
    unsigned short offSet;
    unsigned char i;

    mAD1IntEnable(INT_DISABLED);
    mAD1ClearIntFlag();

    // Determine which buffer is idle and create an offset
    offSet = 8 * ((~ReadActiveBufferADC10() & 0x01));

    for (i = 0; i < MAXPOTS; i++)
        ADresult[i] = (unsigned short) ReadADC10(offSet + i); // read the result of channel 0 conversion from the idle buffer
}

void UserInit(void) {
    // Turn off JTAG so we get the pins back
    mJTAGPortEnable(false);

    //Initialize all of the LED pins
    mInitAllLEDs();

    //Initialize all of the push buttons
    mInitAllSwitches();

    //PORTSetPinsDigitalIn(IOPORT_C, BIT_14 | BIT_13);
    //mCNOpen(CN_ON, CN0_ENABLE | CN1_ENABLE, CN0_PULLUP_ENABLE | CN1_PULLUP_ENABLE);
    //ConfigIntCN(CHANGE_INT_ON | CHANGE_INT_PRI_2);

    // Set up main UART
    UARTConfigure(HOSTuart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(HOSTuart, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(HOSTuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
#define SYS_FREQ 80000000
    UARTSetDataRate(HOSTuart, SYS_FREQ, 57600);
    UARTEnable(HOSTuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure UART #2 Interrupts
    INTEnable(INT_U2TX, INT_DISABLED);
    INTEnable(INT_SOURCE_UART_RX(HOSTuart), INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(HOSTuart), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(HOSTuart), INT_SUB_PRIORITY_LEVEL_0);

    // Set up XBEE at 115200 baud
    UARTConfigure(XYZuart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(XYZuart, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(XYZuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(XYZuart, SYS_FREQ, 115200);
    UARTEnable(XYZuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure XBEE Interrupts
    INTEnable(INT_SOURCE_UART_TX(XYZuart), INT_DISABLED);
    INTEnable(INT_SOURCE_UART_RX(XYZuart), INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(XYZuart), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(XYZuart), INT_SUB_PRIORITY_LEVEL_0);

    /*
    // Set up MIDI at 31250 baud
    UARTConfigure(MIDIuart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(MIDIuart, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(MIDIuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(MIDIuart, SYS_FREQ, 31250);
    UARTEnable(MIDIuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure MIDI Interrupts
    INTEnable(INT_SOURCE_UART_TX(MIDIuart), INT_DISABLED);
    INTEnable(INT_SOURCE_UART_RX(MIDIuart), INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(MIDIuart), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(MIDIuart), INT_SUB_PRIORITY_LEVEL_0);

    // Set up DMX512 UART @ 25000 baud   	
    UARTConfigure(DMXuart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(DMXuart, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(DMXuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_2);
    UARTSetDataRate(DMXuart, SYS_FREQ, 250000);
    UARTEnable(DMXuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure DMX interrupts
    INTEnable(INT_SOURCE_UART_RX(DMXuart), INT_ENABLED);
    INTEnable(INT_SOURCE_UART_TX(DMXuart), INT_DISABLED);
    INTSetVectorPriority(INT_VECTOR_UART(DMXuart), INT_PRIORITY_LEVEL_1);
    INTSetVectorSubPriority(INT_VECTOR_UART(DMXuart), INT_SUB_PRIORITY_LEVEL_0);

    PORTSetPinsDigitalOut(IOPORT_G, BIT_0);
    PORTClearBits(IOPORT_G, BIT_0); // RS485 control pin should be set to receive

    PORTSetPinsDigitalIn(IOPORT_C, BIT_14 | BIT_13);
    PORTSetPinsDigitalOut(IOPORT_B, BIT_2);
    PORTSetBits(IOPORT_B, BIT_2);
    */
    
    // Set up Port C outputs:
    //PORTSetPinsDigitalOut(IOPORT_C, BIT_3);
    //PORTSetBits(IOPORT_C, BIT_3);

    // Set up Port E outputs:
    //PORTSetPinsDigitalOut(IOPORT_E, BIT_8);
    //PORTSetBits(IOPORT_E, BIT_8);


    ConfigAd();

    // Set up Timer 2 for 100 microsecond roll-over rate
    //OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_64, 1250);
    // set up the core timer interrupt with a priority of 5 and zero sub-priority
    //ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_5);

    //PORTSetPinsDigitalOut(IOPORT_D, BIT_0 | BIT_1 | BIT_2 | BIT_3 | BIT_4 | BIT_5 | BIT_6 | BIT_7);
    //PORTWrite(IOPORT_D, 0x0000);

    // Turn on the interrupts
    INTEnableSystemMultiVectoredInt();

}//end UserInit

void __ISR(_TIMER_2_VECTOR, ipl5) Timer2Handler(void) {

    mT2ClearIntFlag(); // clear the interrupt flag

}


unsigned char getCommand(unsigned char *ptrString) {
    unsigned char ch = 0;
    short i = 0;

    while (ptrString[i] != NULL && i < MAXBUFFER) {
        ch = ptrString[i];
        if (isalpha(ch)) return (ch);
    }
    return (0);
}

#define ENTER 13
#define BACKSPACE 8
// HOST UART interrupt handler it is set at priority level 2

void __ISR(HOST_VECTOR, ipl2) IntHostUartHandler(void) {
    unsigned char ch;
    static unsigned short HOSTRxIndex = 0;

    if (HOSTbits.OERR || HOSTbits.FERR) {
        if (UARTReceivedDataIsAvailable(HOSTuart))
            ch = UARTGetDataByte(HOSTuart);
        HOSTbits.OERR = 0;
        HOSTRxIndex = 0;
    } else if (INTGetFlag(INT_SOURCE_UART_RX(HOSTuart))) {
        INTClearFlag(INT_SOURCE_UART_RX(HOSTuart));
        if (UARTReceivedDataIsAvailable(HOSTuart)) {
            ch = toupper(UARTGetDataByte(HOSTuart));
            if (ch == '\n' || ch == 0);
            else if (ch == BACKSPACE) {
                while (!UARTTransmitterIsReady(HOSTuart));
                UARTSendDataByte(HOSTuart, ' ');
                while (!UARTTransmitterIsReady(HOSTuart));
                UARTSendDataByte(HOSTuart, BACKSPACE);
                if (HOSTRxIndex > 0) HOSTRxIndex--;
            } else if (ch == ENTER) {
                HOSTRxBuffer[HOSTRxIndex] = '\0'; // $$$$
                HOSTRxBufferFull = true;
                HOSTRxIndex = 0;
            } else if (ch < 27) controlCommand = ch;
            else if (HOSTRxIndex < MAXBUFFER) {
                HOSTRxBuffer[HOSTRxIndex++] = ch;
            }
        }
    }
    if (INTGetFlag(INT_SOURCE_UART_TX(HOSTuart))) {
        INTClearFlag(INT_SOURCE_UART_TX(HOSTuart));
    }
}

void __ISR(XBEE_VECTOR, ipl2) IntXbeeHandler(void) 
{
    static unsigned short RxIndex = 0;
    static unsigned short TxIndex = 0;
    unsigned char ch;

    if (XBEEbits.OERR || XBEEbits.FERR) 
    {
        if (UARTReceivedDataIsAvailable(XYZuart))
            ch = UARTGetDataByte(XYZuart);
        XBEEbits.OERR = 0;
    } 
    else if (INTGetFlag(INT_SOURCE_UART_RX(XYZuart))) 
    {
        INTClearFlag(INT_SOURCE_UART_RX(XYZuart));
        if (UARTReceivedDataIsAvailable(XYZuart)) 
        {
            ch = UARTGetDataByte(XYZuart);
            if (ch == 0xFF) RxIndex = 0;            
            if (RxIndex < MAXBUFFER - 2)
            {
                XBEERxBuffer[RxIndex++] = ch;
                XBEERxLength = RxIndex;             
            }
        }
    }

    if (INTGetFlag(INT_SOURCE_UART_TX(XYZuart))) 
    {
        INTClearFlag(INT_SOURCE_UART_TX(XYZuart));
        /*
        if (XBEETxLength) {
            if (TxIndex < MAXBUFFER) ch = XBEETxBuffer[TxIndex++];
            while (!UARTTransmitterIsReady(XYZuart));
            UARTSendDataByte(XYZuart, ch);
            if (TxIndex >= XBEETxLength) {
                INTEnable(INT_SOURCE_UART_TX(XYZuart), INT_DISABLED);
                XBEETxLength = 0;
                TxIndex = 0;
            }
        } else INTEnable(INT_SOURCE_UART_TX(XYZuart), INT_DISABLED);        
        */
    }
    
}


void sendRequest(uint8_t cmd,  const uint8_t * data1, uint8_t data1Size)
{
    int i;
    unsigned char ch;
     
#define HEADER_SIZE 7
  uint8_t header[HEADER_SIZE];

  uint8_t size = data1Size + HEADER_SIZE;

  uint8_t checksum = size ^ id ^ cmd;
  for (i = 0; i < data1Size; i++) { checksum ^= data1[i]; }
  
  header[0] = 0xFF;
  header[1] = 0xFF;
  header[2] = size;
  header[3] = id;
  header[4] = cmd;
  header[5] = checksum & 0xFE;
  header[6] = ~checksum & 0xFE;

    for (i = 0; i < HEADER_SIZE; i++)
    {
        ch = header[i];
        while (!UARTTransmitterIsReady(XYZuart));  
        UARTSendDataByte(XYZuart, ch);           
    }
  
    for (i = 0; i < data1Size; i++)
    {
        ch = data1[i];
        while (!UARTTransmitterIsReady(XYZuart));  
        UARTSendDataByte(XYZuart, ch);                                      
    }
}

void setSpeed(int16_t speed)
{
    uint8_t playtime = 0x00;
    sendIJog(speed, SET_SPEED_CONTROL, playtime);
}

void setPosition(uint16_t position, uint8_t playtime)
{  
  sendIJog(position, SET_POSITION_CONTROL, playtime);
}

void sendIJog(uint16_t goal, uint8_t type, uint8_t playtime)
{
  #define DATA_SIZE 5  
  uint8_t data[DATA_SIZE];
  data[0] = goal & 0xFF;
  data[1] = goal >> 8 & 0xFF;
  data[2] = type;
  data[3] = id;
  data[4] = playtime;
  sendRequest(CMD_I_JOG, data, DATA_SIZE);
}


/** EOF main.c *************************************************/
#endif
