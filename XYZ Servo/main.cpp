/***********************************************************************************************************
 * PROJECT:     XYZ Servo
 * FileName:    main.c
 * 
 * 
 * 7-4-18:      Adapted from PIC32MX795 Test
 *              
 ************************************************************************************************************/

#ifndef MAIN_C
#define MAIN_C

#define	DMX_STANDBY	0
#define	DMX_BREAK 	1
#define DMX_MARK 	2
#define DMX_START 	3
#define DMX_DATA	4
#define DMXLENGTH 	16
#define TICK		10000

#define MAXSCALE 24
#define MAXPOTVALUE 255
#define FRAME_DELAY 8
#define MAXFILTER 8
#define MAXWINDOW 5
#define MAXSERVO 7 // 12
#define MAXPOTS 1
#define MAXBOARD 16

#define true TRUE
#define false FALSE

#define USE_USB

union {
    unsigned char byte[2];
    unsigned short integer;
} convert;

#define lowByte byte[0]
#define highByte byte[1]  

/** INCLUDES *******************************************************/
// #include <XC.h>
#include <plib.h>


#include "HardwareProfile.h"
#include "Delay.h"
//#include "AT45DB161.h"

#include "GenericTypeDefs.h"
#include "Compiler.h"

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include "XYZrobotServo.h"

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


int main(void) {
    short i = 0;
    unsigned char ch;
    unsigned short goal = 0x0000;
    unsigned char ServoCommand = 0x09, checkSum1, checkSum2;
    XYZrobotServo servo(1);

    InitializeSystem();

    printf("\r Testing XYZ servo control");
    
    while(1)
    {
        DelayMs(1);               
        if (HOSTRxBufferFull)
        {
            if (ServoCommand > 0x06)
            {
                HOSTRxBufferFull = false;
                printf("\rCommand: %d, Goal: %d", ServoCommand, goal);
                XBEETxBuffer[0] = XBEETxBuffer[1] = 0xFF;
                XBEETxBuffer[2] = 7;
                XBEETxBuffer[3] = 0x01;
                XBEETxBuffer[4] = ServoCommand;   
                #define PLAYTIME 10
                XBEETxBuffer[7] = PLAYTIME;
                #define SET 0x03;
                XBEETxBuffer[10] = SET;
                XBEETxBuffer[11] = 0x01;
                /*
                if (ServoCommand == 0x06)
                {
                    goal = goal + 32;
                    if (goal > 1023) goal = 0;
                    XBEETxBuffer[8] = goal & 0x00FF;
                    XBEETxBuffer[9] = (goal >> 8) & 0x00FF;   
                    XBEETxBuffer[2] = 12;
                    checkSum1 = ((((((((XBEETxBuffer[2] ^ XBEETxBuffer[3]) ^ XBEETxBuffer[4])) ^ XBEETxBuffer[7]) ^ XBEETxBuffer[8]) ^ XBEETxBuffer[9]) ^ XBEETxBuffer[10]) ^ XBEETxBuffer[11]);
                }           
                else 
                */
                checkSum1 = (((XBEETxBuffer[2] ^ XBEETxBuffer[3]) ^ XBEETxBuffer[4]));    
            
                //checkSum2 = 0x00;
                //for (i = 0; i < 12; i ++)
                //{
                //    if (i != 5 && i != 6) 
                //        checkSum2 ^= XBEETxBuffer[i];
                //}
            
                //printf("\rChecksum #1 = %02X, Checksum #2 = %02X", checkSum1, checkSum2);

                XBEETxBuffer[5] = checkSum1 & 0xFE;
                XBEETxBuffer[6] = ~checkSum1 & 0xFE;
                for (i = 0; i < 7; i++)
                {
                    ch = XBEETxBuffer[i];
                    while (!UARTTransmitterIsReady(XYZuart));
                    UARTSendDataByte(XYZuart, ch);   
                }
                printf(" Sent");
            }
            if (ServoCommand > 0x06) ServoCommand--;
        }
        
        if (ServoCommand == 0x06)
        {
            DelayMs(2500);

            // Move the servo output counter-clockwise for some time,
            // ramping up to the specified speed.
            servo.setSpeed(400);
            DelayMs(2000);

            // Set the speed to 0 to make the servo stop abruptly.
            servo.setSpeed(0);
            DelayMs(1000);

            // Move the servo output clockwise for some time, ramping up to
            // the specified speed.
            servo.setSpeed(-400);
            DelayMs(1000);

            // Set the speed to -1 to make the servo smoothly ramp down to
            // a speed that is effectively zero.
            servo.setSpeed(-1);        
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

/** EOF main.c *************************************************/
#endif
