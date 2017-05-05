#ifndef MAIN_C
#define MAIN_C

// includes
#include <string.h>
#include <usart.h>
#include <delays.h>
#include <stdio.h>
#include <math.h>
#include <EEP.h>
#include "config.h"
#include "types.h"


//define
//=================================
// Comment out the following line if you do not want the debug
// feature of the firmware (saves code and RAM space when off)//
// Note: if you use this feature you must compile with the large
// memory model on (for 24-bit pointers) so that the sprintf()
// function will work correctly.  If you do not require debug it's
// recommended that you compile with the small memory model and 
// remove any references to <strings.h> and sprintf().
#define DEBUGON

//Use LATA to write, PORTA to read.
//Remember to configure input/output
#define SRR         LATAbits.LATA5 
//#define Switch      PORTBbits.RB6 //Switch 1 input

#define ADDRESS 7//XBee address.
#define SERVER_ADDRESS 0x00
#define SWITCH_ADDRESS 0x08

/* --- configure DS1820 temparture sensor pin --- */
#if defined(__18CXX)
#define DS18B20_POWER_PIN   LATBbits.LATB5
#define DS1820_DATAPIN      PORTBbits.RB4
#define output_low(pin)     TRISBbits.TRISB4 = 0;(LATBbits.LATB4 = 0)
#define output_high(pin)    TRISBbits.TRISB4 = 0;(LATBbits.LATB4 = 1)
#define input(pin)          input_func()
bool input_func(void)
{
    TRISBbits.TRISB4 = 1;
    return (PORTBbits.RB4);
}
#endif

#include "ds1820.h"

// Define the globals 
#pragma udata
int deviceState = 0;
char ADDRH = 0x00;
//UART variables
char RxBuffer[100];
char TxBuffer[100];
int receivePos = 0;
//boiler variables
double time=0;
double boilerSecondsCounter=0;
double hours;
double minutes;
//DS18B20 variables
int temperatureSecondsCounter=0;
double temperature;
#pragma udata

// Private function prototypes
static void initialisePic(void);
void highPriorityISRCode();
void lowPriorityISRCode();
void initUsart(void);
void initXbee(void);
void ReceiveUsart(void);
void TransmitUsart(char ADDRL, char device, char data);
void TransmitUsartAT(void);
void delay1sec(void);
void handleMessge(void);
void boilerCommands(char device);
double readTemperature(void);


#pragma code HIGH_INTERRUPT_VECTOR = 0x08
void High_ISR(void) {
    _asm goto highPriorityISRCode _endasm
}

#pragma code LOW_INTERRUPT_VECTOR = 0x18
void Low_ISR(void) {
    _asm goto lowPriorityISRCode _endasm
}

#pragma code

// High-priority ISR handling function
#pragma interrupt highPriorityISRCode
void highPriorityISRCode() {
    // Application specific high-priority ISR code goes here
    //USART receive interrupt, Cleared when RCREG is read.
    if (PIR1bits.RCIF){
        ReceiveUsart();
    }
}

// Low-priority ISR handling function
#pragma interruptlow lowPriorityISRCode

void lowPriorityISRCode() {
    //TMR0 interrupt on overflow
    if (INTCONbits.TMR0IF) {
        INTCONbits.TMR0IF = 0; //clear the interrupt flag
        TMR0H = 0x0B;
        TMR0L = 0xDC; //TMR0 Preload = 3036; Actual Interrupt Time : 1 sec
        
        //boiler timed operation
        if (deviceState == 3 | deviceState == 4 | deviceState == 5) {
            if (boilerSecondsCounter < hours * 3600 + minutes * 60) {
                boilerSecondsCounter++;
            } else {
                //time has passed
                SRR = 0;
                deviceState = 0; //off
                TransmitUsart(SERVER_ADDRESS, 0x01, (char) deviceState);
                TransmitUsart(SWITCH_ADDRESS, 0x01, (char) deviceState);
            }
        }

        //interval for temperature measurement
        temperatureSecondsCounter++;
        if (temperatureSecondsCounter > 3) {
            temperatureSecondsCounter = 0;
            temperature = readTemperature();
            LATAbits.LATA3 = 0; 
        }

    }
}

// Main program entry point
void main(void) {
    //init device
    initialisePic();
    initUsart();
    
    //init EEPROM with default values
    if(Read_b_eep(0x01)==0xff)
        Write_b_eep(0x01, (char)1 );//minutes
    if(Read_b_eep(0x02)==0xff)
        Write_b_eep(0x02, (char)0 );//hours
    
    initXbee();
    DS18B20_POWER_PIN = 1;
    
   
    //Main loop
    while(1) {
        Sleep();
    }
}

void handleMessage(void) {
    if (RxBuffer[3] == 0x81) //RX (Receive) Packet: 16-bit Address
    {
        switch (RxBuffer[8]) //first data byte, Device selection
        {
            case 1://Device is Boiler
                boilerCommands(RxBuffer[8]);
                break;

            default: // Unknown command received
                break;
        }
    }
}

void boilerCommands(char device) {
    INTCONbits.GIEH = 1;//enable high priority interrupts. enable stopping shutters while on delay.
    switch (RxBuffer[9]) //Device specific command
    {
        case 0://off
            SRR = 0;
            deviceState = 0; //off  
            TransmitUsart(SERVER_ADDRESS, 0x01, (char) deviceState);
            TransmitUsart(SWITCH_ADDRESS, 0x01, (char) deviceState);
            break;
            
        case 1://on manual local
            SRR = 1;
            deviceState = 1; 
            TransmitUsart(SERVER_ADDRESS, 0x01, (char) deviceState);
            TransmitUsart(SWITCH_ADDRESS, 0x01, (char) deviceState);
            break;
            
        case 2://on manual remote
            SRR = 1;
            deviceState = 2; 
            TransmitUsart(SERVER_ADDRESS, 0x01, (char) deviceState);
            TransmitUsart(SWITCH_ADDRESS, 0x01, (char) deviceState);
            break;

        case 3://on pre defined timer local
            SRR = 1;
            minutes = (double) Read_b_eep(0x01);
            hours = (double) Read_b_eep(0x02);
            boilerSecondsCounter = 0;
            deviceState = 3; 
            TransmitUsart(SERVER_ADDRESS, 0x01, (char) deviceState);
            TransmitUsart(SWITCH_ADDRESS, 0x01, (char) deviceState);
            break;
            
        case 4://On Automatic from remote predefined
            SRR = 1;
            minutes = (double) Read_b_eep(0x01);
            hours = (double) Read_b_eep(0x02);
            boilerSecondsCounter = 0;
            deviceState = 4; 
            TransmitUsart(SERVER_ADDRESS, 0x01, (char) deviceState);
            TransmitUsart(SWITCH_ADDRESS, 0x01, (char) deviceState);
            break;
            
        case 5://On Automatic from remote by parameters
            SRR = 1;
            minutes = (double)RxBuffer[10];
            hours = (double) RxBuffer[11];
            boilerSecondsCounter = 0;
            deviceState = 5; //Auto mode 
            TransmitUsart(SERVER_ADDRESS, 0x01, (char) deviceState);
            TransmitUsart(SWITCH_ADDRESS, 0x01, (char) deviceState);
            break;
        
        case 6://Write EEPROM parameter
            Write_b_eep(RxBuffer[10], RxBuffer[11]);
            break;
            
        case 7://Read EEPROM parameter
            TransmitUsart(SERVER_ADDRESS, RxBuffer[10], Read_b_eep(RxBuffer[10]));
            break;

        default: // Unknown command received
            break;
    }
}

double readTemperature(void) {
    int i;
    sint16 temperature_raw; /* temperature raw value (resolution 1/256°C) */
    float temperature_float;
    char temperature[8]; /* temperature as string */

    DS1820_FindFirstDevice();
    /* get temperature raw value (resolution 1/256°C) */
    temperature_raw = DS1820_GetTempRaw();
    /* convert raw temperature to string for output */
    DS1820_GetTempString(temperature_raw, temperature);

    /* get temperature value as float */
    temperature_float = DS1820_GetTempFloat();
    for (i = 0; i < 8; i++) {
        while (TXSTAbits.TRMT != 1);
        WriteUSART(temperature[i]);
    }

}

// Initialise the PIC
static void initialisePic(void) {
    //Internal clock freq 8MHz
    OSCCONbits.IRCF0 = 1;
    OSCCONbits.IRCF1 = 1;
    OSCCONbits.IRCF2 = 1;
    //sleep configuration
    OSCCONbits.IDLEN = 1;
    // Default all pins to digital
    ADCON1 = 0x0F; 
    // Clear all ports
    PORTA = 0b00000000;
    PORTB = 0b00000000;
    PORTC = 0b00000000;
    // Configure ports as inputs (1) or outputs(0)
    TRISA = 0b00000000;
    TRISB = 0b00000000;
    TRISC = 0b00000000;
    //enalble port B pull-up resistors
    INTCON2bits.RBPU = 0; //PORTB weak pullup enabeled
    // Configure interrupts
    INTCONbits.GIEH = 1; //Enable global interrupts
    INTCONbits.GIEL = 1; //Enable lopw priority interrupts
    RCONbits.IPEN = 1; //Enable priority interrupts
    PORTB = PORTB;
    INTCONbits.RBIF = 0; //Reset the interrupt flag
    // Timer0 configuration
    T0CONbits.T0CS = 0; //0 = Internal instruction cycle clock (CLKO)
    T0CONbits.T08BIT = 0;//0=16 bit mode
    T0CONbits.PSA = 0; //Use pre-scalar
    T0CONbits.T0PS0 = 0;
    T0CONbits.T0PS1 = 0;
    T0CONbits.T0PS2 = 1; //prescalar 1:32, overflow every 8.192ms
    TMR0H	 = 0x0B;
    TMR0L	 = 0xDC;//TMR0 Preload = 3036; Actual Interrupt Time : 1 sec
    INTCON2bits.T0IP = 0; //TMR0 interrpt low priority
    INTCONbits.TMR0IE = 1; //Timer0 interrupt on overflow
}

void initUsart() {
    TRISCbits.RC6 = 1;
    TRISCbits.RC7 = 1;
    TXSTAbits.TXEN = 1; // Transmit Enable bit
    RCSTAbits.SPEN = 1; // Serial Port Enable bit
    RCSTAbits.CREN = 1; // Continuous Receive Enable bit
    TXSTAbits.BRGH = 1; // High Baud Rate Select bit
    BAUDCONbits.BRG16 = 1; //16-Bit Baud Rate Register Enable bit
    SPBRGH = 0;
    SPBRG = 103; // baudrate: 103-19230 207-9600
    IPR1bits.RCIP = 1; //Set USART receive interrupt low priority
    PIE1bits.RCIE = 1; //Enable USART receive interrupts
    PIR1bits.TXIF = 1; //The EUSART transmit buffer, TXREG, is empty (cleared when TXREG is written)
}

void initXbee() {
    delay1sec();
    sprintf(TxBuffer, "X");
    TransmitUsartAT();
    delay1sec();
    sprintf(TxBuffer, "+++"); //Enter At command mode
    TransmitUsartAT();
    delay1sec();
    sprintf(TxBuffer, "ATMY%d\r", ADDRESS); //Set address
    TransmitUsartAT();
    sprintf(TxBuffer, "ATBD4\r"); //Set BaudeRate 19,200
    TransmitUsartAT();
    sprintf(TxBuffer, "ATAP1,WR,AC,CN\r"); //Set API mode 1
    TransmitUsartAT();
}

void delay1sec() {
    //TCY = 1sec/(8MHz/4) =0.5 us
    Delay10KTCYx(220);//1 sec is 200, but AT commands were not working, so i increased the delay
}

void ReceiveUsart() {
    char c;
    int i;
    if (PIR1bits.RCIF == 1 && PIE1bits.RCIE == 1) //Check data in RCREG.
    {
        if (RCSTAbits.OERR == 1) {
            RCSTAbits.CREN = 0x0; //Stop continuous reception to clear the error flag FERR.
            RCSTAbits.CREN = 0x1; //Enable continuous reception again.
        }
        c = RCREG; //Read data from RCREG
        if (c == 0x7e)//API Frame start delimeter recived
            receivePos = 0;
        RxBuffer[receivePos] = c;
        //length of data received
        if (receivePos >= 2) {
            if (receivePos == RxBuffer[1]*256 + RxBuffer[2] + 3)
                handleMessage();
        }
        receivePos++;
    }
}

void TransmitUsart(char ADDRL, char device, char data) {
    int i, len;

    TxBuffer[0] = 0x7e; //Start delimiter
    TxBuffer[1] = 0x00; //length MSB
    TxBuffer[2] = 0x07; //length LSB
    TxBuffer[3] = 0x01; //API identifier: TX Request, 16-bit address
    TxBuffer[4] = 0x00; //frame ID
    TxBuffer[5] = ADDRH; //Destination address MSB
    TxBuffer[6] = ADDRL; //Destination address LSB
    TxBuffer[7] = 0x00; //options -Disable ACK
    TxBuffer[8] = device; //Device selection
    TxBuffer[9] = data; //Data
    TxBuffer[10] = 0xff - (TxBuffer[3] + TxBuffer[4] + TxBuffer[5] + TxBuffer[6] + TxBuffer[7] + TxBuffer[8]+TxBuffer[9]); //checksum

    len = TxBuffer[1]*256 + TxBuffer[2] + 4;
    for (i = 0; i < len; i++) {
        while (TXSTAbits.TRMT != 1);
        WriteUSART(TxBuffer[i]);
    }
}

void TransmitUsartAT() {
    int i, len;
    len = strlen(TxBuffer);
    for (i = 0; i < len; i++) {
        while (TXSTAbits.TRMT != 1);
        WriteUSART(TxBuffer[i]);
    }
}

