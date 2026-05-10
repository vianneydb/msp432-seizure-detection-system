/****************
Author: Vianney Diaz-Barraza
Institution: The University of Texas at El Paso

Date:5/7/2026
*****************/

#include <lcdLib_432.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "msp.h"

#define TIMER50HZ 60000
#define TIMER100HZ 30000
#define FREE_FALL 10000
#define SPIKE 11600
#define TONIC 11800
#define END_DELAY 2

volatile uint16_t A0results[8];
volatile uint16_t A1results[8];

volatile float mag = 0.0f;
volatile float peak = 0.0f;
static int index;

//Normal Activity                 => Level 0
//Suspicious Activity/Free fall   => Level 1
//Tonic Phase Detected            => Level 2
//Clonic Phase Detected           => Level 3
//Epileptic                       => Level 4
//Initially at level 0
volatile int levelSeizure = 0;

//Global variables to track seizure time
volatile int tonicTime = 0;
volatile int clonicTime = 0;
volatile int seizureTime = 0;

volatile int endSeizure = END_DELAY;

// ------ RX -------
char cmdBuffer[20];
volatile int cmdIndex = 0;
volatile int selfTestRequested = 0;

volatile int thresholdPercent = 100;

volatile uint8_t payload[2];
volatile uint8_t lastReceivedType = 0;

void Config_GPIO(void) {
    // ADC pin configuration
    // A0 -> P5.5  A1 -> P5.4
    P5->SEL1 |= BIT5 | BIT4;
    P5->SEL0 |= BIT5 | BIT4;

    // Configure pins as UART2 TX and RX pins
    P3->SEL0 |= BIT2 | BIT3; // P3.2->RX
    P3->SEL1 &= ~(BIT2 | BIT3); // P3.3->TX
}

void ADC_Config(void) {

    NVIC->ISER[0] = 1 << ((ADC14_IRQn) & 31);// Enable ADC interrupt in NVIC module

    // ADC14 configuration:
    // SHT0_192 = longer sample time to avoid overflow
    // SHP     = use sampling timer
    // ON      = turn ADC on
    ADC14->CTL0 = ADC14_CTL0_ON |
                  ADC14_CTL0_MSC |
                  ADC14_CTL0_SHT0__192 |
                  ADC14_CTL0_SHP |
                  ADC14_CTL0_CONSEQ_1;

    ADC14->MCTL[0] = ADC14_MCTLN_INCH_0; // ref+=AVcc, channel = A0
    ADC14->MCTL[1] = ADC14_MCTLN_INCH_1 |
                     ADC14_MCTLN_EOS; // ref+=AVcc, channel = A1, end seq.

    // Set resolution to 14-bit
    ADC14->CTL1 = ADC14_CTL1_RES__14BIT;

    ADC14->IER0 = ADC14_IER0_IE1; // Enable ADC14IFG.3
    SCB->SCR &= ~SCB_SCR_SLEEPONEXIT_Msk; // Wake up on exit from ISR
    // Ensures SLEEPONEXIT takes effect immediately
    __DSB();

    ADC14->CTL0 |= ADC14_CTL0_ENC;
}

void TimerA0(void){
    // TIMER A0
    TIMER_A0->CCR[0] = TIMER50HZ - 1; // PWM Period
    TIMER_A0->CCTL[1] = TIMER_A_CCTLN_OUTMOD_7; // CCR1 reset/set

    TIMER_A0->CTL = TIMER_A_CTL_SSEL__SMCLK | // SMCLK
                    TIMER_A_CTL_MC__UP | // Up mode
                    TIMER_A_CTL_CLR; // Clear TAR

    TIMER_A0->CCTL[0] = TIMER_A_CCTLN_CCIE;
    NVIC_EnableIRQ(TA0_0_IRQn); //Interrupt
}

void Systick_Config(void){
    SysTick->CTRL = 0;

    // SMCLK = 3MHz
    SysTick->LOAD = (3000000) - 1; // 1s
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}

void DisplayLCD(void) {
    char buffer[16];

    sprintf(buffer, "ST:%d TT:%d CT:%d  ", seizureTime, tonicTime, clonicTime);
    lcdSetText(buffer, 0, 0);

    sprintf(buffer, "Lvl:%d Mag:%.0f  ", levelSeizure, mag);
    lcdSetText(buffer, 0, 1);
}

void samplingRate(int levelSeizure){
    if(levelSeizure <= 0){
        //SMCLK = 3MHz: CCR0 = 60000 PWM period => PWM frequency = 50Hz
        TIMER_A0->CCR[0] = TIMER50HZ - 1;
    }
    else if(levelSeizure >=1){
        //SMCLK = 3MHz: CCR0 = 30000 PWM period => PWM frequency = 100Hz
        TIMER_A0->CCR[0] = TIMER100HZ - 1;
    }
}

int Oscillations(){
    static float prev = 0;
    static int count = 0;

    if(fabs(mag - prev) > 2000){
        count++;
    }

    prev = mag;

    if(count > 5){
        count = 0;
        return 1;
    }

    return 0;
}

void UART2_Config(void){
    // Configure UART2
    EUSCI_A2->CTLW0 |= EUSCI_A_CTLW0_SWRST; // Hold in reset
    EUSCI_A2->CTLW0 = EUSCI_A_CTLW0_SWRST | EUSCI_B_CTLW0_SSEL__SMCLK; // Use SMCLK as clock source

    // Baud Rate calculation for 3 MHz / 9600 baud, 16x oversampling
    EUSCI_A2->BRW = 3000000/(9600 * 16);
    EUSCI_A2->MCTLW = (9 << EUSCI_A_MCTLW_BRF_OFS) | EUSCI_A_MCTLW_OS16;
    EUSCI_A2->CTLW0 &= ~EUSCI_A_CTLW0_SWRST; // Enable eUSCI_A2

    EUSCI_A2->IE |= EUSCI_A_IE_RXIE;
    NVIC_EnableIRQ(EUSCIA2_IRQn);
}

void sendChar(char c){
    while (!(EUSCI_A2->IFG & EUSCI_A_IFG_TXIFG));
    EUSCI_A2->TXBUF = c;
}

void SendDataUART(uint8_t type){
    uint8_t checksum = type;
    uint8_t payload[4];   // Max 4 bytes
    uint8_t length = 0;
    int i;

    sendChar(0xAA); // Start byte

    sendChar(type); //Send data type

    switch (type){

        case 0x01: // Sensor data
            payload[0] = (uint8_t)mag;        // Casting because is float
            payload[1] = (uint8_t)peak;       // Casting because is float
            payload[2] = levelSeizure;
            length = 3;
        break;

        case 0x02: // Seizure event
            payload[0] = levelSeizure;
            payload[1] = seizureTime;
            length = 2;
        break;

        case 0x05: // Heartbeat
            length = 0;
        break;

        case 0x06: // ACK
            payload[0] = lastReceivedType;
            length = 1;
        break;
    }

    // Send payload array (max 4 bytes)
    for(i = 0; i < length; i++){
        sendChar(payload[i]);
        checksum ^= payload[i]; //XOR
    }

    sendChar(checksum);

    sendChar(0x55); // End byte
}

void RunSelfTest(void){
    levelSeizure = 0;
    seizureTime = 0;
    SendDataUART(0x02);
    __delay_cycles(3000000 * 3);

    levelSeizure = 1;
    seizureTime = 0;
    SendDataUART(0x02);
    __delay_cycles(3000000 * 3);

    levelSeizure = 2;
    seizureTime = 0;
    SendDataUART(0x02);
    __delay_cycles(3000000 * 3);

    levelSeizure = 3;
    seizureTime = 3;
    SendDataUART(0x02);
    __delay_cycles(3000000 * 3);

    levelSeizure = 4;
    seizureTime = 6;
    SendDataUART(0x02);
    __delay_cycles(3000000 * 3);

    levelSeizure = 0;
    seizureTime = 0;
    tonicTime = 0;
    clonicTime = 0;
    SendDataUART(0x02);
}

int main(void){
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD; // Stop watchdog timer
    // Configure GPIO

    lcdInit();
    lcdClear();

    Config_GPIO();
    ADC_Config();
    TimerA0();

    Systick_Config();
    UART2_Config();

    while(1) {
        if (selfTestRequested){
            selfTestRequested = 0;
            RunSelfTest();
        }

        // Start conversion-software trigger
        //ADC14->CTL0 |= ADC14_CTL0_SC;
        __sleep();
        __no_operation(); // For debugger
    }
}

void TA0_0_IRQHandler(void){
    TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;
    ADC14->CTL0 |= ADC14_CTL0_SC; // trigger ADC start conversion
}


// ADC14 interrupt service routine
void ADC14_IRQHandler(void) {
    if (ADC14->IFGR0 & ADC14_IFGR0_IFG1){

        A0results[index] = ADC14->MEM[0]; // X
        A1results[index] = ADC14->MEM[1]; // Y

        mag = sqrtf((float)(A0results[index] * A0results[index]) + (float)(A1results[index] * A1results[index]));

        if(mag > peak){

            peak = mag;
        }

        //----------- FSM seizure -----------
        // LEVEL 0
        if(levelSeizure == 0){
            if(mag>((SPIKE*thresholdPercent)/100) && mag>((FREE_FALL*thresholdPercent)/100)){
                levelSeizure = 1;
                SendDataUART(0x02);
                samplingRate(1);
            }
        }

        //LEVEL 1 to LEVEL 2
        else if(levelSeizure == 1){

            if(mag>((TONIC*thresholdPercent)/100)){
                if(tonicTime >= 3){
                    levelSeizure = 2;
                    SendDataUART(0x02);
                    clonicTime = 0;
                }
            } else if(mag<((SPIKE*thresholdPercent)/100) && endSeizure <= 0){
                levelSeizure = 0;
                SendDataUART(0x02);
                endSeizure = 2;
                samplingRate(0);
            }
        }

        // LEVEL 2 to LEVEL 3
        else if(levelSeizure == 2){

            if(Oscillations()){
                if(clonicTime >= 8){
                    levelSeizure = 3;
                    SendDataUART(0x02);
                }
            } else if(mag<((TONIC*thresholdPercent)/100) && endSeizure <= 0){
                levelSeizure = 1;
                SendDataUART(0x02);
            }
        }

        // LEVEL 3 to LEVEL 4
        else if(levelSeizure == 3){
            if(seizureTime >= 15){
                levelSeizure = 4;
                SendDataUART(0x02);
            } else if(mag<((TONIC*thresholdPercent)/100) && endSeizure <= 0){
               levelSeizure = 2;
               SendDataUART(0x02);
            }
        }

        //LEVEL 4
        else if(levelSeizure == 4){
            if(mag<((TONIC*thresholdPercent)/100) && endSeizure <= 0){
               levelSeizure = 3;
               SendDataUART(0x02);
            }
        }


        //Buffer size 8 (0x7 = 00000111) saves previous result for comparision
        index = (index + 1) & 0x7; // Increment results index, modulo
    }
}

void SysTick_Handler(void){
    DisplayLCD(); //Displays LCD each second
    peak = 0; //Reset peak variable

    SendDataUART(0x01);
    SendDataUART(0x05);

    if(levelSeizure >= 1){
        seizureTime++;

        if(mag<((TONIC*thresholdPercent)/100)){
            endSeizure--;
        }

        if(levelSeizure == 1){
            tonicTime++;
        }

        if(levelSeizure == 2){
            clonicTime++;
        }

    }else if(levelSeizure <= 0){
        endSeizure = END_DELAY;
        seizureTime = 0;
        tonicTime = 0;
        clonicTime = 0;
    }
}

void EUSCIA2_IRQHandler(void){

    if(EUSCI_A2->IFG & EUSCI_A_IFG_RXIFG){

        uint8_t start = EUSCI_A2->RXBUF;

        if(start == 0xAA){

            while(!(EUSCI_A2->IFG & EUSCI_A_IFG_RXIFG));
            uint8_t type = EUSCI_A2->RXBUF;

            uint8_t length = 0;
            uint8_t checksum = type;
            int i;

            // definir longitud
            if(type == 0x03) length = 1;
            else if(type == 0x04) length = 1;
            else if(type == 0x05) length = 0;
            else if(type == 0x06) length = 1;

            // leer payload
            for(i = 0; i < length; i++){

                while(!(EUSCI_A2->IFG & EUSCI_A_IFG_RXIFG));

                payload[i] = EUSCI_A2->RXBUF;
                checksum ^= payload[i];
            }

            // leer checksum
            while(!(EUSCI_A2->IFG & EUSCI_A_IFG_RXIFG));
            uint8_t receivedChecksum = EUSCI_A2->RXBUF;

            // leer end
            while(!(EUSCI_A2->IFG & EUSCI_A_IFG_RXIFG));
            uint8_t end = EUSCI_A2->RXBUF;

            // validar
            if(receivedChecksum == checksum && end == 0x55){
                lastReceivedType = type;
                SendDataUART(0x06);

                if(type == 0x03){
                    if(payload[0] == 0x04){
                        selfTestRequested = 1;
                    }
                }
                else if(type == 0x04){
                    thresholdPercent = payload[0];
                }
            }
        }
    }
}
