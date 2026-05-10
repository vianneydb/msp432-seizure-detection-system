/****************
 * Module B code for Seizure Detection System
 *
 * Author: Mauricio Aranda
 * Institution: The University of Texas at El Paso
 *
 * Date:5/7/2026
 *****************/

#include "msp.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <lcdLib_432.h>


// ================= GLOBAL VARIABLES =================

volatile int currentLevel = 0;  // guarda el nivel actual recibido de Module A
volatile int seizureTime = 0;   // guarda el tiempo de seizure que manda Module A

volatile int rxLevel = 0;       // nivel recibido.
volatile int rxTime = 0;        // tiempo recibido
volatile int newData = 0;       // bandera que dice “ya llegó un dato completo”

//char rxBuffer[20];          // Este buffer guarda los caracteres recibidos por UART
//volatile int rxIndex = 0;   // dice en que posición guardar el siguiente carácter dentro del buffer

volatile int systemArmed = 1;   // sistema encendido/activo.    desarmado en 0
volatile int alarmMuted = 0;    // buzzer suena en 0 y silenciado en 1

volatile int displayValue = 0;  // número que quieres mostrar.
volatile int muxDigit = 0;      // indica qué dígito toca prender: D1, D2, D3 o D4.
//volatile int msCounter = 0;

volatile int commMs = 0;    // Sirve para medir cuánto tiempo ha pasado desde el último dato recibido
volatile int commFault = 0; // Es una bandera (flag) que indica si hay falla de comunicación 0 todoo bien 1 perdio comunicacion
volatile int commFaultChanged = 0;  // Indica que el estado cambió y hay que actualizar LCD/alarma

volatile uint16_t potValue = 0; // Guarda el valor crudo del ADC (0–16383 porque es 14-bit)
volatile int thresholdPercent = 100;    // Es el valor ya convertido a porcentaje útil
volatile int lastThresholdPercent = 100;    // Guarda el último valor enviado
volatile int thresholdMs = 0;   // Se usa para leer el potenciómetro cada cierto tiempo
volatile int readPotFlag = 0;   // Bandera que sirve para avisar que hay q leer el potenciometro

volatile int lcdMs = 0;             // contador de tiempo en ms cada ms en systick lcdms++ sirve para medir los 500 ms
volatile int lcdUpdateFlag = 0; // 0 no hacer nada 1 hace se levanta cuando lcdms llega a 500ms

volatile int servoAngle = 0;    // guardamos el angulo del servo

volatile uint8_t rxType = 0;    // Guarda el tipo de paquete recibido
volatile uint8_t rxPayload[8];  // Es el arreglo donde guardas los datos del paquete
volatile uint8_t rxPayloadIndex = 0;    // Indica en qué posición del payload vas guardando el siguiente byte.
volatile uint8_t rxPayloadLength = 0;   // Guarda cuántos bytes de payload debe tener el paquete.
volatile uint8_t rxChecksum = 0;    // Guarda el checksum calculado mientras llega el paquete. sirve para checar que no este corrupto
volatile uint8_t rxState = 0;   // Esta es la variable más importante de la state machine   Indica en qué parte del paquete estás
volatile int packetErrorCount = 0;  //Cuenta errores de paquetes

// ================= FUNCTION PROTOTYPES =================

void GPIO_Init(void);
void UART2_Init(void);
void LCD_Update(void);
void Alarm_Update(void);
//void SevenSeg_DisplayNumber(int num);
void setSegments(int num);
void allDigitsOff(void);
void delay7seg(int ms);
void Handle_Buttons(void);
void SysTick_Init(void);
void SevenSeg_Refresh_ISR(void);
void sendChar(char c);
void sendString(char *str);
void ADC_Pot_Init(void);
uint16_t ADC_ReadPot(void);
void SendThresholdUpdate(int percent);

void Servo_Init(void);

void Servo_SetAngle(int angle);

void SendPacket(uint8_t type, uint8_t *payload, uint8_t length);
uint8_t GetPayloadLength(uint8_t type);

// ================= SEGMENT TABLE =================
// Order: A B C D E F G
const uint8_t segTable[10][7] = {   // 10 numeros 0-9 7 segmts numero entero de 8bits
    {1,1,1,1,1,1,0}, // 0
    {0,1,1,0,0,0,0}, // 1
    {1,1,0,1,1,0,1}, // 2
    {1,1,1,1,0,0,1}, // 3
    {0,1,1,0,0,1,1}, // 4
    {1,0,1,1,0,1,1}, // 5
    {1,0,1,1,1,1,1}, // 6
    {1,1,1,0,0,0,0}, // 7
    {1,1,1,1,1,1,1}, // 8
    {1,1,1,1,0,1,1}  // 9
};

// ================= MAIN =================

int main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;

    GPIO_Init();    // Configuaramos las funciones que manejan el codigo
    UART2_Init();
    SysTick_Init();
    ADC_Pot_Init();
    Servo_Init();
    Servo_SetAngle(0);


    lcdInit();      // Iniciamos LCD y la limpiamos y llamamos a q se actualice junto con los componentes
    lcdClear();
    LCD_Update();
    Alarm_Update();

    __enable_irq(); //  habilita interrupciones globales

    while (1)
    {
        if (newData)    // 1 significa que llegaron datos nuevos  la bandera la levanta EUASCIA2IRQ hanlder
        {
            newData = 0;    // ya que se recibieron bajamos bandera

            currentLevel = rxLevel;         // ponemos esos datos en las variables del sistema sin usar los de la ISR para evitar problemas
            seizureTime = rxTime;           //
            displayValue = seizureTime;     // El numero que queramos mostrar va ser el del seizure que es el tiempo recibido

            //LCD_Update();   // Actualizamos la LCD
           // Alarm_Update(); // Y actualizamos las alarmas
        }
        if (commFaultChanged)   // si es 1  se llama en systick handler
            {
                commFaultChanged = 0;       // se baja la bandera

                LCD_Update();   // Actualizamos pantalla y alarmas
                Alarm_Update();

                if (commFault)
                        Servo_SetAngle(90);   // WARNING
                    else
                        Servo_SetAngle(0);    // NORMAL
            }
        if (readPotFlag)    // si es 1 y la levanta systick handler
        {
            readPotFlag = 0;    // la bajamos

            potValue = ADC_ReadPot();   // lo que lea el ADC lo guardamos en la variable

            thresholdPercent = 80 + ((potValue * 70UL) / 16383);    //UL unsigned long evita overflow  esta operacion lo deja en un rango de 80 a 150

            if (thresholdPercent > lastThresholdPercent + 2 ||  // el nuevo valor es mayor por 3 unidades
                thresholdPercent < lastThresholdPercent - 2)    // o si el nuevo valor es menor por 3 unidades si una de las dos es true then
            {
                lastThresholdPercent = thresholdPercent;    // actualizamos el valor nuevo en ;a variable que lleva el valor antiguo
                SendThresholdUpdate(thresholdPercent);  // y enviamos el valor por UART
            }
        }
        if (lcdUpdateFlag)  // si se levanto la bandera
        {
            lcdUpdateFlag = 0;  // la bajamos para que este lista

            LCD_Update();   //Actualizamos pantalla y alarmas
            Alarm_Update();
        }

        Handle_Buttons();   // llamamos a los botones


        displayValue = seizureTime; // y asignamos el numero del seizure en la variable que lo va a mostrars
    }
}

// ================= GPIO INIT =================

void GPIO_Init(void)
{
    // 7-segment segments
    P2->DIR |= BIT3 | BIT6 | BIT5 | BIT4;   // A B C D
    P3->DIR |= BIT0;                        // E
    P5->DIR |= BIT6 | BIT7;                 // F G

    // 7-segment digits
    P5->DIR |= BIT5;                        // D1
    P6->DIR |= BIT4 | BIT5 | BIT7;          // D2 D3 D4

    // LEDs and buzzer
    P5->DIR |= BIT2 | BIT0;                 // Buzzer, Red LED
    P1->DIR |= BIT7 | BIT6;                 // Yellow, Green LED

    // Turn off LEDs and buzzer
    P5->OUT &= ~(BIT2 | BIT0);
    P1->OUT &= ~(BIT7 | BIT6);

    // Buttons P3.5, P3.6, P3.7 with pull-up
    P3->DIR &= ~(BIT5 | BIT6 | BIT7);
    P3->REN |=  (BIT5 | BIT6 | BIT7);
    P3->OUT |=  (BIT5 | BIT6 | BIT7);

    allDigitsOff();
}

// ================= UART2 INIT =================

void UART2_Init(void)
{

    //  UART2:      P3.2 = RX     P3.3 = TX       Baud = 9600

    P3->SEL0 |= BIT2 | BIT3;        // cambiamos pines de gpio normal a UART
    P3->SEL1 &= ~(BIT2 | BIT3);

    EUSCI_A2->CTLW0 |= EUSCI_A_CTLW0_SWRST; // Pone el módulo UART en reset. es pausarlo en lo que se configura
    EUSCI_A2->CTLW0 = EUSCI_A_CTLW0_SWRST |
                      EUSCI_A_CTLW0_SSEL__SMCLK; // lo seguimos dejando en reset pero agregamos el reloj para calcular el baud rate

    // 3 MHz SMCLK, 9600 baud, oversampling
    EUSCI_A2->BRW = 3000000 / (9600 * 16); // 19  Esta línea calcula el divisor principal para que UART2 trabaje a 9600 baud usando un reloj de 3 MHz y oversampling de 16.
    EUSCI_A2->MCTLW = (9 << EUSCI_A_MCTLW_BRF_OFS) |   // Esta línea configura la modulación del UART. El valor BRF ajusta finamente el baud rate
                      EUSCI_A_MCTLW_OS16;   // y OS16 activa oversampling por 16 para mejorar la precisión.
//   Configura el registro MCTLW para poner BRF = 9 y activar oversampling por 16

    EUSCI_A2->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;    // quitamos el reset y UART2 empieza a trabajar.

    EUSCI_A2->IE |= EUSCI_A_IE_RXIE;    // Cuando llegue un dato por UART, quiero que se dispare una interrupcion
    NVIC_EnableIRQ(EUSCIA2_IRQn);   // permite la interrupcion
}


void sendChar(char c)
{
    while (!(EUSCI_A2->IFG & EUSCI_A_IFG_TXIFG)); // Registro de flags y Transmit buffer ready  aqui espera hasta que UART este listo para enviar
    EUSCI_A2->TXBUF = c;    // mandamos el caracter C por UART
}

void sendString(char *str)
{
    while (*str)            // Mientras no llegue al final del string
    {
        sendChar(*str++);   // tomamos el caracter actual y lo movemos ++ al siguiente caracterhasta llegar a \0
    }
}

uint8_t GetPayloadLength(uint8_t type)  // 0 a 255
{
    switch (type)   // dependiendo de lo que llegue
    {
        case 0x01: return 3;   // Sensor Data   regresa 3 por que trae 3 bytes mag  peak   level
        case 0x02: return 2;   // Seizure Event regresa 2 pq trae 2 bytes level y seizure time
        case 0x05: return 0;   // Heartbeat     regresa 0 por que no hay datos
        case 0x06: return 1;   // ACK           regresa 1 pq indica el paquete fue reconocido
        case 0x07: return 1;   // Status        regresa 1 pq trae un byte de estado
        default:   return 0xFF; // invalid      si el type no coincide con niguno 255 pq ningun payload tiene esa longitud
    }
}

// ================= UART2 ISR =================

void EUSCIA2_IRQHandler(void)
{
    if (EUSCI_A2->IFG & EUSCI_A_IFG_RXIFG)  // checamos si la interrup fue porque llego un dato
    {
        uint8_t byte = EUSCI_A2->RXBUF; // aqui agarramos el byte que llego

        switch (rxState)
        {
            case 0: // Wait for start byte  es para el inicio de paquete
                if (byte == 0xAA)   // byte de inicio
                {
                    rxState = 1;    // cambiamos al sig estado
                    rxChecksum = 0; // reiniciamos el checksum calculado
                    rxPayloadIndex = 0; // Así cuando lleguen los datos, se empiezan a guardar desde rxpayload[0]
                }
                break;  // salimos de este switch

            case 1: // Packet type   se ejecuta despues de recibir byte de inicio
                rxType = byte;  // guardamos el tipo de paquete
                rxPayloadLength = GetPayloadLength(rxType); // cuantos bytes de payload trae este tipo de paquete y lo ponemos en la variable

                if (rxPayloadLength == 0xFF)    // si es invalido
                {
                    rxState = 0;    // abandonamos el paquete
                    packetErrorCount++; // empezamos a contar un error
                }
                else    // si si es valido
                {
                    rxChecksum = rxType;    //
                    rxPayloadIndex = 0; // reiniciamos el indice de payload

                    if (rxPayloadLength == 0)   // si tiene una longitud de 0 como HEartbeat
                        rxState = 3;   // go to checksum    lo brincamos directo a estado 3 que es leer checksum
                    else
                        rxState = 2;   // read payload  si no tiene longitud 2 osea tiene que leer los datos
                }
                break;  //salimos

            case 2: // Payload
                rxPayload[rxPayloadIndex++] = byte; // Esta línea guarda el byte recibido dentro del arreglo payload y se incrementa la posicion
                rxChecksum ^= byte; // verificamos que no lleguen coruptos

                if (rxPayloadIndex >= rxPayloadLength)  // checamos si ya recibimos todos los bytes del payload
                {
                    rxState = 3;    // cambiamos al siguiente estado checksum
                }
                break;

            case 3: // Checksum verificamos que no este corrupto
                if (byte == rxChecksum) // el recibido es lo mismo al calculado
                {
                    rxState = 4;    // si si es valido y pasamos la end byte
                }
                else
                {
                    rxState = 0;    // abandonamos el paquete
                    packetErrorCount++; // contamos el error de comunicacion
                }
                break;

            case 4: // End byte
                if (byte == 0x55)   // si el ultimo byte recibido es el final del paquete
                {
                    commMs = 0; // reiniciamos el contador de 5 seg
                    if(commFault)
                    {
                        commFault = 0;  // limpiamos falla de comunicacion
                        commFaultChanged = 1;   // avisamoa q hay q cambiar alarmas
                    }

                    if (rxType == 0x02) // Seizure Event
                    {
                        rxLevel = rxPayload[0]; // guardamos los datos en las variables
                        rxTime  = rxPayload[1];
                        newData = 1;    // y avisamos a main que ya llego los datos
                    }
                    else if (rxType == 0x05) // Heartbeat
                    {
                        // just communication alive
                    }
                    else if (rxType == 0x06) // ACK
                    {
                        // optional: handle ACK
                    }
                }
                else
                {
                    packetErrorCount++; // si no es lo anterior sumamos un error
                }

                rxState = 0;    // reiniciamos a esperar un paquete nuevo
                break;

            default:    // si llega con algo inesperado
                rxState = 0;    // reiniciamos
                break;
        }
    }
}

// ================= LCD UPDATE =================

void LCD_Update(void)
{
    char line1[17];
    char line2[17];

        lcdClear();

    if (commFault)  // si labandera esta levantada que muestre lo de abajo
    {

        lcdSetText("COMM FAULT", 0, 0);
        lcdSetText("SENSOR LOST", 0, 1);
        return; // regresamos
    }


    if (!systemArmed)   // Si es 0 osea desarmado
    {
        lcdSetText("System DISARM", 0, 0);
        lcdSetText("Press B1 Arm", 0, 1);
        return;
    }
// sprintf convierte numero a texto guardandolo en line1
    sprintf(line1, "Level:%d", currentLevel);   // mostramos el nivel actual

    if (alarmMuted) //  si es 1 significa muteado
        sprintf(line2, "T:%03d MUTED", seizureTime);
    else
        sprintf(line2, "Time:%03d sec", seizureTime);   // si no es 1 osea 0 muestra tiempo normal

    lcdSetText(line1, 0, 0);
    lcdSetText(line2, 0, 1);    // acomodamos los arrays
}

// ================= ALARM UPDATE =================

void Alarm_Update(void)
{
    P5->OUT &= ~(BIT2 | BIT0);   // Buzzer, Red OFF
    P1->OUT &= ~(BIT7 | BIT6);   // Yellow, Green OFF

    if (commFault)  // si la bandera es 1 osea que se perdio comunicacion
        {
            P5->OUT |= BIT2;      // buzzer ON
            P5->OUT |= BIT0;      // red ON
            P1->OUT |= BIT7;      // yellow ON
            P1->OUT &= ~BIT6;     // green OFF
            return;
        }

    if (!systemArmed)   // si esta desmarmado no hace nada
    {
        return;
    }

    if (currentLevel == 0)
    {
        P1->OUT |= BIT6;         // Green ON
    }
    else if (currentLevel == 1)
    {
        P1->OUT |= BIT6;         // Green ON
    }
    else if (currentLevel == 2)
    {
        P1->OUT |= BIT7;         // Yellow ON

    }
    else if (currentLevel == 3)
    {
        P5->OUT |= BIT0;    // RED ON
        if (!alarmMuted) P5->OUT |= BIT2;   // y prende buzzer si no esta muteada
    }
    else if (currentLevel >= 4)
    {
        P5->OUT |= BIT0;    // RED ON
        P1->OUT |= BIT7;    // YELLOW ON
        if (!alarmMuted) P5->OUT |= BIT2;   // prende buzzer si no esta muteada
    }
}

// ================= 7-SEGMENT FUNCTIONS =================

void setSegments(int num)   // esta funcion no selecciona el digito solo le da la forma
{
    if (num < 0 || num > 9) num = 0;    // no puede ser menor a 0 o mayor 9 si no sera 0

    // A = P2.7
    if (segTable[num][0]) P2->OUT |= BIT3;  // si tiene que estar prendido dependiendo del numero prende el segmento
    else                  P2->OUT &= ~BIT3; // si no lo apaga y lo mismo lo checa con cada letra o segmento

    // B = P2.6
    if (segTable[num][1]) P2->OUT |= BIT6;
    else                  P2->OUT &= ~BIT6;

    // C = P2.5
    if (segTable[num][2]) P2->OUT |= BIT5;
    else                  P2->OUT &= ~BIT5;

    // D = P2.4
    if (segTable[num][3]) P2->OUT |= BIT4;
    else                  P2->OUT &= ~BIT4;

    // E = P3.0
    if (segTable[num][4]) P3->OUT |= BIT0;
    else                  P3->OUT &= ~BIT0;

    // F = P5.6
    if (segTable[num][5]) P5->OUT |= BIT6;
    else                  P5->OUT &= ~BIT6;

    // G = P5.7
    if (segTable[num][6]) P5->OUT |= BIT7;
    else                  P5->OUT &= ~BIT7;
}

void allDigitsOff(void)
{

     // Common cathode:   Digit OFF = HIGH        Digit ON  = LOW


    P5->OUT |= BIT5;                 // D1 OFF
    P6->OUT |= BIT4 | BIT5 | BIT7;   // D2 D3 D4 OFF
}

void SevenSeg_Refresh_ISR(void) // esta selecciona el digito
{
    int value = displayValue;   // copiamos el valor a mostrar en variable value

    int d1, d2, d3, d4; // declaramos los digitos

    if (value < 0) value = 0;       // no puede ser menor a 0
    if (value > 9999) value = 9999; // no pueder mayor a 9999
//1234
    d1 = value / 1000;          // miles    1
    d2 = (value / 100) % 10;    // centenas 2
    d3 = (value / 10) % 10;     // decenas  3
    d4 = value % 10;            // unidades 4

    allDigitsOff(); // apagamos digitos para cambiar de numero sin basura

    switch (muxDigit)   // indica cuál dígito se va a refrescar en esta llamada dependiendo el caso
    {
        case 0:
            setSegments(d1);    // el primer caso agarramos el valor de d1
            P5->OUT &= ~BIT5;   // D1 ON 0 es ON
            break;

        case 1:
            setSegments(d2);
            P6->OUT &= ~BIT4;   // D2 ON
            break;  // break nos ayuda a que no sigamos otro caso y hacer uno a la vez

        case 2:
            setSegments(d3);
            P6->OUT &= ~BIT5;   // D3 ON
            break;

        case 3:
            setSegments(d4);
            P6->OUT &= ~BIT7;   // D4 ON
            break;
    }

    muxDigit++; // despues de prender un digito corre al sig
    if (muxDigit > 3)   // pero cuando se pasa de 3
        muxDigit = 0;   // regresa a 0 pq solo tenemos 4 digitos
}

// ================= DELAY =================

void delay7seg(int ms)
{
    int i;
    for (i = 0; i < ms * 3000; i++);
}


//============= handle bottons=========

void Handle_Buttons(void)
{
    static int lastB1 = 1;  // guardamos el estado de cada boton inician en 1 porque es no presionado 0 es presionado
    static int lastB2 = 1;
    static int lastB3 = 1;

    int b1 = (P3->IN & BIT5) ? 1 : 0;   // Button 1 preguntamos si el bit esta en 1 b1 es 1 si no es 0
    int b2 = (P3->IN & BIT6) ? 1 : 0;   // Button 2
    int b3 = (P3->IN & BIT7) ? 1 : 0;   // Button 3

    // Button 1: ARM / DISARM
    if (lastB1 == 1 && b1 == 0)     // detectamos cambio del sw
    {
        systemArmed = !systemArmed; // invertimos el estado armado del sistema

        if (!systemArmed)   // si esta desarmado
        {
            currentLevel = 0;   // reseteamos todas estas variables
            seizureTime = 0;
            alarmMuted = 0;
        }

        LCD_Update();   // actualizamos LCD
        Alarm_Update(); // actualizamos componentes visuales

        delay7seg(80);   // debounce    de 80 ms para asegurar el funcionamiento y no tener lecturas falsas
    }

    // Button 2: ACK / mute buzzer
    if (lastB2 == 1 && b2 == 0) // detectamos cambios del sw
    {
        if (currentLevel >= 2)  // si el estado de la convulsion esta en 2 o mas arriba
        {
            alarmMuted = 1; // apagamos el buzzer
        }

        LCD_Update();   // actualizamos pantalla y esatdo de componentes
        Alarm_Update();

        delay7seg(80);   // debounce
    }

    // Button 3: RESET / self-test simple
    if (lastB3 == 1 && b3 == 0) // checamos si se presiono el sw
    {
        uint8_t payload[1]; // creamos arreglo de un byte
        payload[0] = 0x04;   // SELF-TEST command
        SendPacket(0x03, payload, 1);   // lo mandamos usando la funcion

        lcdClear(); // limpiamos la pantalla
        lcdSetText("SELF-TEST SENT", 0, 0); // mostramos esto en la pantalla
        lcdSetText("Waiting Mod A", 0, 1);

        delay7seg(80);  // debounce
    }

    lastB1 = b1;    // guardamos el estado actual para las siguientes interacciones
    lastB2 = b2;
    lastB3 = b3;
}


//============Systick
void SysTick_Init(void)
{
    SysTick->CTRL = 0;  // apagar SysTick mientras configuro para no causar errores
    SysTick->LOAD = 3000 - 1;   // 1 ms if clock is 3 MHz
    SysTick->VAL = 0;   // limpiar contador actual y empezar desde 0
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |    //selecciona el reloj del CPU
                    SysTick_CTRL_TICKINT_Msk |  //habilitamos interrumpciones
                    SysTick_CTRL_ENABLE_Msk;    //prendemos el contador
}

void SysTick_Handler(void)
{
    SevenSeg_Refresh_ISR(); //refrescamos el 7 segment
    commMs++;   // avanzamos el contador de datos de comunicacion

        if (commMs >= 5000 && !commFault)   // si pasan 5 seg y no hay falla
        {
            commFault = 1;  // activamos bandera que hay falla de comunicacion
            currentLevel = 5;   // usamos 5 como COMM FAULT
            commFaultChanged = 1;   // levantamos bandera para que main sepa q hay q actualizar alarma y LCD
        }
        thresholdMs++;  // avanzamos contador de trheshold

        if (thresholdMs >= 500) // cuando llega a 500 ms
        {
            thresholdMs = 0;    // reiniciamos el contador
            readPotFlag = 1;    // levantamos bandera para que se vuelva a leer
        }
        lcdMs++;    // contador cada ms esta variable incrementa de uno en uno

        if (lcdMs >= 500)   // si el contador es mayor o igual a 500 ms
        {
            lcdMs = 0;      // reiniciamos el contador
            lcdUpdateFlag = 1;  // levantamos bandera para que lo cheque main
        }
}

void ADC_Pot_Init(void)
{
    // P5.4 = A1
    P5->SEL0 |= BIT4;   // cambiamos el pin a entrada analogica de GPIO a ADC
    P5->SEL1 |= BIT4;

    ADC14->CTL0 = ADC14_CTL0_SHT0__192 |    // sample and hold time por 192 ciclos
                  ADC14_CTL0_SHP |  // Sample and Hold Pulse Mode el ADC se controla solo, no necesitas señales externas
                  ADC14_CTL0_ON;    // activamos el modulo del ADC

    ADC14->CTL1 = ADC14_CTL1_RES__14BIT;    // 2^14 = 16384 salida de 0 a 16383
    //Memory Control Register configura      cada uno le dice al ADC qué canal leer y dónde guardarlo
    ADC14->MCTL[0] = ADC14_MCTLN_INCH_1;    // A1

    ADC14->CTL0 |= ADC14_CTL0_ENC;  // Permite que el ADC pueda empezar conversiones
}

uint16_t ADC_ReadPot(void)
{
    ADC14->CTL0 |= ADC14_CTL0_SC;   // arranca una nueva conversión start conversion

    while (!(ADC14->IFGR0 & ADC14_IFGR0_IFG0)); // Espera hasta que el ADC acabe de medir el voltaje      quedate aquí hasta que IFG3 sea 0

    return ADC14->MEM[0];   // Guardamos el resultado de ADC
}

void SendThresholdUpdate(int percent)
{
    uint8_t payload[1]; // creamos payload de 1 byte pq solo ocupamos el %

    if (percent < 0) percent = 0;   // evitamos valores invalidos 255 por uint8
    if (percent > 255) percent = 255;

    payload[0] = (uint8_t)percent;  // le asignamos el % a payload que es lo que se envia por paquete

    SendPacket(0x04, payload, 1);   // 0x04 paquete tipo threshold payload tiene el % y el 1 la longitud del payload
}

void Servo_Init(void)
{
    // P2.7 -> TA0.4
    P2->DIR |= BIT7;    // pin como salida
    P2->SEL0 |= BIT7;   // estas dos lineas lo configuramos como timer
    P2->SEL1 &= ~BIT7;

    // Timer_A0 PWM
    TIMER_A0->CCR[0] = 60000 - 1; // 20 ms period   3000000*.02=60000   basicamente se configura a 20 ms

    TIMER_A0->CCTL[4] = TIMER_A_CCTLN_OUTMOD_7; // output mode 7  es reset/set

    TIMER_A0->CCR[4] = 4500; // center position duracion del pulso high  4500/3000000= 1.5ms que es 90*

    TIMER_A0->CTL = TIMER_A_CTL_SSEL__SMCLK |   // usamos SMCLK como reloj
                    TIMER_A_CTL_MC__UP |    // cuenta desde 0 hasta CCR0
                    TIMER_A_CTL_CLR;    // Limpia el timer antes de empezar.
}

void Servo_SetAngle(int angle)
{
    int pulse;  // aqui se guardara el valor del PWM que ira  a CCR4

    // Map 0–180° -> 3000–6000 (1ms–2ms)
    pulse = 3000 + ((angle * 3000) / 180);  // esta linea hacemos regla de 3 para dependiendo el angle sacar los pulsos

    TIMER_A0->CCR[4] = pulse;   //aqui actualizamos el ancho del pulso
}

void SendPacket(uint8_t type, uint8_t *payload, uint8_t length)
{
    uint8_t checksum = type;    // lo incluimos para que se verifique que es valido
    int i;

    sendChar(0xAA); // mandamos el first byte
    sendChar(type); // mandamos el tipo de paquete  0x03 coomand packet o 0x04 threshold update

    for (i = 0; i < length; i++)    //  recorremos mientras i sea menor que el length
    {
        sendChar(payload[i]);   // mandamos el byte actual del payload
        checksum ^= payload[i]; // se hace un chequeo para que sea valido
    }

    sendChar(checksum); // mandamos el checksum final q calculamos para que se verifique en module A
    sendChar(0x55); // mandamos el byte de finalizar
}