#include "msp430f5438a.h"

#define MOTORDIR P4DIR
#define MOTOROUT P4OUT
#define MOTORSEL P4SEL
#define MOTOR    BIT1
#define MOTORCONTROL TB0CCTL1
#define MOTORLEVEL TB0CCR1

#define LIGHTDIR P4DIR
#define LIGHTOUT P4OUT
#define LIGHTSEL P4SEL
#define LIGHT    BIT2
#define LIGHTCONTROL TB0CCTL2
#define LIGHTLEVEL TB0CCR2

#define UARTOUT P1OUT
#define UARTSEL P1SEL
#define UARTDIR P1DIR

#define UARTTXBIT BIT1
#define UARTRXBIT BIT2

#define DCO_SPEED 1048000uL
#define BitTime_9600   (DCO_SPEED / 9600)
#define BitTime BitTime_9600
#define DELAY1 (DCO_SPEED / 2)
#define DELAY10 (DCO_SPEED / 10)

void
uart_init()
{
  UARTOUT &= ~(UARTRXBIT + UARTTXBIT); 
  UARTSEL |= UARTTXBIT + UARTRXBIT;   // Timer function for TXD/RXD pins
  UARTDIR |= UARTTXBIT;               // Set all pins but RXD to output
  UARTDIR &= ~UARTRXBIT;

  TA0CCTL0 = OUTMOD0;                 // Set TXD Idle as Mark = '1'
  TA0CTL = TASSEL_2 + MC_2;           // SMCLK, start in continuous mode
}

void ml_init() {
  LIGHTDIR |= LIGHT;
  LIGHTSEL |= LIGHT;

  MOTORSEL |= MOTOR;
  MOTORDIR |= MOTOR;

  TB0CTL |= TBSSEL_1 + MC_1;
  TB0CCR0 = 32; // control PWM freq = 32768/16 = 2048hz

  MOTORCONTROL = OUTMOD_0;
  LIGHTCONTROL = OUTMOD_0;
}

void m_on() {
  MOTORCONTROL = OUTMOD_7;
}

void l_on() {
  LIGHTCONTROL = OUTMOD_7;
}

void ml_on() {
  m_on();
  l_on();
}

void m_level(unsigned int level) {
  MOTORLEVEL = level * 4;
}

void l_level(unsigned int level) {
  LIGHTLEVEL = level * 2;
}

void ml_level(unsigned int level) {
  m_level(level);
  l_level(level);
}

void m_off() {
  MOTORCONTROL = OUTMOD_0;
}

void l_off() {
  LIGHTCONTROL = OUTMOD_0;
}

void ml_off() {
  m_off();
  l_off();
}

volatile unsigned int i; // volatile to prevent optimization
volatile unsigned int j; // volatile to prevent optimization
volatile unsigned int l; // volatile to prevent optimization

int serchar(int data)
{
    int tempData;
    int parity_mask = 0x200;
    char bitCount = 0xB;                    // Load Bit counter, 8data + ST/SP +parity

    TA0CCR0 = TA0R;                       // Current state of TA counter
    TA0CCR0 += BitTime;
    tempData = 0x200 + (int)data;           // Add mark stop bit to Data
    tempData = tempData << 1;

    while (bitCount != 0)
    {
        while (!(TA0CCTL0 & CCIFG)) ;
        TA0CCTL0 &= ~CCIFG;
        TA0CCR0 += BitTime;
        if (tempData & 0x01)
        {
            tempData ^= parity_mask;
            TA0CCTL0 &= ~OUTMOD2;         // TX '1'
        }
        else
        {
            TA0CCTL0 |=  OUTMOD2;             // TX '0'
        }

        parity_mask = parity_mask >> 1;
        tempData = tempData >> 1;
        bitCount--;
    }
    while (!(TA0CCTL0 & CCIFG)) ;         // wait for timer

    return data;
}

void watchdog_init() {
  WDTCTL = WDTPW | WDTHOLD; // Stop watchdog timer
  SFRIFG1 &= ~WDTIFG;
  SFRIE1 |= WDTIE;
}

void main(void) {
  __disable_interrupt();
  watchdog_init();
  __enable_interrupt();

  uart_init();
  __delay_cycles(DELAY1);
  ml_init();

  for (j = 0;j < 3;j++) {
    serchar(65 + j);
    ml_on();
    __delay_cycles(DELAY1);
    for (l = 0;l < 8;l++) {
      ml_level(l);
      serchar(48 + l);
      __delay_cycles(DELAY1 / 2);
    }
    for (l = 0;l < 9;l++) {
      ml_level(8 - l);
      serchar(48 + 8 - l);
      __delay_cycles(DELAY1 / 2);
    }
    ml_off();
    __delay_cycles(DELAY1);
  }

  l_level(8);
  do {
    l_on();
    __delay_cycles(DELAY10 * 1);
    l_off();
    __delay_cycles(DELAY10 * 9);
  } while(1);
}
