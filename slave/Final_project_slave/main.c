#include "intrinsics.h"
#include "msp430fr2355.h"
#include <msp430.h>
#include <stdbool.h>

unsigned int ADC_one;
unsigned int ADC_two;
unsigned int ADC_three;
unsigned int ADC_four;
unsigned int right = 0;
unsigned int left = 0;
unsigned int up_down = 0;
unsigned int diff = 0;
unsigned int left_right_max = 0;
unsigned int base_period = 3678;
volatile bool uart_send_flag = false;
int Data_Cnt = 0;
char Packet[] = {0x00, 0x00, 0x00, 0x00};
unsigned int adc_read_counter = 0;


char message[]= "Hello World\n";
int position;
int i,j;
// PD Controller Parameters
float Kp = 2;     // Position gain (tune as needed)
float Kd = 0.01;   // Derivative gain (tune as needed)

// Variables to track previous error
int prev_diff = 0;

void init_i2c_b0(void){
        WDTCTL = WDTPW | WDTHOLD;               // Stop watchdog timer

        UCB0CTLW0 |= UCSWRST;                   // Put eUSCI_B0 in SW Reset
        UCB0CTLW0 |= UCSSEL__SMCLK;             // Choose BRCLK = SMCLK = 1Mhz
        UCB0BRW = 10;                           // Divide BRCLK by 10 for SCL = 100kHz
        UCB0CTLW0 |= UCMODE_3;                  // Put into I2C mode
        UCB0CTLW0 |= UCMST;                     // Put into master mode
        UCB0CTLW0 |= UCTR;                      // Put into Tx mode
        UCB0I2CSA = 0x0020;                     // Slave address = 0x20
        UCB0CTLW1 |= UCASTP_2;                  // Auto STOP when UCB0TBCNT reached
        UCB0TBCNT = sizeof(Packet);    // # of bytes in packet

        P1SEL1 &= ~BIT3;                        // P1.3 = SCL
        P1SEL0 |= BIT3;
        P1SEL1 &= ~BIT2;                        // P1.2 = SDA
        P1SEL0 |= BIT2;

        PM5CTL0 &= ~LOCKLPM5;                   // Disable low power mode

        UCB0CTLW0 &= ~UCSWRST;                  // Take eUSCI_B0 out of SW Reset

        UCB0IE |= UCTXIE0;                      // Enable I2C Tx0 IR1
    }

int main(void){
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer
    init_i2c_b0();

    //---------------------------------------------------------------------
    // ADC initialization
    //---------------------------------------------------------------------
    P1SEL1  |= (BIT0 | BIT1 | BIT4 | BIT5);     // initialise all four ADCs
    P1SEL0  |= (BIT0 | BIT1 | BIT4 | BIT5);


    ADCCTL0 &= ~ADCSHT;
    ADCCTL0 |= ADCSHT_2;
    ADCCTL0 |= ADCON;           // Turn ADC on

    ADCCTL1 |= ADCSSEL_2;       // clock SMCLK
    ADCCTL1 |= ADCSHP;

    ADCCTL2 &= ~ADCRES;
    ADCCTL2 |= ADCRES_1;

    //-----------------------------------------------------------------------
    // End ADC initialization
    //-----------------------------------------------------------------------

    //-----------------------------------------------------------------------
    //Timer Initialization
    //-----------------------------------------------------------------------

    TB0CTL |= TBCLR;
    TB0CTL |= TBSSEL__SMCLK;
    TB0CTL |= MC__UP;

    TB0CCR0 = base_period;
    TB0CCR1 = 250;
    TB0CCR2 = TB0R + 50000;       // First trigger after ~50ms

    TB0CCTL0 &= ~CCIFG;
    TB0CCTL0 |= CCIE;
    TB0CCTL1 &= ~CCIFG;
    TB0CCTL1 |= CCIE;
    TB0CCTL2 &= ~CCIFG;
    TB0CCTL2 |= CCIE;


    //-----------------------------------------------------------------------
    // End Timer Initialization
    //-----------------------------------------------------------------------

    //-----------------------------------------------------------------------
    //Motor Port Initialization
    //-----------------------------------------------------------------------

    P3DIR |= (BIT6 | BIT7);
    P3OUT &= ~(BIT6 | BIT7);

    //-----------------------------------------------------------------------
    // End Motor Port Initialization
    //-----------------------------------------------------------------------

    //-----------------------------------------------------------------------
    // Bluetooth UART Initialization
    //-----------------------------------------------------------------------
 /*   UCA0CTLW0 |= UCSWRST;

    UCA0CTLW0 |= UCSSEL__SMCLK;
    //UCA0BRW = 26;                          // Divider for 38400 baud at 1 MHz
    //UCA0MCTLW = 0xB600;       // UCBRSx = 0x00, UCBRFx = 10, UCOS16 = 1


    UCA0BRW = 104;                     // Divider
    UCA0MCTLW = (0x2000 | UCOS16);     // UCBRS=0x20, UCBRF=0, UCOS16=1

    P1SEL1 &= ~BIT7;
    P1SEL0 |= BIT7;

    P1SEL1 &= ~BIT6;
    P1SEL0 |= BIT6;

    UCA0CTLW0 &= ~UCSWRST;

*/
    //-----------------------------------------------------------------------
    // End Bluetooth UART Initialization
    //-----------------------------------------------------------------------



    PM5CTL0 &= ~LOCKLPM5;                       // Turn on gpio
    __enable_interrupt();

    init_i2c_b0();

    while(1){

        ADCCTL0 &= ~ADCENC;
        ADCMCTL0 = ADCINCH_0;
        ADCCTL0 |= ADCENC | ADCSC;  // Enable start conversion
        while((ADCIFG & ADCIFG0) == 00);
        ADC_one = ADCMEM0;

        ADCCTL0 &= ~ADCENC;
        ADCMCTL0 = ADCINCH_1;
        ADCCTL0 |= ADCENC | ADCSC;  // Enable start conversion
        while((ADCIFG & ADCIFG0) == 00);
        ADC_two = ADCMEM0;


        ADCCTL0 &= ~ADCENC;
        ADCMCTL0 = ADCINCH_4;
        ADCCTL0 |= ADCENC | ADCSC;  // Enable start conversion
        while((ADCIFG & ADCIFG0) == 00);
        ADC_three = ADCMEM0;

        ADCCTL0 &= ~ADCENC;
        ADCMCTL0 = ADCINCH_5;
        ADCCTL0 |= ADCENC | ADCSC;  // Enable start conversion
        while((ADCIFG & ADCIFG0) == 00);
        ADC_four = ADCMEM0;



 /*       // send UART transmission after ADC reads
        uart_send_flag = true;

        // Schedule the compare event (must be > TB0R)
        TB0CCR2 = TB0R + 100;        // Schedule ~100 clock cycles in future
        TB0CCTL2 = CCIE;             // Enable interrupt
*/
        // Detect direction and compute difference
        if (ADC_one > ADC_two) {
            left_right_max = ADC_one;
            diff = ADC_one - ADC_two;
            right = 1;
            left = 0;
        }
        else if (ADC_two > ADC_one) {
            left_right_max = ADC_two;
            diff = ADC_two - ADC_one;
            right = 0;
            left = 1;
        }
        else {
            right = 0;
            left = 0;
            diff = 0;
            left_right_max = ADC_one; // or ADC_two
        }

        // Compute derivative (simple difference over loop time)
        left_right_max = (ADC_one > ADC_two) ? ADC_one : ADC_two;
        if (left_right_max > 1023) left_right_max = 1023;
        if (left_right_max < 0)    left_right_max = 0;

        int diff = abs(ADC_one - ADC_two);
        int deriv = diff - prev_diff;
        prev_diff = diff;

        float signal_strength = 1024.0 - left_right_max;
        if (signal_strength < 1.0) signal_strength = 1.0;

        float signal_factor = 1024.0 / signal_strength;

        float control_raw = Kp * diff * signal_factor + Kd * deriv * signal_factor;

        if (control_raw >= base_period) {
            TB0CCR1 = base_period - 1;
        }
        else if (control_raw < base_period/8){
            TB0CCR1 = 1;
        }
        else{
            TB0CCR1 = (unsigned int)control_raw;
        }

        send_ADC();
    }

    return 0;
}

void send_ADC(void) {
    Data_Cnt = 0;
    Packet[0] = (ADC_three >> 8) & 0xFF;  // ADC_three MSB
    Packet[1] = ADC_three & 0xFF;         // ADC_three LSB
    Packet[2] = (ADC_four >> 8) & 0xFF;   // ADC_four MSB
    Packet[3] = ADC_four & 0xFF;          // ADC_four LSB
    UCB0TBCNT = sizeof(Packet);
    UCB0CTLW0 |= UCTXSTT;                               // start condition
    __delay_cycles(50000);
}
#pragma vector = TIMER0_B0_VECTOR
__interrupt void motor_pwm_base_period(void){

    if(right == 1){
        P3OUT |= BIT7; // turn on motor current

    }
    else if(left == 1){
        P3OUT |= BIT6;

    }
    else{
        P3OUT &= ~(BIT6 | BIT7);
    }
    TB0CCTL0 &= ~CCIFG;
}

#pragma vector = TIMER0_B1_VECTOR
__interrupt void timer_b1_isr(void) {
    switch (TB0IV) {
        case TB0IV_TBCCR1:
            // PWM off interrupt (motor duty cycle end)
            P3OUT &= ~(BIT6 | BIT7);
            break;

            Packet[0] = (ADC_three >> 8) & 0xFF;
               Packet[1] = ADC_three & 0xFF;
               Packet[2] = (ADC_four >> 8) & 0xFF;
               Packet[3] = ADC_four & 0xFF;
               Data_Cnt = 0;
               UCB0TBCNT = sizeof(Packet);
               UCB0CTLW0 |= UCTXSTT;  // Send I2C

               // Schedule next event ~50ms later
               TB0CCR2 += 50000;
               break;

        default:
            break;
    }
}
#pragma vector=EUSCI_B0_VECTOR
__interrupt void ADC_Send_ISR(void){
    if (Data_Cnt == (sizeof(Packet) - 1)) {
        UCB0TXBUF = Packet[Data_Cnt];
        Data_Cnt = 0;
    } else {
        UCB0TXBUF = Packet[Data_Cnt];
        Data_Cnt++;
    }
}

