#include <msp430.h>

// Step sequence (1, 2, 4, 8) = P6.0, P6.1, P6.2, P6.3
const unsigned char steps[4] = {1, 2, 4, 8};  // forward
int step_index = 0;
int direction = 1;         // 1 = forward, 0 = backward
int step_count = 0;
int position = 0;  // Track current step offset, -40 to +40
int position_limit = 40;

float Kp = 1.0;
float Kd = 0.05;

int busy = 0;
   int Data_Cnt = 0;
   int Data_In[] = {0x00, 0x00, 0x00, 0x00};

   int ADC3_thou = 0;
   int ADC3_tens = 0;
   int ADC3_ones = 0;
   int ADC3_decimal = 0;

   int ADC4_thou = 0;
   int ADC4_tens = 0;
   int ADC4_ones = 0;
   int ADC4_decimal = 0;

   unsigned int ADC_3_high = 0;
   unsigned int ADC_3_low = 0;
   unsigned int ADC_4_high = 0;
   unsigned int ADC_4_low = 0;

   unsigned int ADC_3 = 0;
   unsigned int ADC_4 = 0;

void i2c_b0_init(void) {
    WDTCTL = WDTPW | WDTHOLD;               // Stop watchdog timer

    UCB0CTLW0 |= UCSWRST;                   // Put eUSCI_B0 in SW Reset
    UCB0CTLW0 |= UCMODE_3;                  // Put into I2C mode
    UCB0CTLW0 |= UCSYNC;                    // Synchronous
    UCB0CTLW0 &= ~UCMST;                    // Put into slave mode
    UCB0I2COA0 = 0x0020 | UCOAEN;           // Own address + enable bit
    UCB0CTLW1 &= ~UCASTP_3;                 // Use manual stop detection

    P1SEL1 &= ~BIT3;                        // P1.3 = SCL
    P1SEL0 |= BIT3;
    P1SEL1 &= ~BIT2;                        // P1.2 = SDA
    P1SEL0 |= BIT2;

    PM5CTL0 &= ~LOCKLPM5;                   // Disable low power mode

    UCB0CTLW0 &= ~UCSWRST;                  // Take eUSCI_B0 out of SW Reset

    UCB0IE |= UCRXIE0 | UCSTPIE | UCSTTIE;  // Enable I2C Rx0 IR1
    __enable_interrupt();                   // Enable Maskable IRQs
}

void main(void){

    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer

    // Set motor pins as output (P6.0 - P6.3)
    //P6DIR |= 0x0F;
    //P6OUT &= ~0x0F;
    P6DIR   |= 0b00001111;
    P6OUT   &= ~0b00001111;

    //-- control bits
    P2DIR   |= 0b00000111;
    P2OUT   &= ~0b00000111;
    // Unlock GPIO
    PM5CTL0 &= ~LOCKLPM5;

    // TimerB setup for ~60ms interval (with SMCLK at 1 MHz)
    TB0CTL = TBSSEL__SMCLK | MC__UP | TBCLR;  // SMCLK, up mode
    TB0CCR0 = 6000;                          // ~60ms period
    TB0CCTL0 = CCIE;                          // Enable interrupt

    __enable_interrupt();  // Global interrupt enable
    i2c_b0_init();
    setup();
    while (1){
        __delay_cycles(100000);
        i2c_b0_init();
        process_i2c_data();


    }
}

void process_i2c_data(void) {
    ADC_3_high = Data_In[0] & 0x03;  // Keep only lowest 2 bits
    ADC_3_low = Data_In[1];
    ADC_4_high = Data_In[2] & 0x03;  // Keep only lowest 2 bits
    ADC_4_low = Data_In[3];

    ADC_3 = (ADC_3_high << 8) | ADC_3_low;
    ADC_4 = (ADC_4_high << 8) | ADC_4_low;

    int diff = ADC_3 - ADC_4;
    int abs_diff = (diff > 0) ? diff : -diff;
    int max_adc = (ADC_3 > ADC_4) ? ADC_3 : ADC_4;

    float signal_strength = 1024.0 - max_adc;
    if (signal_strength < 1.0) signal_strength = 1.0;

    float signal_factor = 1024.0 / signal_strength;
    static int prev_diff = 0;
    int deriv = abs_diff - prev_diff;
    prev_diff = abs_diff;

    float control = Kp * abs_diff * signal_factor + Kd * deriv * signal_factor;

    // Map control (speed) to timer period: faster = lower CCR0
    // Clamp: min 1500 (fast), max 12000 (slow)
    unsigned int new_period = 12000 - (unsigned int)(control);
    if (new_period < 1500) new_period = 1500;
    if (new_period > 12000) new_period = 12000;
    TB0CCR0 = new_period;

    // Change direction to follow stronger signal
    if (position <= -position_limit) {
        direction = 1;  // force toward center
    } else if (position >= position_limit) {
        direction = 0;  // force toward center
    } else {
        direction = (ADC_3 < ADC_4) ? 1 : 0;  // normal tracking
    }
    print_ADC(ADC_3, ADC_4);
    __delay_cycles(1000000);
}

void print_ADC(int ADC3, int ADC4){

    // Return Home -------------------------------------------
    byte(0b0000, 0b0010, 0, 0);
    clear_display();

    ADC3_tens = ADC3 / 100;
    ADC3_ones = ADC3 / 10 - ADC3_tens * 10;
    ADC3_decimal = ADC3 - ADC3_tens * 100 - ADC3_ones * 10;


    ADC4_tens = ADC4 / 100;
    ADC4_ones = ADC4 / 10 - ADC4_tens * 10;
    ADC4_decimal = ADC4 - ADC4_tens * 100 - ADC4_ones * 10;


    byte(0b0100, 0b0001,1,0);
    byte(0b0100, 0b0100,1,0);
    byte(0b0100, 0b0011,1,0);
    byte(0b0011, 0b0011,1,0);
    byte(0b0011, 0b1010,1,0);
    byte(0b011, ADC3_tens,1,0);
    byte(0b0011,ADC3_ones,1,0);
    byte(0b0011,ADC3_decimal,1,0);

    byte(0b1100,0b0000,0,0);        // move to second row
    byte(0b0100, 0b0001,1,0);
    byte(0b0100, 0b0100,1,0);
    byte(0b0100, 0b0100,1,0);
    byte(0b0011, 0b0100,1,0);
    byte(0b0011, 0b1010,1,0);
    byte(0b011,ADC4_tens,1,0);
    byte(0b0011,ADC4_ones,1,0);
    byte(0b0011,ADC4_decimal,1,0);

}
int shift_right(void){

    byte(0b0010,0b0000,1,0);
}

void clear_top_row(){
    return_home();
    int i = 0;
    for(i=0; i<16; i=i+1){
        shift_right();
    }

}
int byte(int upper, int lower, int RS, int RW){
    check_busy();
    reset_flags(RS,RW);
    __delay_cycles(4);
    P6OUT   &= ~0b1111;
    P6OUT   |= upper;
    P2OUT   |= BIT0;        //Enable pin
    __delay_cycles(4);
    P2OUT   &= ~BIT0;
    __delay_cycles(4);
    //check_busy();
    //reset_flags(RS,RW);
    __delay_cycles(4);
    P6OUT   &= ~0b1111;
    P6OUT   |= lower;
    P2OUT   |= BIT0;
    __delay_cycles(4);
    P2OUT   &= ~BIT0;
    __delay_cycles(20);
    // End DD Ram set key-------------------------------------------
    return 0;
}

void reset_flags(RS, RW){
    if(RS == 1){
        P2OUT   |= BIT2;
    }
    else if (RS == 0) {
        P2OUT  &= ~BIT2;
    }

    if(RW == 1){
        P2OUT   |= BIT1;
    }
    else if (RW == 0) {
        P2OUT &= ~BIT1;
    }
}
int clear_display(void){
    // Clear display
    byte(0b0000, 0b0001, 0, 0);

    // End Clear display -------------------------------------------
}

int return_home(void){
    // Return Home -------------------------------------------
    byte(0b0000, 0b0010, 0, 0);

    // End Return Home -------------------------------------------
}
int setup(void){
    // 4 Bit Mode -----------------------------------------
    //__delay_cycles(4000);
    byte(0b0010, 0b1100, 0, 0);
    //__delay_cycles(200);

    //  End 4 Bit Mode -----------------------------------------

    clear_display();
     //__delay_cycles(200);

    return_home();
    //__delay_cycles(200);
    // Display on -------------------------------------------
    byte(0b0000, 0b1100,0,0);
    //__delay_cycles(200);
    //End Display on  -------------------------------------------
    return 0;
}

void check_busy(void){
    busy = 0;
    P6DIR   &= ~0b1111;
    P6REN   |= 0b1111;
    P6OUT   &= ~0b1111;

    P2OUT   &= ~BIT2;
    P2OUT   |= BIT1;
    __delay_cycles(1000);
    internal_check_busy();
    P6DIR   |= 0b00001111;
    P6OUT   &= ~0b00001111;

    P2OUT   &= ~BIT2;
    P2OUT   &= ~BIT1;
    __delay_cycles(1000);

}
void internal_check_busy(){
    P2OUT   |= BIT0;
    busy    = P6IN;
    busy    &= BIT3;
    __delay_cycles(1000);


    P2OUT   &= ~BIT0;
    __delay_cycles(1000);

    P2OUT   |= BIT0;

    __delay_cycles(1000);

    P2OUT   &= ~BIT0;
    __delay_cycles(1000);

    if(busy != 0){
        internal_check_busy();

    }
}


// Timer ISR — advance motor step
#pragma vector = TIMER0_B0_VECTOR
__interrupt void Timer0_B0_ISR(void)
{
    // Step only if within position bounds
    if ((direction && position < position_limit) ||
        (!direction && position > -position_limit)) {

        if (direction) {
            step_index = (step_index + 1) % 4;
            position++;
        } else {
            step_index = (step_index + 3) % 4;
            position--;
        }

        P6OUT = steps[step_index];
        step_count++;
    }
}

#pragma vector=EUSCI_B0_VECTOR
__interrupt void LED_I2C_ISR(void){
    switch(__even_in_range(UCB0IV, USCI_I2C_UCBIT9IFG)) {
        case USCI_NONE: break;

        case USCI_I2C_UCSTTIFG:   // START condition
            Data_Cnt = 0;         // Reset buffer index
            break;

        case USCI_I2C_UCRXIFG0:   // Byte received
            if (Data_Cnt < 4) {
                Data_In[Data_Cnt++] = UCB0RXBUF;
            }
            if (Data_Cnt == 4) {
                process_i2c_data();
            }
            break;

        case USCI_I2C_UCSTPIFG:   // STOP condition = end of transmission
            UCB0IFG &= ~UCSTPIFG;  // Clear stop flag
            break;

        default: break;


    }
}
