#include <msp430.h> 

    int col_1 = 0;
    int col_2 = 0;
    int col_3 = 0;
    int col_4 = 0;
    int col = 0;

    int key = 20;


/**
 * main.c
 */
int main(void)
{
    init_keypad();
    WDTCTL = WDTPW | WDTHOLD;                    // Stop watchdog timer
    PM5CTL0 &= ~LOCKLPM5;                        // Disable High Z mode

    while(1){
        get_key();
    }

}
void init_keypad(void) {

    //--set up ports
    P1DIR |=  (BIT4   |   BIT5   |   BIT6   |   BIT7);  // Set keypad row pins as outputs
    P1OUT |=  (BIT4   |   BIT5   |   BIT6   |   BIT7);  // sets output high to start

    // Set column pins as input with pull-down resistor
    P2DIR &= ~(BIT0   |   BIT1   |   BIT2   |   BIT3); // Set P1.4 - p1.7 as input
    P2REN |=  (BIT0   |   BIT1   |   BIT2   |   BIT3); // Enable pull-up/down resistors
    P2OUT &= ~(BIT0   |   BIT1   |   BIT2   |   BIT3); // Set as pull-down
}

void get_column()   // Determine which column out of the four has the button being pressed
{
    int col_1 = (P2IN & BIT0) ? 1 : 0; // Check if the pressed button is in column 1
    int col_2 = (P2IN & BIT1) ? 1 : 0; // Check if the pressed button is in column 2
    int col_3 = (P2IN & BIT2) ? 1 : 0; // Check if the pressed button is in column 3
    int col_4 = (P2IN & BIT3) ? 1 : 0; // Check if the pressed button is in column 4

    if (col_1 == 1) {
        col = 1;
    }

    else if (col_2 == 1) {
        col = 2;
    }

    else if (col_3 == 1) {
        col = 3;
    }

    else if (col_4 == 1) {
        col = 4;
    }

    else {
        col = 0;  // No key pressed
    }
}

void get_key()
{
    P1OUT &= ~(BIT5 | BIT6 | BIT7);  // Clear outputs to start, except BIT4
    P1OUT |= BIT4; // Activate first row
    get_column();

    if (col == 1) {
        key = 0x1;
    } else if (col == 2) {
        key = 0x2;
    } else if (col == 3) {
        key = 0x3;
    } else if (col == 4) {
        key = 0xa;
    }

    P1OUT &= ~BIT4;

    P1OUT |= BIT5; // Activate second row
    get_column();

    if (col == 1) {
        key = 0x4;
    } else if (col == 2) {
        key = 0x5;
    } else if (col == 3) {
        key = 0x6;
    } else if (col == 4) {
        key = 0xb;
    }

    P1OUT &= ~BIT5;

    P1OUT |= BIT6; // Activate third row
    get_column();

    if (col == 1) {
        key = 0x7;
    } else if (col == 2) {
        key = 0x8;
    } else if (col == 3) {
        key = 0x9;
    } else if (col == 4) {
        key = 0xc;
    }

    P1OUT &= ~BIT6;

    P1OUT |= BIT7; // Activate fourth row
    get_column();

    if (col == 1) {
        key = 0xe;
    } else if (col == 2) {
        key = 0x0;
    } else if (col == 3) {
        key = 0xf;
    } else if (col == 4) {
        key = 0xd;
    }

    P1OUT &= ~BIT7;
}

