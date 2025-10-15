#include <msp430.h>

volatile unsigned char blinking = 0;
volatile unsigned char debounce = 0;    // DEBOUNCE FLG
volatile unsigned int blink_count = 0;  // COUNTER FOR BLINKS
volatile unsigned int phase = 0;
volatile unsigned int red_flash_count = 0; // COUNT RED FLASH
volatile unsigned char toggle_red = 0;

#define TOT_Y_BLINK 8  // 3 -> 4 BLINKS (TOGGLES 8 TIMES)
#define TOT_R_BLINK 152    // 57 seconds -> 152 toggles (76 full blinks)
#define TIMER_INTERVAL 4500     // SETTING LF ON ACLK
#define FLASH_INTERVAL 1
#define DEBOUNCE_TIME 12500

void DEBOUNCE(void) {
    volatile unsigned int i;
    for (i = 0; i < DEBOUNCE_TIME; i++); // De-bounce time
}


 // RESET ALL VARIABLES, SWITCH OFF RED LEDS AND LISTEN FOR P1.3
void RESET_PROG(void) {
    blinking = 0;
    blink_count = 0;
    phase = 0;
    red_flash_count = 0;
    toggle_red = 0;
    P1OUT &= ~BIT6;
    P2OUT &= ~(BIT1 | BIT3);
    TA0CTL = MC_0;
    TA0CTL |= TACLR;
}

void START_PROG(void) {
    blinking = 1;                 // START BLINK COUNTING
    blink_count = 0;
    TACCR0 = TIMER_INTERVAL - 1;  // TIMEVAL = 0.375 seconds
    TA0CTL |= TASSEL_1 + MC_1;   // ACLK, UP MODE
}

void main(void)
{    WDTCTL = WDTPW | WDTHOLD;


    //OUT SETUP
    P2DIR |= BIT1 | BIT3;
    P2OUT &= ~(BIT1 | BIT3);


    P1DIR |= BIT6;
    P1OUT &= ~BIT6;

    //IN SETUP
    P1DIR &= ~BIT3;               // Set P1.3 as input (BTN)
    P1REN |= BIT3;                // Enable pull-up/down resistor
    P1OUT |= BIT3;                // Set to pull-up resistor
    P1IE  |= BIT3;                // Enable interrupt for P1.3
    P1IES |= BIT3;                // Interrupt on high-to-low transition (button press)
    P1IFG &= ~BIT3;               // Clear interrupt flag for P1.3


    BCSCTL3 |= LFXT1S_2;         // Use VLO AS LFO (12 kHz internal oscillator)


    TACCTL0 = CCIE;               // Enable Timer A interrupt
    TACCR0 = TIMER_INTERVAL - 1;  // Set timer value for 0.375 seconds
    TA0CTL = TASSEL_1 + MC_1 + TACLR;    // ACLK, clear timer


    __bis_SR_register(LPM3_bits + GIE); // LPM3

    while (1) {
        __bis_SR_register(LPM3_bits + GIE); // Sleep until button press

        while (blinking) {
            if (P1IN & BIT3) {
                RESET_PROG();
            }
        }
    }

}


#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer_A(void)
{
    if (blinking) {
        if (phase == 0) { // Yellow phase
            P2OUT = (BIT1 | BIT3);  // Toggle yellow LEDs (on P2.1 and P2.3)
            blink_count++;  // Count the yellow blinks (toggles)

            if (blink_count >= TOT_Y_BLINK) {
                phase = 1;  // Switch to red LED phase
                blink_count = 0;   // Reset the blink count for red phase
                P2OUT &= ~(BIT1 | BIT3);  // Turn off yellow LEDs
            }
        } else {
            red_flash_count++;
            if (red_flash_count >= FLASH_INTERVAL) {
                if (toggle_red) {
                    P1OUT ^= BIT6;  // Toggle red LED (on P1.6)
                    P2OUT &= ~BIT1;

                } else {
                    P2OUT ^= BIT1;   // Toggle red LED (on P2.1)
                    P1OUT &= ~BIT6;
                }
                toggle_red ^= 1; // Switch the toggle flag for the next interval
                red_flash_count = 0; // Reset flash count
            }
            blink_count++;  // Count red cycle

            if (blink_count >= TOT_R_BLINK) {
                RESET_PROG();   // Stop blinking and reset the program after 60 seconds
            }
        }
    }
}


#pragma vector = PORT1_VECTOR
__interrupt void Port_1(void)
{
    if (!debounce) {  // Only process if not in de-bounce state
        if (P1IES & BIT3) {  // If button is pressed (high-to-low transition)
            if (!blinking) {  // Start blinking only if not already blinking
                START_PROG();
                __bic_SR_register_on_exit(LPM3_bits); // Exit low power mode
            }
        } else {  // Button is released (low-to-high transition)
            RESET_PROG();  // Reset the program immediately
        }


        debounce = 1;
        DEBOUNCE();


        P1IFG &= ~BIT3;  // Clear the interrupt flag for P1.3
        P1IES ^= BIT3;   // Toggle edge detection
    }
    debounce = 0;  // Reset de-bounce flag after interrupt
}
