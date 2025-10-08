#include <msp430.h>

volatile unsigned char blinking = 0;    // State to track blinking status
volatile unsigned char debounce = 0;    // De-bounce flag
volatile unsigned int blink_count = 0;  // Variable to count the number of blinks
volatile unsigned int phase = 0;         // 0: Yellow phase, 1: Red phase
volatile unsigned int red_flash_count = 0; // Counter for red flash intervals
volatile unsigned char toggle_red = 0;   // Flag to toggle between red LEDs

#define TOTAL_YELLOW_BLINKS 8  // 3 seconds -> 8 toggles (4 full blinks)
#define TOTAL_RED_BLINKS 152    // 57 seconds -> 152 toggles (76 full blinks)
#define TIMER_INTERVAL 4500      // Timer interval for 0.375 seconds with ACLK = 12 kHz
#define RED_FLASH_INTERVAL 1      // Number of timer intervals between red flashes
#define DEBOUNCE_TIME 12500      // De-bounce time

void debounceDelay(void) {
    volatile unsigned int i;
    for (i = 0; i < DEBOUNCE_TIME; i++); // De-bounce time
}

void reset_program(void) {
    blinking = 0;                 // Stop blinking
    blink_count = 0;              // Reset blink count
    phase = 0;                    // Reset to yellow phase
    red_flash_count = 0;          // Reset red flash counter
    toggle_red = 0;               // Reset red LED toggle flag
    P1OUT &= ~BIT6;               // Turn off red LED
    P2OUT &= ~(BIT1 | BIT3);      // Turn off yellow LEDs
    TA0CTL = MC_0;                // Stop the timer
    TA0CTL |= TACLR;              // Clear the timer
}

void start_blinking(void) {
    blinking = 1;                 // Start blinking
    blink_count = 0;              // Reset blink count
    TACCR0 = TIMER_INTERVAL - 1;  // Set timer value for 0.375 seconds
    TA0CTL |= TASSEL_1 + MC_1;   // ACLK, up mode
}

void main(void)
{
    WDTCTL = WDTPW | WDTHOLD;     // Stop watchdog timer

    // Set P2.1 and P2.3 as output for yellow LEDs
    P2DIR |= BIT1 | BIT3;
    P2OUT &= ~(BIT1 | BIT3);      // Turn off yellow LEDs initially

    // Set P1.6 as output for red LED
    P1DIR |= BIT6;
    P1OUT &= ~BIT6;               // Turn off red LEDs initially

    // Set P1.3 as input for button
    P1DIR &= ~BIT3;               // Set P1.3 as input
    P1REN |= BIT3;                // Enable pull-up/down resistor
    P1OUT |= BIT3;                // Set to pull-up resistor
    P1IE  |= BIT3;                 // Enable interrupt for P1.3
    P1IES |= BIT3;                // Interrupt on high-to-low transition (button press)
    P1IFG &= ~BIT3;               // Clear interrupt flag for P1.3

    // Set low-frequency oscillator
    BCSCTL3 |= LFXT1S_2;          // Use VLO (12 kHz internal oscillator)

    // Timer setup
    TACCTL0 = CCIE;               // Enable Timer A interrupt
    TACCR0 = TIMER_INTERVAL - 1;  // Set timer value for 0.375 seconds
    TA0CTL = TASSEL_1 + TACLR;    // ACLK, clear timer

    // Enter low-power mode
    __bis_SR_register(LPM3_bits + GIE); // Enter LPM3 with interrupts enabled

    while (1) {
        if (!blinking) {
            __bis_SR_register(LPM3_bits + GIE); // Go back to low power mode if not blinking
        } else {
            // Check button state continuously during blinking
            if (P1IN & BIT3) {  // If the button is released
                reset_program(); // Reset the program if button is released
            }
        }
    }
}

// Timer A interrupt service routine
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer_A(void)
{
    if (blinking) {
        if (phase == 0) { // Yellow phase
            P2OUT = (BIT1 | BIT3);  // Toggle yellow LEDs (on P2.1 and P2.3)
            blink_count++;  // Count the yellow blinks (toggles)

            if (blink_count >= TOTAL_YELLOW_BLINKS) {
                phase = 1;  // Switch to red LED phase
                blink_count = 0;   // Reset the blink count for red phase
                P2OUT &= ~(BIT1 | BIT3);  // Turn off yellow LEDs
            }
        } else { // Red phase (intermittent flashing)
            red_flash_count++;
            if (red_flash_count >= RED_FLASH_INTERVAL) {
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
            blink_count++;  // Count the red phase cycles

            if (blink_count >= TOTAL_RED_BLINKS) {
                reset_program();   // Stop blinking and reset the program after 60 seconds
            }
        }
    }
}

// Port 1 interrupt service routine (Button press)
#pragma vector = PORT1_VECTOR
__interrupt void Port_1(void)
{
    if (!debounce) {  // Only process if not in de-bounce state
        if (P1IES & BIT3) {  // If button is pressed (high-to-low transition)
            if (!blinking) {  // Start blinking only if not already blinking
                start_blinking();
                __bic_SR_register_on_exit(LPM3_bits); // Exit low power mode
            }
        } else {  // Button is released (low-to-high transition)
            reset_program();  // Reset the program immediately
        }

        debounce = 1;  // Set de-bounce flag
        debounceDelay();  // Call de-bounce delay

        // Clear interrupt flag
        P1IFG &= ~BIT3;  // Clear the interrupt flag for P1.3
        P1IES ^= BIT3;   // Toggle edge detection (for press and release)
    }
    debounce = 0;  // Reset de-bounce flag after handling the interrupt
}
