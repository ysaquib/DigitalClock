#include <stdint.h>
#include <stdbool.h>
#include <tm4c123gh6pm.h>
#include <sysctl.h>
#include "timer.h"
#include "Serial.h"

/**
 * main.c
 */

#define PB_DATA GPIO_PORTB_DATA_R
#define PF_DATA GPIO_PORTF_DATA_R

#define ZERO 0x01
#define ONE 0x4F
#define TWO 0x12
#define THREE 0x06
#define FOUR 0x4C
#define FIVE 0x24
#define SIX 0x20
#define SEVEN 0x0F
#define EIGHT 0x00
#define NINE 0x04
#define EMPTY 0x7F

#define D_ZERO 0x07
#define D_ONE 0x0B
#define D_TWO 0x0D
#define D_THREE 0x0E
#define D_HR_OFF
#define D_MIN_OFF

uint16_t digits[4] = {D_ZERO,D_ONE,D_TWO,D_THREE};
uint16_t numbers[11] = {ZERO, ONE,TWO,THREE,FOUR,FIVE,SIX,SEVEN,EIGHT,NINE,EMPTY};

int mode = 0; // 0 for clock, 1 for timer
bool isCountDownDone = false;

int seconds = 7;
int hour_tens = 0;
int hour_ones = 0;
int min_tens = 0;
int min_ones = 0;
int colon = 1;

bool isEditingTimer = false;

int sec = 0;

void Timer0A_Handler(void)
{
    TIMER0_ICR_R = 0x000000FF; // acknowledge the interrupt


    sec++;
    if (sec == 60)
        sec = 0;

    if (isEditingTimer)
        colon = 1;
    if (mode == 0 && !isEditingTimer)
    {
        colon = colon ^ 1;
        seconds++;
        if (seconds == 60)
        {
            seconds = 0;
            min_ones++;
            if (min_ones == 10)
            {
                min_tens++;
                min_ones = 0;
                if (min_tens == 6)
                {
                    min_tens = 0;
                    hour_ones++;

                    if (hour_tens == 2 && hour_ones == 4)
                    {
                        hour_ones = 0;
                        hour_tens = 0;
                    }
                    if (hour_ones == 10)
                    {
                        hour_tens++;
                        hour_ones = 0;
                    }


                }
            }
        }
    }
    if (mode == 1 && !isEditingTimer)
    {
        colon = colon ^ 1;
        seconds--;
        if (seconds == -1)
        {
            seconds = 59;
            min_ones--;
            if (min_ones == -1)
            {
                min_ones = 9;
                min_tens--;
                if (min_tens == -1)
                {
                    min_tens = 5;
                    hour_ones--;
                    if (hour_ones == -1)
                    {
                        if (hour_tens == 0)
                        {
                            hour_ones = 0;
                            isCountDownDone = true;
                        }
                        else
                        {
                            hour_ones = 9;
                            hour_tens--;
                        }
                    }
                }

            }
        }
    }
}

void PLLInit()
{
    SYSCTL_RCC2_R |= 0x80000000;
    SYSCTL_RCC2_R |= 0x00000800;
    SYSCTL_RCC_R = (SYSCTL_RCC_R & ~0x000007C0) + 0x00000540;
    SYSCTL_RCC2_R &= ~0x00000070;
    SYSCTL_RCC2_R &= ~0x00002000;
    SYSCTL_RCC2_R |= 0x40000000;
    SYSCTL_RCC2_R = (SYSCTL_RCC2_R & ~0x1FC00000) + (4 << 22);
    while ((SYSCTL_RIS_R &0x00000040)==0){};
    SYSCTL_RCC2_R &= ~0x00000800;
}

void TimerInit(void)
{
    // ENABLE TIMER
    SYSCTL_RCGCTIMER_R |= 0x01;//|= 0x01;                 //  Activate Timer 0
    TIMER0_CTL_R = 0x00000000;                  //  Disable Timer 0 for configuration
    TIMER0_CFG_R = 0x00000008;                  //  Configure for 32-Bit capture mode
    TIMER0_TAMR_R = 0x00000002;
    TIMER0_TAILR_R = 0x04C4B400;                //  Starting value for count-down
    TIMER0_TAPR_R = 0x000000; //0xFFFFFF;       //  Deactivate pre-scaler
    TIMER0_IMR_R |= 0x00000001;                 //  Enable input capture interrupts
    TIMER0_ICR_R = 0x00000004;//0x00000001;     //  Clear Timer 0 A Capture Match flags

    NVIC_PRI4_R = (NVIC_PRI4_R & 0x00FFFFFF) | 0x40000000;
    NVIC_PRI5_R = (NVIC_PRI5_R & 0x00FFFFFF) | 0x80000000;
    NVIC_EN0_R = 1 << 19;
    TIMER0_CTL_R |= 0x00000001;
}

void Init(void)
{
    PLLInit();
    SystickInit();
    TimerInit();
    SetupSerial();

    SYSCTL_RCGCGPIO_R |= 0x22;              // enable clock for PORT B & F

    GPIO_PORTB_LOCK_R = 0x4C4F434B;         // this value unlocks the GPIOCR register.
    GPIO_PORTB_CR_R = 0xFF;
    GPIO_PORTB_AMSEL_R = 0x00;              // disable analog functionality
    GPIO_PORTB_PCTL_R = 0x00000000;         // Select GPIO mode in PCTL
    GPIO_PORTB_DIR_R = 0x07;                // Pins B0-B2 are outputs
    GPIO_PORTB_AFSEL_R |= 0x40;//= 0x00;              // Disable alternate functionality
    GPIO_PORTB_DEN_R = 0xFF;                // Enable digital ports

    GPIO_PORTB_DEN_R |= 0x40; // Enable Digital Functionality
    GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & 0xF0FFFFFF) + 0x07000000;


    GPIO_PORTF_LOCK_R = 0x4C4F434B; // this value unlocks the GPIOCR register.
    GPIO_PORTF_CR_R = 0xFF;
    GPIO_PORTF_AMSEL_R = 0x00; // disable analog functionality
    GPIO_PORTF_PCTL_R = 0x00000000; // Select GPIO mode in PCTL
    GPIO_PORTF_DIR_R = 0x0E; // Port 0 and 4 are input
    GPIO_PORTF_AFSEL_R = 0x00; // Disable alternate functionality
    GPIO_PORTF_DEN_R = 0xFF; //Enable digital ports
    GPIO_PORTF_PUR_R = 0x11; // enable pullup resistors on PF0 and PF4
}

// GREEN:        PB0         DATA
// BLUE:         PB1         CLOCK
// YELLOW:       PB2         LATCH

/*
 * @requires: seq >= 0
 */
int i;
int bit;
void shift12Bits(uint16_t seq)
{
    PB_DATA &= (~0x07); // LATCH AND CLOCK LOW
    for (i = 0; i < 12; i++)
    {
        bit = seq & 0x01;
        PB_DATA = (PB_DATA & 0x00) + bit; // Set the Data with least significant bit of seq
        seq >>= 1; // Shift seq
        PB_DATA |= 0x02; // Set the clock to high.
        SysTick_Wait100nanosec(1);
        PB_DATA &= 0x01; // Set the clock to low
    }
    // All 12 bits are shifted to the bit shift register
    PB_DATA |= 0x04; // LATCH HIGH
}

/*
 *  @requires: 0 <= colon && colon <= 1
 *          && 0 <= digit && digit <= 0x0E
 *          && 0 <= number && number <= 0x09
 */

int createSequence (int digit, int number, int colon)
{
    return ((digits[digit] << 8) + (numbers[number] << 1) + (colon ^ 1));
}

int main(void)
{
    Init();
    bool b1pressed = false;
    bool b1released = false;
    bool b2pressed = false;
    bool b2released = false;
    int b1counter = 0;
    int b2counter = 0;

    int delay = 2;

    int editingMode = 0;


    while (1)
    {

        if (PF_DATA == 0x01) // SW 1
        {
            b1pressed = true;
            b1counter++;
        }
        else if (PF_DATA == 0x10) // SW 2
        {
            b2pressed = true;
            b2counter++;
        }
        else
        {
            if (b1pressed && b1counter)
            {
                b1released = true;
                b1pressed = false;
                b1counter = 0;
            }
            else if (b2pressed && b2counter)
            {
                b2released = true;
                b2pressed = false;
                b2counter = 0;
            }
        }

        if (b1released)
        {
            editingMode = (editingMode + 1) % 3; // 1 is hours; 2 is minutes
            if (editingMode == 0)
                isEditingTimer = false;
            else
                isEditingTimer = true;
            b1released = false;
        }
        if (b2released)
        {
            if (editingMode == 1)
            {
                hour_ones++;
                if (hour_ones == 10)
                {
                    hour_ones = 0;
                    hour_tens++;
                }
                if (hour_ones == 4 && hour_tens == 2)
                {
                    hour_ones = 0;
                    hour_tens = 0;
                }
            }
            if (editingMode == 2)
            {
                min_ones++;
                if (min_ones == 10)
                {
                    min_ones = 0;
                    min_tens++;
                    if (min_tens == 6)
                    {
                        min_tens = 0;
                    }
                }
            }
            else
            {
                seconds = 0;
            }
            b2released = false;
        }

        if (isEditingTimer)
        {
            if (sec % 2 > 0)
            {
                shift12Bits (createSequence(0,hour_tens, colon));       // Enable digit 0
                SysTick_Wait1ms(delay);                                     // Wait 2 ms
                shift12Bits (createSequence(1,hour_ones, colon));       // Enable digit 1
                SysTick_Wait1ms(delay);                                     // Wait 2 ms
                shift12Bits (createSequence(2,min_tens, colon));        // Enable digit 2
                SysTick_Wait1ms(delay);                                     // Wait 2 ms
                shift12Bits (createSequence(3,min_ones, colon));        // Enable digit 3
                SysTick_Wait1ms(delay);                                     // Wait 2 ms
            }
            else
            {
                if (editingMode == 1){
                    shift12Bits (createSequence(0,10, colon));       // Enable digit 0
                    SysTick_Wait1ms(delay);                                     // Wait 2 ms
                    shift12Bits (createSequence(1,10, colon));       // Enable digit 1
                    SysTick_Wait1ms(delay);                                     // Wait 2 ms
                    shift12Bits (createSequence(2,min_tens, colon));        // Enable digit 2
                    SysTick_Wait1ms(delay);                                     // Wait 2 ms
                    shift12Bits (createSequence(3,min_ones, colon));        // Enable digit 3
                    SysTick_Wait1ms(delay);
                }
                else if (editingMode == 2)
                {
                    shift12Bits (createSequence(0,hour_tens, colon));       // Enable digit 0
                    SysTick_Wait1ms(delay);                                     // Wait 2 ms
                    shift12Bits (createSequence(1,hour_ones, colon));       // Enable digit 1
                    SysTick_Wait1ms(delay);                                     // Wait 2 ms
                    shift12Bits (createSequence(2,10, colon));        // Enable digit 2
                    SysTick_Wait1ms(delay);                                     // Wait 2 ms
                    shift12Bits (createSequence(3,10, colon));        // Enable digit 3
                    SysTick_Wait1ms(delay);
                }
                else
                {
                    shift12Bits (createSequence(0,hour_tens, colon));       // Enable digit 0
                    SysTick_Wait1ms(delay);                                     // Wait 2 ms
                    shift12Bits (createSequence(1,hour_ones, colon));       // Enable digit 1
                    SysTick_Wait1ms(delay);                                     // Wait 2 ms
                    shift12Bits (createSequence(2,min_tens, colon));        // Enable digit 2
                    SysTick_Wait1ms(delay);                                     // Wait 2 ms
                    shift12Bits (createSequence(3,min_ones, colon));        // Enable digit 3
                    SysTick_Wait1ms(delay);
                }
            }
        }


        if (mode == 0 && isEditingTimer == false)
        {
            shift12Bits (createSequence(0,hour_tens, colon));       // Enable digit 0
            SysTick_Wait1ms(delay);                                     // Wait 2 ms
            shift12Bits (createSequence(1,hour_ones, colon));       // Enable digit 1
            SysTick_Wait1ms(delay);                                     // Wait 2 ms
            shift12Bits (createSequence(2,min_tens, colon));        // Enable digit 2
            SysTick_Wait1ms(delay);                                     // Wait 2 ms
            shift12Bits (createSequence(3,min_ones, colon));        // Enable digit 3
            SysTick_Wait1ms(delay);                                     // Wait 2 ms
        }
        else if (mode == 1 && !isEditingTimer)
        {
            if (isCountDownDone == false)
            {
                shift12Bits (createSequence(0,hour_tens, colon));       // Enable digit 0
                SysTick_Wait1ms(delay);                                     // Wait 2 ms
                shift12Bits (createSequence(1,hour_ones, colon));       // Enable digit 1
                SysTick_Wait1ms(delay);                                     // Wait 2 ms
                shift12Bits (createSequence(2,min_tens, colon));        // Enable digit 2
                SysTick_Wait1ms(delay);                                     // Wait 2 ms
                shift12Bits (createSequence(3,min_ones, colon));        // Enable digit 3
                SysTick_Wait1ms(delay);                                     // Wait 2 ms
            }
            else
            {
                sec = 50;
                while (sec < 59)
                {
                    if(sec % 2 == 0)
                    {
                        shift12Bits (createSequence(0,0, colon));       // Enable digit 0
                        SysTick_Wait1ms(delay);                                     // Wait 2 ms
                        shift12Bits (createSequence(1,0, colon));       // Enable digit 1
                        SysTick_Wait1ms(delay);                                     // Wait 2 ms
                        shift12Bits (createSequence(2,0, colon));        // Enable digit 2
                        SysTick_Wait1ms(delay);                                     // Wait 2 ms
                        shift12Bits (createSequence(3,0, colon));        // Enable digit 3
                        SysTick_Wait1ms(delay);
                    }
                    else
                    {
                        shift12Bits (0xFFF);       // Write an empty
                        SysTick_Wait1ms(500);                                    // Wait 2 ms
                    }
                }
                isCountDownDone = false;
                hour_tens = 0;
                hour_ones = 0;
                min_tens = 0;
                min_ones = 0;
                seconds = -1;

            }
        }



    }
}
