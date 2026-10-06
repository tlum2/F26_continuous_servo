/****************************************************************
* Author: T. LUM and R. MARTIN
* CPEG222 DemoProgram9, 10/6/26
* NucleoF446ZE CMSIS STM32F4xx example F26Demo9_Cont_Servo
* Generates a 50 Hz PWM signal on PD12 (TIM4_CH1, robot PWM1) to control
* a continuous rotation servo (Parallax Feedback 360). The pulse width
* sets speed and direction, not position. The feedback (4th) wire is not
* used in this demo.
* The servo runs automatically (no input needed):
*   Part 1: stop, half speed CCW, stop, half speed CW, stop
*   Part 2: ramp up and down in each direction
* System clock is 180 MHz, so the TIM4 clock is 90 MHz (APB1 45 MHz x 2).
*
* Revision History:
* V1  10/6/26  Initial version
****************************************************************/
#include "stm32f4xx.h"       // Include the STM32F4xx CMSIS header file
#include "SysClock180MHz.h"  // 180 MHz system clock configuration
#include <stdint.h>          // Include stdint.h for uint32_t and int32_t types

#define SERVO_PORT     GPIOD
#define SERVO_PIN      12              // PD12 = TIM4_CH1 (AF2), robot PWM1
#define TIMER_CLOCK    90000000UL      // TIM4 clock at 180 MHz SYSCLK (APB1 45 MHz x 2)
#define PWM_PERIOD_US  20000           // 20,000 us = 20 ms period = 50 Hz
// Parallax Feedback 360 pulse widths (from the Parallax datasheet)
#define CW_MAX_US      1280            // full speed clockwise
#define CW_MIN_US      1480            // slowest clockwise (edge of dead band)
#define STOP_US        1500            // stop (dead band is 1480-1520 us)
#define CCW_MIN_US     1520            // slowest counterclockwise (edge of dead band)
#define CCW_MAX_US     1720            // full speed counterclockwise

void PWM_Output_PD12_Init(void);
void Servo_SetSpeed(int32_t speed);
void delay_ms(uint32_t ms);

int main(void) {
    SystemClock_Config();    // Configure the system clock to 180 MHz
    SystemCoreClockUpdate(); // Update SystemCoreClock variable after clock change
    PWM_Output_PD12_Init();  // Start 50 Hz PWM on PD12, servo starts stopped

    while (1) {
        // Part 1: fixed speeds
        Servo_SetSpeed(0);      // stop
        delay_ms(2000);
        Servo_SetSpeed(50);     // half speed counterclockwise
        delay_ms(2000);
        Servo_SetSpeed(0);      // stop
        delay_ms(1000);
        Servo_SetSpeed(-50);    // half speed clockwise
        delay_ms(2000);
        Servo_SetSpeed(0);      // stop
        delay_ms(1000);
        // Part 2: ramp counterclockwise 0 -> 100 -> 0, one step per PWM period (20 ms)
        for (int32_t speed = 0; speed <= 100; speed++) {
            Servo_SetSpeed(speed);
            delay_ms(20);
        }
        for (int32_t speed = 100; speed >= 0; speed--) {
            Servo_SetSpeed(speed);
            delay_ms(20);
        }
        // Ramp clockwise 0 -> -100 -> 0
        for (int32_t speed = 0; speed >= -100; speed--) {
            Servo_SetSpeed(speed);
            delay_ms(20);
        }
        for (int32_t speed = -100; speed <= 0; speed++) {
            Servo_SetSpeed(speed);
            delay_ms(20);
        }
    }
    return 0; // Program should never reach here due to infinite loop
}

void PWM_Output_PD12_Init(void) {
    // 1. Enable GPIOD and TIM4 clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    // 2. Set PD12 to alternate function mode (MODER = 10b)
    SERVO_PORT->MODER &= ~(0x3 << (SERVO_PIN * 2));
    SERVO_PORT->MODER |=  (0x2 << (SERVO_PIN * 2));
    // 3. Select AF2 (TIM4_CH1) for PD12. Pins 8-15 use AFR[1], so subtract 8
    SERVO_PORT->AFR[1] &= ~(0xF << ((SERVO_PIN - 8) * 4));
    SERVO_PORT->AFR[1] |=  (0x2 << ((SERVO_PIN - 8) * 4));
    // 4. Configure TIM4 for PWM output on CH1
    TIM4->CR1 &= ~TIM_CR1_CEN;                   // Disable TIM4 before configuring it
    TIM4->PSC = (TIMER_CLOCK / 1000000UL) - 1;   // 90 MHz / 90 = 1 MHz timer clock (1 us per tick)
    TIM4->ARR = PWM_PERIOD_US - 1;               // 20,000 ticks = 20 ms period (50 Hz)
    TIM4->CCR1 = STOP_US;                        // 1,500 ticks = 1.5 ms pulse (stopped)
    TIM4->CCMR1 &= ~(TIM_CCMR1_CC1S | TIM_CCMR1_OC1M); // CH1 as output, clear output mode
    TIM4->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos);    // PWM mode 1: high while CNT < CCR1
    TIM4->CCMR1 |= TIM_CCMR1_OC1PE;              // Preload enable: new CCR1 takes effect at next period
    TIM4->CCER |= TIM_CCER_CC1E;                 // Enable CH1 output on PD12
    TIM4->CR1 |= TIM_CR1_ARPE;                   // Auto-reload preload enable
    TIM4->EGR = TIM_EGR_UG;                      // Generate update event to load PSC, ARR, CCR1
    TIM4->CR1 |= TIM_CR1_CEN;                    // Enable timer
}

void Servo_SetSpeed(int32_t speed) {
    // speed: -100 (full CW) to 0 (stop) to +100 (full CCW)
    uint32_t pulse_us;
    if (speed > 100) {
        speed = 100;
    } else if (speed < -100) {
        speed = -100;
    }
    if (speed == 0) {
        pulse_us = STOP_US;
    } else if (speed > 0) { // counterclockwise: 1520-1720 us
        pulse_us = CCW_MIN_US + (speed * (CCW_MAX_US - CCW_MIN_US)) / 100;
    } else {                // clockwise: 1480-1280 us
        pulse_us = CW_MIN_US + (speed * (CW_MIN_US - CW_MAX_US)) / 100;
    }
    TIM4->CCR1 = pulse_us; // 1 tick = 1 us, so CCR1 is the pulse width in us
}

void delay_ms(uint32_t ms) {
    if (SystemCoreClock == 16000000U) { // If the system clock is 16 MHz
        for (uint32_t i = 0; i < ms * 2000; i++) {
            __NOP();
        }
    } else if (SystemCoreClock == 180000000U) { // If the system clock is 180 MHz
        for (uint32_t i = 0; i < ms * 22500; i++) {
            __NOP();
        }
    }
}
