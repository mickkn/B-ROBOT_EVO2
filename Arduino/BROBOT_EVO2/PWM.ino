/**
 * @file        PWM.ino
 *
 * @brief       PWM control code for BROBOT EVO 2, the self balance robot with stepper motors by JJROBOTS.
 *
 * @details     This file containes the definitions and functions to initialize and receive PWM signals from an
 *              RC receiver, which can be used to control the robot in RC mode. The code uses the Pin Change
 *              Interrupts to detect changes in the PWM signal on specific pins.
 *
 * @date        08-05-2026
 * @author      Mick K
 *
 */

void PWM_init()
{
    // Configure the pins for PWM input (Pin 2 and Pin 3 on Arduino Leonardo)
    pinMode(PWM_CH1, INPUT);
    pinMode(PWM_CH2, INPUT);
    pinMode(PWM_CH3, INPUT);

    // Enable Pin Change Interrupts for the corresponding pins
    PCICR |= (1 << PCIE0); // Enable pin change interrupt for PCINT0-7 (which includes Pin 2 and Pin 3)
    PCMSK0 |= (1 << PCINT2) | (1 << PCINT3); // Enable pin change interrupt for Pin 2 and Pin 3
}