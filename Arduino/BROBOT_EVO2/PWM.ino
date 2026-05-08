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

volatile uint32_t pwm_rise_us_ch1 = 0;
volatile uint32_t pwm_rise_us_ch2 = 0;
volatile uint32_t pwm_rise_us_ch3 = 0;

volatile uint16_t pwm_ch1_us = 1500;
volatile uint16_t pwm_ch2_us = 1500;
volatile uint16_t pwm_ch3_us = 1500;

static inline uint16_t PWM_sanitizePulse(uint32_t pulse)
{
    // Keep only sane RC pulse widths, fallback to neutral if signal glitches.
    if (pulse < 900 || pulse > 2200)
        return 1500;
    return (uint16_t)pulse;
}

void PWM_ISR_CH1()
{
    uint32_t now = micros();
    if (digitalRead(PWM_CH1))
        pwm_rise_us_ch1 = now;
    else
        pwm_ch1_us = PWM_sanitizePulse(now - pwm_rise_us_ch1);
}

void PWM_ISR_CH2()
{
    uint32_t now = micros();
    if (digitalRead(PWM_CH2))
        pwm_rise_us_ch2 = now;
    else
        pwm_ch2_us = PWM_sanitizePulse(now - pwm_rise_us_ch2);
}

void PWM_ISR_CH3()
{
    uint32_t now = micros();
    if (digitalRead(PWM_CH3))
        pwm_rise_us_ch3 = now;
    else
        pwm_ch3_us = PWM_sanitizePulse(now - pwm_rise_us_ch3);
}

void PWM_init()
{
    int8_t irq1 = digitalPinToInterrupt(PWM_CH1);
    int8_t irq2 = digitalPinToInterrupt(PWM_CH2);
    int8_t irq3 = digitalPinToInterrupt(PWM_CH3);

    if (irq1 != NOT_AN_INTERRUPT) attachInterrupt(irq1, PWM_ISR_CH1, CHANGE);
    if (irq2 != NOT_AN_INTERRUPT) attachInterrupt(irq2, PWM_ISR_CH2, CHANGE);
    if (irq3 != NOT_AN_INTERRUPT) attachInterrupt(irq3, PWM_ISR_CH3, CHANGE);
}

uint16_t PWM_getChannelUs(uint8_t channel)
{
    uint16_t value = 1500;

    noInterrupts();
    if (channel == 1)
        value = pwm_ch1_us;
    else if (channel == 2)
        value = pwm_ch2_us;
    else if (channel == 3)
        value = pwm_ch3_us;
    interrupts();

    return value;
}

void PWM_debugPrint(uint16_t period_ms)
{
    static uint32_t last_print = 0;
    uint32_t now_ms = millis();

    if ((now_ms - last_print) < period_ms)
        return;

    last_print = now_ms;

    uint16_t ch1 = PWM_getChannelUs(1);
    uint16_t ch2 = PWM_getChannelUs(2);
    uint16_t ch3 = PWM_getChannelUs(3);

    Serial.print("RC us | CH1:");
    Serial.print(ch1);
    Serial.print(" CH2:");
    Serial.print(ch2);
    Serial.print(" CH3:");
    Serial.println(ch3);
}