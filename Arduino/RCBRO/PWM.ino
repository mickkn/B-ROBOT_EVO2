/**
 * @file        PWM.ino
 *
 * @brief       PWM control code for RCBRO.
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
volatile uint8_t pwm_update_mask = 0;
volatile uint8_t pwm_frame_seq = 0;

#define PWM_CH1_UPDATED 0x01
#define PWM_CH2_UPDATED 0x02
#define PWM_CH3_UPDATED 0x04
#define PWM_ALL_UPDATED (PWM_CH1_UPDATED | PWM_CH2_UPDATED | PWM_CH3_UPDATED)

/**
 * @brief     Function to mark a PWM channel as updated. This function takes a bitmask representing the channel that has been updated and sets the corresponding bit in the global variable pwm_update_mask. It then checks if all channels have been updated (i.e., if the pwm_update_mask has all the bits set for the channels). If all channels have been updated, it resets the pwm_update_mask to 0 and increments the pwm_frame_seq variable, which can be used to track the sequence of PWM frames received.
 * @param     bit The bitmask representing the channel that has been updated (e.g., PWM_CH1_UPDATED, PWM_CH2_UPDATED, or PWM_CH3_UPDATED).
 */
static inline void PWM_markUpdated(uint8_t bit)
{
    pwm_update_mask |= bit;
    // Consider a frame complete once all 3 channels have fresh values.
    if ((pwm_update_mask & PWM_ALL_UPDATED) == PWM_ALL_UPDATED)
    {
        pwm_update_mask = 0;
        pwm_frame_seq++;
    }
}

/**
 * @brief     Function to sanitize the pulse width of the PWM signal. This function takes a pulse width value in microseconds and checks if it falls within a reasonable range for RC signals (typically between 900 and 2200 microseconds). If the pulse width is outside this range, it returns a default value of 1500 microseconds (neutral position). This is done to ensure that any glitches or noise in the signal do not cause erratic behavior in the robot.
 * @param     pulse   The pulse width value in microseconds to be sanitized.
 * @return    A sanitized pulse width value in microseconds, constrained to a reasonable range for RC signals.
 */
static inline uint16_t PWM_sanitizePulse(uint32_t pulse)
{
    // Keep only sane RC pulse widths, fallback to neutral if signal glitches.
    if (pulse < 900 || pulse > 2200)
        return 1500;
    return (uint16_t)pulse;
}

/**
 * @brief     Interrupt Service Routine for PWM channel 1. This ISR is triggered on any change (rising or falling edge)
 *            of the signal on the PWM_CH1 pin. When the signal goes HIGH, it records the current time in microseconds
 *            as the rising edge time. When the signal goes LOW, it calculates the pulse width by taking the difference
 *            between the current time and the recorded rising edge time, sanitizes the pulse width to ensure it falls
 *            within a reasonable range, updates the global variable for channel 1 with the new pulse width, and marks
 *            that channel 1 has been updated.
 */
void PWM_ISR_CH1()
{
    uint32_t now = micros();
    if (digitalRead(PWM_CH1))
        pwm_rise_us_ch1 = now;
    else
    {
        pwm_ch1_us = PWM_sanitizePulse(now - pwm_rise_us_ch1);
        PWM_markUpdated(PWM_CH1_UPDATED);
    }
}

/**
 * @brief     Interrupt Service Routine for PWM channel 2. This ISR is triggered on any change (rising or falling edge)
 *            of the signal on the PWM_CH2 pin. When the signal goes HIGH, it records the current time in microseconds
 *            as the rising edge time. When the signal goes LOW, it calculates the pulse width by taking the difference
 *            between the current time and the recorded rising edge time, sanitizes the pulse width to ensure it falls
 *            within a reasonable range, updates the global variable for channel 2 with the new pulse width, and marks
 *            that channel 2 has been updated.
 */
void PWM_ISR_CH2()
{
    uint32_t now = micros();
    if (digitalRead(PWM_CH2))
        pwm_rise_us_ch2 = now;
    else
    {
        pwm_ch2_us = PWM_sanitizePulse(now - pwm_rise_us_ch2);
        PWM_markUpdated(PWM_CH2_UPDATED);
    }
}

/**
 * @brief     Interrupt Service Routine for PWM channel 3. This ISR is triggered on any change (rising or falling edge)
 *            of the signal on the PWM_CH3 pin. When the signal goes HIGH, it records the current time in microseconds
 *            as the rising edge time. When the signal goes LOW, it calculates the pulse width by taking the difference
 *            between the current time and the recorded rising edge time, sanitizes the pulse width to ensure it falls
 *            within a reasonable range, updates the global variable for channel 3 with the new pulse width, and marks
 *            that channel 3 has been updated.
 */
void PWM_ISR_CH3()
{
    uint32_t now = micros();
    if (digitalRead(PWM_CH3))
        pwm_rise_us_ch3 = now;
    else
    {
        pwm_ch3_us = PWM_sanitizePulse(now - pwm_rise_us_ch3);
        PWM_markUpdated(PWM_CH3_UPDATED);
    }
}

/**
 * @brief     Function to initialize the PWM input handling. This function sets up pin change interrupts for the specified PWM input pins (PWM_CH1, PWM_CH2, and PWM_CH3). Each pin is associated with an interrupt service routine (ISR) that will be called whenever there is a change in the signal on that pin (i.e., when the signal goes from LOW to HIGH or from HIGH to LOW). The ISRs will measure the pulse width of the incoming PWM signals and update the corresponding variables with the latest values.
 */
void PWM_init()
{
    int8_t irq1 = digitalPinToInterrupt(PWM_CH1);
    int8_t irq2 = digitalPinToInterrupt(PWM_CH2);
    int8_t irq3 = digitalPinToInterrupt(PWM_CH3);

    if (irq1 != NOT_AN_INTERRUPT) attachInterrupt(irq1, PWM_ISR_CH1, CHANGE);
    if (irq2 != NOT_AN_INTERRUPT) attachInterrupt(irq2, PWM_ISR_CH2, CHANGE);
    if (irq3 != NOT_AN_INTERRUPT) attachInterrupt(irq3, PWM_ISR_CH3, CHANGE);
}

/**
 * @brief     Function to get the latest PWM value for a specific channel. The function takes a channel number (1, 2, or 3) as input and returns the corresponding PWM value in microseconds. The function disables interrupts while reading the shared variables to ensure that it gets a consistent value without being interrupted by the ISR that updates these variables.
 * @param     channel Channel number (1, 2, or 3) for which to get the PWM value.
 * @return    The latest PWM value for the specified channel in microseconds. If an invalid channel number is provided, it returns a default value of 1500 microseconds.
 */
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

/**
 * @brief     Function to consume the latest PWM frame values for all channels. The function reads the current PWM values for channels 1, 2, and 3, and checks if they have been updated since the last time this function was called. If the values have not been updated (i.e., the sequence number has not changed), the function returns false, indicating that there is no new frame to consume. If the values have been updated, it updates the last sequence number and returns true, indicating that new frame data has been consumed.
 * @param     ch1 Reference variable to store the latest PWM value for channel 1 (in microseconds).
 * @param     ch2 Reference variable to store the latest PWM value for channel 2 (in microseconds).
 * @param     ch3 Reference variable to store the latest PWM value for channel 3 (in microseconds).
 * @return    true if a new frame was consumed (values were updated), false if there was no new frame (values were not updated).
 */
bool PWM_consumeLatestFrame(uint16_t &ch1, uint16_t &ch2, uint16_t &ch3)
{
    static uint8_t last_seq = 0;
    uint8_t seq;

    noInterrupts();
    seq = pwm_frame_seq;
    ch1 = pwm_ch1_us;
    ch2 = pwm_ch2_us;
    ch3 = pwm_ch3_us;
    interrupts();

    if (seq == last_seq)
        return false;

    last_seq = seq;
    return true;
}

/**
 * @brief     Debug function to print the latest PWM channel values to the serial console. The function uses a static variable to track the last time it printed, and only prints if the specified period has elapsed since the last print. This allows for rate-limited debug output while still consuming the latest PWM frame data.
 * @param     period_ms  Minimum period (in milliseconds) between consecutive prints to the console. If the function is called more frequently than this period, it will skip printing until the period has elapsed.
 */
void PWM_debugPrint(uint16_t period_ms)
{
    static uint32_t last_print = 0;
    uint32_t now_ms = millis();

    uint16_t ch1;
    uint16_t ch2;
    uint16_t ch3;
    if (!PWM_consumeLatestFrame(ch1, ch2, ch3))
        return;

    // Optional print rate limiter on top of event-driven updates.
    if ((now_ms - last_print) < period_ms)
        return;

    last_print = now_ms;


    Serial.print("RC us | CH1:");
    Serial.print(ch1);
    Serial.print(" CH2:");
    Serial.print(ch2);
    Serial.print(" CH3:");
    Serial.println(ch3);
}