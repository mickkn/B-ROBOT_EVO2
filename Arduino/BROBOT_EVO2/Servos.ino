/**
 * @file        Servos.ino
 *
 * @brief       Servo control code for BROBOT EVO 2, the self balance robot with stepper motors by JJROBOTS.
 *              This file contains the definitions and functions to initialize and control the servos used for
 *              steering and throttle control of the robot.
 *
 * @details     The servos are controlled using Timer4 on the Arduino Leonardo board, which allows for precise PWM
 *              control with a resolution of 11 bits. The functions provided in this file allow for initializing the
 *              servo control and setting the PWM values for two servos, which can be used for various purposes such
 *              as steering and throttle control. The PWM values are constrained to the defined minimum and maximum
 *              pulse widths to ensure safe operation of the servos. The code is based on the standard Arduino Servo
 *              library, but with custom implementation to allow for more precise control and to avoid conflicts with
 *              the stepper motor control which uses Timer1 and Timer3. The servos are connected to specific pins on
 *              the Leonardo board (Pin10 for Servo1 and Pin13 for Servo2), and the Timer4 is configured to generate
 *              the appropriate PWM signals for these pins. The code is designed to be efficient and to minimize the
 *              impact on the main control loop of the robot, allowing for smooth and responsive control of the servos
 *              while maintaining the stability and performance of the robot.
 *
 * @date        08-05-2026
 * @author      Mick K
 *
 */

// Default servo definitions
#define SERVO1_AUX_NEUTRO 1500  // Servo neutral position
#define SERVO1_MIN_PULSEWIDTH 700
#define SERVO1_MAX_PULSEWIDTH 2300
#define SERVO2_AUX_NEUTRO 1500  // Servo neutral position
#define SERVO2_MIN_PULSEWIDTH 700
#define SERVO2_MAX_PULSEWIDTH 2300

/**
 * @brief     Function to initialize the servo control using Timer4. This function configures Timer4 for Fast PWM mode,
 *            sets the appropriate prescaler, and configures the output pins for the servos. It also sets the initial
 *            PWM values for the servos to their neutral positions.
 */
void BROBOT_initServo()
{
    int temp;

    // Initialize Timer4 as Fast PWM
    TCCR4A = (1<<PWM4A)|(1<<PWM4B);
    TCCR4B = 0;
    TCCR4C = (1<<PWM4D);
    TCCR4D = 0;
    TCCR4E = (1<<ENHC4); // Enhaced -> 11 bits

    temp = 1500>>3;
    TC4H = temp >> 8;
    OCR4B = temp & 0xff;

    // Reset timer
    TC4H = 0;
    TCNT4 = 0;

    // Set TOP to 1023 (10 bit timer)
    TC4H = 3;
    OCR4C = 0xFF;

    // OC4A = PC7 (Pin13)  OC4B = PB6 (Pin10)   OC4D = PD7 (Pin6)
    // Set pins as outputs
    DDRB |= (1 << 6);  // OC4B = PB6 (Pin10 on Leonardo board)
    DDRC |= (1 << 7);  // OC4A = PC7 (Pin13 on Leonardo board)
    DDRD |= (1 << 7);  // OC4D = PD7 (Pin6 on Leonardo board)

    // Enable OC4A and OC4B and OCR4D output
    TCCR4A |= (1<<COM4B1)|(1<<COM4A1);
    TCCR4C |= (1<<COM4D1);
    // set prescaler to 256 and enable timer    16Mhz/256/1024 = 61Hz (16.3ms)
    TCCR4B = (1 << CS43)|(1 << CS40);
}

/**
 * @brief     Function to set the PWM value for Servo1. This function takes a desired PWM value, constrains it to the defined
 *            minimum and maximum pulse widths, and updates the Timer4 registers to generate the appropriate PWM signal for
 *            Servo1. The PWM value is shifted right by 3 to account for the resolution of Timer4 (8us steps), and the 11-bit
 *            value is split between the TC4H register (for the 3 most significant bits) and the OCR4B register (for the 8 least
 *            significant bits).
 * @param     pwm Desired PWM value for Servo1 (in microseconds). This value will be constrained to the defined minimum and maximum pulse widths.
 */
void BROBOT_moveServo1(int pwm)
{
    pwm = constrain(pwm, SERVO1_MIN_PULSEWIDTH, SERVO1_MAX_PULSEWIDTH) >> 3;  // Check max values and Resolution: 8us
    // 11 bits => 3 MSB bits on TC4H, LSB bits on OCR4B
    TC4H = pwm >> 8;
    OCR4B = pwm & 0xFF;
}

/**
 * @brief     Function to set the PWM value for Servo2. This function takes a desired PWM value, constrains it to the defined
 *            minimum and maximum pulse widths, and updates the Timer4 registers to generate the appropriate PWM signal for
 *            Servo2. The PWM value is shifted right by 3 to account for the resolution of Timer4 (8us steps), and the 11-bit
 *            value is split between the TC4H register (for the 3 most significant bits) and the OCR4A and OCR4D registers (for the 8 least
 *            significant bits). The function updates both OCR4A and OCR4D to support different board versions where Servo2 may be connected to either Pin13 or Pin6.
 * @param     pwm Desired PWM value for Servo2 (in microseconds). This value will be constrained to the defined minimum and maximum pulse widths.
 */
void BROBOT_moveServo2(int pwm)
{
    pwm = constrain(pwm, SERVO2_MIN_PULSEWIDTH, SERVO2_MAX_PULSEWIDTH) >> 3;  // Check max values and Resolution: 8us
    // 11 bits => 3 MSB bits on TC4H, LSB bits on OCR4B
    TC4H = pwm >> 8;
    OCR4A = pwm & 0xFF;  // 2.0 or 2.3  boards servo2 output
    OCR4D = pwm & 0xFF;  // 2.1 or 2.4  boards servo2 output
}
