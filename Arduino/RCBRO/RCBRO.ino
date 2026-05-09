/**
 * @file        RCBRO.ino
 *
 * @brief       RC Version of BROBOT EVO 2, the self balance robot with stepper motors by JJROBOTS.
 *
 * @details     Based on version 2.82, but with some modifications to be controlled by RC remote instead of WiFi.
 *              The code is simplified and some features are removed to make it more suitable for RC control.
 *
 * @date        08-05-2026
 * @author      Mick K
 *
 */

#include <Wire.h>

// Forward declaration needed because Arduino's auto-prototype generator
// does not preserve default parameters across .ino files.
float speedPIControl(float DT, int16_t input, int16_t setPoint, float Kp, float Ki, bool integrate = true);

// NORMAL MODE PARAMETERS (MAXIMUM SETTINGS)
#define MAX_THROTTLE 400          // Reduced for heavier 18650 pack (was 550)
#define MAX_STEERING 120
#define MAX_TARGET_ANGLE 12       // Reduced for heavier 18650 pack (was 14)

// PRO MODE = MORE AGGRESSIVE (MAXIMUM SETTINGS)
#define MAX_THROTTLE_PRO 550      // Reduced for heavier 18650 pack (was 780)
#define MAX_STEERING_PRO 240      // Max recommended value: 280
#define MAX_TARGET_ANGLE_PRO 16   // Reduced for heavier 18650 pack (was 18)

// Fall detection / raise-up thresholds
#define FALL_ANGLE_NORMAL 74
#define FALL_ANGLE_PRO 82
#define FALL_ANGLE_HARD 86
#define RAISEUP_ANGLE_NORMAL 56
#define RAISEUP_ANGLE_PRO 64

// Default control terms for EVO 2
#define KP 0.32
#define KD 0.050
#define KP_THROTTLE 0.080
#define KI_THROTTLE 0.1
#define KP_POSITION 0.06
#define KD_POSITION 0.45
//#define KI_POSITION 0.02

// Control gains for raiseup (the raiseup movement require special control parameters)
#define KP_RAISEUP 0.1
#define KD_RAISEUP 0.16
#define KP_THROTTLE_RAISEUP 0      // No speed control on raiseup
#define KI_THROTTLE_RAISEUP 0.0

#define MAX_CONTROL_OUTPUT 500
#define ITERM_MAX_ERROR 30        // Iterm windup constants for PI control
#define ITERM_MAX 10000

#define ANGLE_OFFSET 0.0          // Offset angle for balance (to compensate robot own weight distribution)

// Servo definitions
#define SERVO_AUX_NEUTRO 1500     // Servo neutral position
#define SERVO_MIN_PULSEWIDTH 700
#define SERVO_MAX_PULSEWIDTH 2500

#define SERVO2_NEUTRO 1500

#define ZERO_SPEED 65535
#define MAX_ACCEL 14          // Maximum motor acceleration (MAX RECOMMENDED VALUE: 20) (default:14)

#define MICROSTEPPING 16      // 8 or 16 for 1/8 or 1/16 driver microstepping (default:16) A4988, all MS pins high.

#define DEBUG 0               // 0 = No debug info (default) DEBUG 1 for console output
#define DEBUG_MPU 0

// AUX definitions
#define CLR(x,y) (x&=(~(1<<y)))
#define SET(x,y) (x|=(1<<y))
#define RAD2GRAD 57.2957795

// Pins (Teensy 2.0, Arduino Leonardo Clone)
#define ENABLE_MOTORS 4

#define STEP_M1 11          // B7
#define STEP_M1_PORT PORTB
#define STEP_M1_PIN 7
#define DIR_M1  8           // B4
#define DIR_M1_PORT PORTB
#define DIR_M1_PIN 4

#define STEP_M2 12          // D6 - Also LED.
#define STEP_M2_PORT PORTD
#define STEP_M2_PIN 6
#define DIR_M2  5           // C6
#define DIR_M2_PORT PORTC
#define DIR_M2_PIN 6

#define SERVO1_PIN 10       // B6
#define SERVO2_PIN 13       // C7

#define PWM_CH1 1           // D3 (INT3)
#define PWM_CH2 0           // D2 (INT2)
#define PWM_CH3 7           // E6 (INT6)

long timer_old;
long timer_value;
float dt;

// Angle of the robot (used for stability control)
float angle_adjusted;
float angle_adjusted_Old;
float angle_adjusted_filtered = 0.0;

// Default control values from constant definitions
float Kp = KP;
float Kd = KD;
float Kp_thr = KP_THROTTLE;
float Ki_thr = KI_THROTTLE;
float Kp_user = KP;
float Kd_user = KD;
float Kp_thr_user = KP_THROTTLE;
float Ki_thr_user = KI_THROTTLE;
float Kp_position = KP_POSITION;
float Kd_position = KD_POSITION;
float PID_errorSum;
float PID_errorOld = 0;
float setPointOld = 0;
float target_angle;
int16_t throttle;
float steering;
float max_throttle = MAX_THROTTLE;
float max_steering = MAX_STEERING;
float max_target_angle = MAX_TARGET_ANGLE;
float control_output;
float angle_offset = ANGLE_OFFSET;


// Expo curve factor: 0.0 = linear, 1.0 = full expo (very soft centre)
#define EXPO_STEERING 0.5f
#define EXPO_THROTTLE 0.4f

/**
 * @brief  Apply an exponential curve to a normalised stick input.
 *         input and output are in the range [-1.0, 1.0].
 *         expo = 0 -> linear, expo = 1 -> pure cubic (maximum softness around centre).
 */
float applyExpo(float input, float expo)
{
    return input * (expo * input * input + (1.0f - expo));
}

boolean positionControlMode = false;
uint8_t mode = 0;  // mode = 0 Normal mode, mode = 1 Pro mode (More agressive)

int16_t motor1;
int16_t motor2;

// position control
volatile int32_t steps1, steps2;
int32_t target_steps1, target_steps2;
int16_t motor1_control, motor2_control;

int16_t speed_M1, speed_M2;        // Actual speed of motors
volatile int8_t dir_M1, dir_M2;    // Actual direction of steppers motors
int16_t actual_robot_speed;        // overall robot speed (measured from steppers speed)
float estimated_speed_filtered;    // Estimated robot speed

// OSC output variables (kept for potential future use)
bool startupLogPrinted = false;

uint16_t rc_ch3_us = 1500;
bool rc_ch3_wasLow = false;

/**
 * @brief     Function to print the startup banner and firmware information to the Serial console.
              This function is called once during the setup() function after the Serial connection is
              established. It prints a header with the firmware version and author information, and
              sets a flag to indicate that the startup log has been printed.
 */
void printStartupBanner()
{
    Serial.println(F(""));
    Serial.println(F("========================================"));
    Serial.println(F("  RCBRO - Startup"));
    Serial.println(F("========================================"));
    Serial.println(F("[INFO] Firmware: RCBRO v1.0.0 by Mick K"));
    startupLogPrinted = true;
}

/**
 * @brief    Arduino setup function. It is called once at the beginning of the program.
 */
void setup()
{
    // STEPPER PINS ON JJROBOTS BROBOT BRAIN BOARD
    pinMode(ENABLE_MOTORS, OUTPUT);     // ENABLE MOTORS
    pinMode(STEP_M1, OUTPUT);           // STEP MOTOR 1
    pinMode(DIR_M1, OUTPUT);            // DIR MOTOR 1
    pinMode(STEP_M2, OUTPUT);           // STEP MOTOR 2
    pinMode(DIR_M2, OUTPUT);            // DIR MOTOR 2
    digitalWrite(ENABLE_MOTORS, HIGH);  // Disable motors
    pinMode(SERVO1_PIN, OUTPUT);        // Servo1 (arm)
    pinMode(SERVO2_PIN, OUTPUT);        // Servo2 (not wired)
    pinMode(PWM_CH1, INPUT);            // PWM input from RC receiver channel 1 (steering)
    pinMode(PWM_CH2, INPUT);            // PWM input from RC receiver channel 2 (throttle)
    pinMode(PWM_CH3, INPUT);            // PWM input from RC receiver channel 3 (mode switch and arm control)

    Serial.begin(115200); // Serial output to console

    // Leonardo uses native USB; wait briefly so startup logs are not missed.
    uint32_t serialWaitStart = millis();
    while (!Serial && (millis() - serialWaitStart < 2000))
    {
    }
    if (Serial)
        printStartupBanner();

    PWM_init();
    Serial.println(F("[OK] RC PWM input initialized"));

    // Initialize I2C bus (MPU6050 is connected via I2C)
    Wire.begin();

    Serial.println(F("[OK] I2C bus initialized"));
    delay(200);
    Serial.println(F("[INFO] Hold robot still: IMU warmup/calibration..."));

#if DEBUG > 0
    delay(9000);
#else
    delay(1000);
#endif
    MPU6050_setup();  // setup MPU6050 IMU
    delay(500);

    // Calibrate gyros
    MPU6050_calibrate();

    // Init servos
    Serial.println(F("[INIT] Servos"));
    BROBOT_initServo();
    BROBOT_moveServo1(SERVO_AUX_NEUTRO);

    // STEPPER MOTORS INITIALIZATION
    Serial.println(F("[INIT] Stepper drivers/timers"));
    // MOTOR1 => TIMER1
    TCCR1A = 0;                            // Timer1 CTC mode 4, OCxA,B outputs disconnected
    TCCR1B = (1 << WGM12) | (1 << CS11);  // Prescaler=8, => 2Mhz
    OCR1A = ZERO_SPEED;                    // Motor stopped
    dir_M1 = 0;
    TCNT1 = 0;

    // MOTOR2 => TIMER3
    TCCR3A = 0;                            // Timer3 CTC mode 4, OCxA,B outputs disconnected
    TCCR3B = (1 << WGM32) | (1 << CS31);  // Prescaler=8, => 2Mhz
    OCR3A = ZERO_SPEED;                    // Motor stopped
    dir_M2 = 0;
    TCNT3 = 0;

    delay(200);

    // Enable stepper drivers and TIMER interrupts
    digitalWrite(ENABLE_MOTORS, LOW);   // Enable stepper drivers

    // Enable TIMERs interrupts
    TIMSK1 |= (1 << OCIE1A); // Enable Timer1 interrupt
    TIMSK3 |= (1 << OCIE3A); // Enable Timer3 interrupt

    // Little motor vibration and servo move to indicate that robot is ready
    for (uint8_t k = 0; k < 5; k++)
    {
        setMotorSpeed(1, 5);
        setMotorSpeed(2, 5);
        BROBOT_moveServo1(SERVO_AUX_NEUTRO + 100);
        BROBOT_moveServo2(SERVO2_NEUTRO + 100);
        delay(200);
        setMotorSpeed(1, -5);
        setMotorSpeed(2, -5);
        BROBOT_moveServo1(SERVO_AUX_NEUTRO - 100);
        BROBOT_moveServo2(SERVO2_NEUTRO - 100);
        delay(200);
    }
    BROBOT_moveServo1(SERVO_AUX_NEUTRO);
    BROBOT_moveServo2(SERVO2_NEUTRO);

    Serial.println(F("[OK] Calibration complete"));
    Serial.println(F("[READY] Control loop started"));
    timer_old = micros();
}


/**
 * @brief    Arduino loop function. It is called in a loop after setup() is finished.
 *           This is the main control loop of the robot, where the IMU data is read,
 *           calculate the control output and send commands to the motors.
 */
void loop()
{
    // If host opens Serial Monitor after boot, print startup header once when it becomes available.
    if (!startupLogPrinted && Serial)
        printStartupBanner();

    // We read the timer at the beginning of the loop to have a more accurate dt for the control loop.
    timer_value = micros();

    // We read the IMU data only when new data is available
    // (MPU6050 has a data ready flag that we check with MPU6050_newData() function).
    if (MPU6050_newData())
    {
        MPU6050_read_3axis();

        dt = (timer_value - timer_old) * 0.000001; // dt in seconds
        timer_old = timer_value;

        angle_adjusted_Old = angle_adjusted;
        // Get new orientation angle from IMU (MPU6050)
        float MPU_sensor_angle = MPU6050_getAngle(dt);
        angle_adjusted = MPU_sensor_angle + angle_offset;
        if ((MPU_sensor_angle > -15) && (MPU_sensor_angle < 15))
            angle_adjusted_filtered = angle_adjusted_filtered * 0.99 + MPU_sensor_angle * 0.01;

#if DEBUG_MPU==1
        Serial.print(dt);
        Serial.print(" ");
        Serial.print(angle_offset);
        Serial.print(" ");
        Serial.print(angle_adjusted);
        Serial.print(",");
        Serial.println(angle_adjusted_filtered);
#endif

#if DEBUG==1
        static uint32_t lastDebug = 0;
        static uint16_t imuCount = 0;

        imuCount++;

        if (millis() - lastDebug >= 1000)
        {
            Serial.print("IMU Hz: ");
            Serial.print(imuCount);
            Serial.print(" dt: ");
            Serial.println(dt, 6);

            imuCount = 0;
            lastDebug = millis();
        }

        PWM_debugPrint(100);
#endif

        // We calculate the estimated robot speed:
        // Estimated robot speed is calculated from the speed of the stepper motors,
        // which is measured from the time between steps in the TIMER interrupts.
        // We take the average of both motors as the overall robot speed.
        actual_robot_speed = (speed_M1 + speed_M2) / 2;

        float angular_velocity = (angle_adjusted - angle_adjusted_Old) * 25.0f;
        float estimated_speed = -(float)actual_robot_speed + angular_velocity;
        estimated_speed_filtered = estimated_speed_filtered * 0.9f + estimated_speed * 0.1f;

#if DEBUG==2
        Serial.print(angle_adjusted);
        Serial.print(" ");
        Serial.println(estimated_speed_filtered);
#endif

        // --- RC INPUT (CH1 = steering, CH2 = throttle) ---
        if (!positionControlMode)
        {
            uint16_t ch1 = PWM_getChannelUs(1); // 1000-2000 us, 1500 = neutral
            uint16_t ch2 = PWM_getChannelUs(2);
            uint16_t ch3 = PWM_getChannelUs(3); // arm trigger
            rc_ch3_us = ch3;

            // PRO mode toggle: rising edge of CH3 going LOW (<= 1200 us)
            bool ch3IsLow = (ch3 <= 1200);
            if (ch3IsLow && !rc_ch3_wasLow)
            {
                mode = (mode == 0) ? 1 : 0; // toggle between normal(0) and pro(1)
                if (mode == 1)
                {
                    max_throttle     = MAX_THROTTLE_PRO;
                    max_steering     = MAX_STEERING_PRO;
                    max_target_angle = MAX_TARGET_ANGLE_PRO;
                    Kp_user          = KP;
                    Kd_user          = KD;
                    Serial.println(F("[MODE] PRO"));
                }
                else
                {
                    max_throttle     = MAX_THROTTLE;
                    max_steering     = MAX_STEERING;
                    max_target_angle = MAX_TARGET_ANGLE;
                    Kp_user          = KP;
                    Kd_user          = KD;
                    Serial.println(F("[MODE] NORMAL"));
                }
            }
            rc_ch3_wasLow = ch3IsLow;

            // Apply dead-band around neutral (1500 us) to avoid drift
            if (abs((int)ch1 - 1500) < 30) ch1 = 1500;
            if (abs((int)ch2 - 1500) < 30) ch2 = 1500;

            // CH1 = steering, CH2 = throttle
            // Clamp and map from PWM input range (1000-2000 us) to control output range (-max, max)
            steering = map(ch1, 1000, 2000, -max_steering, max_steering);
            throttle = map(ch2, 1000, 2000, -max_throttle, max_throttle);

            // Apply expo curve (soften centre feel)
            steering = applyExpo(steering / max_steering, EXPO_STEERING) * max_steering;
            throttle = applyExpo(throttle / max_throttle, EXPO_THROTTLE) * max_throttle;
        }

        if (positionControlMode)
        {
            // POSITION CONTROL. INPUT: Target steps for each motor. Output: motors speed
            motor1_control = positionPDControl(steps1, target_steps1, Kp_position, Kd_position, speed_M1);
            motor2_control = positionPDControl(steps2, target_steps2, Kp_position, Kd_position, speed_M2);

            // Convert from motor position control to throttle / steering commands
            throttle = (motor1_control + motor2_control) / 2;
            throttle = constrain(throttle, -190, 190);
            steering = motor2_control - motor1_control;
            steering = constrain(steering, -50, 50);
        }

        // ROBOT SPEED CONTROL: This is a PI controller.
        //    input:user throttle(robot speed), variable: estimated robot speed, output: target robot angle to get the desired speed
        //    Anti-windup: freeze the speed I-term when either:
        //      1) speed loop output (target_angle) was saturated, or
        //      2) inner balance loop is near motor authority limit.
        //    This avoids runaway at high speed where more I-term cannot produce more wheel torque.
        {
            static bool speed_pi_saturated = false;
            bool balance_loop_saturated = (abs((int)control_output) > (MAX_CONTROL_OUTPUT - 40));
            bool allow_speed_i = (!speed_pi_saturated) && (!balance_loop_saturated);

            target_angle = speedPIControl(dt, estimated_speed_filtered, throttle, Kp_thr, Ki_thr, allow_speed_i);
            float target_angle_clamped = constrain(target_angle, -max_target_angle, max_target_angle);
            speed_pi_saturated = (target_angle != target_angle_clamped);
            target_angle = target_angle_clamped;
        }

#if DEBUG==3
        Serial.print(angle_adjusted);
        Serial.print(" ");
        Serial.print(estimated_speed_filtered);
        Serial.print(" ");
        Serial.println(target_angle);
#endif

        // Stability control (100Hz loop): This is a PD controller.
        //    input: robot target angle(from SPEED CONTROL), variable: robot angle, output: Motor speed
        //    We integrate the output (sumatory), so the output is really the motor acceleration, not motor speed.
        control_output += stabilityPDControl(dt, angle_adjusted, target_angle, Kp, Kd);

        control_output = constrain(control_output, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT); // Limit max output from control

        // The steering part from the user is injected directly to the output
        motor1 = control_output + steering;
        motor2 = control_output - steering;

        // Limit max speed (control output)
        motor1 = constrain(motor1, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);
        motor2 = constrain(motor2, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);

        float balance_angle_error = angle_adjusted - target_angle;
        int angle_ready = (mode == 1) ? FALL_ANGLE_PRO : FALL_ANGLE_NORMAL;
        if (rc_ch3_us >= 1800)  // Arm active: keep the slightly wider envelope
            angle_ready = FALL_ANGLE_PRO;

        // Keep motors enabled while the robot stays reasonably close to its commanded lean.
        // A separate hard stop remains for true near-horizontal falls.
        if ((abs(balance_angle_error) < angle_ready) && (abs(angle_adjusted) < FALL_ANGLE_HARD))
        {
            // NORMAL MODE
            digitalWrite(ENABLE_MOTORS, LOW);  // Motors enable
            // NOW we send the commands to the motors
            setMotorSpeed(1, motor1);
            setMotorSpeed(2, motor2);
        }
        else   // Robot not ready (flat), angle > angle_ready => ROBOT OFF
        {
            digitalWrite(ENABLE_MOTORS, HIGH);  // Disable motors
            setMotorSpeed(1, 0);
            setMotorSpeed(2, 0);
            PID_errorSum = 0;  // Reset PID I term
            control_output = 0;  // Reset stability integrator
            Kp = KP_RAISEUP;   // CONTROL GAINS FOR RAISE UP
            Kd = KD_RAISEUP;
            Kp_thr = KP_THROTTLE_RAISEUP;
            Ki_thr = KI_THROTTLE_RAISEUP;
            // RESET steps
            steps1 = 0;
            steps2 = 0;
            positionControlMode = false;
            throttle = 0;
            steering = 0;
        }

        // Servo1 arm: CH3 high (>=1800 us) triggers the arm.
        // Forward when upright, backward when robot is on its back.
        if (rc_ch3_us >= 1800)
        {
            if (angle_adjusted > -40)   // Upright or tilting forward
                BROBOT_moveServo1(SERVO_MAX_PULSEWIDTH);  // forward
            else                        // Laying on its back
                BROBOT_moveServo1(SERVO_MIN_PULSEWIDTH);  // backward
        }
        else
            BROBOT_moveServo1(SERVO_AUX_NEUTRO);

        // Servo2 - hold at neutral (no RC channel assigned yet)
        BROBOT_moveServo2(SERVO2_NEUTRO);

        // Normal condition? Use angle error relative to commanded lean so PRO mode can
        // stay on the user gains even while intentionally pitched further forward.
        int raiseup_angle = (mode == 1) ? RAISEUP_ANGLE_PRO : RAISEUP_ANGLE_NORMAL;
        if ((abs(balance_angle_error) < raiseup_angle) && (abs(angle_adjusted) < FALL_ANGLE_HARD))
        {
            Kp = Kp_user;            // Default user control gains
            Kd = Kd_user;
            Kp_thr = Kp_thr_user;
            Ki_thr = Ki_thr_user;
        }
        else    // We are in the raise up procedure => we use special control parameters
        {
            Kp = KP_RAISEUP;         // CONTROL GAINS FOR RAISE UP
            Kd = KD_RAISEUP;
            Kp_thr = KP_THROTTLE_RAISEUP;
            Ki_thr = KI_THROTTLE_RAISEUP;
        }

    } // End of new IMU data
}
