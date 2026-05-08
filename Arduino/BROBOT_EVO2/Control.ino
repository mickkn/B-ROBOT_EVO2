/**
 * @file        Control.ino
 *
 * @brief       Control functions for BROBOT EVO 2, the self balance robot with stepper motors by JJROBOTS.
 *
 * @details     This file contains the implementation of the control algorithms for the robot, including the PD
 *              controller for balance and the PI controller for speed control. It also includes the interrupt
 *              service routines for the stepper motor control and the function to set the motor speed.
 *
 * @date        08-05-2026
 * @author      Mick K
 *
 */

/**
 * @brief     Stability control function. This is a PD controller that calculates the control output based on the current angle and the target angle.
 * @param     DT          Time step in seconds.
 * @param     input       Current angle of the robot (measured by the IMU).
 * @param     setPoint    Target angle for the robot to achieve (calculated by the speed PI controller).
 * @param     Kp          Proportional gain for the PD controller.
 * @param     Kd          Derivative gain for the PD controller.
 * @return    Control output for the motors to achieve the desired angle.
 */
float stabilityPDControl(float DT, float input, float setPoint, float Kp, float Kd)
{
    float error = setPoint - input;

    // Kd is implemented in two parts:
    //   Sensor part: -(input - input(t-1))
    //   Setpoint part: setPoint - setPoint(t-1)  (clamped to avoid aggressive jumps)
    float Kd_setPoint = constrain((setPoint - setPointOld), -8.0f, 8.0f);

    // Kd factored out (saves one float multiply).
    // 1/DT used instead of division (float divide is ~4x slower than multiply on AVR).
    float inv_DT = 1.0f / DT;
    float output = Kp * error + Kd * (Kd_setPoint - (input - PID_errorOld)) * inv_DT;

    PID_errorOld = input;   // D-term uses measurement only, not error
    setPointOld = setPoint;
    return output;
}


/**
 * @brief     Speed control function. This is a PI controller that calculates the target angle for the robot to achieve the desired speed.
 * @param     DT          Time step in seconds.
 * @param     input       Current estimated speed of the robot (calculated from the stepper motors speed and the IMU angle).
 * @param     setPoint    Desired speed for the robot (from user throttle input).
 * @param     Kp          Proportional gain for the PI controller.
 * @param     Ki          Integral gain for the PI controller.
 * @return    Target angle for the robot to achieve the desired speed.
 */
float speedPIControl(float DT, int16_t input, int16_t setPoint, float Kp, float Ki)
{
    int16_t error = setPoint - input;
    PID_errorSum += constrain(error, -ITERM_MAX_ERROR, ITERM_MAX_ERROR);
    PID_errorSum = constrain(PID_errorSum, -ITERM_MAX, ITERM_MAX);

    return Kp * error + Ki * PID_errorSum * DT;
}

/**
 * @brief     Position control function. This is a PD controller that calculates the motor control output based on the current position and the target position.
 * @param     actualPos      Current position of the motor (measured in steps).
 * @param     setPointPos    Desired position for the motor (measured in steps).
 * @param     Kpp            Proportional gain for the position control.
 * @param     Kdp            Derivative gain for the position control.
 * @param     speedM         Current speed of the motor (measured in steps per second).
 * @return    Control output for the motor to achieve the desired position.
 */
float positionPDControl(long actualPos, long setPointPos, float Kpp, float Kdp, int16_t speedM)
{
    float P = constrain(Kpp * (float)(setPointPos - actualPos), -115.0f, 115.0f);
    return P + Kdp * (float)speedM;
}

/**
 * @brief     Interrupt Service Routine for TIMER 1 Compare Match A. This ISR is responsible for controlling the stepper motor 1 speed and direction.
 *            It is triggered at a frequency determined by the OCR1A register, which is set by the setMotorSpeed function.
 *            The ISR sets the STEP pin for motor 1 to generate a step pulse, and updates the steps count based on the direction of the motor.
 */
ISR(TIMER1_COMPA_vect)
{
    if (dir_M1 == 0)
        return;

    SET(STEP_M1_PORT, STEP_M1_PIN);
    //asm volatile("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop");
    CLR(STEP_M1_PORT, STEP_M1_PIN);

    if (dir_M1 > 0)
        steps1--;
    else
        steps1++;
}

/**
 * @brief     Interrupt Service Routine for TIMER 3 Compare Match A. This ISR is responsible for controlling the stepper motor 2 speed and direction.
 *            It is triggered at a frequency determined by the OCR3A register, which is set by the setMotorSpeed function.
 *            The ISR sets the STEP pin for motor 2 to generate a step pulse, and updates the steps count based on the direction of the motor.
 */
ISR(TIMER3_COMPA_vect)
{
    if (dir_M2 == 0)
        return;

    SET(STEP_M2_PORT, STEP_M2_PIN);
    //asm volatile("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop");
    CLR(STEP_M2_PORT, STEP_M2_PIN);

    if (dir_M2 > 0)
        steps2--;
    else
        steps2++;
}

/**
 * @brief     Function to set the speed of the motors. This function takes a desired speed for the motor, applies
 *            acceleration limits, and updates the timer period and direction for the motor control.
 * @param     motorNum   Motor number (1 or 2) to set the speed for.
 * @param     tspeed     Desired speed for the motor (in steps per second). Positive values for forward, negative for backward, and zero for stop.
 */
void setMotorSpeed(uint8_t motorNum, int16_t tspeed)
{
    int32_t speed;
    int32_t timer_period;
    int16_t         &speed_M = (motorNum == 1) ? speed_M1 : speed_M2;
    volatile int8_t &dir_M   = (motorNum == 1) ? dir_M1   : dir_M2;

    // WE LIMIT MAX ACCELERATION of the motors
    if ((speed_M - tspeed) > MAX_ACCEL)
        speed_M -= MAX_ACCEL;
    else if ((speed_M - tspeed) < -MAX_ACCEL)
        speed_M += MAX_ACCEL;
    else
        speed_M = tspeed;

#if MICROSTEPPING==16
    speed = speed_M * 50;
#else
    speed = speed_M * 25;
#endif

    if (speed == 0)
    {
        timer_period = ZERO_SPEED;
        dir_M = 0;
    }
    else if (speed > 0)
    {
        timer_period = 2000000 / speed;
        dir_M = 1;
        if (motorNum == 1) SET(DIR_M1_PORT, DIR_M1_PIN);
        else               CLR(DIR_M2_PORT, DIR_M2_PIN); // Motor2 forward = CLR
    }
    else
    {
        timer_period = 2000000 / -speed;
        dir_M = -1;
        if (motorNum == 1) CLR(DIR_M1_PORT, DIR_M1_PIN);
        else               SET(DIR_M2_PORT, DIR_M2_PIN);
    }

    if (timer_period > 65535)
        timer_period = ZERO_SPEED;

    if (motorNum == 1)
    {
        OCR1A = timer_period;
        if (TCNT1 > OCR1A) TCNT1 = 0;
    }
    else
    {
        OCR3A = timer_period;
        if (TCNT3 > OCR3A) TCNT3 = 0;
    }
}
