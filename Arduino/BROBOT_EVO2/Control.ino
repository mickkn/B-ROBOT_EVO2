// BROBOT EVO 2 by JJROBOTS
// SELF BALANCE ARDUINO ROBOT WITH STEPPER MOTORS
// License: GPL v2
// Control functions (PID controls, Steppers control...)

// PD controller implementation(Proportional, derivative). DT in seconds
float stabilityPDControl(float DT, float input, float setPoint, float Kp, float Kd)
{
    float error;
    float output;

    error = setPoint - input;

    // Kd is implemented in two parts
    //    The biggest one using only the input (sensor) part not the SetPoint input-input(t-1).
    //    And the second using the setpoint to make it a bit more agressive   setPoint-setPoint(t-1)
    float Kd_setPoint = constrain((setPoint - setPointOld), -8, 8); // We limit the input part...
    output = Kp * error + (Kd * Kd_setPoint - Kd * (input - PID_errorOld)) / DT;
    //Serial.print(Kd*(error-PID_errorOld));Serial.print("\t");
    //PID_errorOld2 = PID_errorOld;
    PID_errorOld = input;  // error for Kd is only the input component
    setPointOld = setPoint;
    return (output);
}


// PI controller implementation (Proportional, integral). DT in seconds
float speedPIControl(float DT, int16_t input, int16_t setPoint, float Kp, float Ki)
{
    int16_t error;
    float output;

    error = setPoint - input;
    PID_errorSum += constrain(error, -ITERM_MAX_ERROR, ITERM_MAX_ERROR);
    PID_errorSum = constrain(PID_errorSum, -ITERM_MAX, ITERM_MAX);

    //Serial.println(PID_errorSum);

    output = Kp * error + Ki * PID_errorSum * DT; // DT is in miliseconds...
    return (output);
}


float positionPDControl(long actualPos, long setPointPos, float Kpp, float Kdp, int16_t speedM)
{
    float output;
    float P;

    P = constrain(Kpp * float(setPointPos - actualPos), -115, 115); // Limit command
    output = P + Kdp * float(speedM);
    return (output);
}

// TIMER 1 : STEPPER MOTOR1 SPEED CONTROL
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

// TIMER 3 : STEPPER MOTOR2 SPEED CONTROL
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
