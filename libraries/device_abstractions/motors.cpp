#include <Arduino.h>
#include <elapsedMillis.h>
#include <vector>
#include "motors.hh"
#include "bluetooth.hh"
#include "software_config.hh"
#include <cmath>

using namespace swconst;

const bool debug = false; // set to true for serial debugging statements

// regular mapping config
DriverConfig M0(motorLimitM0, convergenceTimeM0, p_l_M0, i_l_M0, d_l_M0,
    p_a_M0, i_a_M0, d_a_M0);
// mapping config only for turns
DriverConfig M0T(motorLimitM0, convergenceTimeM0, p_l_M0T, i_l_M0T, d_l_M0T,
    p_a_M0T, i_a_M0T, d_a_M0T);
// speed run config
DriverConfig S1(motorLimitS1, convergenceTimeS1, p_l_S1, i_l_S1, d_l_S1,
    p_a_S1, i_a_S1, d_a_S1);
// go mapping -> mapping with straight of ways -> speedrun
std::vector<DriverConfig> driverCfgsLinear = { M0, M0, S1};

/* Motor functions */
Motor::Motor(
    int powerPin,
    int directionPin,
    int encoderPin1,
    int encoderPin2,
    SensorArray sensors) :
    _powerPin(powerPin),
    _directionPin(directionPin),
    _encoder(encoderPin1, encoderPin2),
    _pid(
        &_pidInput,
        &_pidOutput,
        &_pidSetpoint,
        // m for motors
        p_m,
        i_m,
        d_m,
        DIRECT),
    _sensors(sensors)
{
    pinMode(_powerPin, OUTPUT);
    pinMode(_directionPin, OUTPUT);
    analogWriteFrequency(_powerPin, 22000);

    _pidSetpoint = 100000;
    _pidOutput = 0;
    _pid.SetOutputLimits(pidLimit * -1, pidLimit);
    _pid.SetTunings(p_m, i_m, d_m);
    //turn the PID on
    _pid.SetMode(AUTOMATIC);
}


void Motor::drive(int speed) {
    digitalWrite(_directionPin, speed > 0);
    analogWrite(_powerPin, speed >= 0 ? speed : speed * -1);
}


// Reads the current value of the encoder
long Motor::readTicks() {
    return _encoder.read();
}


// Moves a number of specified ticks as measured by the encoder
void Motor::moveTicks(long ticks) {
    // magic number
    int speed = 30;
    _encoder.write(0);
    while (_encoder.read() < abs(ticks)) {
        debug_print("Encoder Value for motorpin ");
        debug_print(_powerPin);
        debug_print(" ");
        debug_println(_encoder.read());
        drive(ticks > 0 ? speed : -1 * speed);
    }
    drive(0);
}

// Moves the ticks specified (mesaured by the encoder, and assisted by the PID)
void Motor::moveTicksPID(long ticks) {
    _pidSetpoint = ticks;
    _pidInput = 0;
    _encoder.write(0);
    long time = millis();
    while (abs(_pidSetpoint - _pidInput) > encoderError &&
        millis() - time < timeout)
    {
        _pidInput = _encoder.read();
        _pid.Compute();
        drive(_pidOutput);
    }
    drive(0);
}

// Returns the speed the motor should go according to the PID to acheive the
// setpoint.
float Motor::getPIDSpeed(float setpoint) {
    _pidSetpoint = setpoint;
    _pidInput = _encoder.read();
    debug_print("Encoder: ");
    debug_println(_pidInput);
    _pid.Compute();
    return _pidOutput;
}




/* Driver functions */
Driver::Driver(
    int powerPinL,
    int directionPinL,
    int powerPinR,
    int directionPinR,
    int motorModePin,
    int encoderPinL1,
    int encoderPinL2,
    int encoderPinR1,
    int encoderPinR2,
    SensorArray sensors,
    RGB_LED rgb) :
    _pid_x(p_l_M0, i_l_M0, d_l_M0),
    _pid_y(p_l_M0, i_l_M0, d_l_M0),
    _pid_a(p_a_M0, i_a_M0, d_a_M0),
    _pid_front_tof(p_tof, i_tof, d_tof),
    _pid_diag_tof(p_diag, i_diag, d_diag),
    _leftMotor(powerPinL, directionPinL, encoderPinL1, encoderPinL2, sensors),
    _rightMotor(powerPinR, directionPinR, encoderPinR1, encoderPinR2, sensors),
    _sensors(sensors),
    _rgb(rgb)
{
    curr_xpos = 0.0;
    curr_ypos = 0.0;
    curr_angle = 0.0;
    cfgNum = 0;
    pinMode(motorModePin, OUTPUT);
    digitalWrite(motorModePin, HIGH);
    encoderOnlyFlag = false;

    clearWallData();
}


void Driver::drive(int speed) {
    speed = floorPWM(speed, motorFloor);

    _leftMotor.drive(speed);
    _rightMotor.drive(speed);
}


void Driver::drive(int speedLeft, int speedRight) {
    _leftMotor.drive(speedLeft);
    _rightMotor.drive(speedRight);
}

void Driver::brake() {
    _leftMotor.drive(0);
    _rightMotor.drive(0);
}


// Sequentially moves each motor a specified amount of ticks
void Driver::moveTicks(long ticks) {
    _leftMotor.moveTicks(ticks);
    _rightMotor.moveTicks(ticks);
}


// Moves the motors to adjust to the input setpoint using PID
void Driver::movePID(float setpoint) {
    float leftSpeed = 1;
    float rightSpeed = 1;
    int start_flag = 0;
    while (fabs(leftSpeed) > 0.5 || fabs(rightSpeed) > 0.5) {
        float leftSpeed = _leftMotor.getPIDSpeed(setpoint);
        float rightSpeed = _rightMotor.getPIDSpeed(setpoint);
        // Debugging code
        if (debug) {
            debug_print("Left motor speed: ");
            debug_print(leftSpeed);
            debug_print(" Right motor speed: ");
            debug_print(rightSpeed);
            debug_print(" Setpoint: ");
            debug_println(setpoint);
        }
        // delay(500);
        if (fabs(leftSpeed) > 1 && fabs(rightSpeed) > 1) {
            start_flag = 1;
        }
        if (start_flag) {
            drive(leftSpeed, rightSpeed);
        }
    }
}

// generate PID outputs based on how far robot has traveled to goal
// (assumes PID setpoints are relative distances from init to goal)
void Driver::computePids(float init_xpos, float init_ypos,
    float angle_travelled) {
    _pid_x.input = curr_xpos - init_xpos;
    _pid_y.input = curr_ypos - init_ypos;
    _pid_a.input = angle_travelled;

    _pid_x.compute();
    _pid_y.compute();
    _pid_a.compute();
}


void Driver::debugPidMovement(float angle_travelled) {
    debug_printvar(_pid_x.input);
    debug_printvar(_pid_x.output);
    debug_printvar(_pid_x.setpoint);
    debug_printvar(curr_xpos);
    debug_println(" ");

    debug_printvar(_pid_y.input);
    debug_printvar(_pid_y.output);
    debug_printvar(_pid_y.setpoint);
    debug_printvar(curr_ypos);
    debug_println(" ");

    this->debugAngle(angle_travelled);

    debug_printvar(_v_left);
    debug_printvar(_v_right);
    debug_println(" ");
}

void Driver::debugAngle(float angle_travelled) {
    debug_printvar(_pid_a.input);
    debug_printvar(_pid_a.output);
    debug_printvar(_pid_a.setpoint);
    debug_printvar(angle_travelled);
    debug_printvar(curr_angle);
    debug_println(" ");
}


/* readWalls()
 *
 * Reads each of the short tof sensors values into an array for wall checking.
 *
 * When called, overwrites the values of the left diagonal, front, and right
 * diagonal time-of-flight sensors in an array on Driver object if they
 * fulfill certain conditions (the max/min value seen so far).
 *
 * TODO: improve this heuristic for choosing how to process wall readings.
 */
void Driver::readWalls() {
    /* debug code */
    if (debug) {
        debug_print("Walls read at ");
        debug_print("x=");
        debug_print(curr_xpos);
        debug_print(" y=");
        debug_println(curr_ypos);
    }
    /* end debug code */

    for (int i = 0; i < 4; i++) {
        if (i == RIGHTFRONT) { continue; } // ignore right front sensor
        if ((i == LEFTFRONT
            && _sensors.readShortTof(i) < shortTofWallReadings[1])
            || (_sensors.readShortTof(i) > shortTofWallReadings[i]))
        {
            shortTofWallReadings[i] = _sensors.readShortTof(i);
        }
    }
}


/* clearWallData()
 * Zeroes out the array of wall readings to prepare for another movement.
 */
void Driver::clearWallData() {
    for (int i = 0; i < 4; i++) {
        shortTofWallReadings[i] = 0;
    }
    shortTofWallReadings[LEFTFRONT] = 1000;
}


/* minTurn()
 * Calculates the minimum angle needed to turn from curr_angle to goal_angle
 * considering both left turn and right turn options.
 */
float minTurn(float goal_angle, float curr_angle) {
    goal_angle = wrapAngle(goal_angle);
    return wrapAngle(goal_angle - curr_angle + PI) - PI;
}


/* heading()
 * Returns 0 for +y axis, 1 for -x axis, 2 for -y axis, 3 for +x axis */
int Driver::heading(float goal_x, float goal_y) {
    if (fabs(goal_y - curr_ypos) > fabs(goal_x - curr_xpos)) {
        return goal_y > curr_ypos ? 0 : 2;
    }
    else {
        return goal_x > curr_xpos ? 3 : 1;
    }
}

/* use PID outputs and goal orientation to determine motor pwm values
 * (works well only when robot is already pointed in the direction it's
 * travelling, or if it's turning in place) */
void Driver::calculateInputPWM(bool angle_flag,
    float goal_x, float goal_y, float angle_diff)
{
    // use distance formula to get positive linear velocity
    float lin_velocity = angle_flag ? 0 :
        sqrt(pow(_pid_x.output, 2) + pow(_pid_y.output, 2));
    float ang_velocity = _pid_a.output;

    // L is width of robot
    // todo: makes sure ceiling doesnt drown out angle correction
    _v_left = lin_velocity - L * ang_velocity / 2;
    _v_right = lin_velocity + L * ang_velocity / 2;
    _v_left = ceilingPWM(_v_left, _v_right, motorLimit);
    _v_right = ceilingPWM(_v_right, _v_left, motorLimit);

    /*if (angle_flag){
        float mag = (fabs(_v_right) + fabs(_v_right)) / 2;
        _v_right = SIGN(_v_right) * mag;
        _v_left = -SIGN(_v_right) * mag;
    }*/

    if (!angle_flag &&
    (angle_diff <= PI / 2 || angle_diff >= 3 * PI / 2))
    {
        // moving forward - force motors forward
        if (_v_left + _v_right < 0) {
        _v_left = -1 * _v_left;
        _v_right = -1 * _v_right;
    }
    } else if (!angle_flag) {
        // moving backwards - force motors backwards
        if (_v_left + _v_right > 0) {
            _v_left = -1 * _v_left;
            _v_right = -1 * _v_right;
        }
    }
}


/* Moves based on absolute position on a coordinate grid */
void Driver::go(float goal_x, float goal_y, float goal_a, size_t interval, bool backwards) {
    float sample_t = 1. / interval;
    elapsedMillis timeElapsed = 1000;
    elapsedMillis pidTimer = 0;
    elapsedMillis sensorTimer = 0;
    elapsedMillis timeout = 0;
    int sensorCounter = 0;
    int end_iter = 0;
    bool angle_flag = goal_x == curr_xpos && goal_y == curr_ypos;

    if (angle_flag) {
        updateConfig(M0T);
    } else {
        updateConfig(driverCfgsLinear[cfgNum]);
    }

    // between -PI to PI
    goal_a = minTurn(goal_a, curr_angle);

    EncoderTicker leftEnc(&_leftMotor._encoder);
    EncoderTicker rightEnc(&_rightMotor._encoder);

    const float init_xpos = curr_xpos;
    const float init_ypos = curr_ypos;
    _pid_x.setpoint = goal_x - init_xpos;
    _pid_y.setpoint = goal_y - init_ypos;
    _pid_a.setpoint = goal_a;

    float init_imu_angle = _sensors.readIMUAngle();
    float last_imu_angle = init_imu_angle;
    float imu_angle = 0;
    float imu_change = 0;
    float last_rangefinder_angle = 0;
    float rangefinder_angle = 0;
    float rangefinder_change = 0;

    float imu_weight = nowall_imu_w;
    float encoder_weight = nowall_encoder_w;
    float rangefinder_weight = nowall_rangefinder_w;

    int ignore_rangefinder = 0; // 0 for use all, 1 for left, 2 for right, 3 for none
    float ignore_init_pos = 0;

    float angle_travelled = 0; // -PI to PI
    float front_dist = 0;
    float front_diff = 0;
    pinMode(13, OUTPUT);


    do {
        /* stores sensor readings to detect walls
         * NOTE: will break if sensors not initialized */
        if ((goal_y != init_ypos || goal_x != init_xpos) &&
            sensorTimer > sensorRefreshTime &&
            sensorCounter < numWallChecks)
        {
            sensorTimer = 0;
            sensorCounter++;
            readWalls();
        }
        // super fast loop where we update state based on encoders
        if (timeElapsed > interval) {
            // reset sample time
            timeElapsed = 0;

            // slower loop where we update our motor outputs using PID
            if (pidTimer > pidSampleTime) {
                pidTimer = 0;

                computePids(init_xpos, init_ypos, angle_travelled);

                // (same calcuation as temp_a in tankGo)
                float travel_angle = atan2f(
                    -1*(goal_x - curr_xpos), goal_y - curr_ypos);

                float angle_diff = fabs(wrapAngle(curr_angle) -
                    wrapAngle(travel_angle));

                calculateInputPWM(angle_flag, goal_x, goal_y, angle_diff);
                debugPidMovement(angle_travelled);
                drive(_v_left, _v_right);

                /* If the movement looks like it's reached the goal position
                or it's converged, stop the movement */
                if ((withinError(goal_x, curr_xpos, errorX) &&
                     withinError(goal_y, curr_ypos, errorY) &&
                     withinError(goal_a, angle_travelled, errorA)) ||
                    (fabs(_v_left) < motorCloseEnough && fabs(_v_right) < motorCloseEnough) ||
                    // perpendicular to goal direction means it's either
                    // right next to destination, or it's hopeless anyway
                    (!angle_flag &&
                        fabs(angle_diff - PI / 2) <= perpendicularError))
                {
                    end_iter++;
                }
                else {
                    end_iter = std::max(end_iter - 5, 0);
                }

                if (end_iter > convergenceTime) {
                    break;
                }
            }

            /* Update positional state, curr_xpos and curr_ypos */
            float true_v_left = leftEnc.diffLastRead() *
                ticksToCm / sample_t;
            float true_v_right = rightEnc.diffLastRead() *
                ticksToCm / sample_t;

            curr_xpos += (true_v_left + true_v_right) / 2  *
                sample_t * -1 * sinf(curr_angle);
            curr_ypos += (true_v_left + true_v_right) / 2 *
                sample_t * cosf(curr_angle);

            // IMU update
            imu_angle = _sensors.readIMUAngle();
            imu_change = wrapAngle(PI+(last_imu_angle - imu_angle) * degToRad)-PI; //imu backwards in angle
            last_imu_angle = imu_angle;

            // integrates rangefinder offset
            if (!angle_flag && !encoderOnlyFlag) {
                if (ignore_rangefinder == 0) {
                    switch (heading(goal_x, goal_y)) {
                        case 0:
                        case 2: {
                            ignore_init_pos = curr_ypos;
                            break;
                        }
                        case 1:
                        case 3: {
                            ignore_init_pos = curr_xpos;
                            break;
                        }
                    }
                }
                float alpha = 0.9;
                float left_diag_dist = _sensors.readShortTof(LEFTDIAG);
                float right_diag_dist = _sensors.readShortTof(RIGHTDIAG);
                float left_front_dist = _sensors.readShortTof(LEFTFRONT);
                float right_front_dist = _sensors.readShortTof(RIGHTFRONT);
                front_dist = .5*alpha*(left_front_dist + right_front_dist) + (1-alpha)*front_dist;
                front_diff = alpha*(left_front_dist - right_front_dist - diag_correction) + (1-alpha)*front_diff;

                imu_weight = nowall_imu_w;
                encoder_weight = nowall_encoder_w;
                rangefinder_weight = nowall_rangefinder_w;
                digitalWrite(13, LOW);


                // not close to a wall on the front
                if (front_dist > front_wall_threshold) {
                    // wall on left side
                    if (((left_diag_dist >= tof_low_bound && left_diag_dist <= tof_high_bound)
                        || (right_diag_dist >= tof_low_bound && right_diag_dist <= tof_high_bound))
                        && ignore_rangefinder != 3)
                    {
                        imu_weight = imu_w;
                        encoder_weight = encoder_w;
                        rangefinder_weight = rangefinder_w;
                        digitalWrite(13, HIGH);


                        // walls on both sides to follow
                        if (((left_diag_dist >= tof_low_bound && left_diag_dist <= tof_high_bound)
                            && (right_diag_dist >= tof_low_bound && right_diag_dist <= tof_high_bound))
                            && ignore_rangefinder == 0)
                        {
                            float ratio = 0.5*(acosf(20./right_diag_dist) - acosf(20./left_diag_dist));
                            if (!std::isnan(ratio) && !std::isinf(ratio)) {
                                rangefinder_angle = alpha*(ratio) + (1-alpha)*rangefinder_angle;
                                rangefinder_change = rangefinder_angle - last_rangefinder_angle;
                                last_rangefinder_angle = rangefinder_angle;
                            }
                        }
                        // just use right wall to wallfollow
                        else if (right_diag_dist >= tof_low_bound && right_diag_dist <= tof_high_bound)
                        {
                            float ratio = acosf(20./right_diag_dist) - 1.05;
                            if (!std::isnan(ratio) && !std::isinf(ratio)) {
                                rangefinder_angle = alpha*(ratio) + (1-alpha)*rangefinder_angle;
                                rangefinder_change = rangefinder_angle - last_rangefinder_angle;
                                last_rangefinder_angle = rangefinder_angle;
                            }
                            ignore_rangefinder = 2;
                        }
                        // just use left wall to wallfollow
                        else {
                            float ratio = 1.05 - acosf(20./left_diag_dist);
                            if (!std::isnan(ratio) && ! std::isinf(ratio)) {
                                rangefinder_angle = alpha*(ratio) + (1-alpha)*rangefinder_angle;
                                rangefinder_change = rangefinder_angle - last_rangefinder_angle;
                                last_rangefinder_angle = rangefinder_angle;
                            }
                            ignore_rangefinder = 1;
                        }
                    }
                    // don't wall follow
                    else {
                        ignore_rangefinder = 3;
                        _rgb._turnOff(0);
                        _rgb._turnOff(1);
                        _rgb._turnOff(2);
                        // rangefinder_angle = 0;
                        // rangefinder_change = 0;
                        // last_rangefinder_angle = 0;
                    }
                    switch (heading(goal_x, goal_y)) {
                        case 0:
                        case 2: {
                            if (fabs(curr_ypos - ignore_init_pos) >= distance_limit) {
                                ignore_rangefinder = 0;
                            }
                            break;
                        }
                        case 1:
                        case 3: {
                            if (fabs(curr_xpos - ignore_init_pos) >= distance_limit) {
                                ignore_rangefinder = 0;
                            }
                            break;
                        }
                    }
                    if (ignore_rangefinder < 3) {
                        _rgb.turnOn(ignore_rangefinder);
                    }
                }
                else {
                    //imu_weight = imu_w;
                    //encoder_weight = encoder_w;
                    //rangefinder_weight = rangefinder_w;

                    //rangefinder_angle = alpha * atan2f(front_diff*2 , 52) + (1-alpha)*rangefinder_angle; // rangefinders 52 mm apart
                    //rangefinder_change = rangefinder_angle - last_rangefinder_angle;
                    //last_rangefinder_angle = rangefinder_angle;
                    //digitalWrite(13,HIGH);
                    rangefinder_angle = 0;
                    rangefinder_change = 0;
                    last_rangefinder_angle = 0;
                }
            }

            /* Update angular state, curr_angle */
            float true_ang_v = (true_v_right - true_v_left) / L;
            float angle_change = imu_weight * imu_change +
                encoder_weight * (true_ang_v * sample_t) +
                rangefinder_weight * rangefinder_change;

            curr_angle = wrapAngle(curr_angle + angle_change);

            // the current angle wrapped from 0 to 2PI
            if (backwards) {
                angle_travelled -= angle_change;
            } else if (imuOn && !angle_flag && rangefinder_weight == 0) { // ignore this condition, TODO
                angle_travelled = wrapAngle(PI+(init_imu_angle - imu_angle) * degToRad)-PI;;
            } else {
                angle_travelled += angle_change;
            }
        }
    } while (timeout < pidLoopTimeout);

    brake();
    if (debug) debug_println("Done with movement.");
}

// move forward given relative distance
void Driver::forward(float distance) {
    float goal_x = curr_xpos - sinf(curr_angle) * distance;
    float goal_y = curr_ypos + cosf(curr_angle) * distance;
    go(goal_x, goal_y, curr_angle);
}

// turn left relative degrees
void Driver::turnLeft(float degrees) {
    float goal_a = curr_angle + degToRad * degrees;
    go(curr_xpos, curr_ypos, goal_a);
}

// turn right relative degrees
void Driver::turnRight(float degrees) {
    turnLeft(-1 * degrees);
}

/* Moves the robot to the input goal state in discrete tank style movements
 * of move forward and turn */
void Driver::tankGo(float goal_x, float goal_y, bool back_wall, bool backwards) {
    // calculation for the desired angle to face the goal coordinates
    float temp_a = atan2f(-1*(goal_x - curr_xpos), goal_y - curr_ypos);

    debug_printvar(curr_angle);
    debug_printvar(temp_a);
    debug_printvar(fabs(temp_a - curr_angle));
    debug_printvar(backwards);

    if (withinError(fabs(temp_a - curr_angle), PI, 0.4) && backwards) {
        // go to the opposite angle
        debug_printvar(temp_a);
        debug_printvar(curr_angle);
        float opp_angle = wrapAngle(temp_a + PI);
        go(curr_xpos, curr_ypos, opp_angle);
        delay(500);
        // go backwards
        go(goal_x, goal_y, opp_angle, 1, true);

    }
    else if (fabs(temp_a - curr_angle) > PI / 12) {
        // Turn
        debug_println(temp_a);
        go(curr_xpos, curr_ypos, temp_a);
        // back align after turning, if there is a wall behind
        if (back_wall){
            backAlign();
        }

        delay(200);
        debug_println("Finished turn of tank go.");
        // Go forward
        go(goal_x, goal_y, temp_a);
        debug_println("Finished straight go.");
    }
    else {
        go(goal_x, goal_y, temp_a);
    }

    // Front Re-align if near the wall
    if (_sensors.readShortTof(LEFTFRONT) < 80) {
        realign(front_wall_align);
    }
}


void Driver::resetState() {
    this->curr_xpos = 0;
    this->curr_ypos = 0;
    this->curr_angle = 0;
}

// realign on a front wall using distance sensors
void Driver::realign(int goal_dist) {
    _pid_front_tof.setpoint = goal_dist;
    // right diag reads less than left diag
    _pid_diag_tof.setpoint = 0;
    float alpha = 0.9;
    int counter = 0;
    int direction = round(wrapAngle(curr_angle) + PI / 4) / (PI / 2);

    float left_front_dist = _sensors.readShortTof(LEFTFRONT);
    float right_front_dist = _sensors.readShortTof(RIGHTFRONT);
    float front_diff = left_front_dist - right_front_dist - diag_correction;
    float front_dist = .5*(left_front_dist + right_front_dist);

    // use single loop to get to proper distance and perpendicular to front wall
    elapsedMillis timeout = 0;
    while (timeout < pidLoopTimeout) {
        left_front_dist = _sensors.readShortTof(LEFTFRONT);
        right_front_dist = _sensors.readShortTof(RIGHTFRONT);
        // float diag_diff = left_diag_dist - right_diag_dist;
        front_diff = alpha*(left_front_dist - right_front_dist - diag_correction) + (1-alpha)*front_diff;
        front_dist = alpha*.5*(left_front_dist + right_front_dist) + (1-alpha)*front_dist;

        // end condition
        if (withinError(front_dist, goal_dist, wall_error) &&
            withinError(front_diff, 0, 2)) {
            counter++;
        }
        else {
            counter = 0;
        }

        if (counter >= convergenceTime) {
            break;
        }

        _pid_front_tof.input = front_dist;
        _pid_diag_tof.input = front_dist < front_threshold ? atan2f(front_diff*2, 52.) : 0;
        _pid_front_tof.compute();
        _pid_diag_tof.compute();

        float pwm = -1 * _pid_front_tof.output;
        // can't input pidlimit so lowered cap by multiplying
        float angle_correction = _pid_diag_tof.output;
        //Serial.println(pwm);
        drive(pwm - angle_correction, pwm + angle_correction);
    }
    brake();

    // correct state based on which wall it realigned on
    // Pointing east or west -> x-axis
    if (direction % 2 == 1) {
        int current_col = round(curr_xpos / cellSize);
        curr_xpos = current_col * cellSize;
    }
    else {
        int current_row = round(curr_ypos / cellSize);
        curr_ypos = current_row * cellSize;
    }
    // angular state updates
    float new_angle = direction * PI / 2;
    curr_angle += (new_angle - curr_angle) * angle_correction_ratio;
}

// align on back wall, then move back to middle of cell
void Driver::backAlign() {
    // if there is a wall behind, back into it
    elapsedMillis encoderTimer = 0;
    debug_println("backaligning");
    int backingPWM = backAlignPWM;
    drive(backingPWM, backingPWM);
    EncoderTicker leftEnc(&_leftMotor._encoder);
    EncoderTicker rightEnc(&_rightMotor._encoder);
    long left_diff = 501;
    long right_diff = 501;
    // stop when encoders show wheels stop turning (hit wall)
    while(abs(left_diff) > 300 || abs(right_diff) > 300) {
        if (encoderTimer > 40) {
            encoderTimer = 0;
            left_diff = leftEnc.diffLastRead();
            right_diff = rightEnc.diffLastRead();
        }
        drive(backingPWM, backingPWM);
    }
    brake();
    // state update
    int direction = round(wrapAngle(curr_angle) + PI / 4) / (PI / 2);
    float diff_pos = direction == 1 || direction == 2 ? -1 * backedOffset : backedOffset;
    // Pointing east or west -> x-axis
    if (direction % 2 == 1) {
        int current_col = round(curr_xpos / cellSize);
        curr_xpos = current_col * cellSize - diff_pos;
    }
    else {
        int current_row = round(curr_ypos / cellSize);
        curr_ypos = current_row * cellSize - diff_pos;
    }

    // angular state update
    float new_angle = direction * PI / 2;
    curr_angle += (new_angle - curr_angle); //* angle_correction_ratio;
    _rgb.flashLED(1);
    delay(200);
    forward(backedOffset);
}

void Driver::updateConfig(DriverConfig cfg) {
    motorLimit = cfg.motorLimit;
    convergenceTime = cfg.convergenceTime;
    _pid_x.setTunings(cfg.p_l, cfg.i_l, cfg.d_l);
    _pid_y.setTunings(cfg.p_l, cfg.i_l, cfg.d_l);
    _pid_a.setTunings(cfg.p_a, cfg.i_a, cfg.d_a);
}
