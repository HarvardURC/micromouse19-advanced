#include <elapsedMillis.h>
#include <EEPROM.h>
#include "bluetooth.hh"       // bluetoothInitialize, bleReady
#include "config.h"           // ::pins,
#include "io.hh"              // RGB_LED, Buzzer, Button
#include "maze.hh"            // Position, Maze
#include "motors.hh"          // Motor, Driver
#include "sensors.hh"         // SensorArray
#include "software_config.hh" // ::swconst
#include <algorithm>

#define BUFSIZE 20  // bluetooth buffer
enum LED_Color { LED_RED, LED_GREEN, LED_BLUE };

// There are several pin definitions in config.h: read it.
using namespace pins;

Maze* maze;
Driver* driver;
SensorArray* sensorArr;
Buzzer* buzz;
Button* backButt;
Button* frontButt;
RGB_LED* backRgb;
RGB_LED* frontRgb;

int command_flag = 0;     // wait for a command or button press
int swap_flag = 0;        // if true return to the start
char command[BUFSIZE];    // buffer to hold bluetooth commands
bool bluetooth = true;    // activate bluetooth (and command system)
bool backupFlag = false;  // back up instead of turning 180 degrees
// bool abort_run = 0;

// this all should be placed into a header file
void makeNextMove(Position next);
bool commandIs(const char* token, const char* cmd, bool firstchar=false);
void waitButton();
void waitCommand();
void celebrate();

void setup() {

    /* MOUSE HARDWARE INTITALIZATION */

    // ?? need to power this pin to make the motors work ??
    pinMode(motorMode, OUTPUT);
    digitalWrite(motorMode, HIGH);

    // set up serial interface at 9600 bits per second; give 500 ms init time
    Serial.begin(9600);
    delay(500);

    // values from ::pins, remember?
    frontRgb = new RGB_LED(frontLedR, frontLedG, frontLedB);
    backRgb  = new RGB_LED(backLedR, backLedG, backLedB);
    // * flash red to indicate that the mouse turned on
    backRgb->flashLED(LED_RED);

    // * why are they named like this?
    // values from ::pins
    buzz = new Buzzer(buzzer);
    backButt = new Button(backButton);
    frontButt = new Button(frontButton);

    sensorArr = new SensorArray(
        // * tof: "time of flight"
        tofDiagL,
        tofFrontL,
        tofFrontR,
        tofDiagR,
        imuRST
    );

    driver = new Driver(
        motorPowerL,
        motorDirectionL,
        motorPowerR,
        motorDirectionR,
        motorMode,
        encoderL1,
        encoderL2,
        encoderR1,
        encoderR2,
        *sensorArr,
        *frontRgb
    );

    driver->encoderOnlyFlag = true;
    driver->imuOn = false;

    /* FLOODFILL INITIALIZATION */

    maze = new Maze();
    maze->initializeMaze();
    maze->setBoundaryWalls();

    // indicate that we reached up until the bluetooth setup
    backRgb->flashLED(LED_GREEN);

    if (bluetooth) {
        backRgb->flashLED(LED_BLUE);
        // bluetoothInitialize polls every 500ms until you connect a BT device
        bluetoothInitialize();
        command[0] = '\0';
    }

    // confirms that a BT device has been connected
    // On Mac, just use Adafruit Bluefruit LE Connect from app store
    // * can't be wrapped in the above if-then block above for some reason
    if (bluetooth) {
      frontRgb->flashLED(LED_BLUE);
    }
}


void loop() {
    if (command_flag >= 0) {
        debug_println("Waiting on command");
        if (bluetooth) {
            waitCommand();
        } else {
            waitButton();
        }
    }

    // * not clear exactly what's going on, but looks like it's for
    // * going back to the start when finishing maze
    // ** even or odd means that the run is coming from start to end
    // * or from end back to start
    if ( (maze->currPos == maze->goalPos  && maze->counter % 2 == 0)
      || (maze->currPos == maze->startPos && maze->counter % 2 == 1))
    {
        maze->counter++;

        debug_println("Swapping goal....");
        if (maze->counter <= 2) {
            buzz->siren();
            maze->writeEEPROM();
        }
        if (maze->currPos == maze->startPos) {
            command[0] = '\0';
            driver->resetState();
            driver->cfgNum = std::min(maze->counter / 2, 2);
            command_flag = 1;
        } else if (maze->currPos == maze->goalPos) {
            celebrate();
        }
    }

    // run the flood-fill algorithm
    maze->floodMaze();

    // determine next cell to move to
    Position next_move = maze->chooseNextCell(maze->currPos, maze->counter >= 2);
    debug_print("Next move -- ");
    next_move.print(bluetooth);

    // move to that cell
    makeNextMove(next_move);
    maze->updatePosition(next_move);

    // update walls
    maze->addWalls(driver->curr_angle,
                   driver->shortTofWallReadings[LEFTDIAG],
                   driver->shortTofWallReadings[LEFTFRONT],
                   driver->shortTofWallReadings[RIGHTDIAG]);

    // only print the walls on the speedrun
    // * adham says maybe only supposed to happen on the mapping run
    if (maze->counter == 0) {
        debug_print("Walls:");
        for (int i = 0; i < 4; i++) {
            // ignore right front tof
            // * but why??
            if (i == RIGHTFRONT) { continue; }
            debug_print(driver->shortTofWallReadings[i]);
            debug_print(" ");
        }
        debug_println(" ");
    }


    driver->clearWallData();
    maze->printWallsCell(next_move);
}


/**
 * void makeNextMove(Position)
 * Given the position of the next cell, converts the position to (x, y)
 * coordinates in centimeters and runs tankGo() to for precise movement
 * in the mapping phase.
 */
void makeNextMove(Position next) {
    Position diff = next - maze->currPos;
    debug_print("Diff direction: ");
    debug_println(diff.direction());

    bool backupMode = backupFlag ? maze->wallBehind(diff.direction()) : false;

    driver->tankGo(
        next.col * swconst::cellSize,
        next.row * swconst::cellSize,
        // maze->wallsOnSides(driver->curr_angle),
        backupMode
    );
    frontRgb->flashLED(LED_GREEN);
}


/**
 * void waitButton()
 * Waits on a button press. When pressed it starts the run of the maze.
 * Allows for a variety of functionality, check README for details.
 */
void waitButton() {
    // time interval for button's commands in milliseconds
    const long ledtime = 2000;

    for (;;) {
        /**
         *  press back button:
         *  0-2 sec for a run
         *  2-4 seconds for an EEPROM read
         *   4+ for EEPROM clear
         */
        if (backButt->read() == LOW) {
            // elapsedMillis type automatically counts time for us
            elapsedMillis timer = 0;
            while(backButt->read() == LOW) {
                if (timer < ledtime) {
                    backRgb->turnOn(0);
                } else if (timer < ledtime * 2) {
                    backRgb->turnOn(1);
                } else {
                    backRgb->turnOn(2);
                }
            }
            backRgb->turnOff();

            if (timer < ledtime) {
                // start the run
                driver->resetState();
                frontRgb->flashLED(LED_BLUE);
                delay(1000);
                frontRgb->flashLED(LED_GREEN);

                command_flag = -1000;
                break;
            } else if (timer < ledtime * 2) {
                maze->readEEPROM();
                buzz->siren();
            } else {
                buzz->siren();
                delay(50);
                buzz->siren();
                delay(50);
                buzz->siren();
                maze->clearEEPROM();
            }
        }

        /**
         *  press front button:
         *  0-2 sec to load next speed run config
         *  2-4 sec to flip the backup flag
         *  4-6 sec to turn on the ToF sensors           for guiding movement
         *   6+ sec to turn on the ToF sensors & the IMU for guiding movement
         */
        // older comment: "press front button to increment speed run counter"

        else if (frontButt->read() == LOW) {
            elapsedMillis timer = 0;
            while(frontButt->read() == LOW) {
                if (timer < ledtime) {
                    backRgb->turnOn(0);
                } else if (timer < ledtime * 2) {
                    backRgb->turnOn(1);
                } else if (timer < ledtime * 3) {
                    backRgb->turnOn(2);
                } else {
                    backRgb->turnOn(0);
                }
            }
            backRgb->turnOff();

            if (timer < ledtime) {
                // load next speed run config

                frontRgb->flashLED(LED_GREEN);
                if (maze->counter == 0) {
                    maze->counter++;
                } else {
                    maze->counter = std::min(maze->counter + 2, 4);
                }
            }
            else if (timer < ledtime * 2) {
                celebrate();
                backupFlag = !backupFlag;
            }
            else if (timer < ledtime * 3) {
                // turn on the ToF sensors for guiding movement
                buzz->siren();
                backupFlag = false;
                driver->encoderOnlyFlag = false;
            } else {
                // turn on the ToF sensors & the IMU for guiding movement
                backupFlag = true;
                driver->imuOn = true;
                driver->encoderOnlyFlag = false;
            }
        }
    }
}


/**
 * void waitCommand()
 * Waits on a command from bluetooth controller.
 * See README.md for valid commands.
 */
void waitCommand() {
    // command interface menu vals
    int tuning = -1;
    float p = 0;
    float i = 0;
    float d = 0;
    PidController nullPid(0, 0, 0);
    PidController* pid = &nullPid;
    const int notifyTime = 8000;
    elapsedMillis timer = 0;
    char* cmds = "reset, "
                 "fullreset, "
                 "go, "
                 "start, "
                 "celebrate, "
                 "setgoal, "
                 "tune, "
                 "quit";


    for (;;) {

        if (timer > notifyTime) {
            debug_println("Waiting on command...");
            timer = 0;
        }

        while (ble.available()) {
            ble.readline(command, BUFSIZE);
        }

        if (command[0] != '\0') {
            char* token = strtok(command, " ");

            /* definitions of command behaviors */

            // reset maze
            if (commandIs(token, "reset")) {
                driver->resetState();
                debug_println("Robot state reset. Ready for next run.");
            }
            else if (commandIs(token, "fullreset")) {
                driver->resetState();
                maze->reset();
                debug_print("Maze reset.\n");
            }
            // go to next cell
            else if (commandIs(token, "go")) {
                int numMoves = atoi(strtok(NULL, " "));
                command_flag = -1 * numMoves;

                frontRgb->flashLED(LED_BLUE);
                delay(1000);
                frontRgb->flashLED(LED_GREEN);
                break;
            }
            // continue without interruption
            else if (commandIs(token, "start")) {
                debug_println("Running maze.");
                driver->resetState();
                command_flag = -1000;
                break;
            }
            else if (commandIs(token, "celebrate")) {
                celebrate();
            }
            else if (commandIs(token, "setgoal")) {
                int row = atoi(strtok(NULL, " "));
                int col = atoi(strtok(NULL, " "));
                Position p(row, col);
                maze->setGoal(p);
            }
            else if (commandIs(token, "tune")) {
                tuning = 0;
                debug_println("Pick PID to tune. [linear, angular, fronttof]");
            }
            else if (commandIs(token, "quit")) {
                tuning = -1;
            }
            else if (tuning >= 0) {
                if (tuning > 0) {
                    p = pid->proportion;
                    i = pid->integral;
                    d = pid->derivative;

                    if (commandIs(token, "proportion")) {
                        token = strtok(NULL, " ");
                        p = atof(token);
                    }
                    else if (commandIs(token, "integral")) {
                        token = strtok(NULL, " ");
                        i = atof(token);
                    }
                    else if (commandIs(token, "derivative")) {
                        token = strtok(NULL, " ");
                        d = atof(token);
                    }
                    pid->setTunings(p, i, d);
                }
                else if (tuning == 0) {
                    if (commandIs(token, "linear")) {
                        tuning = 1;
                    }
                    else if (commandIs(token, "angular")) {
                        tuning = 2;
                    }
                    else if (commandIs(token, "fronttof")) {
                        tuning = 3;
                    }
                }

                if (tuning > 0) {
                    switch(tuning) {
                        case 1: {
                            pid = &driver->_pid_x;
                            debug_print("linear: ");
                            break;
                        }
                        case 2: {
                            pid = &driver->_pid_a;
                            debug_print("angular: ");
                            break;
                        }
                        case 3: {
                            pid = &driver->_pid_front_tof;
                            debug_print("fronttof: ");
                            break;
                        }
                    }
                    pid->printTunings();
                    debug_println(
                        "Pick var to tune. [proportion, integral, derivative]");
                        debug_println("Ex: 'proportion  10.5'");
                }
            }
            else if (commandIs(token, "help")) {
                debug_println("Possible commands: [" cmds "]");
            }
            else {
                debug_println("Invalid command.");
            }
            timer = 0;
        }
        command[0] = '\0';
    }
    command[0] = '\0';
}


/**
 * bool commandIs(const char*, const char*, bool)
 * Checks if the token is the same string as a command. If firstchar is
 * enabled, it passes if the first character is the same.
 */
bool commandIs(const char* token, const char* cmd, bool firstchar) {
    return !strcmp(token, cmd) || (firstchar && token[0] == cmd[0]);
}


/**
 * void celebrate()
 * Flashes the LEDs in celebration.
 */
void celebrate() {
    for (size_t j = 0; j < 4; j++) {
        for (size_t i = 0; i < 2; i++) {
            frontRgb->flashLED(i);
            backRgb->flashLED(i);
            buzz->siren();
            delay(50);
        }
    }
}
