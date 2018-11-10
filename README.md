## Library Setup
To setup the libraries on your machine you will need to symlink your local `/2017-2018/libraries` to your Arduino libraries folder. This will mean that every time you pull down updated libraries from Github they will also be updated for your local Arduino application without any nastiness on your part. The default location for your local Arduino libraries folder is:
* OSX: `/Users/your_username/Documents/Arduino/Libraries`
* Windows: `My Documents\Arduino\libraries\`

(Instructions for OSX/Mac, similar for Linux)
1. `cd` into your local copy of the repository and `cd` into `/2017-2018/libraries`.
2. Use `pwd` to get the current directory information and save this for later.
3. `cd` to where your Arduino libraries are located. Again use `pwd` to save the full directory path.
4. Run `ln -s source destination` and replace `source` with the first directory path and replace `destination` with the second directory path.

Use Finder to verify that the libraries are now in your Arduino folder. This should be a one-time process.

(Instructions for Windows) TODO: needs to be verified
1. `cd` into your local copy of the repository and `cd` into `/2017-2018/libraries`.
2. Use `cd` (no arguments) to get the current directory information and save this for later.
3. `cd` to where your Arduino libraries are located. Again use `cd` (no arguments) to save the full directory path.
4. Run `mklink /D source destination` and replace `source` with the first directory path and replace `destination` with the second directory path.

# Robot Overview

## Code Structure
#### mouse_master
The master `.ino` file for mapping and speed running the maze.
#### mouse_stripped
A stripped down version of the master code without all the bells and whistles of different modes and sensor combinations that the mouse_master allows for.
#### mouse_testing
A series of test sets that can be run to verify the integrity and tune PIDs for different movements and compound movements.
#### tests/
A group of hardware or software specific tests for testing functionality.
#### libraries/
A folder containing all libraries used in the robot code.
#### libraries/device_abstractions/
A set of high-level libraries we wrote for the different parts/hardware of the robot.
#### libraries/hadware_config/
Config files for the robot that are populated with Teensy pin numbers.

## Operation
### mouse_master
#### Setup
1. When the mouse turns on, the back LED should flash red.
2. Wait until the back LED flashes blue, then connect to the bluetooth module using the `Adafruit Bluetooth LE Connect` app.
3. After connecting, the back LED should flash green indicating it is ready for commands. 

#### Bluetooth Commands
- 'reset' : Resets the internal state of the robot to start a new run without accumulated error from the previous run. This is useful for if the robot completed a maze run and you want to reset its start position, or if you moved it a bit when placing it down. This is also called whenever you use the 'start' command so it is enough to just place the robot at the start of the maze and use the 'start' command.
- 'fullreset' : Additionally resets the wall map of the maze to start a fresh mapping run. Useful for fresh mapping maze runs.
- 'go' or 'go X' : The prior advances a single cell in the maze. The latter you can replace 'X' with any positive integer to have the mouse make X number of cell moves in the maze.
- 'start' : Starts the mouse of a full maze run.
- 'celebrate' : Flashes the LEDs and sounds the buzzer in a celebratory pattern.
- 'setgoal x y' : Sets the goal of the maze to be row 'x' and column 'y' (0 indexed). This is useful for testing different goals or maze configurations without having to reset the goal inside the code and upload every maze change.
- 'tune' : Enters the tuning menu for the PIDs. This is a powerful menu that allows setting the PID values of the driver to different values remotely. This can be good for trying different sets of PID values in quick repetition (prevents the recompile and upload step). Note: the values won't persist if you turn the mouse off, but it's useful for debugging. Make sure to save good values into the code.
- 'quit' : Exits out of the tuning menu interface.
- 'help' : Shows available commands.

#### Button Commands
There are visual LED cues for each duration of button press. It will change from red to green after two seconds, then to blue, then back to red.
##### Back Button
- 0-2 second press: Start the maze run.
- 2-4 second press: Read the maze from EEPROM memory.
- 4-6 second press: Clear EEPROM memory.

##### Front Button
- 0-2 second press: Load next speed run config. 
- 2-4 second press: Flip the backup flag (determines whether the mouse backs up or turns 180 degress when it needs to move to the cell immediately behind it.
- 4-6 second press: Turn ToF sensors on for guiding movement.
- 6-8 second press: Turn on the ToF sensors AND the IMU for guiding movement.

### mouse_testing
#### Operation
1. After turning the mouse on, the back LED should flash red indicating wait, and then green indicating ready. 
2. Press the back button on the mouse and the back LED will again flash red, and then green after one second. After flashing green it will start the maze run.

### mouse_testing
#### Setup
1. The back LED will flash red at the beginning of the setup function.
2. The back LED will flash green when it's done doing setup.

If you want to start with a different test suite than the basic level 0 tests, press the front button once. The back LED will flash green and  increment the counter changing the test suite number.

#### Operation
- Press the back button once to run the next test. 
- Press the front button once to repeat a test that's already run.

