# MSP Examples
Two examples of MSP on Arduino. Works on both Arduino Uno and Arduino Due. You
need to run either `make setup_uno` (for Arduino Uno) or `make setup_due` (for
Arduino Due) before compiling the code. This will configure and copy over all
the needed MSP library code.

To remove the MSP library code, run `make clean`.

A use case example of MSP in experiment mode is found in the `experiment/`
directory and a corresponding OBC mode example under the `obc/` directory.

## How to run on Arduino Due
Follow these steps to run an example on an Arduino Due. To instead run it on an
Arduino Uno, skip step 2 and run `make setup_uno` instead of step 3.

1. Install the Arduino IDE. (Can be found here: https://www.arduino.cc/en/Main/Software)
2. Install the plugins required for compiling and flashing code to the Arduino Due. This can be installed via the Boards Manager in the Arduino IDE. The Boards Manager can be found under `Tools > Board: "<board type>" > Boards Manager...`. Search for "Due" in the Boards Manager and the necessary plugin should show up.
3. Run `make setup_due` from the terminal.
4. Open the experiment.ino or obc.ino file depending on which example you want to run.
5. Plug in the Arduino Due to the computer. Check under `Tools > Port` that the port matches that of the Arduino.
6. Hit Verify/Compile.
7. Upload the code to the Arduino.
8. Make sure that the Serial Monitor is open if the code contains any prints or debug messages. The Serial Monitor can be found under `Tools > Serial Monitor`.
9. For more information about Arduino IDE refer link https://www.arduino.cc/en/Guide/Environment
