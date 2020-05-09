/* Following is License message for DualVNH5019MotorShield : https://github.com/pololu/dual-vnh5019-motor-shield
-----------------start of the message---------------------
Copyright (c) 2014 Pololu Corporation.  For more information, see

http://www.pololu.com/
http://forum.pololu.com/

Permission is hereby granted, free of charge, to any person
obtaining a copy of this software and associated documentation
files (the "Software"), to deal in the Software without
restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following
conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.
-----------------end of the message--------------------- */

// parameter to store latest commnad to the motor driver
int last_left_command = 0, last_right_command = 0;

void motor_drive(int left_command, int right_command) {
        // ramp command to motor driver to avoid sudden current
        left_command = ramp_command(left_command, last_left_command);
        right_command = ramp_command(right_command, last_right_command);

        // regulate output for motor driver.
        left_command = regulate_command(left_command);
        right_command = regulate_command(right_command);

        md.setM2Speed(left_command);
        md.setM1Speed(right_command);

        // store latest command
        last_left_command = left_command;
        last_right_command = right_command;

        // for test
        command_L = left_command;
        command_R = right_command;
}


// Maximum change of command to motor driver to avoid steep change of command
int MAXIMUM_OUTPUT_CHANGE = 2;
int IRREGULER_INPUT_THRESHOLD = 200;
int ramp_command(int current_command, int last_command){
        int ramped_command = 0;

        if(abs(current_command - last_command) > IRREGULER_INPUT_THRESHOLD) {
                ramped_command = last_command;
        }
        if(current_command - last_command > MAXIMUM_OUTPUT_CHANGE) {
                ramped_command = last_command + MAXIMUM_OUTPUT_CHANGE;
        }
        else if(current_command - last_command < -1 * MAXIMUM_OUTPUT_CHANGE) {
                ramped_command = last_command - MAXIMUM_OUTPUT_CHANGE;
        }
        else{
                ramped_command = current_command;
        }

        return ramped_command;
}


int regulate_command(int command) {
        if (command > MAXIMUM_OUTPUT) {
                command = MAXIMUM_OUTPUT;
        }
        else if (command < -1 * MAXIMUM_OUTPUT) {
                command = -1 * MAXIMUM_OUTPUT;
        }
        return command;
}
