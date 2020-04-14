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
int MAXIMUM_OUTPUT_CHANGE = 5;
int ramp_command(int current_command, int last_command){
  int ramped_command = 0;

  if(current_command - last_command > MAXIMUM_OUTPUT_CHANGE){
    ramped_command = last_command + MAXIMUM_OUTPUT_CHANGE;
  }
  else if(current_command - last_command < -1 * MAXIMUM_OUTPUT_CHANGE){
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
