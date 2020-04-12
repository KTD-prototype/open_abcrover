void motor_drive(int left_pwm, int right_pwm) {
  // regulate pwm and remap from 0-255 to 0-400(due to config of the motor drive library)
  left_pwm = regulate_output(left_pwm * 80 / 51);
  right_pwm = regulate_output(right_pwm * 80 / 51);

  md.setM2Speed(left_pwm);
  md.setM1Speed(right_pwm);
  pwm_L = left_pwm;
  pwm_R = right_pwm;
}


int regulate_output(int pwm_value) {
  if (pwm_value > MAXIMUM_OUTPUT) {
    pwm_value = MAXIMUM_OUTPUT;
  }
  else if (pwm_value < -1 * MAXIMUM_OUTPUT) {
    pwm_value = -1 * MAXIMUM_OUTPUT;
  }
  return pwm_value;
}
