int countup = 0;
bool led1 = false, led2 = false;

void voltage_alert(float volt1, float volt2) {

  // check battery 1
  if ((volt1 <= 13.5) && (volt1 > 0)) { // if it's zero, it means no battery is connected
    if (countup > 100) {
      led1 = !led1;
      digitalWrite(alert_led1, led1);
    }
  }
  else {
    digitalWrite(alert_led1, LOW);
  }

  // check battery 2
  if ((volt2 <= 13.5) && (volt2 > 0)) {
    if (countup > 100) {
      led2 = !led2;
      digitalWrite(alert_led2, led2);
    }
  }
  else {
    digitalWrite(alert_led2, LOW);
  }

  // reset count
  if (countup > 100) {
    countup = 0;
  }
}
