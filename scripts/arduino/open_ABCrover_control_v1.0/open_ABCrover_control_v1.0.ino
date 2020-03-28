#include <TimerOne.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// constants for an IMU
#define GRAVITATIONAL_ACCEL 9.798 //at TOKYO
#define IMU_SAMPLERATE_HZ 100 //refresh rate of the imu (looks like it should be fixed at 100Hz due to bno055 hardware...?)

// constants for pin assign
#define ENC_LA 2 // input from encoder, left motor, phaseA
#define ENC_LB 3 // left motor phaseB
#define ENC_RA 18 // right motor, phaseA
#define ENC_RB 19 // right motor, phaseB

// constants for motors
#define PULSE_PER_ROUND 723.24 // encoder pulse resolution * gear ratio

// flag to control whether timer interruption is ignited or not
volatile bool interrupt_flag = false;

//paramters to count time
int time1, time2, time3, past_time;

//parameters to store quaternions
float quat_w, quat_x, quat_y, quat_z;

//parameters for motors/encoders
volatile byte pulse_L, last_pulse_L, pulse_R, last_pulse_R;
volatile long count_L = 0, count_R = 0;
int pwm_L, pwm_R; // pwm output for motor driver
float battery1_voltage, battery2_voltage;



// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);



void setup() {
  // timer interrupt initialization
  Timer1.initialize(1000000 / IMU_SAMPLERATE_HZ);
  Timer1.attachInterrupt(interrupt);

  // serial communication initialization
  Serial.begin(230400);


  /* Initialise the bno055_imu */
  if (!bno.begin())
  { /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  delay(100);
  bno.setExtCrystalUse(true);

  // change bitrate of I2C comm to 400kbps (100kbps, usual)
  Wire.setClock(400000);

  // setup arduino GPIO
  pinMode(ENC_LA, INPUT_PULLUP);
  pinMode(ENC_LB, INPUT_PULLUP);
  pinMode(ENC_RA, INPUT_PULLUP);
  pinMode(ENC_RB, INPUT_PULLUP);

  // setup hardware interrupt
  attachInterrupt(2, encoder_read, CHANGE); // pin2
  attachInterrupt(3, encoder_read, CHANGE); // pin3
  attachInterrupt(4, encoder_read, CHANGE); // pin19
  attachInterrupt(5, encoder_read, CHANGE); // pin18
}

void loop() {
  // actual process when timer interruption was ingnited
  if (interrupt_flag == true) {
    get_IMUdata();
    //    print_time();
    interrupt_flag = false;
  }

  if (Serial.available() > 2) {
    if (Serial.read() == 'H') {//only when received data starts from 'H'
      // read data
      pwm_L = Serial.read();
      pwm_R = Serial.read();

      // send data
      Serial.println(quat_w);
      Serial.println(quat_x);
      Serial.println(quat_y);
      Serial.println(quat_z);
      Serial.println(battery1_voltage);
      Serial.println(battery2_voltage);
    }
  }
}


// func at the timer interrupt. Just turn on a flag and actual procedure will in the main loop.
void interrupt() {
  interrupt_flag = true;
}
