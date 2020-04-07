#include <TimerOne.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "DualVNH5019MotorShield.h"

// constants for an IMU
#define GRAVITATIONAL_ACCEL 9.798 //at TOKYO
#define IMU_SAMPLERATE_HZ 100 //refresh rate of the imu (looks like it should be fixed at 100Hz due to bno055 hardware...?)

// constants for pin assign
#define ENC_LA 2 // input from encoder, left motor, phaseA
#define ENC_LB 3 // left motor phaseB
#define ENC_RA 18 // right motor, phaseA
#define ENC_RB 19 // right motor, phaseB
#define volt_input1 A2 // voltage monitor
#define volt_input2 A3 // voltage monitor
#define alert_led1 22 // battery1 voltage alert
#define alert_led2 24 // battery2 voltage alert

// constants for motors
#define PULSE_PER_ROUND 723.24 // encoder pulse resolution * gear ratio
#define MAXIMUM_OUTPUT 200 // maximum pwm output for motor

// flag to control whether timer interruption is ignited or not
volatile bool timer_interrupt_flag = false;

//paramters to count time
int time1, time2, time3, past_time;

//parameters for motors/encoders
volatile byte pulse_L, last_pulse_L, pulse_R, last_pulse_R;
volatile long encoder_count_L = 0, encoder_count_R = 0;


// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

// pin remap of the motor sheild
//                       1a,1b,1pw,1e,1cs,2a,2b,2pw,2e,2cs
DualVNH5019MotorShield md(5, 4, 9, 6, A0, 7, 8, 10, 12, A1);


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


        // initialize the motor driver shield
        md.init();

        // change bitrate of I2C comm to 400kbps (100kbps, usual)
        Wire.setClock(400000);

        // setup arduino GPIO
        pinMode(ENC_LA, INPUT_PULLUP);
        pinMode(ENC_LB, INPUT_PULLUP);
        pinMode(ENC_RA, INPUT_PULLUP);
        pinMode(ENC_RB, INPUT_PULLUP);

        // setup hardware interrupt
        attachInterrupt(0, encoder_read, CHANGE); // pin2 to left encoder phase A
        attachInterrupt(1, encoder_read, CHANGE); // pin3 to left encoder phase B
        attachInterrupt(5, encoder_read, CHANGE); // pin18 to right encoder phase A
        attachInterrupt(4, encoder_read, CHANGE); // pin19 to right encoder phase B
}

void loop() {

        //parameters to store quaternions(w,x,y,z)
        float quaternions[4];
        if (timer_interrupt_flag == true) { // get imu data at timer interruption
                get_quaternion_data(quaternions);
                timer_interrupt_flag = false; // toggle the flag again
        }


        // check the battery voltage (divided by 4, from 14.8V to 3.7V at nominal voltage)
        float battery1_voltage = analogRead(volt_input1) * 4 * 5 / 1023;
        float battery2_voltage = analogRead(volt_input2) * 4 * 5 / 1023;


        // prepare parameters for motor output before receiving commands
        int pwm_L, pwm_R;

        // communicate with host PC
        if (Serial.available() > 2) {
                if (Serial.read() == 'H') {//only when received data starts from 'H'
                        // read data : pwm command for motors
                        pwm_L = shift_pwm(Serial.read());
                        pwm_R = shift_pwm(Serial.read());

                        // send data : data of encoders and IMU
                        Serial.println(encoder_count_L);
                        Serial.println(encoder_count_R);
                        Serial.println(quaternions[0]);
                        Serial.println(quaternions[1]);
                        Serial.println(quaternions[2]);
                        Serial.println(quaternions[3]);

                        // if com speed is fast enough
                        //      Serial.println(battery1_voltage);
                        //      Serial.println(battery2_voltage);
                }
        }

        // drive motors based on command
        if (battery2_voltage > 13.5) {// when the battery is enough
                motor_drive(pwm_L, pwm_R);
        }
        else { // if not
                motor_drive(0, 0);
        }

        // voltage alert by leds
        voltage_alert(battery1_voltage, battery2_voltage);
}

// function for timer interrupt : toggle the flag and process will be in the main.
void interrupt() {
        timer_interrupt_flag = true;
}


// shift pwm command range: from 0 - 254 to -254 - 254
int shift_pwm(int pwm_data) {
        pwm_data = (pwm_data - 127) * 2;
        return pwm_data;
}
