#include <TimerOne.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "DualVNH5019MotorShield.h"
#include <Adafruit_NeoPixel.h>

// constants for an IMU
#define GRAVITATIONAL_ACCEL 9.798 //at TOKYO
#define IMU_SAMPLERATE_HZ 100     //refresh rate of the imu (looks like it should be fixed at 100Hz due to bno055 hardware...?)

// constants for pin assign
#define ENC_LA 2            // input from encoder, left motor, phaseA
#define ENC_LB 3            // left motor phaseB
#define ENC_RA 18           // right motor, phaseA
#define ENC_RB 19           // right motor, phaseB
#define VOLTAGE_MONITOR1 A2 // voltage monitor
#define VOLTAGE_MONITOR2 A3 // voltage monitor
#define ALERT_LED 52        // a pin the series of full-color LEDs are connected
#define NUMPIXELS 2         // the number of full-color LEDs

// constants for motors
#define PULSE_PER_ROUND 723.24 // encoder pulse resolution * gear ratio
#define MAXIMUM_OUTPUT 300     // maximum pwm output for motor

// flag to control whether timer interruption is ignited or not
volatile bool timer_interrupt_flag = false;
volatile bool imu_refresh_enable = true;

//paramters to count time
int time1, time2, time3, past_time;

//parameters for motors/encoders
volatile byte pulse_L, last_pulse_L, pulse_R, last_pulse_R;
volatile long encoder_count_L = 0, encoder_count_R = 0;

// prepare parameters for motor output before receiving commands
int operation_mode = 0; // 0:disabiled, 1:teleop, 2:teleop_turbo, 3:autonomous
int command_L = 0, command_R = 0;

//parameters to store IMU data
float quaternions[4] = {0.0, 0.0, 0.0, 0.0}; //x,y,z,w
float accelerometers[3] = {0.0, 0.0, 0.0};   //x,y,z
float gyros[3] = {0.0, 0.0, 0.0};            //roll,pitch,yaw

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

// pin remap of the motor sheild
//                       1a,1b,1pw,1e,1cs,2a,2b,2pw,2e,2cs
DualVNH5019MotorShield md(13, 4, 9, 6, A0, 7, 8, 10, 12, A1);

// initialize full color LED
Adafruit_NeoPixel pixels(NUMPIXELS, ALERT_LED, NEO_GRB + NEO_KHZ800);

void setup()
{
        // timer interrupt initialization
        Timer1.initialize(1000000 / IMU_SAMPLERATE_HZ);
        Timer1.attachInterrupt(interrupt);

        // serial communication initialization
        Serial.begin(230400);

        /* Initialise the bno055_imu */
        if (!bno.begin())
        { /* There was a problem detecting the BNO055 ... check your connections */
                Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
                while (1)
                        ;
        }
        delay(100);
        bno.setExtCrystalUse(true);

        // initialize the motor driver shield
        md.init();

        // change bitrate of I2C comm to 400kbps (100kbps, usual)
        Wire.setClock(400000);

        // change pwm frequency
        TCCR2B = (TCCR2B & 0b11111000) | 0x01; //31.37255 [kHz]
        // TCCR2B = (TCCR2B & 0b11111000) | 0x02; //3.92116 [kHz]
        // TCCR2B = (TCCR2B & 0b11111000) | 0x03; //980.39 [Hz]
        // TCCR2B = (TCCR2B & 0b11111000) | 0x04; //490.20 [Hz]
        // TCCR2B = (TCCR2B & 0b11111000) | 0x05; //245.10 [Hz]
        // TCCR2B = (TCCR2B & 0b11111000) | 0x06; //122.55 [Hz]
        // TCCR2B = (TCCR2B & 0b11111000) | 0x07; //30.64 [Hz]

        pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)

        // setup arduino GPIO
        pinMode(ENC_LA, INPUT_PULLUP);
        pinMode(ENC_LB, INPUT_PULLUP);
        pinMode(ENC_RA, INPUT_PULLUP);
        pinMode(ENC_RB, INPUT_PULLUP);

        // pinMode(LED_BUILTIN, OUTPUT);

        // setup hardware interrupt
        attachInterrupt(0, encoder_read, CHANGE); // pin2 to left encoder phase A
        attachInterrupt(1, encoder_read, CHANGE); // pin3 to left encoder phase B
        attachInterrupt(5, encoder_read, CHANGE); // pin18 to right encoder phase A
        attachInterrupt(4, encoder_read, CHANGE); // pin19 to right encoder phase B
}

void loop()
{
        if (timer_interrupt_flag == true)
        { // get imu data at timer interruption
                get_quaternion_data(quaternions);
                get_accelerometer_data(accelerometers);
                get_gyro_data(gyros);
                timer_interrupt_flag = false; // toggle the flag again
        }

        // check the battery voltage (divided by 4, from 14.8V to 3.7V at nominal voltage)
        float battery_voltage[2] = {0.0, 0.0};
        battery_voltage[0] = analogRead(VOLTAGE_MONITOR1) * 4.0 * 5.0 / 1023.0;
        battery_voltage[1] = -0.664 + float(analogRead(VOLTAGE_MONITOR2)) * 4.0 * 5.0 / 1023.0;

        // communicate with host PC
        if (Serial.available() > 1)
        { //6 bytes should be received, but arduino hangs up when you wait all 6 bytes are available
                if (Serial.read() == 'H')
                { //only when received data starts from 'H'
                        delayMicroseconds(200);
                        // read data : operation mode
                        operation_mode = Serial.read();

                        // read data : pwm command for motors and shift it
                        int offset = 300; // the value depends how much did you offset befor sending the commands
                        command_L = receive_data() - offset;
                        command_R = receive_data() - offset;

                        // drive motors
                        if (operation_mode == 0)
                        { // if the command is disabled
                                // motor_drive(0, 0);
                                command_L = 0;
                                command_R = 0;
                        }
                        else if (battery_voltage[1] < 13.5)
                        { //if the battery is running out
                                // motor_drive(0, 0);
                                command_L = 0;
                                command_R = 0;
                        }
                        motor_drive(command_L, command_R);

                        // send data : data of encoders and IMU
                        Serial.println(encoder_count_L);
                        Serial.println(encoder_count_R);
                        Serial.println(quaternions[0]);
                        Serial.println(quaternions[1]);
                        Serial.println(quaternions[2]);
                        Serial.println(quaternions[3]);
                        Serial.println(accelerometers[0]);
                        Serial.println(accelerometers[1]);
                        Serial.println(accelerometers[2]);
                        Serial.println(gyros[0]);
                        Serial.println(gyros[1]);
                        Serial.println(gyros[2]);
                        // Serial.println(Serial.available());

                        // if com speed is fast enough
                        Serial.println(battery_voltage[0]);
                        Serial.println(battery_voltage[1]);

                        // to check the commands sent to motor driver
                        // Serial.println(command_L);
                        // Serial.println(command_R);

                        //flush buffer and wait to complete sending
                        // Serial.flush();
                        // while(Serial.available()) {
                        //         Serial.read();
                        // }
                }
        }

        // rover state indicator
        rover_state_indicator(operation_mode, battery_voltage);
}

// function for timer interrupt : toggle the flag and process will be in the main.
void interrupt()
{
        timer_interrupt_flag = true;
}
