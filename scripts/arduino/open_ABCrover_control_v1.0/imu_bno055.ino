
/* This driver reads raw data from the BNO055

   Hardware Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/


double raw_qw, raw_qx, raw_qy, raw_qz;//parameters for raw quaternion data
double euler_x, euler_y, euler_z; //parameters for euler angle

//coefficient to get the raw data
#define LSB 16384 //bno055 deals quaternion data at 2^14(=16384)LSB, that means raw data "1" indicates 1/LSB in actual(physical) data.


void get_IMUdata() {
  // get a new data (quaternion data) from the IMU
  imu::Quaternion quat = bno.getQuat();

  // store into parameters.
  quat_w = quat.w();
  quat_x = quat.x();
  quat_y = quat.y();
  quat_z = quat.z();

  //   print the data
  //  Serial.print("qW: ");
  //  Serial.print(quat.w(), 3);
  //  Serial.print(" qX: ");
  //  Serial.print(quat.x(), 3);
  //  Serial.print(" qY: ");
  //  Serial.print(quat.y(), 3);
  //  Serial.print(" qZ: ");
  //  Serial.print(quat.z(), 3);
  //  Serial.print("\t\t");
}
