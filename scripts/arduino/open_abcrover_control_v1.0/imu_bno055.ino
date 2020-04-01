
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


void get_quaternion_data(float quat_data[4]) {
  // get a new data (quaternion data) from the IMU
  imu::Quaternion quat = bno.getQuat();

  // store into parameters.
  quat_data[0] = quat.w();
  quat_data[1] = quat.x();
  quat_data[2] = quat.y();
  quat_data[3] = quat.z();
}
