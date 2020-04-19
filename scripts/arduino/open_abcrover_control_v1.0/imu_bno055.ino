
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

double raw_qw, raw_qx, raw_qy, raw_qz; //parameters for raw quaternion data
double euler_x, euler_y, euler_z;      //parameters for euler angle

//coefficient to get the raw data
#define LSB 16384 //bno055 deals quaternion data at 2^14(=16384)LSB, that means raw data "1" indicates 1/LSB in actual(physical) data.

void get_quaternion_data(float quat_data[4])
{
  // get a new data (quaternion data) from the IMU
  imu::Quaternion quat = bno.getQuat();

  // store into parameters.
  quat_data[0] = quat.x();
  quat_data[1] = quat.y();
  quat_data[2] = quat.z();
  quat_data[3] = quat.w();
}

void get_accelerometer_data(float acc_data[3])
{
  // get a new data from the IMU
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  //store into parameters
  acc_data[0] = accel.x();
  acc_data[1] = accel.y();
  acc_data[2] = accel.z();
}

void get_gyro_data(float gyro_data[3])
{
  // get a new data from the IMU
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  //store into parameters
  gyro_data[0] = gyro.x();
  gyro_data[1] = gyro.y();
  gyro_data[2] = gyro.z();
}