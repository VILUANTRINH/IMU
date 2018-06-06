#include <Wire.h>
#include "I2Cdev.h"
#include "MPU9250.h"
MPU9250 accelgyro;
I2Cdev I2C_M;
uint8_t buffer_m[6];
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;
float heading;
float tiltheading;
float Ax,Ay,Az;
float Gx,Gy,Gz;
float Mx,My,Mz;
float gx_bias = 0;
float gy_bias = 0;
float gz_bias = 0;
float Mx_E ;
float My_E ; 
float Mz_E ;
float dt=1;
#define sample_num_mdate 5000
volatile float mx_sample[3];
volatile float my_sample[3];
volatile float mz_sample[3];
static float mx_centre = 0;
static float my_centre = 0;
static float mz_centre = 0;
volatile int mx_max = 0;
volatile int my_max = 0;
volatile int mz_max = 0;
volatile int mx_min = 0;
volatile int my_min = 0;
volatile int mz_min = 0;
float temperature;
float pressure;
float atm;
float altitude;
void setup()
{
  Wire.begin();
  Serial.begin(9600);
  accelgyro.initialize();
  get_gyro_bias();
  delay(1000);

}
void loop()
{
 
getAccel_Data();
getGyro_Data();
getMagnetometer_calibrated();


/*Serial.print (Gx);
Serial.print (" ");
Serial.print (Gy);
Serial.print (" ");
Serial.print (Gz);  
Serial.print (" ");
Serial.print (Ax);
Serial.print (" ");
Serial.print (Ay);
Serial.print (" ");
Serial.print (Az); */

Serial.print (" ");
Serial.print (Mx_E);
Serial.print (" ");
Serial.print (My_E);
Serial.print (" ");
Serial.println (Mz_E); 

delay(0.1);
}

void get_gyro_bias()
{
  float gx_temp = 0;
  float gy_temp = 0;
  float gz_temp = 0;
  
  for(int i = 0; i<100; i++)
  {
    getGyro_Data();
    gx_temp += Gx;
    gy_temp+=Gy;
    gz_temp+=Gz;    
  }
  gx_bias = gx_temp / 100;
  gy_bias = gy_temp / 100;
  gz_bias = gz_temp / 100;
//  Serial.print (gx_bias);
//Serial.print (" ");
//Serial.print (gy_bias);
//Serial.print (" ");
//Serial.println (gz_bias); 
}


void get_calibration_Data ()
{
for (int i = 0; i < sample_num_mdate; i++)
{
get_one_sample_date_mxyz();

if (mx_sample[2] >= mx_sample[1])mx_sample[1] = mx_sample[2];
if (my_sample[2] >= my_sample[1])my_sample[1] = my_sample[2]; //find max value
if (mz_sample[2] >= mz_sample[1])mz_sample[1] = mz_sample[2];
if (mx_sample[2] <= mx_sample[0])mx_sample[0] = mx_sample[2];
if (my_sample[2] <= my_sample[0])my_sample[0] = my_sample[2]; //find min value
if (mz_sample[2] <= mz_sample[0])mz_sample[0] = mz_sample[2];
}
mx_max = mx_sample[1];
my_max = my_sample[1];
mz_max = mz_sample[1];
mx_min = mx_sample[0];
my_min = my_sample[0];
mz_min = mz_sample[0];
mx_centre = (mx_max + mx_min) / 2;
my_centre = (my_max + my_min) / 2;
mz_centre = (mz_max + mz_min) / 2;
}
void get_one_sample_date_mxyz()
{
getMagnetometer_Data();
mx_sample[2] = Mx;
my_sample[2] = My;
mz_sample[2] = Mz;
}
void getAccel_Data(void)
{
accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
Ax = (double) ax / 16384;
Ay = (double) ay / 16384;
Az= (double) az / 16384;
}
void getGyro_Data(void)
{
accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
Gx = ((double) gx * 250 / 32768) - gx_bias;
Gy = ((double) gy * 250 / 32768) - gy_bias;
Gz = ((double) gz * 250 / 32768) - gz_bias;
}
void getMagnetometer_Data(void)
{
I2C_M.writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x01); 
delay(10);
I2C_M.readBytes(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_XOUT_L, 6, buffer_m);
mx = ((int16_t)(buffer_m[1]) << 8) | buffer_m[0] ;
my = ((int16_t)(buffer_m[3]) << 8) | buffer_m[2] ;
mz = ((int16_t)(buffer_m[5]) << 8) | buffer_m[4] ;
Mx = (double) mx * 1200 / 4096;
My = (double) my * 1200 / 4096;
Mz = (double) mz * 1200 / 4096;
}
void getMagnetometer_calibrated ()
{
  float Mx_temp= 0;
  float My_temp = 0;
  float Mz_temp = 0;
  for ( int i=0 ;i<10000 ; i++)
  { 
  
getMagnetometer_Data();
Mx_temp += Mx;
My_temp += My;
Mz_temp +=Mz;
}
Mx_E = Mx_temp/10000 ;
My_E = My_temp/10000 ;
Mz_E = Mz_temp/10000 ;
}


