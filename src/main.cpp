#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// only print every DELAY_PRINT number of cycles
#define    DELAY_PRINT                1000 

#define    MPU9250_ADDRESS            0x68
#define    MPU9250_ADDRESS_2          0x69    // for IMU #2

#define    MAG_ADDRESS                0x0C

#define    GYRO_FULL_SCALE_250_DPS    0x00
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

#define    ACC_FULL_SCALE_2_G        0x00
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18

#define    GYRO_RANGE_ADDRESS        GYRO_FULL_SCALE_1000_DPS
#define    ACC_RANGE_ADDRESS         ACC_FULL_SCALE_4_G

// GYRO_RANGE is the number located before "DPS" for the address constant
static const double GYRO_RANGE = 1000;
// ACC_RANGE is the number of g's in the range (e.g. 4*g for ACC_FULL_SCALE_4_G)
static const double ACC_RANGE = 4 * 9.81;
static const int16_t MAX_INT16 = 0x7fff - 1;

/* Bias of the gyro. This will depend on your device
*  To find this, keep your IMU motionless and jot down what the "gx", "gy",
*  and "gz" variables read. Then multiply that by -1
*/
static const int GX_BIAS = -52;
static const int GY_BIAS = -29;
static const int GZ_BIAS = -30;

// blending weight for complementary filter
static const double alpha = 0.5;

// global variables
static int printCounter = 0;

// thetas (from gyro integration, accelerometer, and fusion)
static double th_x_gyro = 0;
static double th_y_gyro = 0;
static double th_z_gyro = 0;
static double th_x_acc = 0;
static double th_y_acc = 0;
static double th_z_acc = 0;
static double th_x_acc_init = 0;
static double th_y_acc_init = 0;
static double th_z_acc_init = 0;
static double th_x = 0;
static double th_y = 0;
static double th_z = 0;
static bool init_acc = false;
static long int tLast = 0;

// This function read Nbytes bytes from I2C device at address Address. 
// Put read bytes starting at register Register in the Data array. 
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
  
  // Read Nbytes
  Wire.requestFrom(Address, Nbytes); 
  uint8_t index=0;
  while (Wire.available())
    Data[index++]=Wire.read();
}


// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}

// Initial time
long int ti;
volatile bool intFlag=false;

void setup() {
  // Arduino initializations
    Wire.begin();
    Serial.begin(9600);

    // Set accelerometers low pass filter at 5Hz
    I2CwriteByte(MPU9250_ADDRESS,29,0x06);
    // Set gyroscope low pass filter at 5Hz
    I2CwriteByte(MPU9250_ADDRESS,26,0x06);


    // Configure gyroscope range
    I2CwriteByte(MPU9250_ADDRESS,27,GYRO_RANGE_ADDRESS);
    // Configure accelerometers range
    I2CwriteByte(MPU9250_ADDRESS,28,ACC_RANGE_ADDRESS);
    // Set by pass mode for the magnetometers
    I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);

    // Request continuous magnetometer measurements in 16 bits
    I2CwriteByte(MAG_ADDRESS,0x0A,0x16);

    // Store initial time
    ti = millis();

    tLast = ti;

    Serial.println("MPU9250");
    delay(1000);
}

void loop() {
    long int tCurr = millis();
    double dt = 1.0 * (tCurr - tLast) / 1000;   // seconds
    tLast = tCurr;

    // ____________________________________
    // :::  accelerometer and gyroscope :::

    // Read accelerometer and gyroscope
    uint8_t Buf[14];
    I2Cread(MPU9250_ADDRESS,0x3B,14,Buf);

    // Create 16 bits values from 8 bits data

    // Accelerometer
    int16_t ax = -(Buf[0]<<8 | Buf[1]);
    int16_t ay = -(Buf[2]<<8 | Buf[3]);
    int16_t az = Buf[4]<<8 | Buf[5];

    double ax_metric = ax * ACC_RANGE / MAX_INT16;
    double ay_metric = ay * ACC_RANGE / MAX_INT16;
    double az_metric = az * ACC_RANGE / MAX_INT16;

    /* Note: one of these will be yaw, and that one will be garbage.
    * Accelerometers cannot estimate yaw (yaw is rotation about the unit vector normal to earth's surface)
    */
    th_x_acc = atan2(ay_metric, az_metric) * RAD_TO_DEG;
    th_y_acc = atan2(az_metric, ax_metric) * RAD_TO_DEG;
    th_z_acc = atan2(ax_metric, ay_metric) * RAD_TO_DEG;

    if (!init_acc) {
      th_x_acc_init = th_x_acc;
      th_y_acc_init = th_y_acc;
      th_z_acc_init = th_z_acc;
      init_acc = true;
    }
    th_x_acc -= th_x_acc_init;
    th_y_acc -= th_y_acc_init;
    th_z_acc -= th_z_acc_init;

    // Gyroscope
    int16_t gx = -(Buf[8]<<8 | Buf[9]);
    int16_t gy = -(Buf[10]<<8 | Buf[11]);
    int16_t gz = Buf[12]<<8 | Buf[13];

    double gx_metric = (gx + GX_BIAS) * GYRO_RANGE / MAX_INT16;
    double gy_metric = (gy + GY_BIAS) * GYRO_RANGE / MAX_INT16;
    double gz_metric = (gz + GZ_BIAS) * GYRO_RANGE / MAX_INT16; 

    th_x_gyro += gx_metric * dt;
    th_y_gyro += gy_metric * dt;
    th_z_gyro += gz_metric * dt;

    // Sensor fusion, assuming acceleration vector points in +z (i.e. th_z is yaw)
    th_x = alpha * (th_x + gx_metric * dt) + (1 - alpha) * th_x_acc;
    th_y = alpha * (th_y + gy_metric * dt) + (1 - alpha) * th_y_acc;
    //th_z is yaw, so ignore here
    //th_z = alpha * (th_z + gy_metric * dt) + (1 - alpha) * th_z_acc;    

    // Display values

    if (printCounter > DELAY_PRINT) {
      Serial.print("th_x_gyro: ");
      Serial.print(th_x_gyro);
      Serial.print ("\t");
      Serial.print("th_y_gyro: ");
      Serial.print(th_y_gyro);
      Serial.print ("\t");
      Serial.print("th_z_gyro: ");
      Serial.print(th_z_gyro);
      Serial.print("\n");

      Serial.print("th_x_acc: ");
      Serial.print(th_x_acc);
      Serial.print ("\t");
      Serial.print("th_y_acc: ");
      Serial.print(th_y_acc);
      Serial.print ("\t");
      Serial.print("th_z_acc: ");
      //Serial.print(th_z_acc);
      Serial.print("yaw");
      Serial.print("\n");

      Serial.print("th_x fusion: ");
      Serial.print(th_x);
      Serial.print("\t");
      Serial.print("th_y fusion: ");
      Serial.print(th_y);
      Serial.print("\t");
      Serial.print("th_z fusion: ");
      //Serial.print(th_z);
      Serial.print("yaw");


      Serial.println("\n");
      printCounter = 0;
    } else {
      printCounter++;
    }

    /*
    // Accelerometer
    Serial.print("ax: ");
    Serial.print (ax,DEC);
    Serial.print ("\t");

    Serial.print("ay: ");
    Serial.print (ay,DEC);
    Serial.print ("\t");

    Serial.print("az: ");
    Serial.print (az,DEC);
    Serial.print ("\t");

    // Gyroscope
    Serial.print("gx: ");
    Serial.print (gx,DEC);
    Serial.print ("\t");

    Serial.print("gy: ");
    Serial.print (gy,DEC);
    Serial.print ("\t");

    Serial.print("gz: ");
    Serial.print (gz,DEC);
    Serial.print ("\t");
        // _____________________
    // :::  Magnetometer :::

    // Read register Status 1 and wait for the DRDY: Data Ready
    uint8_t ST1;
    do
    {
        I2Cread(MAG_ADDRESS,0x02,1,&ST1);
    }
    while (!(ST1&0x01));

    // Read magnetometer data
    uint8_t Mag[7];
    I2Cread(MAG_ADDRESS,0x03,7,Mag);

    // Create 16 bits values from 8 bits data

    // Magnetometer
    int16_t mx = -(Mag[3]<<8 | Mag[2]);
    int16_t my = -(Mag[1]<<8 | Mag[0]);
    int16_t mz = -(Mag[5]<<8 | Mag[4]);

    // Magnetometer
    Serial.print(mx+200,DEC);
    Serial.print("\t");

    Serial.print(my-70,DEC);
    Serial.print("\t");

    Serial.print(mz-700,DEC);
    Serial.print("\t");

    // End of line
    Serial.println("");

    */
}