#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// only print every DELAY_PRINT number of cycles
#define    DELAY_PRINT                300 

// change this depending on number of IMUs
#define    NUM_IMU                    2

#define    MPU9250_ADDRESS            0x68
#define    MPU9250_ADDRESS_2          0x69    // for IMU #2. Connect ADO pin to 3.3V

#define    MAG_ADDRESS                0x0C

#define    GYRO_FULL_SCALE_250_DPS    0x00
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

#define    ACC_FULL_SCALE_2_G        0x00
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18

#define    GYRO_RANGE_INIT        GYRO_FULL_SCALE_1000_DPS
#define    ACC_RANGE_INIT         ACC_FULL_SCALE_4_G

// GYRO_RANGE is the number located before "DPS" for the address constant
static const double GYRO_RANGE = 1000;
// ACC_RANGE is the number of g's in the range (e.g. 4*g for ACC_FULL_SCALE_4_G)
static const double ACC_RANGE = 4 * 9.81;
static const int16_t MAX_INT16 = 0x7fff - 1;
static const int INIT_CUTOFF = 5;   // used to initialize gyro theta

struct MPU9250 {
  /* Bias of the gyro. This will depend on your device
  *  To find this, keep your IMU motionless and jot down what the "gx", "gy",
  *  and "gz" variables read. Then multiply that by -1
  */
  int GX_BIAS;
  int GY_BIAS;
  int GZ_BIAS;

  // blending weight for complementary filter
  double alpha;

  // thetas (from gyro integration, accelerometer, and fusion)
  double th_x_gyro;
  double th_y_gyro;
  double th_z_gyro;
  double th_x_acc;
  double th_y_acc;
  double th_z_acc;
  double th_x;
  double th_y;
  double th_z;

  int init_counter; // used to initialize gyro theta
};

static struct MPU9250 imu1 = {
  .GX_BIAS = -27,
  .GY_BIAS = 33,
  .GZ_BIAS = 26,
  .alpha = 0.5,
  .th_x_gyro = 0,
  .th_y_gyro = 0,
  .th_z_gyro = 0,
  .th_x_acc = 0,
  .th_y_acc = 0,
  .th_z_acc = 0,
  .th_x = 0,
  .th_y = 0,
  .th_z = 0,
  .init_counter = 0
};

static struct MPU9250 imu2 = {
  .GX_BIAS = -52,
  .GY_BIAS = -29,
  .GZ_BIAS = -30,
  .alpha = 0.5,
  .th_x_gyro = 0,
  .th_y_gyro = 0,
  .th_z_gyro = 0,
  .th_x_acc = 0,
  .th_y_acc = 0,
  .th_z_acc = 0,
  .th_x = 0,
  .th_y = 0,
  .th_z = 0,
  .init_counter = 0
};

static int printCounter = 0;
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
    I2CwriteByte(MPU9250_ADDRESS,27,GYRO_RANGE_INIT);
    // Configure accelerometers range
    I2CwriteByte(MPU9250_ADDRESS,28,ACC_RANGE_INIT);
    // Set by pass mode for the magnetometers
    I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);

    if (NUM_IMU == 2) {
      I2CwriteByte(MPU9250_ADDRESS_2,29,0x06);
      I2CwriteByte(MPU9250_ADDRESS_2,26,0x06);
      I2CwriteByte(MPU9250_ADDRESS_2,27,GYRO_RANGE_INIT);
      I2CwriteByte(MPU9250_ADDRESS_2,28,ACC_RANGE_INIT);
      I2CwriteByte(MPU9250_ADDRESS_2,0x37,0x02);
    }

    // Request continuous magnetometer measurements in 16 bits
    I2CwriteByte(MAG_ADDRESS,0x0A,0x16);

    // Store initial time
    ti = millis();

    tLast = ti;

    Serial.println("MPU9250");
    delay(1000);
}

static void populateData(MPU9250 &imu, uint8_t Address, double dt) {
  // ____________________________________
  // :::  accelerometer and gyroscope :::

  // Read accelerometer and gyroscope
  uint8_t Buf[14];
  I2Cread(Address,0x3B,14,Buf);

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
  imu.th_x_acc = atan2(ay_metric, az_metric) * RAD_TO_DEG;
  imu.th_y_acc = atan2(az_metric, ax_metric) * RAD_TO_DEG;
  imu.th_z_acc = atan2(ax_metric, ay_metric) * RAD_TO_DEG;

  if (imu.init_counter < INIT_CUTOFF) {
    // use average of first INIT_CUTOFF accelerometer values to calibrate gyro theta
    imu.th_x_gyro += imu.th_x_acc;
    imu.th_y_gyro += imu.th_y_acc;
    imu.th_z_gyro += imu.th_z_acc;
    imu.init_counter++;
  } else if (imu.init_counter == INIT_CUTOFF) {
    imu.th_x_gyro /= INIT_CUTOFF;
    imu.th_y_gyro /= INIT_CUTOFF;
    imu.th_z_gyro /= INIT_CUTOFF;
    imu.init_counter++;
  }

  // Gyroscope
  int16_t gx = -(Buf[8]<<8 | Buf[9]);
  int16_t gy = -(Buf[10]<<8 | Buf[11]);
  int16_t gz = Buf[12]<<8 | Buf[13];

  double gx_metric = (gx + imu.GX_BIAS) * GYRO_RANGE / MAX_INT16;
  double gy_metric = (gy + imu.GY_BIAS) * GYRO_RANGE / MAX_INT16;
  double gz_metric = (gz + imu.GZ_BIAS) * GYRO_RANGE / MAX_INT16; 

  imu.th_x_gyro += gx_metric * dt;
  imu.th_y_gyro += gy_metric * dt;
  imu.th_z_gyro += gz_metric * dt;

  // Sensor fusion (one of these might be yaw, so that value will be garbage)
  imu.th_x = imu.alpha * (imu.th_x + gx_metric * dt) + (1 - imu.alpha) * imu.th_x_acc;
  imu.th_y = imu.alpha * (imu.th_y + gy_metric * dt) + (1 - imu.alpha) * imu.th_y_acc;
  imu.th_z = imu.alpha * (imu.th_z + gy_metric * dt) + (1 - imu.alpha) * imu.th_z_acc;    
}

static bool printData(MPU9250 &imu) {
  if (printCounter > DELAY_PRINT) {
    if (imu.init_counter >= INIT_CUTOFF) {
      Serial.print("th_x_gyro: ");
      Serial.print(imu.th_x_gyro);
      Serial.print ("\t");
      Serial.print("th_y_gyro: ");
      Serial.print(imu.th_y_gyro);
      Serial.print ("\t");
      Serial.print("th_z_gyro: ");
      Serial.print(imu.th_z_gyro);
      Serial.print("\n");
    }

    Serial.print("th_x_acc: ");
    Serial.print(imu.th_x_acc);
    Serial.print ("\t");
    Serial.print("th_y_acc: ");
    Serial.print(imu.th_y_acc);
    Serial.print ("\t");
    Serial.print("th_z_acc: ");
    Serial.print(imu.th_z_acc);
    //Serial.print("yaw");
    Serial.print("\n");

    Serial.print("th_x fusion: ");
    Serial.print(imu.th_x);
    Serial.print("\t");
    Serial.print("th_y fusion: ");
    Serial.print(imu.th_y);
    Serial.print("\t");
    Serial.print("th_z fusion: ");
    Serial.print(imu.th_z);
    //Serial.print("yaw");

    Serial.println("\n");
    return true;
  } 
  return false;
}

static bool printKneeAngle() {
  if (printCounter > DELAY_PRINT) {
    double knee_x = imu2.th_x - imu1.th_x;
    double knee_y = imu2.th_y - imu1.th_y;
    double knee_z = imu2.th_z - imu1.th_z;

    Serial.print("Knee Angle X: ");
    Serial.print(knee_x);
    Serial.print("\t");
    Serial.print("Knee Angle Y: ");
    Serial.print(knee_y);
    Serial.print("\t");
    Serial.print("Knee Angle Z: ");
    Serial.print(knee_z);
    Serial.print("\n");
    return true;
  } 
  return false;
}

void loop() {
    long int tCurr = millis();
    double dt = 1.0 * (tCurr - tLast) / 1000;   // seconds
    tLast = tCurr;

    populateData(imu1, MPU9250_ADDRESS, dt);
    if (NUM_IMU == 2) {
      populateData(imu2, MPU9250_ADDRESS_2, dt);

      // print knee angle
      if (printKneeAngle()) {
        printCounter = 0;
      } else {
        printCounter++;
      }
    } else if (NUM_IMU == 1) {
      // print angle for single IMU
      if (printData(imu1)) {
        printCounter = 0;
      } else {
        printCounter++;
      }
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