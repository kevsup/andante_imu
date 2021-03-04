/* This is a hacky solution for reading two MPU9250 on an Adafruit nRF52 Feather.
* Typically on an Arduino Uno, we would just connect both IMU to the SCL/SDA pins, and
* change one of the IMU addresses to 0x69 (pull ADO pin high), but this does not work
* on nRF52 for some reason. Instead, we can set pins 15/16 as another set of I2C pins,
* and thus have a separate line of communication for each IMU 
*
* To use the code, keep the address of BOTH IMU as 0x68 (do not pull ADO pin high),
* then connect one IMU to pin 15 (SDA) and pin 16 (SCL). That should do the trick.
*/

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <bluefruit.h>
#include <vector>

#define SDA0 PIN_WIRE_SDA
#define SCL0 PIN_WIRE_SCL

#define SDA1 15
#define SCL1 16

#define I2C_SPEED_STANDARD 100000

TwoWire i2cWire1 = TwoWire(NRF_TWIM0, NRF_TWIS0, SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn, SDA0, SCL0);
TwoWire i2cWire2 = TwoWire(NRF_TWIM1, NRF_TWIS1, SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn, SDA1, SCL1);

std::vector<TwoWire> wires = {i2cWire1, i2cWire2};

// only print every DELAY_PRINT number of cycles
#define    DELAY_PRINT                300 

// change this depending on number of IMUs
#define    NUM_IMU                    2

#define    MPU9250_ADDRESS            0x68
#define    MPU9250_ADDRESS_2          0x69    // Connect ADO pin to 3.3V

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
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data, uint8_t wireNum)
{
  // Set register address
  wires[wireNum].beginTransmission(Address);
  wires[wireNum].write(Register);
  wires[wireNum].endTransmission();
  
  // Read Nbytes
  wires[wireNum].requestFrom(Address, Nbytes); 
  uint8_t index=0;
  while (wires[wireNum].available())
    Data[index++]=wires[wireNum].read();
}


// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data, uint8_t wireNum)
{
  // Set register address

  wires[wireNum].beginTransmission(Address);
  wires[wireNum].write(Register);
  wires[wireNum].write(Data);

  wires[wireNum].endTransmission();
}

// Initial time
long int ti;
volatile bool intFlag=false;

void setup() {
  // Arduino initializations
    Serial.begin(9600);
    Serial.println("Hello World");

    i2cWire1.begin();
    i2cWire1.setClock(I2C_SPEED_STANDARD);

    i2cWire2.begin();
    i2cWire2.setClock(I2C_SPEED_STANDARD);

    // Set accelerometers low pass filter at 5Hz
    I2CwriteByte(MPU9250_ADDRESS,29,0x06, 0);

    // Set gyroscope low pass filter at 5Hz
    I2CwriteByte(MPU9250_ADDRESS,26,0x06, 0);
    // Configure gyroscope range
    I2CwriteByte(MPU9250_ADDRESS,27,GYRO_RANGE_INIT, 0);
    // Configure accelerometers range
    I2CwriteByte(MPU9250_ADDRESS,28,ACC_RANGE_INIT, 0);
    // Set by pass mode for the magnetometers
    I2CwriteByte(MPU9250_ADDRESS,0x37,0x02, 0);

    if (NUM_IMU == 2) {
      I2CwriteByte(MPU9250_ADDRESS,29,0x06,1);
      I2CwriteByte(MPU9250_ADDRESS,26,0x06,1);
      I2CwriteByte(MPU9250_ADDRESS,27,GYRO_RANGE_INIT,1);
      I2CwriteByte(MPU9250_ADDRESS,28,ACC_RANGE_INIT,1);
      I2CwriteByte(MPU9250_ADDRESS,0x37,0x02,1);
    }

    // Request continuous magnetometer measurements in 16 bits
    //I2CwriteByte(MAG_ADDRESS,0x0A,0x16);

    // Store initial time
    ti = millis();

    tLast = ti;

    Serial.println("MPU9250");
    delay(1000);

  
}

static void populateData(MPU9250 &imu, uint8_t Address, double dt, uint8_t wireNum) {
  // ____________________________________
  // :::  accelerometer and gyroscope :::

  // Read accelerometer and gyroscope
  uint8_t Buf[14];
  I2Cread(Address,0x3B,14,Buf,wireNum);

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

    populateData(imu1, MPU9250_ADDRESS, dt, 0);
    if (NUM_IMU == 2) {
      populateData(imu2, MPU9250_ADDRESS, dt, 1);

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
}