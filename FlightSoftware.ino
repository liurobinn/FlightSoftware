#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif


MPU6050 accelgyro;

double angAcc;
double angleFAcc;
double accel_angle_x, accel_angle_y;
double degreeX=0;
double degreeY=0;
double degreeZ=0;



#define OUTPUT_READABLE_ACCELGYRO

struct Gyro {

    MPU6050 accelGyro;

    int16_t gx, gy, gz;
    int16_t ax, ay, az;

    double angRateX, angRateY, angRateZ;
    double degreeX , degreeY, degreeZ;
    double duration;

    void init() {
        #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
            Wire.begin();
        #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
            Fastwire::setup(400, true);
        #endif

        accelgyro.setXGyroOffset(-468);
        accelgyro.setYGyroOffset(161);
        accelgyro.setZGyroOffset(-11);
        accelgyro.setXAccelOffset(1136);
        accelgyro.setXAccelOffset(-1368);
        accelgyro.setXAccelOffset(1470);
    }

    void update_gyro() {

        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        unsigned long timeBegin = micros();
        angRateX = (gx/131.0) + 0.3126;
        angRateY = (gy/131.0) - 0.0334;
        angRateZ = (gz/131.0) - 0.02065;
        unsigned long duration = micros() - timeBegin;
        degreeX += angRateX * duration* 0.000001;
        degreeY += angRateY * duration* 0.000001;
        degreeZ += angRateZ * duration* 0.000001;

        Serial.print(degreeX,"\t");
        Serial.print(degreeY,"\t");
        Serial.println(degreeZ,"\t");
    }

    void update_acc() {
      accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      double accX = ax/16384.0 - 0.23;
      double accY = ay/16384.0 - 0.09;
      double accZ = az/16384.0 + 0.05;

      angleFAcc = atan(accX/accZ);

      Serial.print(degreeX);Serial.print("\t");
      Serial.print(degreeY);Serial.print("\t");
      Serial.print(degreeZ);Serial.print("\t");

      Serial.println(angleFAcc*180/3.1415926);
    }


};

struct Gyro gyro;

void setup() {

    // initialize serial communication
    gyro.init();
    delay(100);

}

void initGyro() {

  delay(100);
  // join I2C bus (I2Cdev library doesn't do this automatically)


  Serial.begin(38400);

  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  // use the code below to change accel/gyro offset values

  Serial.println("Updating internal sensor offsets...");
  //-76  -2359 1688  0 0 0



}

void loop() {

  gyro.update_gyro();
  gyro.update_acc();




}
