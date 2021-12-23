#include "I2Cdev.h"
#include "MPU6050.h"
#include <SPI.h>
#include <Adafruit_BMP280.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define OUTPUT_READABLE_ACCELGYRO

#define BMP_SCK  (19)
#define BMP_MISO (17)
#define BMP_MOSI (18)
#define BMP_CS   (13)

MPU6050 accelgyro;
Adafruit_BMP280 bmp;

double angAcc;
double angleFAcc;
double accel_angle_x, accel_angle_y;
double degreeX=0;
double degreeY=0;
double degreeZ=0;


struct BMP280 {
    void init(){
        if (!bmp.begin()) {
            Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                              "try a different address!"));

            while (1) delay(10);
        }

        bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,/* Operating Mode. */
        Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
        Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
        Adafruit_BMP280::FILTER_X16,      /* Filtering. */
        Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
      }

    void update_Temp(){
        Serial.print(F("Temperature = "));
        Serial.print(bmp.readTemperature());
        Serial.print(" *C"); Serial.println("\t");
    }

    void update_Pressure(){
        Serial.print(F("Pressure = "));
        Serial.print(bmp.readPressure());
        Serial.print(" Pa");Serial.print("\t");
    }

    void update_Alt(){
        Serial.print(F("Approx altitude = "));
        Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
        Serial.println(" m");Serial.println("\t");
    }
};


struct IMU {

    MPU6050 accelGyro;

    int16_t gx, gy, gz;
    int16_t ax, ay, az;
    double gxp = gx;
    double gyp = gy;
    double gzp = gz;
    double angRateX, angRateY, angRateZ;
    double degreeX , degreeY, degreeZ;
    unsigned long duration;

    void init() {


  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  Serial.begin(38400);
  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

        accelgyro.setXGyroOffset(-472);
        accelgyro.setYGyroOffset(156);
        accelgyro.setZGyroOffset(-11);
        accelgyro.setXAccelOffset(1042);
        accelgyro.setXAccelOffset(-1347);
        accelgyro.setXAccelOffset(1471);
    }

    void update_gyro() {

        accelgyro.getRotation(&gx, &gy, &gz);

        unsigned long timerBegin= micros();
        gxp = (gx+gxp)/2;
        gyp = (gy+gyp)/2;
        gzp = (gz+gzp)/2;
        angRateX = ((gx)/131.0);
        angRateY = ((gy)/131.0);
        angRateZ = ((gz)/131.0);
        delay(35);
        unsigned long timerEnd= micros();

        duration = timerEnd-timerBegin;

        degreeX += angRateX*duration*0.000001+0.003407;
        degreeY += angRateY*duration*0.000001+0.003307;
        degreeZ += angRateZ*duration*0.000001-0.001607;

        Serial.print(gx); Serial.print("\t");
        Serial.print(gy);Serial.print("\t");
        Serial.print(gz);Serial.print("\t");

        Serial.print(degreeX); Serial.print("\t");
        Serial.print(degreeY);Serial.print("\t");
        Serial.print(degreeZ);Serial.print("\t");
        Serial.print(duration);




    }




    void update_acc() {
      accelgyro.getAcceleration(&ax, &ay, &az);
      double accX = ax/16384.0 - 0.23;
      double accY = ay/16384.0 - 0.09;
      double accZ = az/16384.0 + 0.05;

      angleFAcc = atan(accX/accZ);


      Serial.println(angleFAcc*180/3.1415926);
    }


};

struct IMU imu;
struct BMP280 bmp280;




void setup() {

    imu.init();
    bmp280.init();
    delay(100);

}


void loop() {

  imu.update_gyro();
  //imu.update_acc();
  bmp280.update_Temp();
  bmp280.update_Alt();
  bmp280.update_Pressure();





}
