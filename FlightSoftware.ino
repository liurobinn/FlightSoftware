#include "I2Cdev.h"
#include <Servo.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <Adafruit_BMP280.h>
#include "Wire.h"


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif


MPU6050 mpu;
Adafruit_BMP280 bmp;

Servo X08_X;
Servo X08_Y;

#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 2
#define LED_PIN 13
bool blinkState = false;

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

// orientation/motion vars
Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;
float euler[3];
float ypr[3];


uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
volatile bool mpuInterrupt = false;
void dmpDataReady() {
        mpuInterrupt = true;
}


struct IMU {


        void init(){
                // join I2C bus (I2Cdev library doesn't do this automatically)
          #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
                Wire.begin();
                Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
          #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
                Fastwire::setup(400, true);
          #endif

                Serial.begin(115200);
                while (!Serial);


                mpu.initialize();
                pinMode(INTERRUPT_PIN, INPUT);

                Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));



                Serial.println(F("Initializing DMP..."));
                devStatus = mpu.dmpInitialize();


                mpu.setXGyroOffset(220);
                mpu.setYGyroOffset(76);
                mpu.setZGyroOffset(-85);
                mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

                // make sure it worked (returns 0 if so)
                if (devStatus == 0) {
                        // Calibration Time: generate offsets and calibrate our MPU6050
                        mpu.CalibrateAccel(6);
                        mpu.CalibrateGyro(6);
                        mpu.PrintActiveOffsets();
                        // turn on the DMP, now that it's ready
                        Serial.println(F("Enabling DMP..."));
                        mpu.setDMPEnabled(true);

                        // enable Arduino interrupt detection
                        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
                        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
                        Serial.println(F(")..."));
                        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
                        mpuIntStatus = mpu.getIntStatus();

                        // set our DMP Ready flag so the main loop() function knows it's okay to use it
                        Serial.println(F("DMP ready! Waiting for first interrupt..."));
                        dmpReady = true;

                        // get expected DMP packet size for later comparison
                        packetSize = mpu.dmpGetFIFOPacketSize();
                } else {
                        // ERROR!
                        // 1 = initial memory load failed
                        // 2 = DMP configuration updates failed
                        // (if it's going to break, usually the code will be 1)
                        Serial.print(F("DMP Initialization failed (code "));
                        Serial.print(devStatus);
                        Serial.println(F(")"));
                }

                // configure LED for output
                pinMode(LED_PIN, OUTPUT);
        }

        void update(){


                if (!dmpReady) return;

                // wait for MPU interrupt or extra packet(s) available
                while (!mpuInterrupt && fifoCount < packetSize) {
                        if (mpuInterrupt && fifoCount < packetSize) {
                                // try to get out of the infinite loop
                                fifoCount = mpu.getFIFOCount();
                        }

                }


                mpuInterrupt = false;
                mpuIntStatus = mpu.getIntStatus();

                fifoCount = mpu.getFIFOCount();
                if(fifoCount < packetSize) {

                }
                // check for overflow (this should never happen unless our code is too inefficient)
                else if ((mpuIntStatus & (0x01 << MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
                        // reset so we can continue cleanly
                        mpu.resetFIFO();
                        //  fifoCount = mpu.getFIFOCount();  // will be zero after reset no need to ask
                        Serial.println(F("FIFO overflow!"));

                        // otherwise, check for DMP data ready interrupt (this should happen frequently)
                } else if (mpuIntStatus & (0x01 << MPU6050_INTERRUPT_DMP_INT_BIT)) {

                        // read a packet from FIFO
                        while(fifoCount >= packetSize) { // Lets catch up to NOW, someone is using the dreaded delay()!
                                mpu.getFIFOBytes(fifoBuffer, packetSize);
                                // track FIFO count here in case there is > 1 packet available
                                // (this lets us immediately read more without waiting for an interrupt)
                                fifoCount -= packetSize;
                        }



        #ifdef OUTPUT_READABLE_YAWPITCHROLL
                        // display Euler angles in degrees
                        mpu.dmpGetQuaternion(&q, fifoBuffer);
                        mpu.dmpGetGravity(&gravity, &q);
                        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
                        Serial.print("ypr\t");
                        Serial.print(ypr[0] * 180/M_PI);
                        Serial.print("\t");

                        double val;
                        double prev;

                        prev = val;
                        val = ypr[1] * 180/M_PI;

                        if(val > prev){
                          Serial.print(90+abs(abs(abs(ypr[1] * 180/M_PI)-90)-90));
                        }
                        if(val < prev){
                          Serial.print(90-abs(abs(abs(ypr[1] * 180/M_PI)-90)-90));
                        }
                        Serial.print("\t");
                        Serial.println(abs(ypr[2] * 180/M_PI));
        #endif
/*
        if(val > prev && 90+abs(abs(abs(ypr[1] * 180/M_PI)-90)-90)>80 && 90+abs(abs(abs(ypr[1] * 180/M_PI)-90)-90)< 90){
          X08_X.write(90+abs(abs(abs(ypr[1] * 180/M_PI)-90)-90));
        }
        if(val < prev && 90+abs(abs(abs(ypr[1] * 180/M_PI)-90)-90)>80 && 90+abs(abs(abs(ypr[1] * 180/M_PI)-90)-90)< 90){
          X08_X.write(90-abs(abs(abs(ypr[1] * 180/M_PI)-90)-90));
        }

*/
                        blinkState = !blinkState;
                        digitalWrite(LED_PIN, blinkState);
                }
        }


};

struct TVC {

        double pos;

        void servo_init(){

                X08_X.attach(3);
                X08_Y.attach(4);

                X08_X.write(83);
                X08_Y.write(92);

        }

        void X80_testX(){

                pos= 83;

                for (pos = 83; pos >= 68; pos -= 1) {

                        X08_X.write(pos);
                        delay(15);
                }
                delay(15);

                for (pos = 68; pos <= 83; pos += 1) {

                        X08_X.write(pos);
                        delay(15);
                }
                delay(15);

                for (pos = 83; pos <= 105; pos += 1) {
                        X08_X.write(pos);
                        delay(15);
                }
                delay(15);

                for (pos = 105; pos >= 83; pos -= 1) {
                        X08_X.write(pos);
                        delay(15);
                }
                delay(15);

        }

        void X80_testY(){

                pos= 92;

                for (pos = 92; pos >= 75; pos -= 1) {

                        X08_Y.write(pos);
                        delay(15);
                }
                delay(15);

                for (pos = 75; pos <= 92; pos += 1) {

                        X08_Y.write(pos);
                        delay(15);
                }
                delay(15);

                for (pos = 92; pos <= 105; pos += 1) {
                        X08_Y.write(pos);
                        delay(15);
                }
                delay(15);

                for (pos = 105; pos >= 92; pos -= 1) {
                        X08_Y.write(pos);
                        delay(15);
                }
                delay(15);

        }

};

struct BMP280 {
        void init(){

                if (!bmp.begin()) {
                        Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                                         "try a different address!"));

                        while (1) delay(10);
                }

                bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,/* Operating Mode. */
                                Adafruit_BMP280::SAMPLING_X2, /* Temp. oversampling */
                                Adafruit_BMP280::SAMPLING_X16, /* Pressure oversampling */
                                Adafruit_BMP280::FILTER_X16, /* Filtering. */
                                Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
        }

        void update_Temp(){
                Serial.print(F("Temp:"));
                Serial.print(bmp.readTemperature());
                Serial.print(" *C"); Serial.println("\t");
        }

        void update_Pressure(){
                Serial.print(F("Pressure:"));
                Serial.print(bmp.readPressure());
                Serial.print(" Pa"); Serial.print("\t");
        }

        void update_Alt(){
                Serial.print(F("Alt:"));
                Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
                Serial.print(" m"); Serial.println("\t");
        }
};

struct BMP280 bmp280;
struct TVC tvc;
struct IMU imu;

void setup(){
        Serial.begin(9600);
        imu.init();

        Wire.begin();
        bmp280.init();
        tvc.servo_init();
        tvc.X80_testX();
        tvc.X80_testY();
}
void loop() {
        imu.update();
        //bmp280.update_Temp();
        //bmp280.update_Alt();
        //bmp280.update_Pressure();
        //tvc.X80_testX();
        //tvc.X80_testY();



        if (abs(ypr[2] * 180/M_PI) >= 80 && abs(ypr[2] * 180/M_PI) <= 100 ) {

                X08_Y.write(abs(ypr[2] * 180/M_PI));
        }


}
