// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include <PID_v1.h>
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file
#include "TBMotorController.h"
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#include <CurieBLE.h>
BLEService carService("19b10000-e8f2-537e-4f6c-d104768a1214"); // BLE LED Service
// BLE LED Switch Characteristic - custom 128-bit UUID, read and writable by central
BLEUnsignedCharCharacteristic cmdCharacteristic("19b10000-e8f2-537e-4f6c-d104768a1214", BLERead | BLEWrite);

const int LED_PIN = LED_BUILTIN; // pin to use for the LED

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
//#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define MIN_ABS_SPEED 30
bool blinkState = false;
const double M_PI=3.1415926535897932384626433832795028841950;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===                    Motor & PID setup                     ===
// ================================================================

const int AIN1 = 8;   // 控制輸入A1
const int AIN2 = 7;   // 控制輸入A2
const int BIN1 = 10;  // 控制輸入B1
const int BIN2 = 11;   // 控制輸入B2
const int PWMA = 6;
const int PWMB = 5;
const int STBY = 9;  // 「待機」控制接Arduino的11腳

double motorSpeedFactorLeft = 0.7;
double motorSpeedFactorRight = 0.7;
TBMotorController motorController(PWMA, AIN1, AIN2, PWMB, BIN1, BIN2, motorSpeedFactorLeft, motorSpeedFactorRight);
//PID
double originalSetpoint = 180;
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;
double input, output;

//adjust these values to fit your own design
double Kp = 40;   
double Kd = 0.5;
double Ki = 30;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

int speed;

int cmd = 0;
int offset = 0;
// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void calibrate(){
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
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
}

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
//    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    // begin initialization
    BLE.begin();
    // set advertised local name and service UUID:
    BLE.setLocalName("CarCar");
    BLE.setAdvertisedService(carService);
    // add the characteristic to the service
    carService.addCharacteristic(cmdCharacteristic);
    // add service
    BLE.addService(carService);
    // set the initial value for the characeristic:
    cmdCharacteristic.setValue(0);
    // start advertising
    BLE.advertise();

    Serial.println("BLE LED Peripheral");
    
    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    //Serial Init version
//    while (Serial.available() && Serial.read()); // empty buffer
//    while (!Serial.available());                 // wait for data
//    while (Serial.available() && Serial.read()); // empty buffer again
    //BLE Init version
    while (!cmdCharacteristic.written()); // wait for data
    
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    calibrate();
    
    //setup PID
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255); 
    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    // set STANDBY pin HIGH
    pinMode(STBY, OUTPUT);
    digitalWrite(STBY, HIGH);
    digitalWrite(LED_PIN, HIGH); //Built-IN LED 
}

void PIDtuning(){
  if (Serial.available() > 0) {
    // read the incoming byte:
    char chr;
    int i = 0;
    char data[5]={};
    float paras[3]={};
    int count = 0;
    while ((chr = Serial.read()) != '\n') {
      // 確認輸入的字元介於'0'和'9'
    if (chr >= '0' && chr <= '9') {
      data[i] = chr;
      i++;  
      }
    else if(chr == ','){
      data[i] = '\0';
      paras[count] = atoi(data);
      count++;
      i = 0;
      }
    }
    Serial.print("Kp\t");
    Serial.print(paras[0]);
    Serial.print("Kd\t");
    Serial.print(paras[1]);
    Serial.print("Ki\t");
    Serial.println(paras[2]);
    pid.SetTunings(paras[0], paras[2], paras[1]/10);//void PID::SetTunings(double Kp, double Ki, double Kd)
  }
}

void mpu_Compute(){
  // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    input = ypr[1] * 180/M_PI + 180.5;
    // blink LED to indicate activity
    Serial.print("Input\t");
    Serial.print(input);
    Serial.print(" Setpoint\t");
    Serial.print(setpoint);
    Serial.print(" Error\t");
    Serial.println(setpoint - input);
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
}
// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady){
    motorController.move(0, 0);
    return;
  }
  // listen for BLE peripherals to connect:
  BLEDevice central = BLE.central();
  // if a central is connected to peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());
    // while the central is still connected to peripheral:
    while (central.connected()) {
      // read a packet from FIFO
      if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
        mpu_Compute();
      }
      PIDtuning();
      if(cmdCharacteristic.written()){
        cmd = cmdCharacteristic.value();
        Serial.println(cmd);
        digitalWrite(STBY, HIGH);
      }
      if (cmd == 0) {
//        Serial.println("Stop");
        setpoint = 180 + offset;
        pid.Compute();
        speed = motorController.move(output, MIN_ABS_SPEED);
      } 
      else if(cmd == 1){
//        Serial.println("Forward");
        setpoint = 178.5 + offset;
        pid.Compute();
        speed = motorController.move(output, MIN_ABS_SPEED);
      }
      else if(cmd == 2){
//        Serial.println("Backward");
        setpoint = 181.5 + offset;
        pid.Compute();
        speed = motorController.move(output, MIN_ABS_SPEED);
      }
      else if(cmd == 3){
        pid.Compute();
        motorController.turnRight(output, MIN_ABS_SPEED);
      }
      else if(cmd == 4){
        pid.Compute();
        motorController.turnLeft(output, MIN_ABS_SPEED);
      }
      else if(cmd == 10){
        offset -= 0.5;
        cmd = 0;
      }
      else if(cmd == 11){
        motorController.move(0, MIN_ABS_SPEED);
        digitalWrite(STBY, LOW);
        calibrate();
        cmd = 0;
      }
      else if(cmd == 12){
        offset += 0.5;
        cmd = 0;
      } 
    }
  }
}
