/* * -------------------------------------------------------------------------
 * TRICOPTER FLIGHT CONTROLLER - CASCADE PID
 * -------------------------------------------------------------------------
 * Dibuat oleh   : Furqon Taufiq Hidayat
 * username      : @frustio
 * * Deskripsi Singkat:
 * Kode ini adalah inti algoritma dari Tricopter yang Menggunakan loop kontrol 
 * PID bertingkat (Cascade) untuk kestabilan roll, pitch, dan yaw. 
 * * Komponen Utama:
 * - Sensor   : MPU6050 (SDA: PB9, SCL: PB8), HMC5883L
 * - Remote   : ELRS via CRSF Protocol (TX/RX pada Serial2)
 * - Aktuator : 3 ESC + 2 Servo Mekanisme Yaw
 * * "Fly safe, crash gracefully!" 
 * -------------------------------------------------------------------------
 */

#include <HardwareSerial.h>
#include <Wire.h>

//MPU
#include "MPU6050_6Axis_MotionApps20.h"

//MAGNETO
#include <HMC5883L.h>

//MOTOR
#include "Servo.h"

// elrs
#include "CRSFforArduino.hpp"

//================================================= ELRS ======================================
CRSFforArduino *crsf = nullptr;
HardwareSerial Serial2(PD6,PD5); // define serial pin in this section
int rcChannelCount = crsfProtocol::RC_CHANNEL_COUNT;
void onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcChannels);

//================================================ IMU ==========================================
MPU6050 mpu;
// MPU6050 accelgyro;
int16_t mx, my, mz;
int16_t ax, ay, az;
int16_t gx, gy, gz;
float gyroX_filt, gyroY_filt, gyroZ_filt;
float accelX, accelY, accelZ;

//----------------------Tuning Drone------------------------------------------

float Kp_roll       = 4.25;             float Kp_roll2  = 2.2;
float Ki_roll       = 0.37;             float Ki_roll2  = 0;//0.35`` 
float Kd_roll       = 0;                float Kd_roll2  = 0;//0.9
 
float Kp_pitch      = 3.4;              float Kp_pitch2 = 1.5;
float Ki_pitch      = 0.47;             float Ki_pitch2 = 0;//0.47
float Kd_pitch      = 0;                float Kd_pitch2 = 0;

float Kp_yaw        = 1.5;              float Kp_yaw2   = 0.6;
float Ki_yaw        = 0;                float Ki_yaw2   = 0;
float Kd_yaw        = 0;                float Kd_yaw2   = 0;

//-----------------------------------------------------------------------------

double yaw_deg;
double pitch_deg;
double roll_deg;
float yaw_reference, set_yaw, yawPrev, yaw_head, yaw_control;
int yaw_ref = 0;
#define INTERRUPT_PIN PB12  // use pin 2 on Arduino Uno & most boards
#define LED_PIN PC13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
float dt,error_roll,error_pitch,error_yaw,
      integral_error_roll,integral_error_pitch,integral_error_yaw,
      derivative_error_roll,derivative_error_pitch,derivative_error_yaw,
      last_error_roll,last_error_pitch,last_error_yaw;
float error_roll1,error_pitch1,error_yaw1,
      integral_error_roll1,integral_error_pitch1,integral_error_yaw1,
      derivative_error_roll1,derivative_error_pitch1,derivative_error_yaw1,
      last_error_roll1,last_error_pitch1,last_error_yaw1;
float PID_value_roll,PID_value_pitch,PID_value_yaw,
      PID_value_roll1,PID_value_pitch1,PID_value_yaw1;
float Proll,Ppitch,Pyaw,Iroll,Ipitch,Iyaw,Droll,Dpitch,Dyaw,
      Proll1,Ppitch1,Pyaw1,Iroll1,Ipitch1,Iyaw1,Droll1,Dpitch1,Dyaw1;

int PID_max = 400;

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

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

//================================================ COMPASS ============================================
HMC5883L compass;
float headingDegrees, headingDegreesPrev;
float headingDegrees1, headingDegreesPrev1;
float fixedHeadingDegrees;
float heading, setHeading;
float heading1, setHeading1;
float heading_reference,heading_reference1;
float heading_control,heading_control1;
int head = 0;
float acc_x, acc_y;
float compensateRoll, compensatePitch;
float cosComRoll, sinComRoll, cosComPitch, sinComPitch;
float Yh, Xh, Ymag_correct, Xmag_correct;
float declinationAngle;

//================================================ REMOTE ==========================================
volatile int channel1         = 0;
volatile int roll_channel     = 0;
volatile int channel2         = 0;
volatile int throttle_channel = 0;
volatile int channel3         = 0;
volatile int pitch_channel    = 0;
volatile int channel4         = 0;
volatile int yaw_channel      = 0;
volatile int channel5         = 0;
volatile int ch5_channel      = 0;
volatile int channel6         = 0;
volatile int ch6_channel      = 0;
volatile int channel7         = 0;
volatile int ch7_channel      = 0;
volatile int channel8         = 0;
volatile int ch8_channel      = 0;
volatile int channel9         = 0;
volatile int ch9_channel      = 0;

int ch5, ch6;
int armStatus = 0;
int roll_input, pitch_input, yaw_input, yaw_target;
int roll_input1, pitch_input1, yaw_input1;
char inChar;

//================================================ MOTOR ===========================================
#define MIN_PULSE_LENGTH 1000 // Minimum pulse length in µs
#define MAX_PULSE_LENGTH 2000 // Maximum pulse length in µs

Servo motA, motB, motC, myservoX, myservoY, myservoA, myservoB;
float rollControl, pitchControl, yawControl,yawFiltered;
unsigned long pulse_length_esc1 = 1000,
              pulse_length_esc2 = 1000,
              pulse_length_esc3 = 1000;

//================================================SERVO===========================================
int servoAngleInit1 = 95;
int servoAngleInit2 = 90;

int servoAngleInitA = 100;
int servoAngleInitB = 108;

int servoAngleInitD = 55;

int servo1_up   = 60;
int servo1_down = 120;
int servo2_up   = 65;
int servo2_down = 125;

int servo1_up1   = 60;
int servo1_down1 = 140;
int servo2_up1   = 68;
int servo2_down1 = 148;

int pulse_length_servo1, pulse_length_servo2;

//================================================ TIMER ==============================================
unsigned long currentTime_pid, previousTime_pid;
uint32_t timeProgram, previousTimeProgram;

HardwareSerial Serial1(PA1,PA0);

void setup() {

  Serial1.begin(57600);
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.setSDA(PB9);
  Wire.setSCL(PB8);
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
  #endif
  // Inisialisasi sensor
  init_MPU();
  elrsinit();
  motor_setup();
  compass_init();
  
}

void loop() {

  get_YPR();
  compass_update();
  SerialEvent();
  mapremote();
  update_motor();
  timeProgram = micros();
  if (timeProgram - previousTimeProgram >= 100000)
  {
    print_out();
    previousTimeProgram = micros();
  } 

}
