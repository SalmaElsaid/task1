#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>
MPU6050 imu;

#define OUTPUT_READABLE_YAWPITCHROLL
int16_t Ax,Ay,Az,Gx,Gy,Gz;
uint8_t devStatus; 
uint16_t packetSize;
uint8_t fifoBuffer[64];
uint16_t fifoCount;
Quaternion q;           
VectorInt16 aa;       
VectorInt16 gy; 
float ypr[3];
VectorFloat gravity;
HardwareSerial Serial3(PB11, PB10);


void setup() {
  Wire.begin();
Wire.setClock(400000);
Serial.begin(9600);
while (!Serial); 
Serial3.begin(115200);
while (!Serial3);
imu.initialize(); 
 Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial3.read()); 
  while (!Serial.available());               
  while (Serial.available() && Serial3.read());
   Serial3.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial3.available() && Serial3.read()); 
  while (!Serial3.available());                
  while (Serial3.available() && Serial3.read()); 
 devStatus = imu.dmpInitialize();
 imu.setXGyroOffset(51);
  imu.setYGyroOffset(8);
  imu.setZGyroOffset(21);
  imu.setXAccelOffset(1150);
  imu.setYAccelOffset(-50);
  imu.setZAccelOffset(1060);

if (devStatus == 0) {
    
    imu.CalibrateAccel(6);
    imu.CalibrateGyro(6);
    imu.setDMPEnabled(true);
 packetSize = imu.dmpGetFIFOPacketSize();
}
}

void loop() {
 imu.dmpGetQuaternion(&q, fifoBuffer);
    imu.dmpGetGravity(&gravity, &q);
    imu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[2] * 180 / M_PI);
    Serial3.print("ypr\t");
    Serial3.print(ypr[0] * 180 / M_PI);
    Serial3.print("\t");
    Serial3.print(ypr[1] * 180 / M_PI);
    Serial3.print("\t");
    Serial3.print(ypr[2] * 180 / M_PI);

}
