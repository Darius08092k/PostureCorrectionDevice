#include <ArduinoBLE.h>
#include <Wire.h>
#include "FastIMU.h"
#include <SPI.h>
#include <Arduino_LSM6DS3.h>

#define IMU_ADDRESS 0x18 // Address for BMI055
#define Alpha 0.98 // Filter constant (g/10)
#define Delta_time 0.01 // Time difference between readings (in seconds)

// BMI055 IMU Definition
BMI055 IMU_Cap;

// Rela pins for vibrating motors
const int RELAY_PIN_CAP = 10;
const int RELAY_PIN_BACK = 11;

// Data received floats
float xA, yA, zA;
float xM, yM, zM;
float angX, angY, angZ;

calData calib = { 0 };  //Calibration data
AccelData IMUAccel;    //Sensor accelerometer data
GyroData IMUGyro; //Sensor gyroscope data
float roll; // Movement angle on x axis
float pitch; // Movement angle on y axis

float roll_Back; // Movement angle on x axis
float pitch_Back; // Movement angle on y axis
bool flag = false;

long previousMillis = 0;
int interval = 0;
int ledState = LOW;

/* Bluetooth Settings */ 
// Serivce Declaration
BLEService mainService("0b30d63c-775b-4f33-b3f7-12466873d5ff"); // BLE LED Service

// Characteristics
BLEByteCharacteristic lbl_char1("0b30d63c-775b-4f34-b3f7-12466873d5ff", BLERead | BLENotify);
BLEByteCharacteristic  lbl_char2("0b30d63c-775b-4f35-b3f7-12466873d5ff", BLERead | BLENotify);

BLEByteCharacteristic motor_Swtich("0b30d63c-775b-4f36-b3f7-12466873d5ff", BLERead | BLEWrite);
BLEByteCharacteristic motor_Back_Swtich("0b30d63c-775b-4f37-b3f7-12466873d5ff", BLERead | BLEWrite);

struct RawValues
{
  uint8_t  PacketID;

  int16_t AccX;
  int16_t AccY;
  int16_t AccZ;

  int16_t GyroX;
  int16_t GyroY;
  int16_t GyroZ;
};
static RawValues RawData;

void setup()
{
  Wire.begin();        // Initialize I2C communication
  Wire.setClock(400000); //500hz clock

  Serial.begin(9600);  // Initialize the serial monitor

  // Init the cap IMU (BMI055)
  int err = IMU_Cap.init(calib, IMU_ADDRESS);
  if (err != 0)
   {
    while(!Serial)
    {
      ;
    }
    Serial.print("Error initializing IMU! e:");
    Serial.println(err);
    while (true) {
      ;
    }
  }
  RawData.PacketID = 3;

  // Init LSM9DS1 IMU
   if (!IMU.begin()) 
   {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  //Serial.print(IMU.magneticFieldSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in g's");
  Serial.println("X\tY\tZ");
  pinMode(RELAY_PIN_CAP, OUTPUT);
  pinMode(RELAY_PIN_BACK, OUTPUT);
  digitalWrite(RELAY_PIN_BACK, HIGH);
  digitalWrite(RELAY_PIN_CAP, HIGH);

   // initialize the Bluetooth® Low Energy hardware
  BLE.begin();

  Serial.println("Bluetooth® Low Energy Central");

  BLE.setLocalName("Nano 33 IoT");
  BLE.setAdvertisedService(mainService);

  mainService.addCharacteristic(lbl_char1);
  mainService.addCharacteristic(lbl_char2);
  mainService.addCharacteristic(motor_Swtich);
  mainService.addCharacteristic(motor_Back_Swtich);

  BLE.addService(mainService);

  // set the initial value for the characteristic:
  lbl_char1.writeValue(0);
  motor_Swtich.writeValue(0);
  motor_Back_Swtich.writeValue(0);
  //lbl_char2.writeValue(0);

  // start advertising
  BLE.advertise();

  Serial.println("BLE LED Peripheral");
}

void loop() 
{
  static long preMillis = 0;
  BLEDevice central = BLE.central();
  // put your main code here, to run repeatedly:
  if(flag == false)
  {
    calibrateSensor();
  }
  if(flag == true)
  {
       if (central) 
       {
         Serial.print("Connected to central: ");
         // print the central's MAC address:
         Serial.println(central.address());
          //delay(2000);
          // while the central is still connected to peripheral:
         while (central.connected()) 
         {
            
      
          //Serial.println(lbl_char1.value());
            long curMillis = millis();
            if (preMillis>curMillis) preMillis=0; // millis() rollover?
            if (curMillis - preMillis >= 1000) // check values every 10mS
            {
              preMillis = curMillis;
              IMU_Cap.update();
              IMU_Cap.getAccel(&IMUAccel);
              IMU_Cap.getGyro(&IMUGyro);
  
              RawData.AccX = (short)(IMUAccel.accelX * 2048);
              RawData.AccY = (short)(IMUAccel.accelY * 2048);
              RawData.AccZ = (short)(IMUAccel.accelZ * 2048);

              RawData.GyroX = (short)(IMUGyro.gyroX * 16);
              RawData.GyroY = (short)(IMUGyro.gyroY * 16);
              RawData.GyroZ = (short)(IMUGyro.gyroZ * 16);

              computeAngles(RawData.AccX, RawData.AccY, RawData.AccZ, RawData.GyroX, RawData.GyroY, &roll, &pitch);
              Serial.print("Roll Cap = ");
              Serial.println(roll*(180/PI));

              Serial.print("Pitch Cap = ");
              Serial.println(pitch*(180/PI));

              if(((roll*(180/PI))<100))
              {
                lbl_char1.writeValue(2);
                
                if(motor_Swtich.value() == 1)
                {
                  //Serial.println("Success");
                  digitalWrite(RELAY_PIN_CAP, LOW);
                }
                
              }
              else
              {
                lbl_char1.writeValue(0);
                digitalWrite(RELAY_PIN_CAP, HIGH);
               }

              if (IMU.accelerationAvailable()) 
              {
                IMU.readAcceleration(xA, yA, zA);
                pitch_Back = xA*90;
                roll_Back = yA*90;

                Serial.print("Roll Back = ");
                Serial.println(roll_Back);
                Serial.print("Pitch Back= ");
                Serial.println(pitch_Back);

                if((pitch_Back<-8 || pitch_Back >8) || (roll_Back<82))
               {
                lbl_char2.writeValue(1);
                if(motor_Back_Swtich.value() == 1)
                {
                  //Serial.println("Success");
                  digitalWrite(RELAY_PIN_BACK, LOW);
                }
                  //
               }
              else
              {
                   lbl_char2.writeValue(0);
                   Serial.println(0);
                   digitalWrite(RELAY_PIN_BACK, HIGH);
              }
              }
            }
         }
        }
    else
    {
      Serial.println("Central disconnected");
    }
  
  delay(500*2);
  }
}

// Calibrate the cap sensor
void calibrateSensor()
{
  do{
      flag = true;
      calib = { 0 };              
      IMU_Cap.init(calib, IMU_ADDRESS);
      Serial.println("Calibrating IMU... Lay IMU flat side down on a level surface...");
      delay(10000);
      IMU_Cap.calibrateAccelGyro(&calib);
      IMU_Cap.init(calib, IMU_ADDRESS);
      Serial.println("Accelerometer and Gyroscope calibrated!");
      Serial.println("IMU Calibration complete!");
      Serial.println("Accel biases X/Y/Z: ");
      Serial.print(calib.accelBias[0]);
      Serial.print(", ");
      Serial.print(calib.accelBias[1]);
      Serial.print(", ");
      Serial.println(calib.accelBias[2]);
      Serial.println("Gyro biases X/Y/Z: ");
      Serial.print(calib.gyroBias[0]);
      Serial.print(", ");
      Serial.print(calib.gyroBias[1]);
      Serial.print(", ");
      Serial.println(calib.gyroBias[2]);

      delay(5000);
  }while(flag == false);
 
}

void printGyro()
{
  Serial.print("Gyroscope Data: X = ");
  Serial.print(RawData.GyroX);
  Serial.print(", Y = ");
  Serial.print(RawData.GyroY);
  Serial.print(", Z = ");
  Serial.println(RawData.GyroZ);
}

void printAccel()
{
  Serial.print("Accelerometer Data: X = ");
  Serial.print(RawData.AccX);
  Serial.print(", Y = ");
  Serial.print(RawData.AccY);
  Serial.print(", Z = ");
  Serial.println(RawData.AccZ);
}

void computeAngles(int16_t accel_x, int16_t accel_y, int16_t accel_z, int16_t gyro_x, int16_t gyro_y, float* roll, float* pitch)
{
  *roll = atan2(accel_y, accel_z);
  *pitch = atan2(-accel_x, sqrt(accel_y * accel_y + accel_z * accel_z));

  // Integrate gyroscope data to get the change in angles
  float delta_roll = gyro_x * Delta_time; // Transforms the gyro data in angle data
  float delta_pitch = gyro_y * Delta_time; // Transforms the gyro data in angle data

  // Apply complementary filter
  *roll = (1 - Alpha) * (*roll + delta_roll) + Alpha * (*roll);
  *pitch = (1 - Alpha) * (*pitch + delta_pitch) + Alpha * (*pitch);
}
















