#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
   
Adafruit_BNO055 bno1 = Adafruit_BNO055(55, BNO055_ADDRESS_A); //Wrist Sensor
Adafruit_BNO055 bno2 = Adafruit_BNO055(56, BNO055_ADDRESS_B); //Hand Sensor
char mode = 0;
char t = 0;
unsigned long StartTime;

imu::Vector<3> acc1;
imu::Vector<3> gyro1;
imu::Vector<3> acc2;
imu::Vector<3> gyro2;

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup() 
{
  Serial1.begin(115200);
  Serial.begin(115200);
  
  bno1.begin();
  bno2.begin();
  
  setCal();       // Set Calibration Values for Sensor 1 (Wrist)
  setCal_2();      // Set Calibration Values for Sensor 2 (Hand)
  bno1.setExtCrystalUse(true);  
  bno2.setExtCrystalUse(true);
  StartTime = 1;
  
}

//Hard coding the Calibration Data, do this after you take the Calibration Values, pg 48 of datasheet to learn how to do this
// or just search the Adafruit forums, this is done for both sensors
void setCal()
{
  byte calData;
  bno1.setMode( bno1.OPERATION_MODE_CONFIG );    // Put into CONFIG_Mode
  calData = bno1.setCalvalARL(232);
  calData = bno1.setCalvalARM(3);
  calData = bno1.setCalvalMRL(254);
  calData = bno1.setCalvalMRM(2);
  calData = bno1.setCalvalAOXL(19);
  calData = bno1.setCalvalAOXM(0);
  calData = bno1.setCalvalAOYL(7);
  calData = bno1.setCalvalAOYM(0);
  calData = bno1.setCalvalAOZL(19);
  calData = bno1.setCalvalAOZM(0);
  calData = bno1.setCalvalMOXL(175);
  calData = bno1.setCalvalMOXM(255);
  calData = bno1.setCalvalMOYL(220);
  calData = bno1.setCalvalMOYM(0);
  calData = bno1.setCalvalMOZL(129);
  calData = bno1.setCalvalMOZM(255);
  calData = bno1.setCalvalGOXL(254);
  calData = bno1.setCalvalGOXM(255);
  calData = bno1.setCalvalGOYL(255);
  calData = bno1.setCalvalGOYM(255);
  calData = bno1.setCalvalGOZL(0);
  calData = bno1.setCalvalGOZM(0);
  bno1.setMode( bno1.OPERATION_MODE_NDOF );    // Put into NDOF Mode
}

void setCal_2()
{
  byte calData;
  bno2.setMode( bno2.OPERATION_MODE_CONFIG );    // Put into CONFIG_Mode

  // Addafruit BNO055 datasheet https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf
  // forum https://forums.adafruit.com/viewtopic.php?f=19&t=73014
  
  calData = bno2.setCalvalARL(232);
  // http://mbbsmc.edu.pk/backup.php?older=1532052035 
  calData = bno2.setCalvalARM(3);
  calData = bno2.setCalvalMRL(214);
  calData = bno2.setCalvalMRM(2);
  calData = bno2.setCalvalAOXL(0);
  calData = bno2.setCalvalAOXM(0);
  calData = bno2.setCalvalAOYL(27);
  calData = bno2.setCalvalAOYM(0);
  calData = bno2.setCalvalAOZL(234);
  calData = bno2.setCalvalAOZM(255);
  calData = bno2.setCalvalMOXL(87);
  calData = bno2.setCalvalMOXM(0);
  calData = bno2.setCalvalMOYL(37);
  calData = bno2.setCalvalMOYM(0);
  calData = bno2.setCalvalMOZL(234);
  calData = bno2.setCalvalMOZM(0);
  calData = bno2.setCalvalGOXL(254);
  calData = bno2.setCalvalGOXM(255);
  calData = bno2.setCalvalGOYL(2);
  calData = bno2.setCalvalGOYM(0);
  calData = bno2.setCalvalGOZL(1);
  calData = bno2.setCalvalGOZM(0);
  bno2.setMode( bno2.OPERATION_MODE_NDOF );    // Put into NDOF Mode
}

void serialFlush()
{
  while(Serial1.available() > 0){ t = Serial1.read(); }
  while(Serial.available() > 0) { t = Serial.read();  }
} 

void loop(void)
{
  if(Serial1.available() > 0)
  {
    mode = Serial1.read();
    if(mode == 'R')// all data transmitted
    {
      StartTime = millis();
      while(Serial1.read() != 'D')
      {
        acc1 = bno1.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);// VECTOR_GRAVITY to check gravity
        gyro1 = bno1.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        acc2 = bno2.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
        gyro2 = bno2.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
       Serial1.print((millis() - StartTime)/1000.0, 3);
       Serial1.print(",");
       Serial1.print(acc1.x()); //Print x axis
       Serial1.print(",");
       Serial1.print(acc1.y()); //Print y axis
       Serial1.print(",");
       Serial1.print(acc1.z()); //Print z axis
       Serial1.print(",");
       
       Serial1.print(gyro1.x()); // Print x axis
       Serial1.print(",");
       Serial1.print(gyro1.y()); // Print y axis
       Serial1.print(",");
       Serial1.print(gyro1.z()); // Print z axis
       Serial1.print(",");

       Serial1.print(acc2.x());
       Serial1.print(",");
       Serial1.print(acc2.y());
       Serial1.print(",");
       Serial1.print(acc2.z());
       Serial1.print(",");
      
       Serial1.print(gyro2.x());
       Serial1.print(",");
       Serial1.print(gyro2.y());
       Serial1.print(",");
       Serial1.print(gyro2.z());
       Serial1.print(",");

        // Read EMG Sensors
        Serial1.print("0");
       Serial1.print(",");
       Serial1.print(analogRead(A1));
       Serial1.print(",");
       Serial1.print(analogRead(A2));
       Serial1.print(",");
       Serial1.print(analogRead(A3));
       Serial1.print("\n");
      }
    }
    else if(mode ==  'C')// just used to test connection
    {
      Serial1.print("S");
    }
    else if(mode ==  'Q')
    {
      if(!bno2.begin())
      {
      /* There was a problem detecting the BNO055 ... check your connections */
        Serial1.println("No IMU detected (bno2)");
      }
      if(!bno1.begin())
      {
      /* There was a problem detecting the BNO055 ... check your connections */
        Serial1.println("No IMU detected (bno1)");
      }
    }
  }
  else if(Serial.available() > 0)
  {
    mode = Serial.read();
    if(mode == 'R')// all data transmitted
    {
      StartTime = millis();
      while(Serial.read() != 'D')
      {
        acc1 = bno1.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
        gyro1 = bno1.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        acc2 = bno2.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
        gyro2 = bno2.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
       Serial.print((millis() - StartTime)/1000.0, 3);
       Serial.print(",");
       Serial.print(acc1.x()); //Print x axis
       Serial.print(",");
       Serial.print(acc1.y()); //Print y axis
       Serial.print(",");
       Serial.print(acc1.z()); //Print z axis
       Serial.print(",");
       
       Serial.print(gyro1.x()); // Print x axis
       Serial.print(",");
       Serial.print(gyro1.y()); // Print y axis
       Serial.print(",");
       Serial.print(gyro1.z()); // Print z axis
       Serial.print(",");

       Serial.print(acc2.x());
       Serial.print(",");
       Serial.print(acc2.y());
       Serial.print(",");
       Serial.print(acc2.z());
       Serial.print(",");
      
       Serial.print(gyro2.x());
       Serial.print(",");
       Serial.print(gyro2.y());
       Serial.print(",");
       Serial.print(gyro2.z());
       Serial.print(",");
      

        // Read EMG Sensors
       Serial.print("0");  //disconnected
       Serial.print(",");
       Serial.print(analogRead(A1));  // sensor UNCOVERED
       Serial.print(",");
       Serial.print(analogRead(A2)); //sensor 2
       Serial.print(",");
       Serial.print(analogRead(A3)); // sensor 1
       Serial.print("\n");
       delay(100);
      }
    }
    else if(mode ==  'C')// just used to test connection
    {
      Serial.print("S");
    }
    else if(mode ==  'Q')
    {
      if(!bno2.begin())
      {
      /* There was a problem detecting the BNO055 ... check your connections */
        Serial.println("No IMU detected (bno2)");
      }
      if(!bno1.begin())
      {
      /* There was a problem detecting the BNO055 ... check your connections */
        Serial.println("No IMU detected (bno1)");
      }
    }
  }
  mode = -1;
}
