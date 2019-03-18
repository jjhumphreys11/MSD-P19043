#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.
   
   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.

   You should also assign a unique ID to this sensor for use with
   the Adafruit Sensor API so that you can identify this particular
   sensor in any data logs, etc.  To assign a unique ID, simply
   provide an appropriate value in the constructor below (12345
   is used by default in this example).
   
   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3-5V DC
   Connect GROUND to common ground
    
   History
   =======
   2015/MAR/03  - First release (KTOWN)
   2015/Mar/12  - Dave's mod - calibration
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)
   
Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDRESS_A);

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void) 
{
  Serial.begin(9600);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  //setCal();       // Set Calibration Values - Comment out to read calibration values
  delay(1000);
    
  /* Display some basic information on this sensor */
  displaySensorDetails();

  bno.setExtCrystalUse(true);  

}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void) 
{
  /* Get a new sensor event */ 
  sensors_event_t event; 
  bno.getEvent(&event);
  //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  getCalStat();                  // Uncomment to get calibration values

  
  /* Display the floating point data */
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z, 4);
  Serial.println("");
  
  delay(BNO055_SAMPLERATE_DELAY_MS);
  
}
void setCal(){
  // DAVES MOD - Writes calibration data to sensor//
  byte calData;
  bno.setMode( bno.OPERATION_MODE_CONFIG );    // Put into CONFIG_Mode
  delay(25);
  
  calData = bno.setCalvalARL(232);
  
  calData = bno.setCalvalARM(3);
  
  calData = bno.setCalvalMRL(87);
  
  calData = bno.setCalvalMRM(3);
  
  calData = bno.setCalvalAOXL(231);
  
  calData = bno.setCalvalAOXM(255);
  
  calData = bno.setCalvalAOYL(253);
  
  calData = bno.setCalvalAOYM(255);
  
  calData = bno.setCalvalAOZL(5);
 
  calData = bno.setCalvalAOZM(0);
  
  calData = bno.setCalvalMOXL(14);
  
  calData = bno.setCalvalMOXM(1);
  
  calData = bno.setCalvalMOYL(127);
 
  calData = bno.setCalvalMOYM(0);
 
  calData = bno.setCalvalMOZL(173);
  
  calData = bno.setCalvalMOZM(1);
  
  calData = bno.setCalvalGOXL(155);
  
  calData = bno.setCalvalGOXM(255);
  
  calData = bno.setCalvalGOYL(255);
  
  calData = bno.setCalvalGOYM(255);
  
  calData = bno.setCalvalGOZL(254);
  
  calData = bno.setCalvalGOZM(255);
  
  bno.setMode( bno.OPERATION_MODE_NDOF );    // Put into NDOF Mode
  delay(25);
}

void getCal(){
  // Dave's Mod - Reads Calibration Data when sensors are calibrated
  byte calData;
  bno.setMode( bno.OPERATION_MODE_CONFIG );    // Put into CONFIG_Mode
  
  calData = bno.getCalvalARL();
  Serial.println(calData);
  
  calData = bno.getCalvalARM();
  Serial.println(calData);
  
  calData = bno.getCalvalMRL();
  Serial.println(calData);
  
  calData = bno.getCalvalMRM();
  Serial.println(calData);
  
  calData = bno.getCalvalAOXL();
  Serial.println(calData);
  
  calData = bno.getCalvalAOXM();
  Serial.println(calData);
  
  calData = bno.getCalvalAOYL();
  Serial.println(calData);
  
  calData = bno.getCalvalAOYM();
  Serial.println(calData);
  
  calData = bno.getCalvalAOZL();
  Serial.println(calData);
 
  calData = bno.getCalvalAOZM();
  Serial.println(calData);
  
  calData = bno.getCalvalMOXL();
  Serial.println(calData);
  
  calData = bno.getCalvalMOXM();
  Serial.println(calData);
  
  calData = bno.getCalvalMOYL();
  Serial.println(calData);
 
  calData = bno.getCalvalMOYM();
  Serial.println(calData);
 
  calData = bno.getCalvalMOZL();
  Serial.println(calData);
  
  calData = bno.getCalvalMOZM();
  Serial.println(calData);
  
  calData = bno.getCalvalGOXL();
  Serial.println(calData);
  
  calData = bno.getCalvalGOXM();
  Serial.println(calData);
  
  calData = bno.getCalvalGOYL();
  Serial.println(calData);
  
  calData = bno.getCalvalGOYM();
  Serial.println(calData);
  
  calData = bno.getCalvalGOZL();
  Serial.println(calData);
  
  calData = bno.getCalvalGOZM();
  Serial.println(calData);
  
  while(1){                              // Stop
    delay(1000);
  }
  
  
}
void getCalStat(){
  // Dave's Mod - Move sensor to calibrate, when status shows calibration, read values
byte cal = bno.getCalib();
  byte calSys = (0xC0 & cal) >> 6;
  byte calGyro = (0x30 & cal) >> 4;
  byte calAccel = (0x0C & cal) >> 2;
  byte calMag = (0x03 & cal) >> 0;
  
  Serial.println(cal);
  Serial.print("System calibration status "); Serial.println(calSys);
  Serial.print("Gyro   calibration status "); Serial.println(calGyro);
  Serial.print("Accel  calibration status "); Serial.println(calAccel);
  Serial.print("Mag    calibration status "); Serial.println(calMag);
  
  delay(1000);
  if (cal==255){
    getCal();
  }
}
