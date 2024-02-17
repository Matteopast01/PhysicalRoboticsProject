#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>

/* Assign a unique ID to the sensors */
Adafruit_9DOF                dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);

/* Update this with the correct SLP for accurate altitude measurements */
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

/**************************************************************************/
/*!
    @brief  Initialises all the sensors used by this example
*/
/**************************************************************************/
void initImuSensors()
{
  if(!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
}

/**************************************************************************/



// Pin per i sensori ultrasonici
#define echoPinRight 4  // filo verde
#define trigPinRight 3  // filo lilla
#define echoPinLeft 8   // filo verde
#define trigPinLeft 7   // filo lilla




/**************************************************************************/
void setup(void)
{
  Serial.begin(9600);

  pinMode(echoPinRight, INPUT);
  pinMode(trigPinRight, OUTPUT);
  pinMode(echoPinLeft, INPUT);
  pinMode(trigPinLeft, OUTPUT);


  
  /* Initialise the sensors */
  initImuSensors();


}



/**************************************************************************/
/*!
    @brief  Constantly check the roll/pitch/heading/altitude/temperature
*/
/**************************************************************************/
void loop(void)
{
  
  Serial.print("#");
  Serial.print(sonarsensor(trigPinRight, echoPinRight));
  Serial.print("#");
  Serial.print(sonarsensor(trigPinLeft, echoPinLeft));
  // Stampa i gradi
  Serial.print("#");
  Serial.println(imu_orientation_reading());

  
}