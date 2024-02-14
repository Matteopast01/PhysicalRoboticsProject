#include <QMC5883LCompass.h>
QMC5883LCompass compass;

void setup() {
  Serial.begin(9600);
  compass.init();

  compass.setCalibrationOffsets(-197.00, -1316.00, -1411.00);
  compass.setCalibrationScales(1.64, 0.87, 0.81);
}

void loop() {
  compass.read(); // Read compass values
  float a = compass.getAzimuth();

  // for having degree from 0° to 360°
  if (a < 0) {
    a = a + 360;
    }
  // Return Azimuth reading
  Serial.println(a);
  delay(100);
}
