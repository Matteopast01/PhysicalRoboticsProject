#include <QMC5883LCompass.h>
QMC5883LCompass compass;

void setup() {
    Serial.begin(9600);
    compass.init();
}

void loop() {
    compass.read(); // legge i valori della bussola
    float degree = compass.getAzimuth();

    // Per avere i gradi da 0° a 360°
    if (degree < 0) {
      degree = degree + 360;
      }
    // Ritorna i gradi
    Serial.println(degree);
    delay(100);
}