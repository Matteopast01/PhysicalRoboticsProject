// Pin per i sensori ultrasonici
#define echoPinRight 4  // filo verde
#define trigPinRight 3  // filo lilla
#define echoPinLeft 8   // filo verde
#define trigPinLeft 7   // filo lilla

// Per la bussola
#include <QMC5883LCompass.h>
QMC5883LCompass compass;

void setup() {
  Serial.begin(9600);
  
  // Inizializzazione dei pin per i sensori ultrasonici
  pinMode(echoPinRight, INPUT);
  pinMode(trigPinRight, OUTPUT);
  pinMode(echoPinLeft, INPUT);
  pinMode(trigPinLeft, OUTPUT);

  // Per la bussola
  compass.init();

  compass.setCalibrationOffsets(-197.00, -1316.00, -1411.00);
  compass.setCalibrationScales(1.64, 0.87, 0.81);
}

// Funzione per il sensore ultrasonico
long sonarsensor(int triggerPin, int echoPin) {
  long distance = 0;
  digitalWrite(triggerPin, LOW);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  long times = pulseIn(echoPin, HIGH);
  if (times < 38000) {
    distance = 0.034 * times / 2;
    return distance;
  }
  return distance;
}

void loop() {
  // Legge la bussola
  compass.read();
  float degree = compass.getAzimuth();
  // trasforma i gradi in formato 0°-360°
  if (degree < 0) {
    degree = degree + 360;
    }

  // Lettura e stampa dei valori dei sensori ultrasonici
  Serial.print("#");
  Serial.print(sonarsensor(trigPinRight, echoPinRight));
  Serial.print("#");
  Serial.print(sonarsensor(trigPinLeft, echoPinLeft));
  // Stampa i gradi
  Serial.print("#");
  Serial.println(degree);
}
