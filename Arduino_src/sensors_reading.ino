// Pin per i sensori ultrasonici
#define echoPinRight 4  // filo verde
#define trigPinRight 3  // filo lilla
#define echoPinLeft 8   // filo verde
#define trigPinLeft 7   // filo lilla

void setup() {
  Serial.begin(9600);
  
  // Inizializzazione dei pin per i sensori ultrasonici
  pinMode(echoPinRight, INPUT);
  pinMode(trigPinRight, OUTPUT);
  pinMode(echoPinLeft, INPUT);
  pinMode(trigPinLeft, OUTPUT);
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
  // Lettura e stampa dei valori dei sensori ultrasonici
  Serial.print("#");
  Serial.print(sonarsensor(trigPinRight, echoPinRight));
  Serial.print("#");
  Serial.println(sonarsensor(trigPinLeft, echoPinLeft));
}
