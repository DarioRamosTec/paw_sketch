#define ledsPin 3
#define ventiladorPin 4

// Arduino Uno
void setup() {
  Serial.begin(9600); // Inicializa el puerto serie para la comunicación con el ESP32
  pinMode(ledsPin, OUTPUT); // Configura el pin 3 como salida para el LED
  pinMode(ventiladorPin, OUTPUT); // Configura el pin 4 como salida para el ventilador
  digitalWrite(ledsPin, HIGH); // Apaga el LED
  digitalWrite(ventiladorPin, HIGH); // Apaga el LED

}

void loop() {
  // Espera un comando del ESP32
  if (Serial.available() > 0) {
    char command = Serial.read();

    // Realiza acciones según el comando recibido
    if (command == 'L') {

        digitalWrite(ledsPin, HIGH); // Enciende el LED
    }
    if (command == 'K') {
        digitalWrite(ledsPin, LOW); // Apaga el LED
    }

    if (command == 'V') {
        digitalWrite(ventiladorPin, HIGH); // Enciende el ventilador
    }
    if (command == 'C') {
        digitalWrite(ventiladorPin, LOW); // Apaga el ventilador
      }



if (command == 'J') {
      if (digitalRead(ledsPin) == HIGH) {
        digitalWrite(ledsPin, LOW); // Apaga el LED
      } else {
        digitalWrite(ledsPin, HIGH); // Enciende el LED
      }
    }

    if (command == 'X') {
      if (digitalRead(ventiladorPin) == HIGH) {
        digitalWrite(ventiladorPin, LOW); // Apaga el ventilador
      } else {
        digitalWrite(ventiladorPin, HIGH); // Enciende el ventilador
      }
    }

    }
  }

