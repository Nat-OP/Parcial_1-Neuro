#include <Arduino.h>

void setup() {
  // Serial2 usa el Pin 17 para TX y el Pin 16 para RX en la placa de 38 pines
  Serial2.begin(115200, SERIAL_8N1, 16, 17); 
}

void loop() {
  Serial2.println("HOLA_S3");
  delay(500);
}