#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#define PIN_RGB 48  // Pin del LED RGB en la S3
Adafruit_NeoPixel pixel(1, PIN_RGB, NEO_GRB + NEO_KHZ800);

void setup() {
  Serial.begin(115200);
  // Cambiamos a pines 1 y 2 para evitar conflictos con el USB
  Serial1.begin(115200, SERIAL_8N1, 1, 2); 
  
  pixel.begin();
  pixel.setBrightness(20);
}

void loop() {
  if (Serial1.available()) {
    // Si hay datos de la Master -> VERDE
    pixel.setPixelColor(0, pixel.Color(0, 255, 0));
    pixel.show();
    while(Serial1.available()) Serial1.read(); // Limpiar buffer
    delay(100); 
  } else {
    // Si NO hay datos -> ROJO
    pixel.setPixelColor(0, pixel.Color(255, 0, 0));
    pixel.show();
  }
}