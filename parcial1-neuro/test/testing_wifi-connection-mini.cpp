#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

WiFiUDP udp;
const char* reply = "Robot_Online";

void setup() {
  // Comunicación con tu PC por USB
  Serial.begin(115200); 
  
  // Comunicación con la placa Master (Pines 44 y 43)
  Serial1.begin(115200, SERIAL_8N1, 44, 43); 

  WiFi.softAP("Neuro-UDP-Test", "123456789");
  udp.begin(4210);
}

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    // Si el PC manda un "ping", respondemos para el gráfico de latencia
    udp.beginPacket(udp.remoteIP(), udp.remotePort());
    udp.write((uint8_t*)reply, strlen(reply));
    udp.endPacket();
  }

  // Si la Master manda telemetría por cable, mándala también al USB para debug
  if (Serial1.available()) {
    Serial.write(Serial1.read()); 
  }
}