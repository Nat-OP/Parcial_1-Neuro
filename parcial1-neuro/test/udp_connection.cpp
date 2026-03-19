#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "esp_wifi.h"

const char* ssid = "Neuro-UDP-Test";
const char* password = "123456789";

WiFiUDP udp;
unsigned int localPort = 4210;

void setup() {
  Serial.begin(115200);
  
  // Configuración de WiFi agresiva para evitar desconexiones
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  esp_wifi_set_ps(WIFI_PS_NONE); // Desactivar ahorro energía

  delay(100); // Dar tiempo al stack de red
  
  udp.begin(localPort);
  Serial.println("\n--- Servidor UDP Reiniciado y Blindado ---");
  Serial.print("IP del AP: ");
  Serial.println(WiFi.softAPIP());
}

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    // 1. Leer el mensaje entrante para vaciar el buffer (crucial)
    char incomingPacket[255];
    int len = udp.read(incomingPacket, 255);
    if (len > 0) incomingPacket[len] = 0;

    // 2. Responder directamente a la IP y puerto que nos habló
    udp.beginPacket(udp.remoteIP(), udp.remotePort());
    udp.print("EEG:45.2,EMG:12.8,LAT:MIN"); 
    udp.endPacket();
    
    // Serial para debug: si ves esto en el monitor, el ESP32 sí está trabajando
    Serial.printf("Recibido de %s:%d\n", udp.remoteIP().toString().c_str(), udp.remotePort());
  }
}