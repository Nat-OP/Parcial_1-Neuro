#include <Arduino.h>
#include "WiFi.h"
#include "ESPAsyncWebServer.h"

// Credenciales del Access Point
const char* ssid = "ESP32-Latency-Test";
const char* password = "123456789";

// Objeto del servidor
AsyncWebServer server(80);

// Funciones para simular datos (sin sensores físicos)
String readTemp() {
  return String(random(2000, 3000) / 100.0); // Simula 20.00 a 30.00
}

String readHumi() {
  return String(random(40, 70)); // Simula 40% a 70%
}

String readPres() {
  return String(random(1000, 1020)); // Simula hPa
}

void setup() {
  Serial.begin(115200);
  Serial.println();

  // Configurar ESP32 como Access Point
  Serial.print("Configurando Access Point...");
  WiFi.softAP(ssid, password);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("IP del AP: ");
  Serial.println(IP);

  // Endpoint para Temperatura
  server.on("/temperature", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", readTemp().c_str());
  });

  // Endpoint para Humedad
  server.on("/humidity", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", readHumi().c_str());
  });

  // Endpoint para Presión
  server.on("/pressure", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", readPres().c_str());
  });

  // Iniciar servidor
  server.begin();
  Serial.println("Servidor iniciado.");
}

void loop() {
  // ESPAsyncWebServer funciona por interrupciones, no requiere nada en el loop
}
