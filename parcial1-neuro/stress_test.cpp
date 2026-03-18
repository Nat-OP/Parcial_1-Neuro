#include <Arduino.h>
#include "WiFi.h"
#include "ESPAsyncWebServer.h"

// Credenciales del Access Point
const char* ssid = "ESP32-Stress-Test";
const char* password = "123456789";

// Objeto del servidor
AsyncWebServer server(80);

// Funciones de simulación
String readTemp() { return String(random(2000, 3000) / 100.0); }
String readHumi() { return String(random(40, 70)); }
String readPres() { return String(random(1000, 1020)); }

void setup() {
  Serial.begin(115200);
  
  // Iniciar WiFi en modo Access Point
  WiFi.softAP(ssid, password);
  Serial.println("\n--- ESP32 Stress Test Server ---");
  Serial.print("IP del AP: ");
  Serial.println(WiFi.softAPIP());

  // 1. Endpoint Simple (Baja latencia)
  server.on("/ping", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", "pong");
  });

  // 2. Endpoint de Datos (JSON Estándar)
  server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request){
    String json = "{";
    json += "\"temp\":" + readTemp() + ",";
    json += "\"humi\":" + readHumi() + ",";
    json += "\"pres\":" + readPres() + ",";
    json += "\"status\":\"OK\"";
    json += "}";
    request->send(200, "application/json", json);
  });

  // 3. Endpoint de Estrés (JSON Pesado + Simulación de proceso)
  server.on("/stress", HTTP_GET, [](AsyncWebServerRequest *request){
    // Simulamos un pequeño retraso de procesamiento de 10ms
    // para ver cómo se suma a la latencia de red.
    delay(10); 
    
    String heavyJson = "{ \"logs\": [";
    for(int i=0; i<10; i++) {
      heavyJson += "{\"id\":" + String(i) + ",\"val\":" + String(random(100, 999)) + "}";
      if(i < 9) heavyJson += ",";
    }
    heavyJson += "], \"device\":\"ASUS-TUF-Node\", \"uptime\":" + String(millis()) + "}";
    
    request->send(200, "application/json", heavyJson);
  });

  server.begin();
  Serial.println("Servidor listo para pruebas.");
}

void loop() {
  // El servidor funciona de forma asíncrona, no bloqueamos el loop.
}