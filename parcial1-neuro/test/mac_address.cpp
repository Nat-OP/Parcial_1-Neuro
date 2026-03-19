#include <WiFi.h>
#include <esp_wifi.h>

void readMacAddress(){
  uint8_t baseMac[6];
  // Esta función es nativa del SDK de Espressif
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  if (ret == ESP_OK) {
    Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x\n",
                  baseMac[0], baseMac[1], baseMac[2],
                  baseMac[3], baseMac[4], baseMac[5]);
  } else {
    Serial.println("Error al leer la dirección MAC");
  }
}

void setup(){
  Serial.begin(115200);
  
  // 1. Establecer el modo estación
  WiFi.mode(WIFI_STA);
  
  // 2. CORRECCIÓN: Usar WiFi.begin() sin el .STA
  // Si no pasas SSID ni Password, solo inicializa el driver de radio
  WiFi.begin(); 
  
  Serial.print("[DEFAULT] ESP32 Board MAC Address: ");
  readMacAddress();
}

void loop(){
  // Nada por ahora
}