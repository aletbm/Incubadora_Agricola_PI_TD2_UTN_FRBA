#include <Arduino.h>
#include <WiFi.h>
#include <WiFiManager.h>      
#include <WiFiClientSecure.h> 

// ---------------------------------------------------------------------------
// IMPORTANTE: ESTE CÓDIGO REQUIERE ARDUINO JSON VERSIÓN 6.21.5
// Si tienes la versión 7.x.x, ve al Gestor de Librerías, busca ArduinoJson
// y selecciona la versión 6.21.5 para instalarla (Downgrade).
// ---------------------------------------------------------------------------
#include <CTBot.h>            

// ==========================================
// CONFIGURACIÓN DE USUARIO
// ==========================================
const char* token = "8289483635:AAEaHNfAq4bn54nVXZgFgaWna9pJ-2CTpdc";

// SOLUCIÓN AL ERROR DE COMPILACIÓN:
// Definimos el ID como Texto (String) para evitar el error de "decrement operand".
// El código lo convertirá a número automáticamente en el setup.
String chatID_Texto = "-5018890121"; 

// ==========================================
// HARDWARE
// ==========================================
#define RX_PIN 20             
#define TX_PIN 21             
#define LED_PIN 8             
#define TRIGGER_PIN 9         

// ==========================================
// OBJETOS GLOBALES
// ==========================================
HardwareSerial SerialPort(1); 
CTBot myBot;                  
WiFiManager wm;
bool wifiConnected = false;
int64_t targetChatID; // Aquí se guardará el ID numérico convertido

// Configuración IP para Portal (Estilo HH3)
IPAddress apIP(8, 8, 8, 8);
IPAddress apGateway(8, 8, 8, 8);
IPAddress apSubnet(255, 255, 255, 0);

// ==========================================
// PROTOTIPOS
// ==========================================
void configModeCallback(WiFiManager *myWiFiManager);
void sendToTelegram(String text);

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  SerialPort.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  
  pinMode(LED_PIN, OUTPUT);
  pinMode(TRIGGER_PIN, INPUT_PULLUP);

  // --- CONVERSIÓN DEL ID (Solución al error) ---
  // strtoll convierte el String a un número de 64 bits (long long) de forma segura
  targetChatID = strtoll(chatID_Texto.c_str(), NULL, 10);
  
  Serial.println("\n--- BRIDGE TELEGRAM (CTBot Version) ---");
  Serial.print("Target ID (Texto): ");
  Serial.println(chatID_Texto);
  
  // CORRECCIÓN: Usamos printf con %lld para ver el número de 64 bits completo.
  Serial.printf("Target ID Numérico: %lld\n", targetChatID); 

  // Configuración Portal WiFi
  wm.setAPCallback(configModeCallback);
  wm.setAPStaticIPConfig(apIP, apGateway, apSubnet); 
  wm.setConnectTimeout(30); 

  if (!wm.autoConnect("ESP32-Telegram-Bridge")) {
    Serial.println("Fallo WiFi. Reiniciando...");
    delay(3000);
    ESP.restart();
  }

  Serial.println("WiFi Conectado.");
  
  // CTBot Setup
  myBot.setTelegramToken(token);
  myBot.useDNS(true); 

  Serial.print("Probando conexión Telegram...");
  if (myBot.testConnection()) {
    Serial.println(" OK!");
    wifiConnected = true;
    digitalWrite(LED_PIN, HIGH); 
    
    // Mensaje de inicio
    String msg = "🟢 Sistema Online.\nIP: " + WiFi.localIP().toString();
    myBot.sendMessage(targetChatID, msg);
  } else {
    Serial.println(" FALLO. (Revisa Token, Internet o Versión de JSON)");
    wifiConnected = false;
  }
}

// ==========================================
// LOOP
// ==========================================
void loop() {
  // 1. Heartbeat
  if (!wifiConnected) {
    static unsigned long lastBlink = 0;
    if (millis() - lastBlink > 500) {
      lastBlink = millis();
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
  }

  // 2. Reset WiFi
  if (digitalRead(TRIGGER_PIN) == LOW) {
    delay(50); 
    if (digitalRead(TRIGGER_PIN) == LOW) {
      unsigned long startPress = millis();
      while (digitalRead(TRIGGER_PIN) == LOW) {
        if (millis() - startPress > 3000) {
          Serial.println("!!! BORRANDO WIFI !!!");
          for(int i=0; i<5; i++) { digitalWrite(LED_PIN, !digitalRead(LED_PIN)); delay(100); }
          wm.resetSettings(); 
          ESP.restart();      
        }
      }
    }
  }

  // 3. Puente UART (Recibir datos del cable y enviar a Telegram)
  if (SerialPort.available() > 0) {
    String receivedData = SerialPort.readString();
    receivedData.trim(); 
    
    if (receivedData.length() > 0) {
      Serial.print("UART >> "); Serial.println(receivedData);
      sendToTelegram("📩 " + receivedData);
    }
  }

  // 4. Comandos de Telegram
  // Verificamos si hay mensajes nuevos para comandos como /test
  TBMessage msg;
  if (myBot.getNewMessage(msg)) {
    
    // Usamos startsWith para detectar "/test" incluso si escriben "/test@NombreBot"
    if (msg.text.startsWith("/test")) {
        Serial.println("Comando /test recibido.");
        
        // MODIFICACIÓN: Respondemos al ID DEL GRUPO (targetChatID) 
        // en lugar de al usuario privado (msg.sender.id).
        // Así confirmamos en el grupo que el bot está activo.
        String respuesta = "✅ Estado: ACTIVO\n";
        respuesta += "📡 Escuchando en Grupo: " + chatID_Texto; // Usamos el String original para evitar errores de visualización
        
        myBot.sendMessage(targetChatID, respuesta);
    }
  }
}

// ==========================================
// FUNCIONES AUXILIARES
// ==========================================
void configModeCallback(WiFiManager *myWiFiManager) {
  Serial.println("MODO AP: Esperando configuración WiFi...");
  Serial.println(WiFi.softAPIP()); 
  digitalWrite(LED_PIN, HIGH); 
}

void sendToTelegram(String text) {
  if (wifiConnected) {
    digitalWrite(LED_PIN, LOW); delay(50); digitalWrite(LED_PIN, HIGH);
    myBot.sendMessage(targetChatID, text);
  } else {
    Serial.println("Error: Sin conexión Telegram");
  }
}