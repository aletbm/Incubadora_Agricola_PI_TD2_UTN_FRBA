#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>

// Configuracion WIFI
//const char* ssid = "MotoG";
//const char* password = "digitales";
//const char* chatID = "1329817360";
/*
const char* ssid = "NoConectarQueTeHackea";
const char* password = "pangea01";
const char* botToken = "8373577792:AAG3qJaATNbAlGfGckmVILiTXOxWi11J1M0";
const char* chatID = "6326727457"; 
*/
String ssid = "";
String password = "";
String chatID = "";
const char* botToken = "8373577792:AAG3qJaATNbAlGfGckmVILiTXOxWi11J1M0";


bool wifiConnected = false;
bool configuracionCompleta = false;
unsigned long connectionStartTime = 0;
const unsigned long WIFI_TIMEOUT = 30000; // 30 segundos máximo


// Configuración UART
#define RX_PIN 20
#define TX_PIN 21
HardwareSerial SerialPort(1); // UART1

const unsigned long CONFIG_TIMEOUT = 120000; // 2 minutos para configuración
const unsigned long INPUT_TIMEOUT = 30000;   // 30 segundos por cada entrada
bool esperarConfiguracionUART() {
    Serial.println();
    Serial.println("=== MODO CONFIGURACIÓN ===");
    Serial.println("Esperando datos de configuración por UART...");
    Serial.println("Timeout: 2 minutos");
    Serial.println("==========================");
    
    unsigned long configStartTime = millis();
    
    while (true) {
        // Verificar timeout general
        if (millis() - configStartTime > CONFIG_TIMEOUT) {
            Serial.println("TIMEOUT: Tiempo de configuración agotado");
            return false;
        }
        
        // Verificar si hay datos disponibles
        if (SerialPort.available() > 0) {
            String input = SerialPort.readString();
            input.trim(); // Eliminar espacios y newlines
            
            if (input.length() > 0) {
                Serial.print("Recibido: ");
                Serial.println(input);
                
                // Parsear datos separados por coma
                int primeraComa = input.indexOf(',');
                int segundaComa = input.indexOf(',', primeraComa + 1);
                
                if (primeraComa != -1 && segundaComa != -1) {
                    // Extraer los tres valores
                    ssid = input.substring(0, primeraComa);
                    password = input.substring(primeraComa + 1, segundaComa);
                    chatID = input.substring(segundaComa + 1);
                    
                    // Limpiar espacios
                    ssid.trim();
                    password.trim();
                    chatID.trim();
                    
                    // Validar que no estén vacíos
                    if (ssid.length() > 0 && password.length() > 0 && chatID.length() > 0) {
                        Serial.println("Datos parseados correctamente");
                        Serial.println("=== DATOS CONFIGURADOS ===");
                        Serial.println("SSID: " + ssid);
                        Serial.println("CHAT_ID: " + chatID);
                        Serial.println("==========================");
                        
                        // LED de confirmación
                        for(int i = 0; i < 6; i++) {
                            digitalWrite(8, !digitalRead(8));
                            delay(150);
                        }
                        digitalWrite(8, HIGH);
                        
                        return true;
                    } else {
                        Serial.println("ERROR: Todos los campos deben tener datos");
                    }
                } else {
                    Serial.println("ERROR: Formato incorrecto");
                    Serial.println("Usa: SSID,PASSWORD,CHAT_ID");
                }
                
                Serial.println();
            }
        }
        
        // LED parpadeante durante configuración
        static unsigned long lastBlink = 0;
        if (millis() - lastBlink > 300) {
            lastBlink = millis();
            digitalWrite(8, !digitalRead(8));
        }
        
        delay(50);
    }
}

void connectToWiFi() {
  Serial.println("Iniciando conexión WiFi...");
  
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  delay(1000);
  
  Serial.print("Conectando a: ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  connectionStartTime = millis();
  
  while (WiFi.status() != WL_CONNECTED) {
    // Verificar timeout de manera segura (evitando desbordamiento)
    unsigned long currentTime = millis();
    unsigned long elapsedTime;
    
    if (currentTime >= connectionStartTime) {
      elapsedTime = currentTime - connectionStartTime;
    } else {
      // Handle millis() overflow
      elapsedTime = (ULONG_MAX - connectionStartTime) + currentTime;
    }
    
    if (elapsedTime > WIFI_TIMEOUT) {
      Serial.println();
      Serial.println("TIMEOUT: No se pudo conectar a WiFi en 30 segundos");
      Serial.println("Estado WiFi: " + String(WiFi.status()));
      return;
    }
    
    delay(500);
    Serial.print(".");
    digitalWrite(8, !digitalRead(8)); // LED parpadeante
    
    // Reset de hardware suave si parece congelado
    if (elapsedTime > 10000) { // Después de 10 segundos
      Serial.println();
      Serial.println("Intentando reconexión WiFi...");
      WiFi.disconnect();
      delay(1000);
      WiFi.begin(ssid, password);
    }
  }
  
  Serial.println();
  Serial.println("WiFi CONECTADO!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  wifiConnected = true;
  digitalWrite(8, HIGH); // LED encendido = conectado
}

void sendTelegramMessage(String message) {
  if (!wifiConnected) {
    Serial.println("ERROR WiFi no conectado");
    return;
  }
  
  HTTPClient http;
  String url = "https://api.telegram.org/bot" + String(botToken) + "/sendMessage";
  String payload = "{\"chat_id\":\"" + String(chatID) + "\",\"text\":\"" + message + "\"}";
  
  Serial.println("Iniciando conexión Telegram");
  
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  
  int httpCode = http.POST(payload);
  
  if (httpCode == 200) {
    Serial.println("Mensaje enviado a Telegram");
  } else {
    Serial.print("Error Telegram: ");
    Serial.println(httpCode);
  }
  
  http.end();
}

void setup() {
  Serial.begin(115200);
  delay(3000); // Espera crítica para USB
  
  Serial.println();
  Serial.println("ESP32-C3 Iniciando...");
  Serial.println("========================");

  // Inicializar UART1 en pines específicos
  SerialPort.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  pinMode(8, OUTPUT);
  digitalWrite(8, LOW);
  
  // Información del sistema
  Serial.print("Free heap: ");
  Serial.println(esp_get_free_heap_size());
  Serial.print("CPU freq: ");
  Serial.println(ESP.getCpuFreqMHz());
  
  // Obtener configuracion
  esperarConfiguracionUART();

  // Conectar WiFi
  connectToWiFi();
  
  if (wifiConnected) {
    delay(2000);
    sendTelegramMessage("ESP32-C3 iniciado correctamente!\n" +
                       String("IP: ") + WiFi.localIP().toString() + "\n" +
                       "Memoria: " + String(esp_get_free_heap_size()) + " bytes");
  } else {
    Serial.println("Modo offline - WiFi falló");
    // Modo error - parpadeo lento
    while(1) {
      digitalWrite(8, HIGH);
      delay(1000);
      digitalWrite(8, LOW);
      delay(1000);
    }
  }

  
}

void loop() {
  // Mantener conexión WiFi
  if (wifiConnected && WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi desconectado, reconectando...");
    wifiConnected = false;
    digitalWrite(8, LOW);
    connectToWiFi();
  }
  /*
  CODIGO PARA RECIBIR MENSAJES
  */
  if (SerialPort.available() > 0) { // Verificar si hay datos disponibles
    String receivedData = SerialPort.readString();
    receivedData.trim(); // Eliminar espacios y newlines
    
    if (receivedData.length() > 0) {
      digitalWrite(8, !digitalRead(8));
      Serial.print("Recibido: \"");
      Serial.print(receivedData);
      Serial.println("\"");
      sendTelegramMessage("Mensaje :"+receivedData+"\n");
      // Mostrar información adicional
      Serial.print("Longitud: ");
      Serial.println(receivedData.length());
      Serial.print("Tiempo: ");
      Serial.println(millis());
      Serial.println();
    }
  }
}




/*
CODIGO PRUEBA BASICA DE COMUNICACION CON TERMINAL
#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  delay(2000);  // Espera larga para asegurar inicialización
  
  Serial.println("TEST - ESP32-C3 Funcionando");
  Serial.println("=================================");
  pinMode(8, OUTPUT);
}

void loop() {
  digitalWrite(8, !digitalRead(8));
  Serial.println("Hola mundo");
  Serial.println("LED: " + String(digitalRead(8) ? "ENCENDIDO" : "APAGADO"));
  delay(1000);
}
*/
