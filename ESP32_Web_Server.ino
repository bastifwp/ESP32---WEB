#include <OneWire.h>
#include <DallasTemperature.h>
#include <Arduino.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "SPIFFS.h"
#include <Arduino_JSON.h>
#include <WiFi.h>

//Variables de entorno (se usa en ppm)
#define VREF 5 //Referencia analógico de voltaje
#define SCOUNT 30 //sum of sample point


//Pines de input que utilizaremos
#define T_PIN 33 // sensor temperatura
#define LIGHT_SENSOR_PIN 34 // sensor de luz
#define TdsSensorPin 32 // sensor de ppm

//Variables para medir luz
int light;


//Variables para medir temperatura
OneWire oneWire(T_PIN);
DallasTemperature DS18B20(&oneWire);

float tempC_1;
float tempC_2;


//Variables para PPM
float averageVoltage = 0;
float tdsValue = 0;
float temperature = 25; //Temperature for compensations

int analogBuffer[SCOUNT]; //Para guardar el valor análogo en array, leido desde el sensor
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
int copyIndex = 0;


//Función para obtener la mediana 
int getMedianNum(int bArray[], int iFilterLen){
  int bTab[iFilterLen];
  for (byte i = 0; i<iFilterLen; i++)
  bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0){
    bTemp = bTab[(iFilterLen - 1) / 2];
  }
  else {
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
  return bTemp;
}

// ----------------- CONFIGURACION PARA WEB SERVICE -------------
const char* ssid = ""; //ssid: nombre del wifi a conectarse
const char* password = ""; //password: clave del wifi a conectarse

//Creamos un objetc AsyncWebServer en puerto 80
AsyncWebServer server(80);

//Creamos un "event source" en /events
AsyncEventSource events("/events");

//Creamos variable de tipo json que reciba los datos de los sensores
JSONVar readings;

//**Timer variables**
unsigned long lastTime = 0;
unsigned long timerDelay = 3000;


//Función que genera objeto JSON a partir de las str
String getSensorReadings(){
  readings["temperature1"] = String(tempC_1);
  readings["temperature2"] = String(tempC_2);
  readings["light"] = String(light);
  readings["tds"] = String(tdsValue);
  String jsonStrings = JSON.stringify(readings);
  return jsonStrings;
}

//Función que inicializa SPIFFS (manejo de archivos)
void initSPIFFS(){
  if (!SPIFFS.begin()){
    Serial.println("An error has occurred while mounting SPIFFS");
  }
  Serial.println("SPIFFS mounted successfully");
}

//Función que iniciaiza el WiFi
void initWiFi(){
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi...");

  //Si es que no funciona queda en un loop infinito
  while(WiFi.status() != WL_CONNECTED){
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
}




void setup() {
  Serial.begin(115200); //Iniciamos serial
  DS18B20.begin(); //Iniciamos sensor de temperatura
  pinMode(TdsSensorPin, INPUT); //Iniciamos pin para sensor de ppm 
  initWiFi(); 
  initSPIFFS();

  // --------- RUTAS PARA EL WEB SERVER ------------

  //Web Server Root URL
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", "text/html");
  });

  server.serveStatic("/", SPIFFS, "/");


  //Request for the lastest sensor readings
  server.on("/readings", HTTP_GET, [](AsyncWebServerRequest *request){
    String json = getSensorReadings();
    request->send(200, "application/json", json);
    json = String();

  });


  events.onConnect([](AsyncEventSourceClient *client){
    if(client->lastId()){
      Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }

    //mandar evento con mensaje "hello", id current millis and set reconnect delay to 1 s
    client->send("hello!", NULL, millis(), 10000);

  });

  server.addHandler(&events);

  //Start server
  server.begin();

}






void loop() {

  //ENVIAR LAS COSAS AL WEB SERVER
  if ((millis() - lastTime) > timerDelay){
    
    //Mandar los datos cada 10 segundos
    events.send("ping", NULL, millis());
    events.send(getSensorReadings().c_str(), "new_readings", millis());
    lastTime = millis();

  }



  // --------- SENSOR DE TEMPERATURA ------------

  //Primero pediremos las temperaturas y almacenaremos en variables respectivas

  //Mostraremos resultados cada 3 segundos
  static unsigned long tempSampleTimepoint = millis();   
  if(millis() - tempSampleTimepoint > 1000U){
    tempSampleTimepoint = millis();

    DS18B20.requestTemperatures();
    tempC_1 = DS18B20.getTempCByIndex(0);
    tempC_2 = DS18B20.getTempCByIndex(1);

    //Mostramos temperaturas de los sensores
    Serial.print("Temperatura sensor 1: ");
    Serial.print(tempC_1);
    Serial.println("°C");

    Serial.print("Temperatura sensor 2: ");
    Serial.print(tempC_2);
    Serial.println("°C");
  }

  // --------- SENSOR DE LUZ -------------
  
  //Cada 3 segundos realizaremos una medición
  static unsigned long lightSampleTimepoint = millis();
  

  if(millis() - lightSampleTimepoint > 1000U ){
    
    lightSampleTimepoint = millis();

    int analogValue = analogRead(LIGHT_SENSOR_PIN);
    light = analogValue;

    //Mostramos valor
    Serial.print("Sensor Luz: ");
    Serial.println(analogValue);
  }


  // ---------- SENSOR PPM --------------

  //Definimos cada cuanto tiempo el sensor mide
  static unsigned long analogSampleTimepoint = millis();

  //Cada 40 mili segundos el sensor tomará valores
  if(millis() - analogSampleTimepoint > 40U ){ //Si han pasado mas de 40 ms
    analogSampleTimepoint = millis();

    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin); //le un valor y lo guarda en el buffer
    
    //Avanzamos una posición en el buffer
    analogBufferIndex++;
    if(analogBufferIndex == SCOUNT){
      analogBufferIndex = 0;
    }

  
  //Ahora debemos definir cada cuanto tiempo se mostrarán los resultados
  static unsigned long printTimepoint = millis();
  
  //cada 3000 milisegundos se mostrarán los datos
  if(millis() - printTimepoint > 1000U){ //Si han pasado mas de 3000 ms
    printTimepoint = millis();

    //Copiamos valores de un buffer a otro
    for(copyIndex = 0; copyIndex < SCOUNT; copyIndex++){
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
    }

    //Calculo del voltaje promedio
    averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 4096.0;

    //Compensaiones de temperatura y voltaje
    float compensationCoefficient = 1.0 + 0.02*(temperature - 25.0);
    float compensationVoltage = averageVoltage/compensationCoefficient;

    //Convertimos valores obtenidos a TDS:
    tdsValue=(133.42*compensationVoltage*compensationVoltage*compensationVoltage - 255.86*compensationVoltage*compensationVoltage + 857.39*compensationVoltage)*0.5;

    //Mostramos resultado
    Serial.print("TDS Value: ");
    Serial.print(tdsValue, 0);
    Serial.println("ppm");
    Serial.println("");

  }

  }

}
