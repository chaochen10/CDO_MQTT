#include <SoftwareSerial.h>

/***************************************************
  Adafruit MQTT Library ESP8266 Adafruit IO SSL/TLS example

  Must use the latest version of ESP8266 Arduino from:
    https://github.com/esp8266/Arduino

  Works great with Adafruit's Huzzah ESP board & Feather
  ----> https://www.adafruit.com/product/2471
  ----> https://www.adafruit.com/products/2821

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Tony DiCola for Adafruit Industries.
  SSL/TLS additions by Todd Treece for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/
#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <Ticker.h>


/************************* WiFi Access Point *********************************/

#define WLAN_SSID       "050216"   //"11055"
#define WLAN_PASS       "?U9ZzarlsDorOA7GYau;@"   //"wux14n$wang%lu0?mima@!"

/************************* Adafruit.io Setup *********************************/

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  8883                   // 8883 for MQTTS
#define AIO_USERNAME    "ccaoen"
#define AIO_KEY         "a89e82870d4247448f3b63841c8bfa16"

/************ Global State (you don't need to change this!) ******************/

// WiFiFlientSecure for SSL/TLS support
WiFiClientSecure client;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// io.adafruit.com SHA1 fingerprint
const char* fingerprint = "26 96 1C 2A 51 07 FD 15 80 96 93 AE F7 32 CE B9 0D 01 55 C4";

/****************************** Feeds ***************************************/

// Setup a feed called 'test' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>

Adafruit_MQTT_Subscribe pir_sensor = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/pir");

Adafruit_MQTT_Subscribe photocell = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/photocell");

Adafruit_MQTT_Subscribe usonic_sensor = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/usonic");

Adafruit_MQTT_Subscribe humidity_sensor = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/humidity");

Adafruit_MQTT_Subscribe temperature_sensor = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/temperature");

//Adafruit_MQTT_Publish pressure_sensor = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/pressure");

//Adafruit_MQTT_Publish indexTemperature_sensor = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/indexTemperature");


/*************************** Sketch Code ************************************/

// Bug workaround for Arduino 1.6.6, it seems to need a function declaration
// for some reason (only affects ESP8266, likely an arduino-builder bug).
void MQTT_connect();
void verifyFingerprint();

Ticker ledMQTT;
Ticker ledUsonic;
Ticker enviarDataSerial;
#define pinRx 14
#define pinTx 12
SoftwareSerial swSer(pinRx, pinTx, false, 256);
#define pinLedMQTT 2
#define pinLed2  0
bool datosPIR = false;
bool datosUsonic = false;
bool datosTemperatura = false;
bool datosHumedad = false;
bool notificarFuncionamiento = false;
int valorUsonicx100 = 0;
int valorTemperaturax100 = 0;
int valorHumedadx100 = 0;

void setup() {
  Serial.begin(115200);
  delay(50);
  swSer.begin(9600);
  delay(50);
  //llamar_tono();

  //delay(500);
  Serial.println(F("Adafruit IO MQTTS (SSL/TLS) Example"));

  // Connect to WiFi access point.
  Serial.println(); Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);

  delay(1000);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  delay(2000);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: "); Serial.println(WiFi.localIP());

  // check the fingerprint of io.adafruit.com's SSL cert
  verifyFingerprint();

  mqtt.subscribe(&pir_sensor);
  mqtt.subscribe(&photocell);
  mqtt.subscribe(&usonic_sensor);
  mqtt.subscribe(&humidity_sensor);
  mqtt.subscribe(&temperature_sensor);

  pinMode(pinLedMQTT, OUTPUT);
  pinMode(pinLed2, OUTPUT);
  
  digitalWrite(pinLed2, LOW);
  digitalWrite(pinLedMQTT, LOW);

  enviarDataSerial.attach_ms(50,enviar_datos);
  Serial.println(F("Iniciando MQTT..."));
  
}

uint32_t x = 0;

void loop() {
  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  MQTT_connect();

  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(5000))) {
    if (subscription == &pir_sensor) {
       if (strcmp((char *)pir_sensor.lastread, "#bf5024") == 0) {
        Serial.print("presencia PIR");
        datosPIR = true;
       }
      else if (strcmp((char *) pir_sensor.lastread, "#353538") == 0) {
        Serial.println("variacion PIR de apagar");
      }
    }
    
   if(subscription == &photocell){
      Serial.println(F(".... MQTT led parpadear OK"));
      ledMQTT.attach(0.5, notificar_led);
      notificarFuncionamiento = true;
    }

    if(subscription == &usonic_sensor){
      float usonic = atof((char *)usonic_sensor.lastread);  // convert to a number
      valorUsonicx100 = int(usonic * 100);
      if(usonic<=2.86){
        Serial.print(F("valor usonic: "));
        Serial.println(usonic);
        ledUsonic.attach(0.25,notificar_led2);
      }
    }
    if(subscription == &humidity_sensor){
      float humidity = atof((char *)humidity_sensor.lastread);
      Serial.print(F("humedad: "));
      Serial.println(humidity);
      valorHumedadx100 = int(humidity * 100);
      datosHumedad = true;
    }
    if(subscription == &temperature_sensor){
      float temperature = atof((char *)temperature_sensor.lastread);
      Serial.print(F("temperatura: "));
      Serial.println(temperature);
      valorTemperaturax100 = int(temperature * 100);
      datosTemperatura = true;
    }
  }
  

  if (! mqtt.ping()) {
    mqtt.disconnect();
  }

}

void verifyFingerprint() {
  const char* host = AIO_SERVER;
  Serial.print("Connecting to ");
  Serial.println(host);

  if (! client.connect(host, AIO_SERVERPORT)) {
    Serial.println("Connection failed. Halting execution.");
    while (1);
  }

  if (client.verify(fingerprint, host)) {
    Serial.println("Connection secure.");
  } else {
    Serial.println("Connection insecure! Halting execution.");
    while (1);
  }

}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;
  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }
  Serial.print("Connecting to MQTT... ");
  uint8_t retries = 10;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    digitalWrite(pinLed2, LOW);
    delay(5000);  // wait 5 seconds
    retries--;
    if (retries == 0) {
      Serial.println(F("Retrying MQTT connection in 5 minutes..."));
      delay(300000);
      retries = 10;
      // basically die and wait for WDT to reset me
      //while (1);
    }
  }

  Serial.println("MQTT Connected!");
  digitalWrite(pinLed2, HIGH);
  digitalWrite(pinLedMQTT, HIGH);
}

void notificar_led(){
  static bool parpadear;
  static int contar;
  contar = (contar+1) % 2; 
  parpadear = !parpadear;
 // Serial.print("contar ");
 // Serial.print(contar);
 // Serial.print(" parpadear ");
  if(parpadear == true) {digitalWrite(pinLedMQTT, LOW); /*Serial.println(parpadear);*/}
  else {digitalWrite(pinLedMQTT, HIGH);/*Serial.println(parpadear);*/}
  if(contar == 0) ledMQTT.detach();
}

void notificar_led2(){
  static bool parpadear;
  static int contar;
  contar = (contar+1) % 60; 
  parpadear = !parpadear;
 // Serial.print("contar ");
 // Serial.print(contar);
 // Serial.print(" parpadear ");
  if(parpadear == true) {digitalWrite(pinLed2, LOW); /*Serial.println(parpadear);*/}
  else {digitalWrite(pinLed2, HIGH);/*Serial.println(parpadear);*/}
  if(contar == 0) ledUsonic.detach();
}

void enviar_datos(){
  if(datosPIR){
    swSer.println(F("%p,0$"));
    datosPIR = false;
  }
  else if(datosUsonic){
    swSer.print(F("%u,"));
    swSer.print(valorUsonicx100);
    swSer.println(F("$"));
    datosUsonic = false;
  }
  else if(datosHumedad){
    swSer.print(F("%h,"));
    swSer.print(valorHumedadx100);
    swSer.println(F("$"));
    datosHumedad = false;
  }
  else if(datosTemperatura){
    swSer.print(F("%t,"));
    swSer.print(valorTemperaturax100);
    swSer.println(F("$"));
      datosTemperatura = false;
  }
  else if(notificarFuncionamiento){
    swSer.println(F("%n,0$"));
    notificarFuncionamiento = false;
  }
}

