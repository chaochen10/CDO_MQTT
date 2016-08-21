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
//#include <SimpleTimer.h>
#include <Ticker.h>
#include "DHT.h"
//#include "Maxbotix.h"

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

Adafruit_MQTT_Publish pir_sensor = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/pir");

Adafruit_MQTT_Publish pressure_sensor = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/pressure");

Adafruit_MQTT_Publish temperature_sensor = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperature");

Adafruit_MQTT_Publish indexTemperature_sensor = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/indexTemperature");

Adafruit_MQTT_Publish humidity_sensor = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humidity");

Adafruit_MQTT_Publish usonic_sensor = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/usonic");

Adafruit_MQTT_Publish photocell = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/photocell");

/*************************** Sketch Code ************************************/

// Bug workaround for Arduino 1.6.6, it seems to need a function declaration
// for some reason (only affects ESP8266, likely an arduino-builder bug).
void MQTT_connect();
void verifyFingerprint();
float digitalSmooth(float *arrayValue, int8_t numSampling);
void timer_acciones(bool acciones);

#define pinPir  12
#define pinDHT  13
#define pinUSonic  14
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321

SimpleTimer timer;
DHT dht(pinDHT, DHTTYPE);

//int8_t timerId_pir;
//int8_t timerId_DHT;
//int8_t timerId_sendMQTT;
//int8_t timerId_Usonic;
//int8_t timerId_led;
//int8_t timerId_MQTTconnection;
//int8_t timerId_photocell;

Ticker timerId_pir;
Ticker timerId_DHT;
Ticker timerId_sendMQTT;
Ticker timerId_Usonic;
Ticker timerId_led;
Ticker timerId_MQTTconnection;
Ticker timerId_photocell;

bool pirValue = false;
bool pirValueTemp = false;
bool pirDetect = false;
bool pirAlarm = false;

bool dhtRead = false;
float arrayTemperature[13];
float arrayIndexTemperature[13];
float arrayHumidity[13];
int8_t arrayTemperatureElement = 13;
float temperatureValue = 0.0;
float indexTemperatureValue = 0.0;
float humidityValue = 0.0;
float tempTemperatureValue = 1.0;
float tempIndexTemperatureValue = 1.0;
float tempHumidityValue = 1.0;

bool usonicRead = false;
float arrayUsonic[13];
int arrayUsonicElement = 13;
float usonicValue = 0.0;
float tempUsonicValue = 0.0;

bool enivarMQTT = false;
bool enviarTemperatura = false;
bool enviarHumedad = false;
bool enviarSensacionTemperatura = false;
bool enviarUsonic = false;
bool enviarPhotocell = false;

/*************************************/
bool lectura_temperaturaSensacion = false;


void setup() {
  Serial.begin(115200);
  delay(50);

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

  // iniciar sensor temperatura y humedad
  dht.begin();
  // iniciar sensor PIR
  pinMode(pinPir, INPUT_PULLUP);//INPUT_PULLUP
  pinMode(0, OUTPUT);
  digitalWrite(0, LOW);
//  timerId_pir = timer.setInterval(50, llamar_lecturaPir);
//  timerId_DHT = timer.setInterval(2000, llamar_lecturaDHT);
//  timerId_sendMQTT = timer.setInterval(600, llamar_EnviarMQTT);
//  timerId_Usonic = timer.setInterval(60, llamar_lecturaUsonic);
//  timerId_led = timer.setInterval(50, llamar_led);
//  timerId_photocell = timer.setInterval(120000, llamar_photocell); // 2min
//  timer.enable(timerId_photocell);

  timerId_pir.attach_ms(50, llamar_lecturaPir);
  timerId_DHT.attach(2, llamar_lecturaDHT);
  timerId_sendMQTT.attach_ms(600, llamar_EnviarMQTT);
  timerId_Usonic.attach_ms(60, llamar_lecturaUsonic);
  timerId_led.attach_ms(50, llamar_led);
  timerId_photocell.attach(120, llamar_photocell); // 2min
  //timer.enable(timerId_photocell);

  timer_acciones(false);
  Serial.println(F("Iniciando MQTT..."));
  digitalWrite(0, HIGH);
}

uint32_t x = 0;

void loop() {
  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  MQTT_connect();
  timer.run();

  if (dhtRead == true) {
    float h;
    float t;
    float hic;

    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    h = dht.readHumidity();
    // Read temperature as Celsius (the default)
    t = dht.readTemperature();

    // Compute heat index in Celsius (isFahreheit = false)
    if (lectura_temperaturaSensacion) hic = dht.computeHeatIndex(t, h, false);

    // Check if any reads failed and exit early (to try again).
    if (isnan(h) || isnan(t)) {
      Serial.println(F("Failed to read from DHT sensor!"));
      //return;
    }
    else {
      static int8_t positionArray;
      positionArray = (positionArray + 1) % arrayTemperatureElement;
      static int8_t calcSmoothValue;
      calcSmoothValue = (calcSmoothValue + 1) % 10;
      arrayTemperature[positionArray] = t;
      if (lectura_temperaturaSensacion) arrayIndexTemperature[positionArray] = hic;
      arrayHumidity[positionArray] = h;

      if (calcSmoothValue == 0) {
        // calcular mediana del vector temperatura
        tempTemperatureValue = digitalSmooth( arrayTemperature, arrayTemperatureElement);
        if (tempTemperatureValue > (temperatureValue - 0.05) && tempTemperatureValue < (temperatureValue + 0.05)) {
          Serial.print(F("no cambiar valor temperatura "));
          Serial.print(tempTemperatureValue);
          Serial.print(F("\t\tanterior "));
          Serial.println(temperatureValue);
        }
        else {
          enviarTemperatura = true;
        }
        // calcular mediana del vector sensacion temperatura
        if (lectura_temperaturaSensacion) {
          tempIndexTemperatureValue = digitalSmooth( arrayIndexTemperature, arrayTemperatureElement);
          if (tempIndexTemperatureValue > (indexTemperatureValue - 0.05) && tempIndexTemperatureValue < (indexTemperatureValue + 0.05)) {
            Serial.print(F("no cambiar valor temperatura sensacacion "));
            Serial.print(tempIndexTemperatureValue);
            Serial.print(F("\tanterior "));
            Serial.println(indexTemperatureValue);
          }
          else {
            enviarSensacionTemperatura = true;
          }
        }
        // calcular mediana del vector humedad
        tempHumidityValue = digitalSmooth( arrayHumidity, arrayTemperatureElement);
        if (tempHumidityValue > (humidityValue - 0.05) && tempHumidityValue < (humidityValue + 0.05)) {
          Serial.print(F("no cambiar valor humedad "));
          Serial.print(tempHumidityValue);
          Serial.print(F("\t\tanterior "));
          Serial.println(humidityValue);
        }
        else {
          enviarHumedad = true;
        }
      }
    }
    dhtRead = false;
  }

  if (usonicRead == true) {
    static int8_t arrayIndexUsonic;
    static int8_t smoothUsonic;
    float distanceM;

    arrayIndexUsonic = (arrayIndexUsonic + 1) % arrayUsonicElement;
    smoothUsonic = (smoothUsonic + 1) % 10;
    distanceM = pulseIn(pinUSonic, HIGH) / 147.0 * 2.54 / 100.0;
    arrayUsonic[arrayIndexUsonic] = distanceM;
    if (smoothUsonic == 0) {
      tempUsonicValue = digitalSmooth(arrayUsonic, arrayUsonicElement);
      if (tempUsonicValue > (usonicValue - 0.05) && tempUsonicValue < (usonicValue + 0.05)) {
        Serial.print(F("no cambiar valor sensor ultrasonido "));
        Serial.print(tempUsonicValue);
        Serial.print(F("\t\tanterior "));
        Serial.println(usonicValue);
      }
      else enviarUsonic = true;
    }
    usonicRead = false;
  }

  if (enivarMQTT == true) {
    timer_acciones(false);
    if (enviarUsonic == true) {
      if (!usonic_sensor.publish(tempUsonicValue))Serial.print(F("** error envio"));
      else {
        usonicValue = tempUsonicValue;
        timer.enable(timerId_led);
      }
      Serial.print(F("\tsmooth usonic: "));
      Serial.print(tempUsonicValue);
      Serial.println(F(" m"));
      enviarUsonic = false;
    }
    else if (pirDetect == true && pirAlarm == true) {
      if (!pir_sensor.publish("#bf5024"))Serial.print(F("** error envio MQTT "));
      else {
        Serial.println(F(" PIR [<->]"));
        timer.enable(timerId_led);
      }
      pirDetect = false;
    }
    else if (pirDetect == true && pirAlarm == false) {
      if (!pir_sensor.publish("#353538"))Serial.print(F("** error envio MQTT "));
      else {
        Serial.println(F(" PIR [···]"));
        timer.enable(timerId_led);
      }
      pirDetect = false;
    }
    else if (enviarTemperatura == true) {
      if (!temperature_sensor.publish(tempTemperatureValue))Serial.print(F("** error envio"));
      else temperatureValue = tempTemperatureValue;
      Serial.print(F("\t smooth temperature: "));
      Serial.print(tempTemperatureValue);
      Serial.println(F("*C"));
      timer.enable(timerId_led);
      enviarTemperatura = false;
    }
    else if (enviarSensacionTemperatura == true) {
      if (!indexTemperature_sensor.publish(tempIndexTemperatureValue))Serial.print(F("** error envio"));
      else indexTemperatureValue = tempIndexTemperatureValue;
      Serial.print(F("\tMQTT smooth heat index: "));
      Serial.print(tempIndexTemperatureValue);
      Serial.println(F("*C"));
      timer.enable(timerId_led);
      enviarSensacionTemperatura = false;
    }
    else if (enviarHumedad == true) {
      if (!humidity_sensor.publish(tempHumidityValue))Serial.print(F("** error envio"));
      else  humidityValue = tempHumidityValue;
      Serial.print(F("\tsmooth Humidity: "));
      Serial.print(tempHumidityValue);
      Serial.println(F(" %"));
      timer.enable(timerId_led);
      enviarHumedad = false;
    }
    else if (enviarPhotocell == true) {
      int valorAleatorio = random(0, 100);
      if (!photocell.publish(valorAleatorio)) Serial.print(F("** error envio"));
      else {
        timer.enable(timerId_led);
        Serial.print(F("\tvalor aleatorio photocell "));
        Serial.println(valorAleatorio);
        //enviarPhotocell = false;
      }
      enviarPhotocell = false;
    }

    timer_acciones(true);
    enivarMQTT = false;
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
    digitalWrite(0, LOW);
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

  timer_acciones(true);
  Serial.println("MQTT Connected!");
  digitalWrite(0, HIGH);
}

float digitalSmooth(float *arrayValue, int8_t numSampling) {
  bool cambios = 0;
  float sorted[numSampling];
  float temp;
  float smoothValue;
  int j, i, k;

  //copiar vector
  for (i = 0; i < numSampling; i++) {
    sorted[i] = arrayValue[i];
  }
  //ordenar de mayor a menor
  while (cambios == false) {
    cambios = true;
    for (j = 0; j < numSampling - 1; j++) {
      if (sorted[j] > sorted[j + 1]) {
        temp = sorted[j + 1];
        sorted[j + 1] = sorted[j];
        sorted[j] = temp;
        j = 0;
        cambios = false;
      }
    }
  }
  // Calcular el valor promedio de las posiciones más relevantes
  k = (numSampling + 1) / 2 - 1; // posicion medio del vector

  return smoothValue = ((sorted[k - 3] + sorted[k - 2] + sorted[k - 1] + sorted[k] + sorted[k + 1] + sorted[k + 2] + sorted[k + 3]) / 7.0);
}

void llamar_lecturaPir() {
  pirValue = digitalRead(pinPir);
  //Serial.println(F("leer PIRT"));

  if (pirValue != pirValueTemp) {
    if (pirValue == HIGH) {
      pirAlarm = true;
      pirDetect = true;
    }
    else {
      pirAlarm = false;
      pirDetect = true;
    }
  }
  pirValueTemp = pirValue;
}

void llamar_lecturaDHT() {
  dhtRead = true;
  //Serial.println(F("leer DHT"));
}

void llamar_EnviarMQTT() {
  enivarMQTT = true;
}

void llamar_lecturaUsonic() {
  usonicRead = true;
}

void llamar_led() {
  static uint8_t cont;
  cont = (cont + 1) % 3;
  digitalWrite(0, LOW);
  if (cont == 0) {
    digitalWrite(0, HIGH);
    timer.disable(timerId_led);
  }
}

void llamar_photocell() {
  enviarPhotocell = true;
}

void timer_acciones(bool estado) {
  if (estado == true) {
    timer.enable(timerId_pir);
    timer.enable(timerId_DHT);
    timer.enable(timerId_Usonic);
    timer.enable(timerId_sendMQTT);
  }
  else {
    timer.disable(timerId_pir);
    timer.disable(timerId_DHT);
    timer.disable(timerId_Usonic);
    timer.disable(timerId_sendMQTT);
  }
}

