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
#include "pitches.h"
#include <Ticker.h>


/************************* WiFi Access Point *********************************/

#define WLAN_SSID        "050216"   //"11055"
#define WLAN_PASS        "?U9ZzarlsDorOA7GYau;@"   //"wux14n$wang%lu0?mima@!"

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

//Adafruit_MQTT_Publish pressure_sensor = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/pressure");

//Adafruit_MQTT_Publish temperature_sensor = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperature");

//Adafruit_MQTT_Publish indexTemperature_sensor = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/indexTemperature");

//Adafruit_MQTT_Publish humidity_sensor = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humidity");


/*************************** Sketch Code ************************************/

// Bug workaround for Arduino 1.6.6, it seems to need a function declaration
// for some reason (only affects ESP8266, likely an arduino-builder bug).
void MQTT_connect();
void verifyFingerprint();
void llamar_tono(void) ;
void llamar_tonoMarioBros(void);

Ticker ledMQTT;
Ticker ledPIR;
Ticker ledUsonic;
Ticker buzzerStop;
Ticker buzzer2Stop;
#define pinSonido  14
#define pinSonido2 13
#define pinLedMQTT 2
#define pinLed2  0
//#define pinLed0 0
#define pinLed1  12
bool buzzerDesactivar = true;
bool buzzer2Desactivar = true;
//int8_t contarBuzzerOff = 0;


// notes in the melody:
//int melody[] = {
//   NOTE_G3, NOTE_B3, NOTE_B3, NOTE_C4
//};
//  int noteDurations[] = {
//     4, 4, 4, 4
//  };
int melody[] = {
  NOTE_G6, NOTE_E7, NOTE_G7,
  NOTE_A7, 1
};
int noteDurations[] = {
   9, 9, 9,
  12, 12
};

int MarioBrosMelody[] = {
  NOTE_E7, NOTE_E7, 1, NOTE_E7
};
//Mario main them tempo
int MarioBrosTempo[] = {
  12, 12, 12, 12
};

void setup() {
  Serial.begin(115200);
  delay(200);

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


  pinMode(pinSonido, OUTPUT);
  pinMode(pinSonido2, OUTPUT);
  pinMode(pinLedMQTT, OUTPUT);
  pinMode(pinLed2, OUTPUT);
  pinMode(pinLed1, OUTPUT);
  
  digitalWrite(pinLed2, LOW);
  digitalWrite(pinLed1, LOW);
  digitalWrite(pinLedMQTT, LOW);

  Serial.println("tono 1");
  llamar_tono();
//  dingdong(14);
  delay(3000);
 Serial.println("tono 2");
  llamar_tonoMarioBros();
  delay(3000);
  
  Serial.println(F("Iniciando MQTT->"));
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
        ledPIR.attach(0.5, notificar_led1);
        buzzerStop.attach(1, tiempo_sin_buzzer); 
        if(buzzerDesactivar == false){
          llamar_tono();
          buzzerDesactivar = true;
        } //else contarBuzzerOff++;
        
      }
      else if (strcmp((char *)pir_sensor.lastread, "#353538") == 0) {
        Serial.println("variacion PIR de apagar");
      }
    }
    
    if(subscription == &photocell){
      Serial.println(F(".... MQTT led parpadear OK"));
      ledMQTT.attach(0.5, notificar_ledON);
    }

    if(subscription == &usonic_sensor){
      float usonicValue = atof((char *)usonic_sensor.lastread);  // convert to a number
      if(usonicValue<=2.80){
        Serial.print(F("valor usonic: "));
        Serial.println(usonicValue);
        ledUsonic.attach(0.25,notificar_led2);
        buzzer2Stop.attach(1, tiempo_sin_buzzer2); 
        if(buzzer2Desactivar == false){
          llamar_tonoMarioBros();
          buzzer2Desactivar = true;
        }
      }
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
  digitalWrite(pinLed2, LOW);
  digitalWrite(pinLed1, LOW);
  digitalWrite(pinLedMQTT, HIGH);
}

void llamar_tono(void) {
  Serial.println(F("notificar sonido"));
  for (int thisNote = 0; thisNote < sizeof(melody)/sizeof(int); thisNote++) {
    //Serial.print("this note: ");
    //Serial.println(thisNote);
    // to calculate the note duration, take one second
    // divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000 / noteDurations[thisNote];
    tone(pinSonido, melody[thisNote], noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(pinSonido);
  }
}

void llamar_tonoMarioBros(void) {
  Serial.println(F("notificar sonido Bros"));
  for (int thisNote = 0; thisNote < sizeof(MarioBrosMelody)/sizeof(int); thisNote++) {
    //Serial.print("this note: ");
    //Serial.println(thisNote);
    // to calculate the note duration, take one second
    // divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000 / MarioBrosTempo[thisNote];
    tone(pinSonido2, MarioBrosMelody[thisNote], noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(pinSonido2);
  }
}

void notificar_ledON(){
  static bool parpadear;
  static int contar;
  contar = (contar+1) % 4; 
  parpadear = !parpadear;
 // Serial.print("contar ");
 // Serial.print(contar);
 // Serial.print(" parpadear ");
  if(parpadear == true) {digitalWrite(pinLedMQTT, LOW); /*Serial.println(parpadear);*/}
  else {digitalWrite(pinLedMQTT, HIGH);/*Serial.println(parpadear);*/}
  if(contar == 0) ledMQTT.detach();
}

void notificar_led1(){
  static bool parpadear;
  static int contar;
  contar = (contar+1) % 12; 
  parpadear = !parpadear;
 // Serial.print("contar ");
 // Serial.print(contar);
 // Serial.print(" parpadear ");
  if(parpadear == true) {digitalWrite(pinLed1, LOW); /*Serial.println(parpadear);*/}
  else {digitalWrite(pinLed1, HIGH);/*Serial.println(parpadear);*/}
  if(contar == 0) ledPIR.detach();
}

void notificar_led2(){
  static bool parpadear;
  static int contar;
  contar = (contar+1) % 12; 
  parpadear = !parpadear;
 // Serial.print("contar ");
 // Serial.print(contar);
 // Serial.print(" parpadear ");
  if(parpadear == true) {digitalWrite(pinLed2, LOW); /*Serial.println(parpadear);*/}
  else {digitalWrite(pinLed2, HIGH);/*Serial.println(parpadear);*/}
  if(contar == 0) ledUsonic.detach();
}

void tiempo_sin_buzzer(){
  static int contarTiempo;
  contarTiempo = (contarTiempo +1) % 4;
  if(contarTiempo == 0){
    buzzerDesactivar = false;
    digitalWrite(pinLed1, LOW);
    buzzerStop.detach();
  }
}

void tiempo_sin_buzzer2(){
  static int contarTiempo;
  contarTiempo = (contarTiempo +1) % 4;
  if(contarTiempo == 0){
    buzzer2Desactivar = false;
    digitalWrite(pinLed2, LOW);
    buzzer2Stop.detach();
  }
}

void dingdong(int pin) {
    tone(pin, 622, 7.52508361204);
    delay(8.36120401338);
    tone(pin, 3135, 37.6254180602);
    delay(41.8060200669);
    tone(pin, 587, 2581.10367893);
    delay(2867.89297659);
    tone(pin, 739, 135.451505017);
    delay(150.501672241);
    noTone(pin);
}
