/*
 MQTT IOT Example
 */

#include <SPI.h>
#include <WiFi.h>
const char* ssid = "....";
const char* password = "....";
 

#include <PubSubClient.h>




#define MAC "xxxxxxxxxx" 
#define PASS "xxxxx"
#define USERNAME "......" 
#define TOPIC "login/esp32/"


char server[] = "narodmon.ru";
char authMethod[] = USERNAME;
char token[] = PASS;
char clientId[] = MAC;
char conntopic[] = TOPIC "status";


#include <Wire.h>
#include "cactus_io_BME280_I2C.h"
// Create the BME280 object
BME280_I2C bme;              // I2C using default 0x77 
// or BME280_I2C bme(0x76);  // I2C using address 0x76
float pressure = 0.0;
float temp = 0.0;
float humidity = 0.0;

//sleep
#define GPIO_DEEP_SLEEP_DURATION     420  // sleep ... seconds and then wake up
RTC_DATA_ATTR static time_t last;        // remember last boot in RTC Memory



WiFiClient wifiClient;
PubSubClient client(server, 1883, wifiClient);

void setup()
{

  // Start the ethernet client, open up serial port for debugging
  Serial.begin(115200);
  setup_wifi();


 // init sensor
   if (!bme.begin()) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  Serial.println("BME280 sensor activated");

  bme.setTempCal(-1);  //correct data

 
//=====
    //sleep - wakeup
    struct timeval now;
    Serial.println("wakeup: start ESP32 loop \n");
    gettimeofday(&now, NULL);

     bme.readSensor();      //получили данные с датчика
     delay(1000);
     bme.readSensor();      //получили данные с датчика
     delay(10);
     // отправка на сервер
     gotTemp();
     delay(10);
     gotHumidity();
     delay(10);
     gotPressure();
     delay(10);

    // close mqtt-connection
    client.disconnect();
    // .. and go sleep
//    Serial.println("deep sleep (%lds since last reset, %lds since last boot)\n",now.tv_sec,now.tv_sec-last);
    Serial.print("last=");  Serial.println(last);
    Serial.print("go deep sleep (%lds since last reset, %lds since last boot)\n");     Serial.print(now.tv_sec); Serial.print("; "); Serial.println(now.tv_sec-last);
    last = now.tv_sec;
    Serial.print("last=");  Serial.println(last);
    esp_deep_sleep(1000000LL * GPIO_DEEP_SLEEP_DURATION);

 


}



void loop()
{
/* 
    //sleep - wakeup
    struct timeval now;
    Serial.println("wakeup: start ESP32 loop \n");
    gettimeofday(&now, NULL);

     bme.readSensor();      //получили данные с датчика
     // отправка на сервер
     gotTemp();
     delay(5);
     gotHumidity();
     delay(5);
     gotPressure();
     delay(5);

    // close mqtt-connection
    client.disconnect();
    // .. and go sleep
//    Serial.println("deep sleep (%lds since last reset, %lds since last boot)\n",now.tv_sec,now.tv_sec-last);
    Serial.print("last=");  Serial.println(last);
    Serial.print("go deep sleep (%lds since last reset, %lds since last boot)\n");     Serial.print(now.tv_sec); Serial.print("; "); Serial.println(now.tv_sec-last);
    last = now.tv_sec;
    Serial.print("last=");  Serial.println(last);
    esp_deep_sleep(1000000LL * GPIO_DEEP_SLEEP_DURATION);
*/
  //delay(5000);
}



// ================
// ================
void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

// ==публикация
void doPublish(String id, String value) {
// если не подключен, то подключаемся. Висит пока не подключится!!
  if (!!!client.connected()) {
     Serial.print("Reconnecting client to "); Serial.println(server);
     while (!!!client.connect(clientId, authMethod, token, conntopic,0,0,"online")) {
        Serial.print(".");
        delay(500);
     }
     Serial.print("connected with: "); Serial.print(clientId); Serial.print(authMethod); Serial.print(token); 
     Serial.println();
  }

  String topic = TOPIC;
  String payload = value ;
 // String topic += id;
  topic.concat(id);
  Serial.print("Publishing on: "); Serial.println(topic);
  Serial.print("Publishing payload: "); Serial.println(payload);
  if (client.publish(topic.c_str(), (char*) payload.c_str())) {
    Serial.println("Publish ok");
  } else {
    Serial.println("Publish failed");
  }
}
//==


// получаем данные - gotXxxx и публикуем
/*  слизанный кусок кода  - получаем данные
void gotReading(uint8_t *adv_data) {
  // This data is from the test ESP32 beacon
  uint16_t wakeupCount = (int16_t)((adv_data[11] << 8) | adv_data[10]);
  Serial.printf("My BLE device wakeup count=%i\n", wakeupCount);
  doPublish("wakeup", String(wakeupCount));
}

void gotTempoReading(uint8_t *adv_data) {
  // this data is from the "Tempo" device sends which send the temperature in a two byte field
  float temp = ((int16_t)((adv_data[27] << 8) | adv_data[26])) / 10.0;
  Serial.printf("My Tempo temp=%0.1f\n", temp);
  doPublish("temp", String(temp, 1));
}
*/

void gotTemp() {
     //
    float  temp = bme.getTemperature_C();
    //Serial.print(temp); Serial.print("*C  \t");
    Serial.printf("My  temp=%0.1f\n", temp);
    doPublish("t0", String(temp, 1));
}

void gotHumidity() {
     //
    float  humidity = bme.getHumidity();
    //Serial.print(humidity); Serial.print("H  \t");
    Serial.printf("My  humidity=%0.1f\n", humidity);
    doPublish("h0", String(humidity, 1));
}

void gotPressure() {
     //
    float  pressure = (bme.getPressure_MB() * 0.7500638);
    //Serial.print(pressure); Serial.print("p  \t");
    Serial.printf("My pressure=%0.1f\n", pressure);
    doPublish("p0", String(pressure, 1));
}




