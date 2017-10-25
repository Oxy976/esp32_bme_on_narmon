/*
transmiting data from wemos r32 (esp32) with sensor BME280 to narodmon.ru across mqtt
*/

#include <SPI.h>
#include <WiFi.h>
// --
const char* ssid = "....";
const char* password = "....";
// --

#include <WiFiUdp.h>
IPAddress timeServerIP;
const char* ntpServerName = "time.nist.gov";
int TIMEZONE=3;
const int NTP_PACKET_SIZE = 48;
byte packetBuffer[ NTP_PACKET_SIZE];
unsigned long ntp_time = 0;
WiFiUDP udp;

#include <PubSubClient.h>
//---
#define MAC "xxxxxxxxxx"
#define PASS "xxxxx"
#define USERNAME "......"
#define TOPIC "login/esp32/"
//--

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
//  delay(300);  //пишут, что надо время на запуск...
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
    gettimeofday(&now, NULL);  //получить приблизительное время от системы. для индикации сколько спали.

     GetNTP();    //получили время, записано в ntp_time, в seril отобразилось. можно использовать где-нибудь еще

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
}



// ================
// ==WIFI ================
void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  int i = 0;
  while (WiFi.status() != WL_CONNECTED && i < 50) {
    delay(500);
    i++;
    Serial.print(".");
  }
  // если до сих пор нет подключения - перегрузить.
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi Connection failed, restart");
    WiFi.disconnect();
    ESP.restart();
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

// ==MQTT ==публикация
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

// ==== NTP===
/**
 * Посылаем и парсим запрос к NTP серверу
 */
bool GetNTP(void) {
  WiFi.hostByName(ntpServerName, timeServerIP);
  sendNTPpacket(timeServerIP);
  delay(1000);

  int cb = udp.parsePacket();
  if (!cb) {
    Serial.println("No packet yet");
    return false;
  }
  else {
    Serial.print("packet received, length=");
    Serial.println(cb);
// Читаем пакет в буфер
    udp.read(packetBuffer, NTP_PACKET_SIZE);
// 4 байта начиная с 40-го сождержат таймстамп времени - число секунд
// от 01.01.1900
    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
// Конвертируем два слова в переменную long
    unsigned long secsSince1900 = highWord << 16 | lowWord;
// Конвертируем в UNIX-таймстамп (число секунд от 01.01.1970
    const unsigned long seventyYears = 2208988800UL;
    unsigned long epoch = secsSince1900 - seventyYears;
// Делаем поправку на местную тайм-зону
    ntp_time = epoch + TIMEZONE*3600;
    Serial.print("Unix time = ");
    Serial.println(ntp_time);
// и в привычном виде:
   uint16_t s = ( ntp_time )%60;
   uint16_t m = ( ntp_time/60 )%60;
   uint16_t h = ( ntp_time/3600 )%24;
   Serial.print("Time: ");
   Serial.print(h);
   Serial.print(":");
   Serial.print(m);
   Serial.print(":");
   Serial.println(s);
   //---

  }
  return true;
}

/**
 * Посылаем запрос NTP серверу на заданный адрес
 */
unsigned long sendNTPpacket(IPAddress& address)
{
  Serial.println("sending NTP packet...");
// Очистка буфера в 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
// Формируем строку зыпроса NTP сервера
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
// Посылаем запрос на NTP сервер (123 порт)
  udp.beginPacket(address, 123);
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
}
