// Network settings
// --
const char* ssid = "....";
const char* password = "....";
// --

// Time settings
const char* ntpServerName = "time.nist.gov";
const int TIMEZONE=3;

// narmon
//---
/*#define MAC "xxxxxxxxxx"
#define PASS "xxxxx"
#define USERNAME "......"
#define TOPIC "login/esp32/"
*/
const char* SRV = "narodmon.ru";
const char* MAC = "....";
const char* PASS = "....";
const char* USERNAME = "....";
const char* TOPIC = "login/esp32/";

//--

//mqtt4narmon
char server[] = SRV ;
char authMethod[] = USERNAME;
char token[] = PASS;
char clientId[] = MAC;
char conntopic[] = TOPIC "status";

// Application settings
