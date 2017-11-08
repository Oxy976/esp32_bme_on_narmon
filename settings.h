// Network settings
// --
const char* ssid = "....";
const char* password = "....";
// --

// Time settings
const char* ntpServerName = "time.nist.gov";
int TIMEZONE=3;

// narmon
//---
#define MAC "xxxxxxxxxx"
#define PASS "xxxxx"
#define USERNAME "......"
#define TOPIC "login/esp32/"
//--

//mqtt4narmon
char server[] = "narodmon.ru";
char authMethod[] = USERNAME;
char token[] = PASS;
char clientId[] = MAC;
char conntopic[] = TOPIC "status";

// Application settings
