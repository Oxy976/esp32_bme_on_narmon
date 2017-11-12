// Network settings
// --
const char* ssid = "xxx";
const char* password = "xxx";
// --

// Time settings
const char* ntpServerName = "pool.ntp.org";
const int TIMEZONE=3;

// narmon

#define SRV   "narodmon.ru"
#define MAC  "xxx"
#define PASS  "xxx"
#define USERNAME  "xxx"
#define TOPIC  "login/xxx"

//--

//mqtt4narmon
char server[] = SRV ;
char authMethod[] = USERNAME;
char token[] = PASS;
char clientId[] = MAC;
char conntopic[] = TOPIC "status";

// LED
const int LED_BUILTIN=2;  //Onboard LED Pin


// Application settings
