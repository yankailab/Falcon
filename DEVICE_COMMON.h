
HardwareSerial* g_pRFSerial;

//Common classes
VehicleLink g_VLink;
config_t config;

//Program
int g_counter = 0;

//switches
bool g_bPrintIMU = false;
bool g_bBootSuccess = true;
bool g_bHostConnected = false;

//General
uint8_t g_opeMode = /*OPE_SERIAL_BRIDGE;//*/OPE_RC_BRIDGE;// OPE_MANUAL;
