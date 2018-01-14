
#include <Adafruit_ST7735.h>

#include <Ticker.h>

#include <ArduinoJson.h>

#include "DallasTemperature.h"
#include "DoubleResetDetector.h"
#include "Config.h"


#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPUpdateServer.h>
#include <DNSServer.h>
//#include <ESP8266mDNS.h>
#include <FS.h>
#include <Wire.h>

#include <PID_v1.h>

#define DEBUG 1

#define BUZZ_OFF 0
#define BUZZ_ALARM 4
#define BUZZ_NACK  2
#define BUZZ_ACK  1
#define BUZZ_WARN 3

#define DATA 0
#define INFO 1
#define WARN 2
#define ALARM 3



#define min(a,b) (((a)<(b)) ? (a) : (b))
#define max(a,b) (((a)>(b)) ? (a) : (b))

bool alarmActivated=false;

#define CFGFILE "FERMENTCONTROL"
bool isDebugEnabled() {
#ifdef DEBUG
  return true;
#endif // DEBUG
  return false;
}
// generic serial output
template <typename T> void SerialOut(const T aValue, bool newLine = true) {
  if (!isDebugEnabled())
    return;
  Serial.print(aValue);
  if (newLine)
    Serial.print("\n");
}
// ------------------- DoubleReset ------------------------- //
// Number of seconds after reset during which a
// subseqent reset will be considered a double reset.
#define DRD_TIMEOUT 1

// RTC Memory Address for the DoubleResetDetector to use
#define DRD_ADDRESS 0
DoubleResetDetector drd(DRD_TIMEOUT, DRD_ADDRESS);


void printStatus(String);
// --------------- Config ------------------------------- //



std::shared_ptr<char> readConfigFile() {
  SerialOut(F("mounting FS..."), false);
  if (SPIFFS.begin()) {
    SerialOut(F(" mounted!"));
    if (SPIFFS.exists(CFGFILE)) {
      // file exists, reading and loading
      SerialOut(F("reading config file"));
      File configFile = SPIFFS.open(CFGFILE, "r");
      if (configFile) {
        SerialOut(F("opened config file"));
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        char *buf = new char[size];
        configFile.readBytes(buf, size);
        std::shared_ptr<char> ptr(buf);
        return ptr;
      }
    }
  }
  return NULL;
}


bool readConfig() {

  std::shared_ptr<char> ptr = readConfigFile();
  if (ptr != NULL) {
    DynamicJsonBuffer jsonBuffer;
    JsonObject &json = jsonBuffer.parseObject((const char *)ptr.get());

    if (json.success()) {
      deserializeConfigData(json, myConfiguration);

      SerialOut(F("parsed config:"));
      if (isDebugEnabled)
        json.printTo(Serial);
      if (myConfiguration.Magic != sizeof(Configuration)) {
        SerialOut(F(".. is NOT valid:"));
        myConfiguration.Magic = sizeof(Configuration);
        writeConfig();

      }
      return true;
    } else {
      SerialOut(F("ERROR: failed to load json config"));
      writeConfig();
      return false;
    }
  }
}


void writeConfig() {
  // if SPIFFS is not usable
  if (!SPIFFS.begin() || !SPIFFS.exists(CFGFILE) ||
      !SPIFFS.open(CFGFILE, "w")) {
    SerialOut(F("\nneed to format SPIFFS: "), false);
    SPIFFS.end();
    SPIFFS.begin();
    SerialOut(SPIFFS.format());
  }

  DynamicJsonBuffer jsonBuffer;
  JsonObject &json = jsonBuffer.createObject();

  serializeConfigData(json, myConfiguration);

  File configFile = SPIFFS.open(CFGFILE, "w+");
  if (!configFile) {
    SerialOut(F("failed to open config file for writing"), true);
  } else
    SerialOut(F("saved successfully"), true);

  if (isDebugEnabled)
    json.printTo(Serial);

  json.printTo(configFile);
  configFile.close();
  SPIFFS.end();

}
// --------------- Relais ------------------------------------//
#define REL_COOLER D0
#define REL_HEATER D7


void initRelais() {
  pinMode(REL_COOLER , OUTPUT);
  digitalWrite(REL_COOLER , HIGH);
  pinMode(REL_HEATER , OUTPUT);
  digitalWrite(REL_HEATER , HIGH);

}
bool setRelais(int relnum, bool val) {
  bool oldVal = getRelais(relnum);
  if (val == oldVal) return false;

  digitalWrite(relnum, !val);
  return true;
}
bool getRelais(int relnum) {

  bool oldVal = digitalRead(relnum);
  return  !oldVal;
}
bool AllRelaisOff() {
  return !(getRelais(REL_COOLER) || getRelais(REL_HEATER) );
}

// ------------------ Events ----------------------------------//
String lastEvent;



void logEvent(int sev, const char *txt) {
  String msg = "";
  if (sev == INFO) msg += String("INFO ");
  if (sev == WARN) msg += String("WARN ");
  if (sev == ALARM) msg += String("ALARM ");
  if (sev == DATA) msg += String("DATA ");
  msg += txt;
  if (sev != DATA)  lastEvent = msg;
  SerialOut("Event: ", false);
  SerialOut(msg);
  
  if (sev != DATA) printStatus(msg);
}


// --------------- Buzzer -------------------------------------//
Ticker buzzerTicker;


int currentBuzz = BUZZ_OFF;
bool buzz(int buzzType) {

  buzzerTicker.detach();
  switch (buzzType) {

    case BUZZ_OFF:
      buzzerSet(false);
      break;
    case BUZZ_ALARM:
    case BUZZ_WARN:
      currentBuzz = buzzType;
      buzzerToggle(buzzType);
      break;
    case BUZZ_NACK:
      buzzerSet(true);
      buzzerTicker.once_ms(1000, buzzerSet, false);
      break;
    case BUZZ_ACK:
      buzzerSet(true);
      buzzerTicker.once_ms(100, buzzerSet, false);
      break;

  }
  return true;
}

void buzzerSet(bool b) {
  digitalWrite(D8, b);
}
void buzzerToggle(int buzzType) {
  bool s = digitalRead(D8);
  digitalWrite(D8, !s);
  switch (buzzType) {
    case BUZZ_ALARM:
      buzzerTicker.once_ms(1000, buzzerToggle, buzzType);
      break;
    case BUZZ_WARN:
      if (s) buzzerTicker.once_ms(5000, buzzerToggle, buzzType) ;
      else buzzerTicker.once_ms(10, buzzerToggle, buzzType);
      break;
  }
}
void initBuzzer() {
  pinMode(D8, OUTPUT);
  digitalWrite(D8 , LOW);
}

// --------------- Temperature ------------------------------- //
float OuterTemperature=20.0, InnerTemperature=20.0;


OneWire oneWireInner(D5);
DallasTemperature DS18B20Inner(&oneWireInner);
OneWire oneWireOuter(D1);
DallasTemperature DS18B20Outer(&oneWireOuter);



Ticker TemperatureReading;
#define RESOLUTION 12
DeviceAddress tempOuterDeviceAddress;
DeviceAddress tempInnerDeviceAddress;
void initTempInner()
{

  // workaround for DS not enough power to boot
  pinMode(D5, OUTPUT);
  digitalWrite(D5, LOW);
  delay(100);
  // digitalWrite(ONE_WIRE_BUS, HIGH);
  // delay(500);
  // oneWire.reset();
  // Start up the DS18B20
  DS18B20Inner.begin();
  DS18B20Inner.setWaitForConversion(false);
  DS18B20Inner.getAddress(tempInnerDeviceAddress, 0);
  DS18B20Inner.setResolution(tempInnerDeviceAddress, RESOLUTION);

}
void initTempOuter()
{

  // workaround for DS not enough power to boot
  pinMode(D1, OUTPUT);
  digitalWrite(D1, LOW);
  delay(100);
  // digitalWrite(ONE_WIRE_BUS, HIGH);
  // delay(500);
  // oneWire.reset();
  // Start up the DS18B20
  DS18B20Outer.begin();
  DS18B20Outer.setWaitForConversion(false);
  DS18B20Outer.getAddress(tempOuterDeviceAddress, 0);
  DS18B20Outer.setResolution(tempOuterDeviceAddress, RESOLUTION);
  
}
long DSreqTime;
bool tempInit=false;
void initTemp() {
  initTempInner();
  initTempOuter();
    DS18B20Inner.requestTemperatures();
    DS18B20Outer.requestTemperatures();
    DSreqTime = millis();
}


float accIT=0.0;
float accOT=0.0;
int accCtr=0;
void handleTemp()
{
  // we need to wait for DS18b20 to finish conversion

  if (millis() - DSreqTime >= 1000)
  {
    accIT += DS18B20Inner.getTempCByIndex(0);
    accOT += DS18B20Outer.getTempCByIndex(0);
    
    DS18B20Inner.requestTemperatures();
    DS18B20Outer.requestTemperatures();
    DSreqTime = millis();
    if (++accCtr==4) {
      InnerTemperature=accIT/accCtr;
      OuterTemperature=accOT/accCtr;
      accCtr=0;
      accIT=0.0;
      accOT=0.0;
      tempInit=true;      
    }

  }

}
//Define Variables we'll be connecting to
double PIDSetpoint, PIDInput, PIDOutput;

unsigned long lastPIDSample=0;
//Specify the links and initial tuning parameters
PID myPID(&PIDInput, &PIDOutput, &PIDSetpoint,2,5,1, DIRECT);
void initPID() {
    PIDSetpoint=(double)myConfiguration.TargetTemp;
    
    myPID.SetMode(MANUAL);
    myPID.SetSampleTime(1000); 
    myPID.SetOutputLimits(-10.0,+10.0);
    PIDOutput=0.0;
    myPID.SetTunings(myConfiguration.PidP, myConfiguration.PidI, myConfiguration.PidD, P_ON_E);
    myPID.SetMode(AUTOMATIC);

    lastPIDSample=millis()-60*1000*22;
}
bool isPIDInit=false;

void controlTemp() {
  if (!tempInit) return ;

  PIDSetpoint=myConfiguration.TargetTemp;
  PIDInput=InnerTemperature;
 
  if (!isPIDInit) {
    isPIDInit=true;
    initPID();
    myPID.Compute();    
  }
  unsigned long now = millis();
  if (now-lastPIDSample < 60*1000*24)
    return;
  lastPIDSample=now;
   myPID.Compute();
 
    // limit output to 0 for only-cool or only-heat configs
    if (PIDOutput <0 && myConfiguration.CoolActivated==false  ) {
       PIDOutput=0.0;
    } else if (PIDOutput >0 && myConfiguration.HeatActivated==false  )  {
       PIDOutput=0.0;
    } 
    
     double pidValue=abs(PIDOutput);

     // avoid switching cooler on/off too  often     
     if ((pidValue<1)&&(PIDOutput<0) ) pidValue=0;
    
     long time=(long)(pidValue*60*1000*2);


   
    if (PIDOutput <0 && myConfiguration.CoolActivated==true && Allowed2Cool() ) {
       CoolerOnFor(time);
    } else if (PIDOutput >0 && myConfiguration.HeatActivated==true && Allowed2Heat() )  {
       HeaterOnFor(time);
    } else {
      CoolerHeaterOff();
    }
    

    
  
}
unsigned long lastCoolTime=0;
unsigned long lastHeatTime=0;
bool Allowed2Cool() {
  if (lastHeatTime==0) return true;
  if (millis()-lastHeatTime > 1000*myConfiguration.CoolToHeatDelaySec)
    return true;
  return false;
}
bool Allowed2Heat() {
  if (lastCoolTime==0) return true;
  if (millis()-lastCoolTime > 1000*myConfiguration.CoolToHeatDelaySec)
    return true;
  return false;
}

unsigned long heaterOff=0;
void HeaterOnFor(unsigned long msec) {
  heaterOff=millis()+msec;
  if (msec !=0) {
       setRelais(REL_COOLER, false);
        if (setRelais(REL_HEATER, true)) {
          logEvent(INFO, "Heater on");
        }
  }
}
unsigned long coolerOff=0;
void CoolerOnFor(unsigned long msec) {
  coolerOff=millis()+msec;
  if (msec!=0) {
      setRelais(REL_HEATER, false);
        if (setRelais(REL_COOLER, true)) {
          logEvent(INFO, "Cooler on");
        } 
  }
}
void CoolerHeaterOff() {
  CoolerOnFor(0);
  HeaterOnFor(0);
}
void HeaterCoolerControl() {
  unsigned long now = millis();
  if (now >= heaterOff) {
      if (setRelais(REL_HEATER, false)) {
          lastHeatTime=now;
          logEvent(INFO, "Heater off");
        }
  };
  if (now >= coolerOff) {
     if (setRelais(REL_COOLER, false)) {
          lastCoolTime=now;
          logEvent(INFO, "Cooler off");
        }
  }
  
}


// --------------- Webserver  ------------------------------- //

ESP8266WebServer server ( 80 );
ESP8266HTTPUpdateServer httpUpdater;

bool STAMode = true;

/** Is this an IP? */
boolean isIp(String str) {
  for (int i = 0; i < str.length(); i++) {
    int c = str.charAt(i);
    if (c != '.' && (c < '0' || c > '9')) {
      return false;
    }
  }
  return true;
}
String myHostname = String("192.168.4.1");
boolean captivePortal() {
  if (STAMode) return false;
  if (!isIp(server.hostHeader()) && server.hostHeader() != (myHostname)) {
    SerialOut("Redirect to captive Portal");
    server.sendHeader("Location", ("http://") + myHostname, true);
    server.setContentLength(0);
    server.send ( 302, "text/plain", ""); // Empty content inhibits Content-length header so we have to close the socket ourselves.
    //    server.client().stop(); // Stop is needed because we sent no content length
    return true;
  }
  return false;
}

void handleRoot() {
  if (captivePortal()) return;
  char temp[1000];
  int sec = millis() / 1000;
  int min = sec / 60;
  int hr = min / 60;

  snprintf ( temp, sizeof(temp),

             "<html>\
  <head>\
    <title>FermentControl</title>\
    <style>\
      body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }\
    </style>\
  </head>\
  <body>\
    <h1>FermentControl</h1>\
    <p>Uptime: %02d:%02d:%02d</p>\
<form method=\"get\" action=\"setConfig\"><label>SSID</label><input id=\"SSID\" name=\"SSID\" length=32 placeholder=\"SSID\"><label>Password</label><input id=\"Password\" name=\"Password\" length=64 placeholder=\"password\"> \
<button class=\"btn\" type=\"submit\">save</button></form> \
    <p><a href='getConfig'>Read Configuration</a></p>\
    <p><a href='getStatus'>Read Status</a></p>\
    <p><a href='setConfig'>Set Configuration</a></p>\
    <p><a href='reset'>Reset</a></p>\
    <p><a href='update'>Firmware Update</a></p>\
  </body>\
</html>",

             hr, min % 60, sec % 60
           );
  server.send ( 200, "text/html", temp );
}

void handleNotFound() {
  if (captivePortal()) { // If caprive portal redirect instead of displaying the error page.
    return;
  }
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += ( server.method() == HTTP_GET ) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";

  for ( uint8_t i = 0; i < server.args(); i++ ) {
    message += " " + server.argName ( i ) + ": " + server.arg ( i ) + "\n";
  }

  server.send ( 404, "text/plain", message );
}
void handleGetConfig() {
  DynamicJsonBuffer jsonBuffer;
  JsonObject &json = jsonBuffer.createObject();


  serializeConfigData(json, myConfiguration);
  json["Password"] = "********";

  WiFiClient client = server.client();
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: application/json");
  client.println("Connection: close");
  client.println();

  json.prettyPrintTo(client);


}
unsigned long millisLastStartup = 0;

void handleGetStatus() {
  DynamicJsonBuffer jsonBuffer;
  JsonObject &json = jsonBuffer.createObject();
  char *ON = "ON";
  char *OFF = "OFF";


  json["OuterTemperature"] = String(OuterTemperature);
  json["InnerTemperature"] = String(InnerTemperature);
  json["TargetTemperature"] = String(myConfiguration.TargetTemp);
  json["PIDOutput"] = String(PIDOutput);

  json["Cooling"] = getRelais(REL_COOLER) ? ON : OFF;
  json["Heating"] = getRelais(REL_HEATER) ? ON : OFF;
  json["LastEvent"] = lastEvent;
  unsigned long mins=(millis()-millisLastStartup)/1000/60;
  String alive=String(mins/60); 
  alive += ":";
  alive += mins % 60;
  json["Alive"] = alive;

  WiFiClient client = server.client();
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: application/json");
  client.println("Connection: close");
  client.println();

  json.prettyPrintTo(client);


}

void handleSetConfig() {

  StaticJsonBuffer<400> newBuffer;
  JsonObject& newjson = newBuffer.parseObject(server.arg("plain"));

  deserializeConfigData(newjson, myConfiguration);
  writeConfig();
  alarmActivated=false;   // wait until temp meets target -> then alarm is on again

  initPID();
  CoolerHeaterOff();
  lastCoolTime=0;
  lastHeatTime=0;
  server.send ( 200, "text/json", "{ \"success\":true}" );



}

void handleSetConfigGet() {

  DynamicJsonBuffer newBuffer;
  JsonObject& newjson = newBuffer.createObject();
  for (int i = 0; i < server.args(); i++) {
    String v = server.arg(i);
    String n = server.argName(i);
    newjson[n] = v;
  }

  deserializeConfigData(newjson, myConfiguration);
  writeConfig();
  alarmActivated=false;   // wait until temp meets target -> then alarm is on again
  initPID();
  CoolerHeaterOff();
  lastCoolTime=0;
  lastHeatTime=0;
  server.send ( 200, "text/json", "{ \"success\":true}" );



}

bool toBool(String in) {
  if (in.equalsIgnoreCase("ON")) return true;
  if (in.equalsIgnoreCase("TRUE")) return true;
  if (in.equalsIgnoreCase("1")) return true;
  return false;
}

void handleSetStatus() {
  for (int i = 0; i < server.args(); i++) {
    String v = server.arg(i);
    String n = server.argName(i);
    int r = 0;

    if (n == "Cooling") r = REL_COOLER;
    if (n == "Heating") r = REL_HEATER;

    if (r != 0)
    {
      bool b = toBool(v);
      setRelais(r, b);
      String txt = n + String(b ? " On" : " Off");
      logEvent(INFO, txt.c_str());
    }
  }


  server.send ( 200, "text/json", "{ \"success\":true}" );



}
unsigned long  resetTime=0;
void handleReset() {
    resetTime=millis()+20000;
    server.send ( 200, "text/json", "{\"resetInSeconds\":20}" );
}

void initWebServer() {
  server.on ( "/", handleRoot );
  // server.on ( "/test.svg", drawGraph );
  server.on ( "/inline", []() {
    server.send ( 200, "text/plain", "this works as well" );
  } );
  server.onNotFound ( handleNotFound );
  server.on ( "/getConfig", handleGetConfig );
  server.on ( "/setConfig", HTTP_POST, handleSetConfig );
  server.on ( "/setConfig", HTTP_GET, handleSetConfigGet );
  server.on ( "/getStatus", handleGetStatus );
  server.on ( "/setStatus", HTTP_GET, handleSetStatus );
  server.on ( "/reset", HTTP_GET, handleReset );

  httpUpdater.setup(&server);
  server.begin();
  SerialOut ( "HTTP server started" );

}

// --------------- Setup  ------------------------------- //
DNSServer dnsServer;

void initWIFI(bool startSSID) {
  int connRes = 0;
  SerialOut("");
  WiFi.setOutputPower(20.5);
  if (!startSSID) {
    if (WiFi.status() == WL_CONNECTED) return;
    const char *ssid = myConfiguration.SSID.c_str();
    const char *password = myConfiguration.Password.c_str();
    SerialOut (F("SSID is:" ), false);
    SerialOut (ssid, false);
    SerialOut (F("Pwd is:" ), false);
    SerialOut (password, true);
    WiFi.mode(WIFI_STA);
    WiFi.begin (ssid, password );
    // Wait for connection
    connRes = WiFi.waitForConnectResult();
    SerialOut ("Connresult:" , false);
    SerialOut (connRes );

    if ( connRes != WL_CONNECTED ) {
      SerialOut ("Retry..." );
      WiFi.reconnect();
      connRes = WiFi.waitForConnectResult();
      SerialOut ("Connresult:" , false);
      SerialOut (connRes );

    }
    SerialOut ( "Connected to " , false);
    SerialOut ( WiFi.SSID() );
    SerialOut ( "IP address: ", false );
    SerialOut ( WiFi.localIP() );
    dnsServer.stop();
    STAMode = true;
  } else {
    buzz(BUZZ_NACK);
    SerialOut ("Try setup own SSID" );
    if (WiFi.getAutoConnect() == 0)WiFi.setAutoConnect(1);
    //dnsServer.reset(new DNSServer());

    WiFi.softAP("FermentControl");

    IPAddress myIP = WiFi.softAPIP();
    SerialOut("AP IP address: ", false);
    SerialOut(myIP);

    connRes = WiFi.waitForConnectResult();
    dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
    dnsServer.start(53, "*", WiFi.softAPIP());

    SerialOut ("Connresult:" , false);
    SerialOut (connRes );
    STAMode = false;

  }

  SerialOut ( "" );

  SerialOut ( "Status: " , false);
  SerialOut ( WiFi.status() );

}
#define TFT_CS     D6
#define TFT_RST    -1  // you can also connect this to the Arduino reset
                      // in which case, set this #define pin to -1!
#define TFT_DC     D2

// Option 1 (recommended): must use the hardware SPI pins
// (for UNO thats sclk = 13 and sid = 11) and pin 10 must be
// an output. This is much faster - also required if you want
// to use the microSD card (see the image drawing example)
//Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);

// Option 2: use any pins but a little slower!
#define TFT_SCLK D4   // set these to be whatever pins you like!
#define TFT_MOSI D3   // set these to be whatever pins you like!
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

bool cleanDisplay=true;

void tftInit() {
  tft.initR(INITR_BLACKTAB);  
  tft.setRotation(3);
  tft.setTextWrap(false);
  tft.setTextSize(2);
  tft.fillScreen(ST7735_BLACK);
  tft.drawFastHLine(0, 41, tft.width(), ST7735_MAGENTA);
}
void printStatus(String txt) {
   tft.setTextSize(1);
   tft.setTextColor(ST7735_BLACK);
     tft.fillRect(0, 38, tft.width(),9, ST7735_MAGENTA);
     tft.setCursor(10,39);
     tft.println(txt);
}
void printTempValues(float out, float in, float tgt) {
    
  tft.fillRect(0,0, 180,40,ST7735_BLACK);
  tft.setTextSize(2);

  tft.setTextColor(ST7735_GREEN);
  tft.setCursor(110, 0);
  tft.println("Tgt");
  tft.setCursor(110, 20);
  printfloat(tgt);

  tft.setTextColor(ST7735_RED);
  tft.setCursor(0,0);
  tft.println("Out");
  tft.setCursor(0, 20);
  printfloat(out);

  tft.setTextColor(ST7735_YELLOW);
  tft.setCursor(55, 0);
  tft.println("In");
  tft.setCursor(55, 20);
  printfloat(in);


 
}


void printfloat(float x) {
  int v=(int)x;
  tft.print(v);
  tft.print(".");
  tft.print(abs((int)(x*10) % 10));
}


void printActivity() {
   int coolColor=myConfiguration.CoolActivated ? ST7735_BLUE : ST7735_BLACK;
   tft.fillRect(0, 48, tft.width()/2,79, coolColor);
   int heatColor=myConfiguration.HeatActivated ? ST7735_RED : ST7735_BLACK;
   tft.fillRect(80, 48, tft.width()/2,79, heatColor);
   if (getRelais(REL_COOLER)) {
     tft.fillRect(20,68,40,40,ST7735_WHITE);
   }
   if (getRelais(REL_HEATER)) {
     tft.fillRect(100,68,40,40,ST7735_WHITE);
   }

}

unsigned long millisLastDisplayUpdate=0;

void handleDisplay() {
  if (millis() -millisLastDisplayUpdate <5000)
    return;

  millisLastDisplayUpdate=millis();

  printTempValues(OuterTemperature, InnerTemperature, myConfiguration.TargetTemp);

    printActivity();

    
}


unsigned long millisLastWifiCheck = 0;

unsigned long millisNextReset=0;
void setup() {
  initRelais();
  tftInit();
  printStatus("initializing");

  bool _dblreset = drd.detectDoubleReset();
  Serial.begin(115200);
  if (_dblreset) SerialOut(F("DoubleReset detected"));

  readConfig() ;


  initBuzzer();

  initTemp();

  initWIFI(_dblreset);
  initWebServer();


  buzz(BUZZ_ACK);
  logEvent(INFO, "FermentControl started");
  millisLastWifiCheck = millis();
  millisLastStartup = millis();
  millisNextReset=myConfiguration.ResetHours * 60 * 60 * 1000;
}

void loop() {
  int currentMillis = millis();
  handleTemp();
  controlTemp();
  HeaterCoolerControl();
  server.handleClient();
  if (currentMillis - millisLastWifiCheck > 10 * 60 * 1000) {
    millisLastWifiCheck = currentMillis;
    initWIFI(false);
  }




  if ((myConfiguration.ResetHours>0)&&(currentMillis - millisLastStartup > millisNextReset)) {
    if (AllRelaisOff())
      ESP.reset();
  }
  if ((resetTime>0) && (currentMillis >resetTime)) {
  
      ESP.reset();
  }

  handleDisplay();


  if (!STAMode)
    dnsServer.processNextRequest();

  drd.loop();
}
