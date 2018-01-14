
#ifndef _CONFIG
#define _CONFIG 1

typedef struct _Config {
  short Magic = sizeof(_Config);
  String SSID = "SSID";
  String Password = "password";
  float TargetTemp = 20.0;


  int ResetHours=24*7;
  bool CoolActivated=true;
  bool HeatActivated=true;
  double PidP=4.0;
  double PidI=0.5;
  double PidD=0.5;
  unsigned long CoolToHeatDelaySec=3600;
} Configuration;


Configuration myConfiguration;
Configuration tmpConfig;
bool _toBool(String in) {
  if (in.equalsIgnoreCase("ON")) return true;
  if (in.equalsIgnoreCase("TRUE")) return true;
  if (in.equalsIgnoreCase("1")) return true;
  return false;
}
void deserializeConfigData(JsonObject &json, Configuration &cfg) {
  if (json.containsKey("Magic"))  cfg.Magic = String((const char*) json["Magic"]).toInt();
  if (json.containsKey("SSID"))  cfg.SSID = (const char *)json["SSID"];
  if (json.containsKey("PASSWORD"))  cfg.Password = (const char *)json["PASSWORD"];
  if (json.containsKey("TargetTemp"))  cfg.TargetTemp = String((const char*) json["TargetTemp"]).toFloat();

 

  if (json.containsKey("ResetHours"))  cfg.ResetHours = String((const char*) json["ResetHours"]).toInt();
  if (json.containsKey("CoolActivated"))  cfg.CoolActivated = _toBool(String((const char*) json["CoolActivated"]));
  if (json.containsKey("HeatActivated"))  cfg.HeatActivated = _toBool(String((const char*) json["HeatActivated"]));
  if (json.containsKey("PidP"))  cfg.PidP = String((const char*) json["PidP"]).toFloat();
  if (json.containsKey("PidD"))  cfg.PidD = String((const char*) json["PidD"]).toFloat();
  if (json.containsKey("PidI"))  cfg.PidI = String((const char*) json["PidI"]).toFloat();  
  if (json.containsKey("CoolToHeatDelaySec"))  cfg.CoolToHeatDelaySec = String((const char*) json["CoolToHeatDelaySec"]).toInt();

}

void serializeConfigData(JsonObject &json, Configuration &cfg) {
  json["Magic"] = String(cfg.Magic);
  json["SSID"] = cfg.SSID;
  json["Password"] = cfg.Password;
  json["TargetTemp"] = String(cfg.TargetTemp);

  json["ResetHours"] = String(cfg.ResetHours);
  json["CoolActivated"] = String(cfg.CoolActivated);
  json["HeatActivated"] = String(cfg.HeatActivated);
  json["PidP"] = String(cfg.PidP);
  json["PidI"] = String(cfg.PidI);
  json["PidD"] = String(cfg.PidD);
  json["CoolToHeatDelaySec"] = String(cfg.CoolToHeatDelaySec);

}
#endif

