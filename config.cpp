#include "Arduino.h"
#include "config.h"

bool config::save() {
  DynamicJsonBuffer jsonBuffer;
  JsonObject& json = jsonBuffer.createObject();  

  json["client_ssid"] = config::client_ssid;
  json["client_pass"] = config::client_pass;

  json["softap_ssid"] = config::softap_ssid;
  json["softap_pass"] = config::softap_pass;

  json["admin_login"] = config::admin_login;
  json["admin_pass"]  = config::admin_pass;

  json["mqtt_server"] = config::mqtt_server;
  json["mqtt_login"]  = config::mqtt_login;
  json["mqtt_pass"]   = config::mqtt_pass;
  json["mqtt_path"]   = config::mqtt_path;

  json["narodmon_id"] = config::narodmon_id;

  File configFile = SPIFFS.open(config::file, "w");
  if (!configFile) return false;
  json.printTo(configFile);
  configFile.close();

  return true;
}

bool config::save(String apiSave) {
  StaticJsonBuffer<1024> jsonBuffer;
  JsonObject& json = jsonBuffer.parseObject(apiSave);

  if (!json.success()) return false;

  if (json.containsKey("client_ssid")) config::client_ssid = config::toString(json["client_ssid"]);
  if (json.containsKey("client_pass")) config::client_pass = config::toString(json["client_pass"]);

  if (json.containsKey("softap_ssid")) config::softap_ssid = config::toString(json["softap_ssid"]);
  if (json.containsKey("softap_pass")) config::softap_pass = config::toString(json["softap_pass"]);

  if (json.containsKey("admin_login")) config::admin_login = config::toString(json["admin_login"]);
  if (json.containsKey("admin_pass"))  config::admin_pass  = config::toString(json["admin_pass"]);

  if (json.containsKey("mqtt_server")) config::mqtt_server = config::toString(json["mqtt_server"]);
  if (json.containsKey("mqtt_login"))  config::mqtt_login  = config::toString(json["mqtt_login"]);
  if (json.containsKey("mqtt_pass"))   config::mqtt_pass   = config::toString(json["mqtt_pass"]);
  if (json.containsKey("mqtt_path"))   config::mqtt_path   = config::toString(json["mqtt_path"]);

  if (json.containsKey("narodmon_id")) config::narodmon_id = config::toString(json["narodmon_id"]);

  return config::save();
}

bool config::load() {
  File configFile = SPIFFS.open(config::file, "r");
  if (!configFile) return false;

  size_t size = configFile.size();
  if (size > 1024) return false;

  std::unique_ptr<char[]> buf(new char[size]);
  configFile.readBytes(buf.get(), size);

  DynamicJsonBuffer jsonBuffer;
  JsonObject& json = jsonBuffer.parseObject(buf.get());

  configFile.close();

  if (!json.success()) {
    SPIFFS.remove(config::file);
    return false;
  }
  
  if (json.containsKey("client_ssid")) config::client_ssid = config::toString(json["client_ssid"]);
  if (json.containsKey("client_pass")) config::client_pass = config::toString(json["client_pass"]);

  if (json.containsKey("softap_ssid")) config::softap_ssid = config::toString(json["softap_ssid"]);
  if (json.containsKey("softap_pass")) config::softap_pass = config::toString(json["softap_pass"]);

  if (json.containsKey("admin_login")) config::admin_login = config::toString(json["admin_login"]);
  if (json.containsKey("admin_pass"))  config::admin_pass  = config::toString(json["admin_pass"]);

  if (json.containsKey("mqtt_server")) config::mqtt_server = config::toString(json["mqtt_server"]);
  if (json.containsKey("mqtt_login"))  config::mqtt_login  = config::toString(json["mqtt_login"]);
  if (json.containsKey("mqtt_pass"))   config::mqtt_pass   = config::toString(json["mqtt_pass"]);
  if (json.containsKey("mqtt_path"))   config::mqtt_path   = config::toString(json["mqtt_path"]);

  if (json.containsKey("narodmon_id")) config::narodmon_id = config::toString(json["narodmon_id"]);

  return true;
}

void config::recreate() {
  if (SPIFFS.exists(config::file)) SPIFFS.remove(config::file);
  config::save();
}

void config::remove() {
  if (SPIFFS.exists(config::file)) SPIFFS.remove(config::file);
}

String config::jsonSecurityLoad() {
  if (SPIFFS.exists(config::file)) {
    File configFile = SPIFFS.open(config::file, "r");
    if (!configFile) {
      config::recreate();
      return "";
    }

    size_t size = configFile.size();

    std::unique_ptr<char[]> buf(new char[size]);
    configFile.readBytes(buf.get(), size);

    StaticJsonBuffer<1024> jsonBuffer;
    JsonObject& json = jsonBuffer.parseObject(buf.get());
    configFile.close();

    if (!json.success()) {
      SPIFFS.remove(config::file);
      return "";
    }

    String answer;
    for (JsonObject::iterator obj = json.begin(); obj != json.end(); ++obj) {
      String key = obj->key;
      String val = obj->value;
      if (key.endsWith("pass")) val = val.length() ? "********" : "";
      answer += ",\"" + key + "\":\"" + val + "\"";
    }
    return "{\"status\":true" + answer + "}";
  }
  return "";
}

String config::toString(const char *cstr) {
  byte i = 0; String str;
  while (cstr[i] != '\0' or i > 32) str += cstr[i++];
  return str;
}
