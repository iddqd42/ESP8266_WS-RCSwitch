#ifndef CONFIG_H
#define CONFIG_H

#include "Arduino.h"
#include "FS.h"
#include <ArduinoJson.h>

class config {
  public:
    String file = "/config.json";
    
    bool save();
    bool save(String apiStr);
    bool load();
    void recreate();
    void remove();
    String jsonSecurityLoad();
    String toString(const char *cstr);    
    
    // Режим клиента
    String client_ssid = "";
    String client_pass = "";
    
    // Редим точки доступа
    String softap_ssid = "espWeatherStation";
    String softap_pass = "";
    
    // Доступ к панели администратора
    String admin_login = "admin";
    String admin_pass  = "admin";

    // MQTT
    String mqtt_server = "";
    String mqtt_login  = "";
    String mqtt_pass   = "";
    String mqtt_path   = "";

    // ID доступа к "Народному мониторингу"
    String narodmon_id = "";
};

#endif
