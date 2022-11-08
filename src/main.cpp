/*********************************************************
 * ESP32 WROOM va SIM7000G module bilan birgalikda
 * rivojlantirilgan plata suv xo'jaligi uchun xizmat qiladi.
 ********************************************************
*/ 
#include <Arduino.h>
#include <WebServer.h>
#include "WebConfig.h"
#include "images.h"
#include <SoftwareSerial.h>
#include <DHT.h>
#include "FS.h"
#include "SD.h"
#include <TimeLib.h>

#define DEBUG_SERIAL
// #define AT_SERIAL
// #define SDMMC_ENABLE

#define DHTPIN      27
#define DHTTYPE     DHT11

#define VMET_PIN  34
#define AMET_PIN  35
#define GSMP_PIN  25
#define GSMR_PIN  26
#define FRST_PIN  0

#define AT_CHK      0
#define AT_CSQ      1
#define AT_APN      2
#define AT_NET_ON   3
#define AT_NET_OFF  4
#define AT_NET_CHK  5
#define AT_HTTPGET  6
#define AT_GPS_ON   7
#define AT_GPS_OFF  8
#define AT_LOCATION 9
#define AT_IP_CHK   10
#define AT_COPS     11
#define AT_CCLK     12
#define ATE_OFF     13

String commands[] = {
  "AT",
  "AT+CSQ",
  "AT+CGACT=1",
  "AT+CGACT=0",
  "AT+CGACT?",
  "AT+CGNSPWR=1",
  "AT+CGNSPWR=0",
  "AT+CGNSINF",
  "AT+COPS?",
  "AT+CCLK?",
  "ATE0"
};

tmElements_t tm;
int Year, Month, Day, Hour, Minute, Second, Zone, lastMinute = 0, lastSecond = 0;
DHT dht(DHTPIN, DHTTYPE);
SoftwareSerial gsmSerial(33, 32);
SoftwareSerial RS485Serial(18, 19);

// byte readDistance [8] = {0x01, 0x03, 0x00, 0x01, 0x00, 0x01, 0xD5, 0xCA};
// byte readLevel [8] = {0x01, 0x03, 0x00, 0x06, 0x00, 0x01, 0x64, 0x0B};

byte readDistance [8] = {0x01, 0x03, 0x20, 0x01, 0x00, 0x02, 0x9E, 0x0B};
byte readLevel [8] = {0x01, 0x03, 0x20, 0x02, 0x00, 0x02, 0x6E, 0x0B};
byte ultrasonic_data [7];

uint8_t _counter_httpget = 0, prgs = 0, cmd_timeout_count = 0;
uint8_t old_cmd_gsm = 0, csq = 0, httpget_time = 0, signal_quality = 0;
uint16_t distance = 0, cops = 0, err_http_count = 0;
uint16_t mcc_code[] = {43401, 43402, 43403, 43404, 43405, 43406, 43407, 43408, 43409};
uint32_t per_hour_time = 0, per_minute_time = 0, per_second_time = 0, per_mill_time = 0;
uint32_t vcounter = 0, curtime_cmd = 0, res_counter = 0, frst_timer = 0;
uint32_t message_count = 0, httpget_count = 0, start_cmd_time = 0, end_cmd_time = 0;
String location = "00.0000,00.0000", ip_addr = "0.0.0.0", device_id = "", server_url = "", server_url2 = "";
String sopn[] = {"Buztel", "Uzmacom", "UzMobile", "Beeline", "Ucell", "Perfectum", "UMS", "UzMobile", "EVO"};
String file_name = "", gsm_data = "", date_time = "";
bool sdmmc_detect = 0, gps_state = 0, frst_btn = 0, no_sim = 0;
bool next_cmd = true, waitHttpAction = false, star_project = false, device_lost = 0;
bool internet = false, queue_stop = 0, time_update = 0;
float voltage = 0.0, tmp = 0.0, hmt = 0.0, water_level = 0.0;
int water_cntn = 0, v_percent = 0, fix_length = 0;

String params = "["
  "{"
    "'name':'ssid',"
    "'label':'WLAN nomi',"
    "'type':"+String(INPUTTEXT)+","
    "'default':'AQILLI SUV'"
  "},"
  "{"
    "'name':'pwd',"
    "'label':'WLAN paroli',"
    "'type':"+String(INPUTPASSWORD)+","
    "'default':'12345678'"
  "},"
  "{"
    "'name':'username',"
    "'label':'WEB login',"
    "'type':"+String(INPUTTEXT)+","
    "'default':'admin'"
  "},"
  "{"
    "'name':'password',"
    "'label':'WEB parol',"
    "'type':"+String(INPUTPASSWORD)+","
    "'default':'admin'"
  "},"
  "{"
    "'name':'server_url',"
    "'label':'URL1 server',"
    "'type':"+String(INPUTTEXT)+","
    "'default':'http://m.and-water.uz/bot/app.php?'"
  "},"
  "{"
    "'name':'server_url2',"
    "'label':'URL2 server',"
    "'type':"+String(INPUTTEXT)+","
    "'default':''"
  "},"
  "{"
    "'name':'timeout',"
    "'label':'Xabar vaqti (minut)',"
    "'type':"+String(INPUTTEXT)+","
    "'default':'1'"
  "},"
  "{"
    "'name':'fixing',"
    "'label':'Tuzatish (cm)',"
    "'type':"+String(INPUTTEXT)+","
    "'default':'0'"
  "}"
"]";

struct cmdQueue {
  String cmd [16];
  int8_t cmd_id[16];
  int k;
  void init () {
    k = 0;
    for (int i = 0; i < 16; i++) {
      cmd_id[i] = -1;
      cmd[i] = "";
    }
  }
  void addQueue (String msg, uint8_t msg_id) {
    cmd[k] = msg;
    cmd_id[k++] = msg_id;
    if (k > 15) k = 0;
  }
  void sendCmdQueue () {
    if (k > 0) {
      if (!waitHttpAction) {
//        Serial.println(cmd[0]);
        old_cmd_gsm = cmd_id[0];
        if (cmd_id[0] == AT_HTTPGET) waitHttpAction = true;
        gsmSerial.println(cmd[0]);
        start_cmd_time = millis();
        k --;
        next_cmd = false;
        for (int i = 0; i < k; i++) {
          cmd[i] = cmd[i+1];
          cmd[i+1] = "";
          cmd_id[i] = cmd_id[i+1];
          cmd_id[i+1] = -1;
        }
      }
    }
  }
};

cmdQueue queue;
WebServer server;
WebConfig conf;
void checkCommandGSM ();
void check_CMD (String str);
void createElements(const char *str);

void configRoot() {
  if (!server.authenticate(conf.values[2].c_str(), conf.values[3].c_str())) {
    return server.requestAuthentication();
  }
  conf.handleFormRequest(&server, 2);
}

void handleRoot() {
  if (!server.authenticate(conf.getValue("username"), conf.getValue("password"))) {
    return server.requestAuthentication();
  }
  conf.handleFormRequest(&server, 1);
}
void logoutRoot() {
  if (!server.authenticate(conf.getValue("username"), conf.getValue("password"))) {
    return server.requestAuthentication();
  }
  conf.handleFormRequest(&server, 0);
}
void tableRoot1 () {
  if (!server.authenticate(conf.getValue("username"), conf.getValue("password"))) return server.requestAuthentication(); conf.handleFormRequest(&server, 3);
}
void tableRoot2 () {
  if (!server.authenticate(conf.getValue("username"), conf.getValue("password"))) return server.requestAuthentication(); conf.handleFormRequest(&server, 4);
}
void tableRoot3 () {
  if (!server.authenticate(conf.getValue("username"), conf.getValue("password"))) return server.requestAuthentication(); conf.handleFormRequest(&server, 5);
}
void tableRoot4 () {
  if (!server.authenticate(conf.getValue("username"), conf.getValue("password"))) return server.requestAuthentication(); conf.handleFormRequest(&server, 6);
}
void tableRoot5 () {
  if (!server.authenticate(conf.getValue("username"), conf.getValue("password"))) return server.requestAuthentication(); conf.handleFormRequest(&server, 7);
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return float((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

// String make_param() {
//   return ("?id=" + device_id + "&location={" + location + "}&data=" + String(water_cntn) + "&temperature=" + String(tmp) + "&humidity=" + String(hmt) + "&power=" + String(int(voltage)));
// }

String make_param() {
  return ("?id=" + device_id + "&location={" + location + "}&water_level=" + String(water_level) + "&water_volume=" + String(water_cntn) + "&temperature=" + String(tmp) + "&humidity=" + String(hmt) + "&power=" + String(int(voltage)));
}

void appendFile(fs::FS &fs, const char * path, const char * message){
    File file = fs.open(path, FILE_APPEND);
    if (date_time.length() > 10) file.println(date_time + "\t" + String(message));
    else file.println(message);
    file.close();
}

void setup() {
  #ifdef DEBUG_SERIAL
  Serial.begin(115200);
  #endif
  pinMode(VMET_PIN, INPUT);
  pinMode(AMET_PIN, INPUT);
  pinMode(GSMP_PIN, OUTPUT);
  pinMode(GSMR_PIN, OUTPUT);
  pinMode(FRST_PIN, INPUT_PULLUP);
  gsmSerial.begin(9600);
  RS485Serial.begin(9600);
  digitalWrite(GSMR_PIN, 1);
  delay(5000);
  digitalWrite(GSMR_PIN, 0);
  digitalWrite(GSMP_PIN, 1);
  #ifdef DEBUG_SERIAL
  Serial.println("Wait for GSM modem...");
  #endif
  while (!star_project) {
    checkCommandGSM();
    gsmSerial.println("AT");
  }
  #ifdef DEBUG_SERIAL
  Serial.println("Start.");
  #endif
  queue.init();
  queue.addQueue(commands[ATE_OFF], ATE_OFF);
  queue.addQueue(commands[AT_CHK], AT_CHK);
  queue.addQueue(commands[AT_CSQ], AT_CSQ);
  queue.addQueue(commands[AT_COPS], AT_COPS);
  queue.addQueue(commands[AT_APN], AT_APN);
  queue.addQueue(commands[AT_CCLK], AT_CCLK);
  queue.addQueue(commands[AT_NET_CHK], AT_NET_CHK);
  queue.addQueue(commands[AT_GPS_ON], AT_GPS_ON);
  queue.addQueue(commands[AT_IP_CHK], AT_IP_CHK);
  dht.begin();
  device_id = WiFi.macAddress();
  device_id.replace(":","");
  conf.clearStatistics();
  conf.addStatistics("Qurilma ID", device_id);                        // 0 - index Qurilma id
  conf.addStatistics("Joylashuv nuqtasi", location);                  // 1 - index Location
  conf.addStatistics("Internet IP", ip_addr);                         // 2 - index IP
  conf.addStatistics("Xabarlar soni", String(message_count/6));         // 3 - index Counter
  conf.addStatistics("Harorat", "26.35");                             // 4 - index Harorat
  conf.addStatistics("Namlik", "30.00");                              // 5 - index Namlik
  conf.addStatistics("Quvvat", "12.45");                              // 6 - index Quvvat
  conf.addStatistics("ANT Signal", "29");                             // 7 - index ANT
  conf.addStatistics("Suv sarfi", "0");                               // 8 - index Suv sarfi
  conf.addStatistics("Suv tahi", "0");                                // 9 - index Suv sathi
  conf.setDescription(params);
  conf.readConfig();
  fix_length = conf.getInt("fixing");
  httpget_time = uint8_t(conf.getInt("timeout"));
  server_url = conf.getString("server_url");
  server_url2 = conf.getString("server_url2");
  WiFi.softAP(conf.getValue("ssid"), conf.getValue("pwd"));
  #ifdef DEBUG_SERIAL
  Serial.print("WebServer IP-Adress = ");
  Serial.println(WiFi.softAPIP());
  #endif
  server.on("/config", configRoot);
  server.on("/", HTTP_GET, handleRoot);
  server.on("/table1", tableRoot1);
  server.on("/table2", tableRoot2);
  server.on("/table3", tableRoot3);
  server.on("/table4", tableRoot4);
  server.on("/table5", tableRoot5);
  server.on("/logout", HTTP_GET, logoutRoot);
  server.begin(80);
  curtime_cmd = millis();
  per_hour_time = millis();
  per_second_time = millis();
  per_mill_time = millis();
  _counter_httpget = httpget_time;
}

void ESP_restart() {
  digitalWrite(GSMR_PIN, 1);
  delay(5000);
  digitalWrite(GSMR_PIN, 0);
  ESP.restart();
}

void loop() {
  server.handleClient();
  checkCommandGSM();
  #ifdef AT_SERIAL
  if (Serial.available()) {
    gsmSerial.println(Serial.readStringUntil('\n'));
  }
  #endif
  // Sozlamalarni tozalash
  if (!digitalRead(FRST_PIN) && !frst_btn) {
    frst_btn = 1;
    frst_timer = millis()/1000;
  } else if (!digitalRead(FRST_PIN) && frst_btn) {
    if (millis()/1000 - frst_timer >= 10) {
      frst_timer = 0;
      conf.deleteConfig(CONFFILE);
      conf.deleteConfig(CONFTABLE);
      ESP_restart();
    }
  } else if (digitalRead(FRST_PIN)) {
    frst_btn = 0;
  }
  if (!time_update) {
    // har 1 sekundda 1 marta ishlash;
    if (millis() - per_second_time >= 1000) {
      per_second_time = millis();
      RS485Serial.write(readLevel, 8);
      per_minute_time ++;
    }
    // har 1 minutda 1 marta ishlash
    if (per_minute_time >= 60) {
      _counter_httpget ++;
      per_minute_time = 0;
    }
  }
  else {
    if (minute() != lastMinute) {
      _counter_httpget ++;
      lastMinute = minute();
      per_minute_time ++;
    }
    if (second() != lastSecond) {
      lastSecond = second();
      RS485Serial.write(readLevel, 8);
    }
  }

  uint8_t pRAM = map(ESP.getFreeHeap(), 0, ESP.getHeapSize(), 0, 100);
  if (pRAM > 90) {
    if (sdmmc_detect) {
      appendFile(SD, "/exp.log", " Large heap size !!!");
    }
    ESP_restart();
  }

  // batareyka voltini hisoblash va ekranga chiqarish;
  // har 100 millisikundda 1 marta ishlash;
  if (millis() - per_mill_time > 100) {
    per_mill_time = millis();
    voltage += float(analogRead(VMET_PIN));
    vcounter ++;
  }
  if (vcounter >= 30) {
    voltage = voltage / vcounter;
    voltage = fmap(voltage, 0, 4096, 0, 21.72);              // Voltage value
    if (voltage < 5.0) {
      device_lost = 1;
    }
    voltage = fmap(voltage, 9.0, 12.0, 0, 100);              // Voltage percent
    if (voltage < 0) voltage = 0;
    if (voltage > 100) voltage = 100;
    v_percent = int(voltage);
    fix_length = conf.getInt("fixing");
    if (conf.check_reset == 1){
      prgs=0;
      ESP_restart();
    }
    tmp = dht.readTemperature();
    hmt = dht.readHumidity();
    conf.setStatistics(1, "<a href=https://maps.google.com?q="+location + ">"+location+"</a>");                        // 1 - index Location
    conf.setStatistics(2, ip_addr);                         // 2 - index IP
    conf.setStatistics(3, String(message_count) + " - E(" + String(int(err_http_count) * -1) + ")");           // 3 - index Counter
    conf.setStatistics(4, String(tmp) + " C");
    conf.setStatistics(5, String(hmt) + " %");
    conf.setStatistics(6, String(v_percent) + "%");
    int dbm = map(csq, 0, 31, -113, -51);
    conf.setStatistics(7, String(dbm) + " dBm");
    conf.setStatistics(8, String(water_cntn) + " T/s");
    conf.setStatistics(9, String(water_level) + " cm");
    if (next_cmd && !waitHttpAction && !queue_stop) {
      queue.addQueue(commands[AT_CSQ], AT_CSQ);
      queue.addQueue(commands[AT_NET_CHK], AT_NET_CHK);
      queue.addQueue(commands[AT_LOCATION], AT_LOCATION);
      if (!internet && signal_quality > 1) {
        queue.addQueue(commands[AT_NET_OFF], AT_NET_OFF);
        queue.addQueue(commands[AT_APN], AT_APN);
        queue.addQueue(commands[AT_NET_ON], AT_NET_ON);
        queue.addQueue(commands[AT_IP_CHK], AT_IP_CHK);
      }
      if (!cops) {
        queue.addQueue(commands[AT_COPS], AT_COPS);
      }
      if (!time_update) {
        queue.addQueue(commands[AT_CCLK], AT_CCLK);
      }
    }
    if (_counter_httpget >= httpget_time) {
      file_name = "/" + String(year()) + "-" + String(month()) + "-" + String(day()) + ".txt";
      date_time = String(year()) + "-" + String(month()) + "-" + String(day()) + " " + String(hour()) + ":" + String(minute()) + ":" + String(second());
      if (sdmmc_detect && file_name.length() > 1) {
        appendFile(SD, file_name.c_str(), make_param().c_str());
        File f = SD.open("/cntr.a", FILE_WRITE);
        f.println(message_count);
        f.close();
      }
      if (!queue_stop && internet) {
        queue.addQueue(commands[AT_HTTPGET] + server_url + make_param() + "\"", AT_HTTPGET);
        if (server_url2.length() > 5) {
          queue.addQueue(commands[AT_HTTPGET] + server_url2 + make_param() + "\"", AT_HTTPGET);
        }
        _counter_httpget = 0;
        httpget_count ++;
      }
    }
    vcounter = 0;
    voltage = 0;
  }
  // masofa sensoridan ma'lumot olish;
  if (RS485Serial.available()) {
    RS485Serial.readBytes(ultrasonic_data, 7);
    distance = ultrasonic_data[3] << 8 | ultrasonic_data[4];
    if (distance > 10000) {
      distance = 10000;
    }
    if (distance < 0) {
      distance = 0;
    }
    water_level = float(distance/10.0) + float(fix_length);
    distance = 0;
    if (water_level >= 0 && water_level < 500) {
      water_cntn = conf.table_values[int(water_level)];
    } else {
      water_cntn = 0;
    }
  }
  // navbatni bo'shatish
  if (next_cmd && millis() - curtime_cmd > 500 && !queue_stop) {
    curtime_cmd = millis();
    queue.sendCmdQueue();
  }
  if (millis() - start_cmd_time > 10000) {
    next_cmd = true;
    cmd_timeout_count ++;
    if (sdmmc_detect) {
      File f = SD.open("/gsmerr.log", FILE_APPEND);
      f.println(date_time + "\t\tGSM timeout: " + String(cmd_timeout_count) + " in 5");
      f.close();
    }
  }
  if (cmd_timeout_count >= 5) {
    cmd_timeout_count = 0;
    queue_stop = 1;
    next_cmd = 0;
    star_project = 0;
    if (sdmmc_detect) {
      File f = SD.open("/gsmerr.log", FILE_APPEND);
      f.println(date_time + "\t\tGSM is turn power off");
      f.close();
    }
    digitalWrite(GSMR_PIN, 1);
    delay(2000);
    digitalWrite(GSMR_PIN, 0);
    delay(10000);
    digitalWrite(GSMP_PIN, 1);
    delay(2000);
    digitalWrite(GSMP_PIN, 0);
    uint32_t start_t = millis();
    while (!star_project) {
      checkCommandGSM();
      if (millis() - start_t >= 2000) {
        gsmSerial.println("AT");
        start_t = millis();
      }
    }
    if (sdmmc_detect) {
      File f = SD.open("/gsmerr.log", FILE_APPEND);
      f.println(date_time + "\t\tGSM is turn power off");
      f.close();
    }
    queue_stop = 0;
    cops = 0;
    internet = 0;
    waitHttpAction = 0;
    ip_addr = "0.0.0.0";
    queue.init();
    queue.addQueue(commands[AT_CHK], AT_CHK);
    queue.addQueue(commands[AT_CSQ], AT_CSQ);
    queue.addQueue(commands[AT_COPS], AT_COPS);
    queue.addQueue(commands[AT_APN], AT_APN);
    queue.addQueue(commands[AT_CCLK], AT_CCLK);
    queue.addQueue(commands[AT_NET_CHK], AT_NET_CHK);
    queue.addQueue(commands[AT_GPS_ON], AT_GPS_ON);
    queue.addQueue(commands[AT_IP_CHK], AT_IP_CHK);
    _counter_httpget = httpget_time;
    httpget_count = message_count + err_http_count;
  }
}

void checkCommandGSM () {
  if (gsmSerial.available()) {
    String dt = gsmSerial.readStringUntil('\n');
    if (dt.length() > 1) {
      #ifdef DEBUG_SERIAL
      Serial.println(dt);
      #endif
      check_CMD(dt);
      next_cmd = true;
    }
  }
}

void check_CMD (String str) {
  ///////////////////////// CHECK COMMANDS ///////////////////////////////////
  switch (old_cmd_gsm) {
    case AT_CSQ:
      if (str.indexOf("+CSQ") >= 0) {
        csq = str.substring(str.indexOf("+CSQ: ") + 5, str.indexOf(",")).toInt();
      }
    case AT_IP_CHK:
      if (str.indexOf("+CGDCONT:1") >= 0) {
        int start_id = str.indexOf("DEFAULT");
        int end_id = str.indexOf("\",0,0");
        ip_addr = str.substring(start_id + 10, end_id);
      }
    case AT_NET_CHK:
      if (str.indexOf("+CGACT: 1") >= 0) {
        internet = true;
      } else if (str.indexOf("+CGACT: 0") >= 0){
        internet = false;
      }
    case AT_LOCATION:
      if (str.indexOf("+LOCATION") >= 0) {
        location = "";
        gps_state = 0;
      } else if (str.substring(0, str.indexOf(",")).toDouble() > 0.0){
        location = str;
        location.trim();
        gps_state = 1;
      }
    case AT_COPS:
      if (str.indexOf("+COPS:") >= 0) {
        cops = str.substring(str.indexOf(",\"") + 2, str.length()-1).toInt();
      }
    case AT_HTTPGET:
      if (str.indexOf("HTTP") >= 0) {
        waitHttpAction = false;
      }
      if (str.indexOf("HTTP/1.1  200") >= 0) {
        message_count ++;
      }
      else if (str.indexOf("HTTP/1.1  400") >= 0) {
        err_http_count ++;
      }
    case AT_CCLK:
      if (str.indexOf("+CCLK:") >= 0) {
        date_time = str.substring(str.indexOf("\"") + 1, str.lastIndexOf("\""));
        createElements(date_time.c_str());
        date_time = "";
      }
    default:
      ////////////////////////// CHECK STRING VALUES //////////////////////////////
      if (str.indexOf("OK") >= 0 || str.indexOf("STN:") >= 0) {
        star_project = true;
      }
      if (str.indexOf("NO SIM") >= 0) {
        no_sim = true;
        queue_stop = true;
      }
      break;
  }
}

void createElements(const char *str) {
  sscanf(str, "%d/%d/%d,%d:%d:%d+%d", &Year, &Month, &Day, &Hour, &Minute, &Second, &Zone);
  tm.Year = CalendarYrToTm(2000 + Year);
  tm.Month = Month;
  tm.Day = Day;
  tm.Hour = (Hour + Zone) % 24;
  tm.Minute = Minute;
  tm.Second = Second;
  setTime(makeTime(tm));
  time_update = 1;
  if (year() < 2022) time_update = 0;
}
