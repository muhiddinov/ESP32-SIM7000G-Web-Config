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
#include <TimeLib.h>

#define WIFI_CONFIGURED 0 // 1 - WiFi orqali sozlamalar saqlangandan keyingi holatida ishlaydi;
                          // 0 - sozlamalarning default holatida ishlaydi;
#define DEBUG_SERIAL
// #define AT_SERIAL
// #define SSD1306_LCD
// #define SDTYPE_MMC
#define SDMMC_ENABLE

#ifdef SDTYPE_MMC
#include <SD_MMC.h>
#include "FS.h"
#else
#include <SD.h>
#include "FS.h"
#endif
#ifdef SSD1306_LCD
#include <SSD1306.h>
#endif

#define DHTPIN      25
#define DHTTYPE     DHT11

#define VMET_PIN  34
#define AMET_PIN  35
#define GSMP_PIN  4
#define FRST_PIN  0
#define GSM_RX    26
#define GSM_TX    27
#define RS_RX     16
#define RS_TX     17

#define AT_CHK      0
#define AT_CSQ      1
#define AT_GPRS     2
#define AT_APN      3
#define AT_NET_CHK  4
#define AT_NET_ON   5
#define AT_GPS_ON   6
#define AT_GPS_OFF  7
#define AT_GPS_INF  8
#define AT_COPS     9
#define AT_CCLK     10
#define AT_ECHO_OFF 11
#define AT_HTTPINIT 12
#define AT_HTTPCID  13
#define AT_HTTPURL  14
#define AT_HTTPGET  15
#define AT_HTTPTERM 16
#define AT_HTTPREAD 17
#define AT_GPS_PWR  18
#define AT_NET_OFF  19

String commands[] = {
  "AT",
  "AT+CSQ",
  "AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"",
  "AT+SAPBR=3,1,\"APN\",\"INTERNET\"",
  "AT+SAPBR=2,1",
  "AT+SAPBR=1,1",
  "AT+CGNSPWR=1",
  "AT+CGNSPWR=0",
  "AT+CGNSINF",
  "AT+COPS?",
  "AT+CCLK?",
  "ATE0",
  "AT+HTTPINIT",
  "AT+HTTPPARA=\"CID\",1",
  "AT+HTTPPARA=\"URL\",\"",
  "AT+HTTPACTION=0",
  "AT+HTTPTERM",
  "AT+HTTPREAD",
  "AT+SGPIO=0,4,1,1",
  "AT+SAPBR=0,1"
};

tmElements_t tm;
int Year, Month, Day, Hour, Minute, Second, Zone, lastMinute = 0, lastSecond = 0;
DHT dht(DHTPIN, DHTTYPE);
SoftwareSerial gsmSerial(GSM_RX, GSM_TX);
SoftwareSerial RS485Serial(RS_RX, RS_TX);

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
String file_name = "", gsm_data = "", date_time = "", cops_name = "";
bool sdmmc_detect = 0, gps_state = 0, frst_btn = 0, sim_card = 0;
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
    "'default':'5'"
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
      // Serial.println(cmd[0]);
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
// void tableRoot1 () {
//   if (!server.authenticate(conf.getValue("username"), conf.getValue("password"))) return server.requestAuthentication(); conf.handleFormRequest(&server, 3);
// }
// void tableRoot2 () {
//   if (!server.authenticate(conf.getValue("username"), conf.getValue("password"))) return server.requestAuthentication(); conf.handleFormRequest(&server, 4);
// }
// void tableRoot3 () {
//   if (!server.authenticate(conf.getValue("username"), conf.getValue("password"))) return server.requestAuthentication(); conf.handleFormRequest(&server, 5);
// }
// void tableRoot4 () {
//   if (!server.authenticate(conf.getValue("username"), conf.getValue("password"))) return server.requestAuthentication(); conf.handleFormRequest(&server, 6);
// }
// void tableRoot5 () {
//   if (!server.authenticate(conf.getValue("username"), conf.getValue("password"))) return server.requestAuthentication(); conf.handleFormRequest(&server, 7);
// }

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
  pinMode(FRST_PIN, INPUT_PULLUP);
  gsmSerial.begin(9600);
  RS485Serial.begin(9600);
  digitalWrite(GSMP_PIN, 1);
  delay(300);
  digitalWrite(GSMP_PIN, 0);
  #ifdef DEBUG_SERIAL
  Serial.println("Wait for GSM modem...");
  #endif
  while (!star_project) {
    if (gsmSerial.available()) {
      String dt = gsmSerial.readStringUntil('\n');
      Serial.println(dt);
      if (dt.indexOf("+CPIN: READY") >= 0) {
        star_project = 1;
        sim_card = 1;
        break;
      } else if (dt.indexOf("+CPIN: NOT INSERTED") >= 0) {
        star_project = 1;
        sim_card = 0;
      }
    }
  }
  #ifdef DEBUG_SERIAL
  Serial.println("Start.");
  #endif
  if (sim_card) {
    queue.init();
    queue.addQueue(commands[AT_ECHO_OFF], AT_ECHO_OFF);
    queue.addQueue(commands[AT_CHK], AT_CHK);
    queue.addQueue(commands[AT_CSQ], AT_CSQ);
    queue.addQueue(commands[AT_COPS], AT_COPS);
    queue.addQueue(commands[AT_GPRS], AT_GPRS);
    queue.addQueue(commands[AT_APN], AT_APN);
    queue.addQueue(commands[AT_NET_ON], AT_NET_ON);
    queue.addQueue(commands[AT_NET_CHK], AT_NET_CHK);
    queue.addQueue(commands[AT_CCLK], AT_CCLK);
    queue.addQueue(commands[AT_GPS_PWR], AT_GPS_PWR);
    queue.addQueue(commands[AT_GPS_ON], AT_GPS_ON);
    queue.addQueue(commands[AT_GPS_INF], AT_GPS_INF);
  }
  dht.begin();
  device_id = WiFi.macAddress();
  device_id.replace(":","");
  conf.clearStatistics();
  conf.addStatistics("Qurilma ID", device_id);                        // 0 - index Qurilma id
  conf.addStatistics("Joylashuv nuqtasi", location);                  // 1 - index Location
  conf.addStatistics("Internet IP", ip_addr);                         // 2 - index IP
  conf.addStatistics("Xabarlar soni", String(message_count));         // 3 - index Counter
  conf.addStatistics("Harorat", "26.35");                             // 4 - index Harorat
  conf.addStatistics("Namlik", "30.00");                              // 5 - index Namlik
  conf.addStatistics("Quvvat", "12.45");                              // 6 - index Quvvat
  conf.addStatistics("ANT Signal", "29");                             // 7 - index ANT
  conf.addStatistics("Suv sarfi", "0");                               // 8 - index Suv sarfi
  conf.addStatistics("Suv tahi", "0");                                // 9 - index Suv sathi
  conf.setDescription(params);
  conf.readConfig();
  httpget_time = uint8_t(conf.getInt("timeout"));
  uint64_t wakeup_time = uint64_t(httpget_time * 60000000); // micro seconds 
  Serial.printf("Deep sleep time: %d min -->> ", httpget_time);
  Serial.println(wakeup_time);
  esp_sleep_enable_timer_wakeup(wakeup_time);
  fix_length = conf.getInt("fixing");
  server_url = conf.getString("server_url");
  server_url2 = conf.getString("server_url2");
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_UNDEFINED) {
    WiFi.softAP(conf.getValue("ssid"), conf.getValue("pwd"));
    server.on("/config", configRoot);
    server.on("/", HTTP_GET, handleRoot);
    // server.on("/table1", tableRoot1);
    // server.on("/table2", tableRoot2);
    // server.on("/table3", tableRoot3);
    // server.on("/table4", tableRoot4);
    // server.on("/table5", tableRoot5);
    server.on("/logout", HTTP_GET, logoutRoot);
    server.begin(80);
    #ifdef DEBUG_SERIAL
    Serial.print("WebServer IP-Adress = ");
    Serial.println(WiFi.softAPIP());
    #endif
  }
  curtime_cmd = millis();
  per_hour_time = millis();
  per_second_time = millis();
  per_mill_time = millis();
  _counter_httpget = httpget_time;
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
    frst_timer = millis();
  } else if (!digitalRead(FRST_PIN) && frst_btn) {
    if (millis() - frst_timer >= 10000) {
      frst_timer = 0;
      conf.deleteConfig(CONFFILE);
      conf.deleteConfig(CONFTABLE);
      // esp_restart();
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
    if (next_cmd && !waitHttpAction && sim_card) {
      queue.addQueue(commands[AT_CSQ], AT_CSQ);
      queue.addQueue(commands[AT_NET_CHK], AT_NET_CHK);
      queue.addQueue(commands[AT_GPS_INF], AT_GPS_INF);
      if (!internet && signal_quality > 1) {
        queue.addQueue(commands[AT_NET_OFF], AT_NET_OFF);
        queue.addQueue(commands[AT_APN], AT_APN);
        queue.addQueue(commands[AT_NET_ON], AT_NET_ON);
        queue.addQueue(commands[AT_NET_CHK], AT_NET_CHK);
      }
      if (cops_name.length() == 0) {
        queue.addQueue(commands[AT_COPS], AT_COPS);
      }
      if (!time_update) {
        queue.addQueue(commands[AT_CCLK], AT_CCLK);
      }
    }
    if (_counter_httpget >= httpget_time) {
      char temp[30];
      sprintf(temp, "/%02d-%02d-%02d.txt", year(), month(), day());
      file_name = String (temp);
      sprintf(temp, "%02d-%02d-%02d %02d:%02d:%02d", year(), month(), day(), hour(), minute(), second());
      date_time = String(temp);
      if (sdmmc_detect && file_name.length() > 1) {
        appendFile(SD, file_name.c_str(), make_param().c_str());
        File f = SD.open("/cntr.a", FILE_WRITE);
        f.println(message_count);
        f.close();
      }
      if (internet && sim_card) {
        queue.addQueue(commands[AT_HTTPINIT], AT_HTTPINIT);
        queue.addQueue(commands[AT_HTTPCID], AT_HTTPCID);
        queue.addQueue(commands[AT_HTTPURL] + server_url + make_param() + "\"", AT_HTTPURL);
        queue.addQueue(commands[AT_HTTPGET], AT_HTTPGET);
        queue.addQueue(commands[AT_HTTPTERM], AT_HTTPTERM);
        if (server_url2.length() > 5) {
          queue.addQueue(commands[AT_HTTPINIT], AT_HTTPINIT);
          queue.addQueue(commands[AT_HTTPCID], AT_HTTPCID);
          queue.addQueue(commands[AT_HTTPURL] + server_url2 + make_param() + "\"", AT_HTTPURL);
          queue.addQueue(commands[AT_HTTPGET], AT_HTTPGET);
          queue.addQueue(commands[AT_HTTPTERM], AT_HTTPTERM);
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
  if (millis() - curtime_cmd > 500 && next_cmd && !waitHttpAction && sim_card) {
    curtime_cmd = millis();
    queue.sendCmdQueue();
  }
  if (millis() - start_cmd_time > 10000) {
    start_cmd_time = millis();
    next_cmd = true;
    cmd_timeout_count ++;
    if (sdmmc_detect) {
      File f = SD.open("/gsmerr.log", FILE_APPEND);
      f.println(date_time + "\t\tGSM timeout: " + String(cmd_timeout_count) + " in 5");
      f.close();
    }
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
        signal_quality = map(csq, 0, 31, 0, 4);
      }
      break;
    case AT_NET_CHK:
      if (str.indexOf("+SAPBR") >= 0) {
        ip_addr = str.substring(str.indexOf("\"") + 1, str.lastIndexOf("\""));
        int bearer = str.substring(str.indexOf(",") + 1, str.lastIndexOf(",")).toInt();
        if (bearer == 1) internet = 1;
        else internet = 0;
      }
      break;
    case AT_GPS_INF:
      if (str.indexOf("++CGNSINF") >= 0) {
        String temp = str.substring(str.indexOf(":") + 2, str.length());
        Serial.println(temp);
        float llong=0.0, llat=0.0, f1=0, f2=0, f3=0, f4=0, f5=0, f6=0;
        int i1=0, i2=0, i3=0, i4=0, i5=0, i6=0, i7=0, i8=0, i9=0, i10=0, i11=0;
        char* s1 = NULL;
        Serial.println(temp);
        sscanf(temp.c_str(), "%d,%d,%s,%f,%f,%f,%f,%f,%d,%d,%f,%f,%f,%d,%d,%d,%d,%d,%d,%d,", &i1, &i2, s1, &llong, &llat, &f1, &f2, &f3, &i3, &i4, &f4, &f5, &f6, &i5, &i6, &i7, &i8, &i9, &i10, &i11);
        char loc[15];
        if (i2 == 1) {
          gps_state = 1;
        }
        sprintf(loc, "%.6f, %.6f", llong, llat);
        location = String (loc);
      }
      break;
    case AT_COPS:
      if (str.indexOf("+COPS:") >= 0) {
        cops_name = str.substring(str.indexOf(",\"") + 2, str.lastIndexOf(" "));
      }
      break;
    case AT_HTTPGET:
      if (str.indexOf("+HTTPACTION:") >= 0) {
        waitHttpAction = false;
        int result = str.substring(str.indexOf(',') + 1, str.lastIndexOf(',')).toInt();
        if (result == 200) {
          message_count ++;
          if (conf.configured == WIFI_CONFIGURED) {
            digitalWrite(GSMP_PIN, 1);
            delay(2000);
            digitalWrite(GSMP_PIN, 0);
            esp_deep_sleep_start();
          }
        } else {
          err_http_count ++;
        }
      }
      break;
    case AT_CCLK:
      if (str.indexOf("+CCLK:") >= 0) {
        date_time = str.substring(str.indexOf("\"") + 1, str.lastIndexOf("\""));
        createElements(date_time.c_str());
        date_time = "";
      }
    default:
      break;
  }
}

void createElements(const char *str) {
  sscanf(str, "%d/%d/%d,%d:%d:%d+%d", &Year, &Month, &Day, &Hour, &Minute, &Second, &Zone);
  tm.Year = CalendarYrToTm(2000 + Year);
  tm.Month = Month;
  tm.Day = Day;
  tm.Hour = Hour;
  tm.Minute = Minute;
  tm.Second = Second;
  setTime(makeTime(tm));
  time_update = 1;
  if (year() < 2022) time_update = 0;
}
