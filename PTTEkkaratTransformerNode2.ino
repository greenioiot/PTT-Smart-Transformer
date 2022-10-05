#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <ModbusMaster.h>
#include "REG_CONFIG.h"
#include <HardwareSerial.h>
#include "HardwareSerial_NB_BC95.h"
#include <TaskScheduler.h>

#define _TASK_TIMECRITICAL

HardwareSerial modbus(2);
HardwareSerial_NB_BC95 AISnb;

BluetoothSerial SerialBT;

#include <ArduinoOTA.h>
#include <WiFi.h>
#include <Wire.h>

// thingcontrol.io setup
String deviceToken = "";
String serverIP = "147.50.151.130";
String serverPort = "19956";
String json = "";

ModbusMaster node;
void t1CallgetMeter();
void t2CallsendViaNBIOT();
void t2CallsendViaNBIOT_Notification();
void t4Restart();

//TASK
Task t1(250000, TASK_FOREVER, &t1CallgetMeter);
Task t2(300000, TASK_FOREVER, &t2CallsendViaNBIOT);
Task t3(60000, TASK_FOREVER, &t2CallsendViaNBIOT_Notification);

#define trigWDTPin    32
#define ledHeartPIN   0

Scheduler runner;
String _config = "{\"_type\":\"retrattr\",\"Tn\":\"8966031940014308310\",\"keys\":[\"epoch\",\"ip\"]}";
unsigned long _epoch = 0;
String _IP = "";
String dataJson = "";
boolean validEpoc = false;

StaticJsonDocument<400> doc;

struct Meter
{
  String cA;
  String cB;
  String cC;
  String cN;
  float cG;
  String cAvg;
  String vAB;
  String vBC;
  String vCA;
  String vAN;
  String vBN;
  String vCN;
  String vLLAvg;
  String vLNAvg;
  String freq;
  String apTotal;

  ////////////////////////
  String cuA;  // Current Unbalance A
  String cuB;  // Current Unbalance B
  String cuC;  // Current Unbalance C
  String cuW;  // Current Unbalance Worst

  String vuAB;
  String vuBC;
  String vuCA;
  String vuLLW; // Voltage Unbalance L-L Worst

  String vuAN;
  String vuBN;
  String vuCN;
  String vuLNW; // Voltage Unbalance L-N Worst

  String apA;
  String apB;
  String apC;
  String apT; // Active Power Total
  String rpA;
  String rpB;
  String rpC;
  String rpT; // Reactive Power Total
  String appA;
  String appB;
  String appC;
  String appT; // Apparent Power Total
  String pfA; //   Power Factor
  String pfB; //
  String pfC; //
  String pfT; //
  String pf; // Power Factor alter format
  String poA; //   Energy
  String poB; //   Energy
  String poC; //   Energy
  String poT; //   Energy
  String Temp1;
  String Temp2;
  String di1;
  String di2;
  String di3;
  String di4;
  String di5;
  String di6;
  String di7;
  String di8;
};
Meter meter;
signal meta;

unsigned long ms;

void _writeEEPROM(String data) {
  Serial.print("Writing Data:");
  Serial.println(data);
  writeString(10, data);  //Address 10 and String type data
  delay(10);
}

void _loadConfig() {
  serverIP = read_String(10);
  serverIP.trim();
  Serial.print("IP:");
  Serial.println(serverIP);
}

char  char_to_byte(char c)
{
  if ((c >= '0') && (c <= '9'))
  {
    return (c - 0x30);
  }
  if ((c >= 'A') && (c <= 'F'))
  {
    return (c - 55);
  }
}
void _init() {
  Serial.println(_config);
  do {
    UDPSend udp = AISnb.sendUDPmsgStr(serverIP, serverPort, _config);
    dataJson = "";
    deviceToken = AISnb.getNCCID();
    Serial.print("nccid:");
    Serial.println(deviceToken);
    UDPReceive resp = AISnb.waitResponse();
    AISnb.receive_UDP(resp);
    Serial.print("waitData:");
    Serial.println(resp.data);
    for (int x = 0; x < resp.data.length(); x += 2)
    {
      char c =  char_to_byte(resp.data[x]) << 4 | char_to_byte(resp.data[x + 1]);
      dataJson += c;
    }
    Serial.println(dataJson);
    DeserializationError error = deserializeJson(doc, dataJson);

    // Test if parsing succeeds.
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      validEpoc = true;
      delay(4000);
    } else {
      validEpoc = false;
      unsigned long epoch = doc["epoch"];
      _epoch = epoch;
      String ip = doc["ip"];
      _IP = ip;
      Serial.println(dataJson);
      Serial.print("epoch:");  Serial.println(_epoch);
      _writeEEPROM(_IP);
      Serial.println(_IP);
    }
    delay(5000);
    HeartBeat();
  } while (validEpoc);
}

void writeString(char add, String data)
{
  EEPROM.begin(512);
  int _size = data.length();
  int i;
  for (i = 0; i < _size; i++)
  {
    EEPROM.write(add + i, data[i]);
  }
  EEPROM.write(add + _size, '\0'); //Add termination null character for String Data
  EEPROM.commit();
}

String read_String(char add)
{
  int i;
  char data[100]; //Max 100 Bytes
  int len = 0;
  unsigned char k;
  k = EEPROM.read(add);
  while (k != '\0' && len < 500) //Read until null character
  {
    k = EEPROM.read(add + len);
    data[len] = k;
    len++;
  }
  data[len] = '\0';
  Serial.print("Debug:");
  Serial.println(String(data));
  return String(data);
}

/**********************************************  WIFI Client 注意编译时要设置此值 *********************************
   wifi client
*/
const char* ssid = "greenioGuest"; //replace "xxxxxx" with your WIFI's ssid
const char* password = "green7650"; //replace "xxxxxx" with your WIFI's password

//WiFi&OTA 参数
String HOSTNAME = "Ekarat-";
#define PASSWORD "green7650" //the password for OTA upgrade, can set it in any char you want

void setupOTA()
{
  //Port defaults to 8266
  //ArduinoOTA.setPort(8266);
  //Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(HOSTNAME.c_str());
  //No authentication by default
  ArduinoOTA.setPassword(password);
  //Password can be set with it's md5 value as well
  //MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  //ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");
  ArduinoOTA.onStart([]()
  {
    Serial.println("Start Updating....");
    SerialBT.println("Start Updating....");
    HeartBeat();
    SerialBT.printf("Start Updating....Type:%s\n", (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem");
    Serial.printf("Start Updating....Type:%s\n", (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem");
  });
  ArduinoOTA.onEnd([]()
  {
    SerialBT.println("Update Complete!");
    Serial.println("Update Complete!");
    ESP.restart();
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
  {
    String pro = String(progress / (total / 100)) + "%";
    //    int progressbar = (progress / (total / 100));
    //int progressbar = (progress / 5) % 100;
    //int pro = progress / (total / 100);
    SerialBT.printf("Progress: %u%%\n", (progress / (total / 100)));
    Serial.printf("Progress: %u%%\n", (progress / (total / 100)));
    ms = millis();
    if (ms % 10000 == 0)
    {
      HeartBeat();
    }
  });

  ArduinoOTA.onError([](ota_error_t error)
  {
    Serial.printf("Error[%u]: ", error);
    String info = "Error Info:";
    switch (error)
    {
      case OTA_AUTH_ERROR:
        info += "Auth Failed";
        Serial.println("Auth Failed");
        break;

      case OTA_BEGIN_ERROR:
        info += "Begin Failed";
        Serial.println("Begin Failed");
        break;

      case OTA_CONNECT_ERROR:
        info += "Connect Failed";
        Serial.println("Connect Failed");
        break;

      case OTA_RECEIVE_ERROR:
        info += "Receive Failed";
        Serial.println("Receive Failed");
        break;

      case OTA_END_ERROR:
        info += "End Failed";
        Serial.println("End Failed");
        break;
    }
    Serial.println(info);
    ESP.restart();
  });

  ArduinoOTA.begin();
}

void setupWIFI()
{
  Serial.println("Connecting...");
  Serial.println(String(ssid));
  //连接WiFi，删除旧的配置，关闭WIFI，准备重新配置
  WiFi.disconnect(true);
  delay(1000);
  WiFi.mode(WIFI_STA);
  //WiFi.onEvent(WiFiEvent);
  WiFi.setAutoConnect(true);
  WiFi.setAutoReconnect(true);    //断开WiFi后自动重新连接,ESP32不可用
  WiFi.setHostname(HOSTNAME.c_str());
  WiFi.begin(ssid, password);
  //等待5000ms，如果没有连接上，就继续往下
  //不然基本功能不可用
  byte count = 0;
  while (WiFi.status() != WL_CONNECTED && count < 10)
  {
    count ++;
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED)
    Serial.println("Connecting...OK.");
  else
    Serial.println("Connecting...Failed");
}

String getMacAddress() {
  uint8_t baseMac[6];
  // Get MAC address for WiFi station
  esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
  char baseMacChr[18] = {0};
  sprintf(baseMacChr, "%02X%02X%02X%02X%02X%02X", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
  return String(baseMacChr);
}

//********************************************************************//
//*********************** HeartBeat Function **************************//
//********************************************************************//
void HeartBeat() {
  //   Sink current to drain charge from watchdog circuit
  pinMode(trigWDTPin, OUTPUT);
  digitalWrite(trigWDTPin, LOW);
  // Led monitor for Heartbeat
  digitalWrite(ledHeartPIN, LOW);
  delay(300);
  digitalWrite(ledHeartPIN, HIGH);
  // Return to high-Z
  pinMode(trigWDTPin, INPUT);
  Serial.println("Heartbeat");
  SerialBT.println("Heartbeat");
}

void setup()
{
  HeartBeat();
  Serial.begin(115200);
  modbus.begin(9600, SERIAL_8N1, 16, 17);
  runner.init();
  Serial.println("Initialized scheduler");
  runner.addTask(t1);
  Serial.println("added t1");
  runner.addTask(t2);
  Serial.println("added t2");
  runner.addTask(t3);
  Serial.println("added t3");
  HeartBeat();
  delay(2000);
  t1.enable();  Serial.println("Enabled t1");
  t2.enable();  Serial.println("Enabled t2");
  t3.enable();  Serial.println("Enabled t3");
  HeartBeat();
  HOSTNAME.concat(getMacAddress());
  SerialBT.begin(HOSTNAME); //Bluetooth
  SerialBT.begin(HOSTNAME); //Bluetooth device name
  SerialBT.println(HOSTNAME);
  AISnb.debug = true;
  AISnb.setupDevice(serverPort);
  HeartBeat();
  _init();
  HeartBeat();
  _loadConfig();

  Serial.println();
  Serial.println(F("***********************************"));
  Serial.println("Initialize...");
  HeartBeat();
  setupWIFI();
  HeartBeat();
  setupOTA();
  HeartBeat();
}

void t2CallsendViaNBIOT ()
{
  meta = AISnb.getSignal();
  Serial.print("RSSI:"); Serial.println(meta.rssi);
  json = "";
  json.concat("{\"Tn\":\"");
  json.concat(deviceToken);
  json.concat("\",\"C_A\":");
  json.concat(meter.cA);
  json.concat(",\"C_B\":");
  json.concat(meter.cB);
  json.concat(",\"C_C\":");
  json.concat(meter.cC);
  json.concat(",\"V_A\":");
  json.concat(meter.vAN);
  json.concat(",\"V_B\":");
  json.concat(meter.vBN);
  json.concat(",\"V_C\":");
  json.concat(meter.vCN);

  json.concat(",\"P_A\":");
  json.concat(meter.apA);
  json.concat(",\"P_B\":");
  json.concat(meter.apB);
  json.concat(",\"P_C\":");
  json.concat(meter.apC);
  json.concat(",\"P_TOT\":");
  json.concat(meter.apT);
  json.concat(",\"Q_A\":");
  json.concat(meter.rpA);
  json.concat(",\"Q_B\":");
  json.concat(meter.rpB);
  json.concat(",\"Q_C\":");
  json.concat(meter.rpC);
  json.concat(",\"Q_TOT\":");
  json.concat(meter.rpT);
  json.concat(",\"S_A\":");
  json.concat(meter.appA);
  json.concat(",\"S_B\":");
  json.concat(meter.appB);
  json.concat(",\"S_C\":");
  json.concat(meter.appC);
  json.concat(",\"S_TOT\":");
  json.concat(meter.appT);

  json.concat(",\"PF\":");
  json.concat(meter.pf);
  json.concat(",\"KWH_A\":");
  json.concat(meter.poA);
  json.concat(",\"KWH_B\":");
  json.concat(meter.poB);
  json.concat(",\"KWH_C\":");
  json.concat(meter.poC);
  json.concat(",\"KWH_T\":");
  if (meter.poT.equals("nan")) {
    meter.poT = "0";
  }
  json.concat(meter.poT);
  json.concat(",\"F\":");
  json.concat(meter.freq);
  json.concat(",\"apTotal\":");
  json.concat(meter.apTotal);
  json.concat(",\"TEMP_AMB\":");
  json.concat(meter.Temp1);
  json.concat(",\"TEMP_OIL\":");
  json.concat(meter.Temp2);

  json.concat(",\"rssi\":");
  json.concat(meta.rssi);
  json.concat("}");
  Serial.println(json);
  SerialBT.println(json);
  //
  UDPSend udp = AISnb.sendUDPmsgStr(serverIP, serverPort, json);
  //  UDPReceive resp = AISnb.waitResponse();
  Serial.print("rssi:");
  Serial.println(meta.rssi);
  //  SerialBT.print("rssi:");
  //  SerialBT.println(meta.rssi);
  Serial.print("serverIP:");
  Serial.println(serverIP);
}

void t2CallsendViaNBIOT_Notification ()
{
  meta = AISnb.getSignal();
  Serial.print("RSSI:"); Serial.println(meta.rssi);
  json = "";
  json.concat("{\"Tn\":\"");
  json.concat(deviceToken);
  /////
  json.concat("\",\"cuA\":");
  json.concat(meter.cuA);
  json.concat(",\"cuB\":");
  json.concat(meter.cuB);
  json.concat(",\"cuC\":");
  json.concat(meter.cuC);
  json.concat(",\"cuW\":");
  json.concat(meter.cuW);

  json.concat(",\"vuAB\":");
  json.concat(meter.vuAB);
  json.concat(",\"vuBC\":");
  json.concat(meter.vuBC);
  json.concat(",\"vuCA\":");
  json.concat(meter.vuCA);
  json.concat(",\"vuLLW\":");
  json.concat(meter.vuLLW);

  json.concat(",\"vuAN\":");
  json.concat(meter.vuAN);
  json.concat(",\"vuBN\":");
  json.concat(meter.vuBN);
  json.concat(",\"vuCN\":");
  json.concat(meter.vuCN);
  json.concat(",\"vuLNW\":");
  json.concat(meter.vuLNW);
  json.concat(",\"d1\":");
  json.concat(meter.di1);
  json.concat(",\"d2\":");
  json.concat(meter.di2);
  json.concat(",\"d3\":");
  json.concat(meter.di3);
  json.concat(",\"d4\":");
  json.concat(meter.di4);
  json.concat(",\"d5\":");
  json.concat(meter.di5);
  json.concat(",\"d6\":");
  json.concat(meter.di6);
  json.concat(",\"d7\":");
  json.concat(meter.di7);
  json.concat(",\"d8\":");
  json.concat(meter.di8);

  json.concat(",\"rssi\":");
  json.concat(meta.rssi);
  json.concat(",\"csq\":");
  json.concat(meta.csq);
  json.concat("}");
  Serial.println(json);
  SerialBT.println(json);
  //
  UDPSend udp = AISnb.sendUDPmsgStr(serverIP, serverPort, json);
  UDPReceive resp = AISnb.waitResponse();
  Serial.print("rssi:");
  Serial.println(meta.rssi);
  SerialBT.print("rssi:");
  SerialBT.println(meta.rssi);
}

void readMeter()
{
  meter.cA = read_Modbus(c_A);
  meter.cB = read_Modbus(c_B);
  meter.cC = read_Modbus(c_C);
  meter.cN = read_Modbus(c_N);
  //  meter.cG = read_Modbus(c_G);
  meter.cAvg = read_Modbus(c_Avg);
  HeartBeat();
  meter.vAB = read_Modbus(v_A_B);
  meter.vBC = read_Modbus(v_B_C);
  meter.vCA = read_Modbus(v_C_A);
  meter.vLLAvg = read_Modbus(v_LL_Avg);
  meter.vAN = read_Modbus(v_A_N);
  meter.vBN = read_Modbus(v_B_N);
  meter.vCN = read_Modbus(v_C_N);
  meter.vLNAvg = read_Modbus(v_LN_Avg);
  meter.apTotal = read_Modbus(ap_Total);
  meter.freq = read_Modbus(v_Freq);
  HeartBeat();
  //
  meter.cuA = read_Modbus(cu_A);  // Current Unbalance A
  meter.cuB = read_Modbus(cu_B);  // Current Unbalance B
  meter.cuC = read_Modbus(cu_C);  // Current Unbalance C
  meter.cuW = read_Modbus(cu_W);  // Current Unbalance Worst

  meter.vuAB = read_Modbus(vu_AB);
  meter.vuBC = read_Modbus(vu_BC);
  meter.vuCA = read_Modbus(vu_CA);
  meter.vuLLW = read_Modbus(vu_LLW); // Voltage Unbalance L-L Worst
  HeartBeat();
  meter.vuAN = read_Modbus(vu_AN);
  meter.vuBN = read_Modbus(vu_BN);
  meter.vuCN = read_Modbus(vu_CN);
  meter.vuLNW = read_Modbus(vu_LNW); // Voltage Unbalance L-N Worst

  meter.apA = read_Modbus(ap_A);
  meter.apB = read_Modbus(ap_B);
  meter.apC = read_Modbus(ap_C);
  meter.apT = read_Modbus(ap_T); // Active Power Total
  meter.rpA = read_Modbus(rp_A);
  meter.rpB = read_Modbus(rp_B);
  meter.rpC = read_Modbus(rp_C);
  HeartBeat();
  meter.rpT = read_Modbus(rp_T); // Reactive Power Total
  meter.appA = read_Modbus(app_A);
  meter.appB = read_Modbus(app_B);
  meter.appC = read_Modbus(app_C);
  meter.appT = read_Modbus(app_T); // Apparent Power Total

  HeartBeat();
  meter.pfA = read_Modbus_PF(PF_A); // Power Factor
  meter.pfB = read_Modbus_PF(PF_B); // Power Factor
  meter.pfC = read_Modbus_PF(PF_C); // Power Factor
  meter.pfT = read_Modbus_PF(PF_T); //  Power Factor Total
  meter.pf = read_Modbus(PF);
  HeartBeat();
  meter.poA = read_Modbus(KWH_A); // Energy
  meter.poB = read_Modbus(KWH_B); // Energy
  meter.poC = read_Modbus(KWH_C); // Energy
  meter.poT = read_Modbus(KWH_T); //  Energy
  HeartBeat();
  meter.Temp1 = read_Modbus_1Byte(ID_Temp1, pvTemp) / 10;
  meter.Temp2 = read_Modbus_1Byte(ID_Temp2, pvTemp) / 10;
  HeartBeat();
  meter.di1 = read_Modbus_1Byte(ID_Drycontact1, DI1);
  meter.di2 = read_Modbus_1Byte(ID_Drycontact1, DI2);
  meter.di3 = read_Modbus_1Byte(ID_Drycontact1, DI3);
  meter.di4 = read_Modbus_1Byte(ID_Drycontact1, DI4);

  meter.di5 = read_Modbus_1Byte(ID_Drycontact2, DI5);
  meter.di6 = read_Modbus_1Byte(ID_Drycontact2, DI6);
  meter.di7 = read_Modbus_1Byte(ID_Drycontact2, DI7);
  meter.di8 = read_Modbus_1Byte(ID_Drycontact2, DI8);

  Serial.print("Current A: ");  Serial.print(meter.cA);  Serial.println(" Amp");
  Serial.print("Current B: ");  Serial.print(meter.cB);  Serial.println(" Amp");
  Serial.print("Current C: ");  Serial.print(meter.cC);  Serial.println(" Amp");
  Serial.print("Current N: ");  Serial.print(meter.cN);  Serial.println(" Amp");
  //  Serial.print("Current G: ");  Serial.print(meter.cG);  Serial.println(" Amp");
  Serial.print("Current Avg: ");  Serial.print(meter.cAvg);  Serial.println(" Amp");
  Serial.print("Voltage A-B: ");  Serial.print(meter.vAB);  Serial.println(" Volt");
  Serial.print("Voltage B-C: ");  Serial.print(meter.vBC);  Serial.println(" Volt");
  Serial.print("Voltage C-A: ");  Serial.print(meter.vCA);  Serial.println(" Volt");
  Serial.print("Voltage L-L Avg: ");  Serial.print(meter.vLLAvg);  Serial.println(" Volt");
  Serial.print("Voltage A-N: ");  Serial.print(meter.vAN );  Serial.println(" Volt");
  Serial.print("Voltage B-N: ");  Serial.print(meter.vBN );  Serial.println(" Volt");
  Serial.print("Voltage C-N: ");  Serial.print(meter.vCN );  Serial.println(" Volt");

  Serial.print(" pfA:");  Serial.println(meter.pfA);
  Serial.print(" pfB:");  Serial.println(meter.pfB);
  Serial.print(" pfC:");  Serial.println(meter.pfC);
  Serial.print(" pfT:");  Serial.println(meter.pfT);

  Serial.print(" poA:");  Serial.println(meter.poA);
  Serial.print(" poB:");  Serial.println(meter.poB);
  Serial.print(" poC:");  Serial.println(meter.poC);
  Serial.print(" poT:");  Serial.println(meter.poT);

  Serial.print("Frequency: ");  Serial.print(read_Modbus(v_Freq));  Serial.println(" Hz");

  Serial.print("Active Power Total: ");  Serial.print(read_Modbus(ap_Total));  Serial.println(" Kw");

  Serial.print("Temp1: ");  Serial.print(meter.Temp1);  Serial.println(" c");
  Serial.print("Temp2: ");  Serial.print(meter.Temp2);  Serial.println(" c");
  Serial.println("__________________________________________________________________");
  Serial.print("Volt UnbalanceAB:"); Serial.println(meter.vuAB);
  Serial.print("Volt UnbalanceBC:"); Serial.println(meter.vuBC);
  Serial.print("Volt UnbalanceCA:"); Serial.println(meter.vuCA);
  Serial.print("Volt UnbalanceLLW:"); Serial.println(meter.vuLLW);  //Voltage Unbalance L-N Worst
  Serial.print("DryContact1: ");  Serial.println(meter.di1);
  Serial.print("DryContact2: ");  Serial.println(meter.di2);
  Serial.print("DryContact3: ");  Serial.println(meter.di3);
  Serial.print("DryContact4: ");  Serial.println(meter.di4);
  Serial.print("DryContact5: ");  Serial.println(meter.di5);
  Serial.print("DryContact6: ");  Serial.println(meter.di6);
  Serial.print("DryContact7: ");  Serial.println(meter.di7);
  Serial.print("DryContact8: ");  Serial.println(meter.di8);
  Serial.println("");

  SerialBT.print("Current A: ");  SerialBT.print(meter.cA);  SerialBT.println(" Amp");
  SerialBT.print("Current B: ");  SerialBT.print(meter.cB);  SerialBT.println(" Amp");
  SerialBT.print("Current C: ");  SerialBT.print(meter.cC);  SerialBT.println(" Amp");
  SerialBT.print("Current N: ");  SerialBT.print(meter.cN);  SerialBT.println(" Amp");
  //  SerialBT.print("Current G: ");  SerialBT.print(meter.cG);  SerialBT.println(" Amp");
  SerialBT.print("Current Avg: ");  SerialBT.print(meter.cAvg);  SerialBT.println(" Amp");
  SerialBT.print("Voltage A-B: ");  SerialBT.print(meter.vAB);  SerialBT.println(" Volt");
  SerialBT.print("Voltage B-C: ");  SerialBT.print(meter.vBC);  SerialBT.println(" Volt");
  SerialBT.print("Voltage C-A: ");  SerialBT.print(meter.vCA);  SerialBT.println(" Volt");
  SerialBT.print("Voltage L-L Avg: ");  SerialBT.print(meter.vLLAvg);  SerialBT.println(" Volt");
  SerialBT.print("Voltage A-N: ");  SerialBT.print(meter.vAN );  SerialBT.println(" Volt");
  SerialBT.print("Voltage B-N: ");  SerialBT.print(meter.vBN );  SerialBT.println(" Volt");
  SerialBT.print("Voltage C-N: ");  SerialBT.print(meter.vCN );  SerialBT.println(" Volt");

  SerialBT.print(" pfA:");  SerialBT.println(meter.pfA);
  SerialBT.print(" pfB:");  SerialBT.println(meter.pfB);
  SerialBT.print(" pfC:");  SerialBT.println(meter.pfC);
  SerialBT.print(" pfT:");  SerialBT.println(meter.pfT);

  SerialBT.print(" poA:");  SerialBT.println(meter.poA);
  SerialBT.print(" poB:");  SerialBT.println(meter.poB);
  SerialBT.print(" poC:");  SerialBT.println(meter.poC);
  SerialBT.print(" poT:");  SerialBT.println(meter.poT);

  SerialBT.print("Frequency: ");  SerialBT.print(read_Modbus(v_Freq));  SerialBT.println(" Hz");

  SerialBT.print("Active Power Total: ");  SerialBT.print(read_Modbus(ap_Total));  SerialBT.println(" Kw");

  SerialBT.print("Temp1: ");  SerialBT.print(meter.Temp1);  SerialBT.println(" c");
  SerialBT.print("Temp2: ");  SerialBT.print(meter.Temp2);  SerialBT.println(" c");
  SerialBT.println("__________________________________________________________________");
  SerialBT.print("Volt UnbalanceAB:"); SerialBT.println(meter.vuAB);
  SerialBT.print("Volt UnbalanceBC:"); SerialBT.println(meter.vuBC);
  SerialBT.print("Volt UnbalanceCA:"); SerialBT.println(meter.vuCA);
  SerialBT.print("Volt UnbalanceLLW:"); SerialBT.println(meter.vuLLW);  //Voltage Unbalance L-N Worst
  SerialBT.print("DryContact1: ");  SerialBT.println(meter.di1);
  SerialBT.print("DryContact2: ");  SerialBT.println(meter.di2);
  SerialBT.print("DryContact3: ");  SerialBT.println(meter.di3);
  SerialBT.print("DryContact4: ");  SerialBT.println(meter.di4);
  SerialBT.print("DryContact5: ");  SerialBT.println(meter.di5);
  SerialBT.print("DryContact6: ");  SerialBT.println(meter.di6);
  SerialBT.print("DryContact7: ");  SerialBT.println(meter.di7);
  SerialBT.print("DryContact8: ");  SerialBT.println(meter.di8);
  SerialBT.println("");

  delay(500);
}

void t1CallgetMeter() {     // Update read all data
  readMeter();
}

float HexTofloat(uint32_t x)
{
  return (*(float*)&x);
}

float read_Modbus(uint16_t  REG)
{
  static uint32_t i;
  uint8_t j, result;
  uint16_t data[2];
  uint32_t value = 0;
  float val = 0.0;
  // communicate with Modbus slave ID 1 over Serial (port 2)
  node.begin(ID_PowerMeter, modbus);
  // slave: read (6) 16-bit registers starting at register 2 to RX buffer
  result = node.readHoldingRegisters(REG, 2);
  // do something with data if read is successful
  if (result == node.ku8MBSuccess)
  {
    for (j = 0; j < 2; j++)
    {
      data[j] = node.getResponseBuffer(j);
      //      SerialBT.print(REG); SerialBT.print(":"); SerialBT.print(j); SerialBT.print(":");  SerialBT.println(data[j]);
      //      Serial.print(REG); Serial.print(":"); Serial.print(j); Serial.print(":");  Serial.println(data[j]);
    }
    value = data[0];
    value = value << 16;
    value = value + data[1];
    val = HexTofloat(value);
    return val;
  } else {
    Serial.print("Connec modbus fail. REG >>> "); Serial.println(REG, HEX); // Debug
    //    delay(1000);
    return 0;
  }
}

String decToHex(int decValue) {
  String hexString = String(decValue, HEX);
  return hexString;
}

unsigned int hexToDec(String hexString) {
  unsigned int decValue = 0;
  int nextInt;
  for (int i = 0; i < hexString.length(); i++) {
    nextInt = int(hexString.charAt(i));
    if (nextInt >= 48 && nextInt <= 57) nextInt = map(nextInt, 48, 57, 0, 9);
    if (nextInt >= 65 && nextInt <= 70) nextInt = map(nextInt, 65, 70, 10, 15);
    if (nextInt >= 97 && nextInt <= 102) nextInt = map(nextInt, 97, 102, 10, 15);
    nextInt = constrain(nextInt, 0, 15);
    decValue = (decValue * 16) + nextInt;
  }
  return decValue;
}

int getResult( unsigned int x_high, unsigned int x_low)
{
  String hex2 = "";
  hex2.concat(decToHex(x_low));
  hex2.concat(decToHex(x_high));
  Serial.print("hex:");  Serial.println(hex2);
  Serial.print("dec:");  Serial.println(hexToDec(hex2));                                                               //rightmost 8 bits
  return hexToDec(hex2);
}

int read_Modbus_PF(uint16_t  REG)
{
  static uint32_t i;
  uint8_t j, result;
  uint16_t data[2];
  uint32_t value = 0;
  float val = 0.0;

  // communicate with Modbus slave ID 1 over Serial (port 2)
  node.begin(ID_PowerMeter, modbus);
  // slave: read (6) 16-bit registers starting at register 2 to RX buffer
  result = node.readHoldingRegisters(REG, 2);
  // do something with data if read is successful
  if (result == node.ku8MBSuccess)
  {
    for (j = 0; j < 2; j++)
    {
      data[j] = node.getResponseBuffer(j);
      //      SerialBT.print(REG); SerialBT.print(":"); SerialBT.print(j); SerialBT.print(":");  SerialBT.println(data[j]);
      //      Serial.print(REG); Serial.print(":"); Serial.print(j); Serial.print(":");  Serial.println(data[j]);
    }
    value = data[0];
    value = value << 16;
    value = value + data[1];
    Serial.print("value:");
    //      Serial.println(value);
    val = HexTofloat(value);
    return val;
  } else {
    Serial.print("Connec modbus fail. REG >>> "); Serial.println(REG, HEX); // Debug
    //    delay(1000);
    return 0;
  }
}

int read_Modbus_1Byte(char addr, uint16_t  REG)
{
  static uint32_t i;
  uint8_t j, result;
  uint16_t data[2];
  int32_t value = 0;
  float val = 0.0;
  // communicate with Modbus slave ID 1 over Serial (port 2)
  node.begin(addr, modbus);
  // slave: read (6) 16-bit registers starting at register 2 to RX buffer
  result = node.readHoldingRegisters(REG, 2);
  // do something with data if read is successful
  if (result == node.ku8MBSuccess)
  {
    for (j = 0; j < 2; j++)
    {
      data[j] = node.getResponseBuffer(j);
      SerialBT.print(REG); SerialBT.print(":"); SerialBT.print(j); SerialBT.print(":");  SerialBT.println(data[j]);
      Serial.print(REG); Serial.print(":"); Serial.print(j); Serial.print(":");  Serial.println(data[j]);
    }
    value = data[0];
    return value;
  } else {
    Serial.print("Connec modbus fail. REG >>> "); Serial.println(REG, HEX); // Debug
    //    delay(1000);
    return 0;
  }
}

void loop()
{
  runner.execute();
  ArduinoOTA.handle();
  ms = millis();
  if (ms % 600000 == 0)
  {
    Serial.println("Attach WiFi for，OTA "); Serial.println(WiFi.RSSI() );
    SerialBT.println("Attach WiFi for OTA"); SerialBT.println(WiFi.RSSI() );
    setupWIFI();
    HeartBeat();
    setupOTA();
  }
  if (ms % 60000 == 0)
  {
    Serial.println("Waiting for，OTA now"); Serial.println(WiFi.RSSI() );
    SerialBT.println("Waiting for, OTA now"); SerialBT.println(WiFi.RSSI() );
  }
  if (ms % 10000 == 0)
  {
    HeartBeat();
  }
}

/****************************************************
   [通用函数]ESP32 WiFi Kit 32事件处理
*/
void WiFiEvent(WiFiEvent_t event)
{
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch (event)
  {
    case SYSTEM_EVENT_WIFI_READY:               /**< ESP32 WiFi ready */
      break;
    case SYSTEM_EVENT_SCAN_DONE:                /**< ESP32 finish scanning AP */
      break;

    case SYSTEM_EVENT_STA_START:                /**< ESP32 station start */
      break;
    case SYSTEM_EVENT_STA_STOP:                 /**< ESP32 station stop */
      break;

    case SYSTEM_EVENT_STA_CONNECTED:            /**< ESP32 station connected to AP */
      break;

    case SYSTEM_EVENT_STA_DISCONNECTED:         /**< ESP32 station disconnected from AP */
      break;

    case SYSTEM_EVENT_STA_AUTHMODE_CHANGE:      /**< the auth mode of AP connected by ESP32 station changed */
      break;

    case SYSTEM_EVENT_STA_GOT_IP:               /**< ESP32 station got IP from connected AP */
    case SYSTEM_EVENT_STA_LOST_IP:              /**< ESP32 station lost IP and the IP is reset to 0 */
      break;

    case SYSTEM_EVENT_STA_WPS_ER_SUCCESS:       /**< ESP32 station wps succeeds in enrollee mode */
    case SYSTEM_EVENT_STA_WPS_ER_FAILED:        /**< ESP32 station wps fails in enrollee mode */
    case SYSTEM_EVENT_STA_WPS_ER_TIMEOUT:       /**< ESP32 station wps timeout in enrollee mode */
    case SYSTEM_EVENT_STA_WPS_ER_PIN:           /**< ESP32 station wps pin code in enrollee mode */
      break;

    case SYSTEM_EVENT_AP_START:                 /**< ESP32 soft-AP start */
    case SYSTEM_EVENT_AP_STOP:                  /**< ESP32 soft-AP stop */
    case SYSTEM_EVENT_AP_STACONNECTED:          /**< a station connected to ESP32 soft-AP */
    case SYSTEM_EVENT_AP_STADISCONNECTED:       /**< a station disconnected from ESP32 soft-AP */
    case SYSTEM_EVENT_AP_PROBEREQRECVED:        /**< Receive probe request packet in soft-AP interface */
    case SYSTEM_EVENT_AP_STA_GOT_IP6:           /**< ESP32 station or ap interface v6IP addr is preferred */
      break;

    case SYSTEM_EVENT_ETH_START:                /**< ESP32 ethernet start */
    case SYSTEM_EVENT_ETH_STOP:                 /**< ESP32 ethernet stop */
    case SYSTEM_EVENT_ETH_CONNECTED:            /**< ESP32 ethernet phy link up */
    case SYSTEM_EVENT_ETH_DISCONNECTED:         /**< ESP32 ethernet phy link down */
    case SYSTEM_EVENT_ETH_GOT_IP:               /**< ESP32 ethernet got IP from connected AP */
    case SYSTEM_EVENT_MAX:
      break;
  }
}
