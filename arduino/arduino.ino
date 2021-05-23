// Wifi Duet3D monitor for the M5Stack core.

// Expects a SD card with config file named /duetbuddy.txt
// and fields like these:
//
// {
//   "wifi_ssid" : "xxx",
//   "wifi_password" : "yyy",
//   "printer_name" : "zzz",
//   "printer_ip" : "xxx.xxx.xxx.xxx"
// }
//
// Status Url: http://xx.xx.xx.xx/rr_status?type=3
//
// The program polls the duet's status URL and and displayes
// a few selected values such as state and progress.
//
// Arduino IDE configuration:
// --------------------------
// Board:             M5Stack-Core-ESP32
// Upload Speed:      921600
// Flash Frequencey:  80Mhz
// Flash Mode:        QIO
// Partition Schema:  No OTA (Large APP)
// Core Debug Level:  None


// TODO: add beeping in pause mode.

#include <Arduino.h>
#include <ArduinoJson.h>
#include <M5Stack.h>
#include <stdint.h>
#include <WiFi.h>
#include <FS.h>
#include <SD.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>
#include <pgmspace.h>

// 16 bits RGB565 colors. Picked using
// http://www.barth-dev.de/online/rgb565-color-picker/

static const uint16_t kBlack  = 0x0000;
static const uint16_t kBlue   = 0x001F;
static const uint16_t kRed    = 0xF800;
static const uint16_t kYellow = 0xFFE0;
static const uint16_t kGreen  = 0x07E0;
static const uint16_t kPurple = 0x801F;
static const uint16_t kGray   = 0xC618;

#define temp_width 30
#define temp_height 54
PROGMEM const unsigned char temp[] = {
  0x00, 0xF8, 0x07, 0x00, 0x00, 0xFE, 0x1F, 0x00, 0x80, 0xFF, 0x3F, 0x00, 
  0xC0, 0xFF, 0x7F, 0x00, 0xC0, 0xFF, 0xFF, 0x00, 0xE0, 0x0F, 0xFC, 0x01, 
  0xF0, 0x07, 0xF8, 0x01, 0xF0, 0x03, 0xF0, 0x03, 0xF0, 0x01, 0xE0, 0x03, 
  0xF8, 0x01, 0xE0, 0x03, 0xF8, 0x00, 0xE0, 0x07, 0xF8, 0x00, 0xC0, 0x07, 
  0xF8, 0x00, 0xC0, 0x07, 0xF8, 0x00, 0xC0, 0x07, 0xF8, 0x00, 0xC0, 0x07, 
  0xF8, 0x00, 0xC0, 0x07, 0xF8, 0x00, 0xC0, 0x07, 0xF8, 0x00, 0xC0, 0x07, 
  0xF8, 0x00, 0xC0, 0x07, 0xF8, 0x00, 0xC0, 0x07, 0xF8, 0xC0, 0xC0, 0x07, 
  0xF8, 0xE0, 0xC0, 0x07, 0xF8, 0xE0, 0xC1, 0x07, 0xF8, 0xE0, 0xC1, 0x07, 
  0xF8, 0xE0, 0xC1, 0x07, 0xF8, 0xE0, 0xC1, 0x07, 0xF8, 0xE0, 0xC1, 0x07, 
  0xF8, 0xE0, 0xC1, 0x07, 0xF8, 0xE0, 0xC1, 0x07, 0xF8, 0xE0, 0xC1, 0x07, 
  0xFC, 0xE0, 0xC1, 0x0F, 0xFC, 0xE0, 0xC1, 0x0F, 0x7E, 0xE0, 0x81, 0x1F, 
  0x3E, 0xE0, 0x01, 0x1F, 0x3F, 0xF0, 0x03, 0x1F, 0x1F, 0xF8, 0x07, 0x3F, 
  0x1F, 0xFC, 0x07, 0x3E, 0x1F, 0xFC, 0x0F, 0x3E, 0x1F, 0xFC, 0x0F, 0x3E, 
  0x1F, 0xFC, 0x0F, 0x3E, 0x1F, 0xFC, 0x0F, 0x3E, 0x1F, 0xF8, 0x07, 0x3E, 
  0x3F, 0xF0, 0x03, 0x1F, 0x3E, 0xC0, 0x00, 0x1F, 0x7E, 0x00, 0x80, 0x1F, 
  0xFE, 0x00, 0xC0, 0x0F, 0xFC, 0x01, 0xE0, 0x0F, 0xF8, 0x03, 0xF0, 0x07, 
  0xF0, 0x1F, 0xFE, 0x03, 0xF0, 0xFF, 0xFF, 0x01, 0xC0, 0xFF, 0xFF, 0x00, 
  0x80, 0xFF, 0x7F, 0x00, 0x00, 0xFE, 0x1F, 0x00, 0x00, 0xE0, 0x00, 0x00, 
  };

static WiFiMulti wifiMulti;
static HTTPClient http;

// When true, we do nothing and stay with the
// fatal error screen.
static bool fatal_error = false;
static bool wifi_connected = false;

// Counts consecutive duet connection errors. Used to filter
// out transient errors.
static int duet_error_allowance = 0;

struct Config {
  char wifi_ssid[10];
  char wifi_password[20];
  String printer_name;
  String printer_ip;
};

// Represents parsed duet status.
struct DuetStatus {
  String state_char;
  float progress_percents;
  float z_height;
  float temp1;
  float temp2;
};

// Per duet status char screen configurations.
// Colors are 16 bit RGB565 format.
struct StatusConfig {
  // The status char as returned by dutet. The special
  // char '*' indicates default catch all terminator.
  const String c;
  // User friendly status name.
  const char* text;
  const bool display_progress;
  const bool display_temps;
  const bool display_z;
  const uint16_t bg_color;
  const uint16_t text_color;
};

static const StatusConfig status_configs[] = {
  {"A", "PAUSED", true, true, false, kPurple, kBlack},
  {"B", "BUSY", false, true, false, kRed, kBlack},
  {"C", "CONFIG", false,  false, false, kYellow, kBlack},
  {"D", "PAUSING", true, true, false, kYellow, kBlack},
  {"F", "FLASHING", false, false, false, kYellow, kBlack},
  {"I", "IDLE", false, true, false,  kGreen, kBlack},  // fix
  {"P", "PRINTING", true, true, true, kRed, kBlack},
  {"R", "RESUMING", true, true, false, kYellow, kBlack},
  {"S", "PAUSED", true, true, false, kYellow, kBlack},
  // Terminator and default. Must be last.
  {"*", "[UNKNOWN]", false, false, false, kGray, kBlack}
};

// Finds the configuration for a given duet status char.
static const StatusConfig& decodeStatusChar(String c) {
  const StatusConfig* p = status_configs;
  for (;;) {
    // Actual match or a catch all default terminator.
    if (p->c == c || p->c == "*") {
      return *p;
    }
    p++;
  }
}

const char *filename = "/duetbuddy.txt";  // <- SD library uses 8.3 filenames
Config config;                              // <- global configuration object

// Loads the configuration from a file
void loadConfiguration(const char *filename, Config &config) {
  // Open file for reading
  File file = SD.open(filename);

  if (!file) {
    Serial.println(F("Failed to open file."));
    drawFatalErrorScreen("Failed to open file.");
    return;
  }

  // Allocate a temporary JsonDocument
  // Don't forget to change the capacity to match your requirements.
  // Use arduinojson.org/v6/assistant to compute the capacity.
  StaticJsonDocument<192> doc;

  // Deserialize the JSON document
  DeserializationError error = deserializeJson(doc, file);
  if (error) {
    Serial.println(F("Failed to read config file."));
    Serial.println(error.c_str());
    drawFatalErrorScreen("Failed to read config file.");
  }

  String protocol = "http://";
  String query = "/rr_status?type=3";
  String status_url = protocol + doc["printer_ip"].as<String>() + query;
  
  Serial.println(status_url);
  
  // Copy values from the JsonDocument to the Config
  strlcpy(config.wifi_ssid, doc["wifi_ssid"] | "", sizeof(config.wifi_ssid));
  strlcpy(config.wifi_password, doc["wifi_password"] | "", sizeof(config.wifi_password));
  config.printer_name = doc["printer_name"].as<String>();
  config.printer_ip = status_url;

  // Close the file (Curiously, File's destructor doesn't close the file)
  file.close();

  if (config.wifi_ssid == "" || config.wifi_password == "" || config.printer_ip == "") {
    drawFatalErrorScreen("Missing required config\n field.");
    return;
  }
}

// Common helper for header.
static void initHeader() {
  // Battery level
  M5.Power.begin();
  M5.Lcd.drawRect(10, 10, 28, 14, BLACK);
  
  switch (M5.Power.getBatteryLevel()) {
    case 25:
      M5.Lcd.fillRect(10, 10, 7, 14, BLACK);
      break;
    case 50:
      M5.Lcd.fillRect(10, 10, 14, 14, BLACK);
      break;
    case 75:
      M5.Lcd.fillRect(10, 10, 21, 14, BLACK);
      break;
    case 100:
      M5.Lcd.fillRect(10, 10, 28, 14, BLACK);
      break;
    default:
      break;
  }
  
  M5.Lcd.setCursor(42, 10, 1);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(BLACK);
  M5.Lcd.printf("%i%%", M5.Power.getBatteryLevel());

  // Printer Name  
  M5.Lcd.setTextDatum(CC_DATUM);
  M5.Lcd.drawRightString(config.printer_name, 310, 10, 1);
}

// Common helper for text screens.
static void initTextScreen() {
  M5.Lcd.fillScreen(kBlue);
  M5.Lcd.setTextColor(WHITE, kBlue);
  M5.Lcd.setTextSize(2);
}

// Common rendering for all fatal error messages.
static void drawFatalErrorScreen(const char* msg) {
  fatal_error = true;
  initTextScreen();
  M5.Lcd.setCursor(0, 12, 1);
  M5.Lcd.print(" FATAL ERROR.\n\n ");
  M5.Lcd.print(msg);
}

static void drawNoWifiScreen() {
  initTextScreen();
  wifi_connected = false;
  M5.Lcd.setTextDatum(CC_DATUM);
  M5.Lcd.drawString("Connecting to WIFI", (int)(M5.Lcd.width()/2), (int)(M5.Lcd.height()/2), 2);
}

static void drawWifiConnectedScreen() {
  initTextScreen();
  wifi_connected = true;
  //M5.Lcd.print(" WIFI connected.\n\n\n Connecting to duet.");
  M5.Lcd.setTextDatum(CC_DATUM);
  M5.Lcd.drawString("WIFI connected", (int)(M5.Lcd.width()/2), (int)((M5.Lcd.height()/2)-16), 2);
  M5.Lcd.drawString("Connecting to duet", (int)(M5.Lcd.width()/2), (int)((M5.Lcd.height()/2)+16), 2);
}

static void drawNoHttpConnectionScreen(const char* error_message) {
  initTextScreen();
  M5.Lcd.setCursor(0, 12, 1);
  M5.Lcd.print(" Duet connection failed.\n\n\n ");
  M5.Lcd.print(error_message);  
  M5.Lcd.setCursor(0, 160, 2);
  M5.Lcd.print(" ");
  M5.Lcd.setTextSize(1);
  //M5.Lcd.printf("[%s]", config_parser.ParsedData().status_url.c_str());
}

static void drawBadDuetResponseScreen() {
  initTextScreen();
  //M5.Lcd.print(" Bad response from duet.");
  M5.Lcd.setTextDatum(CC_DATUM);
  M5.Lcd.drawString("Bad response from duet", (int)(M5.Lcd.width()/2), (int)(M5.Lcd.height()/2), 2);
}

static void drawInfoScreen(const DuetStatus& duet_status) {
  // Map the duet status char to screen configuration.
  Serial.println(duet_status.state_char);

  const StatusConfig& statusConfig = decodeStatusChar(duet_status.state_char);
  M5.Lcd.fillScreen(statusConfig.bg_color);
  M5.Lcd.setTextColor(statusConfig.text_color);
  
  initHeader();

  // Status name
  M5.Lcd.setTextSize(5);
  M5.Lcd.setTextDatum(CC_DATUM);
  M5.Lcd.drawString(statusConfig.text, (int)(M5.Lcd.width()/2), 70, 1);

  if (statusConfig.display_progress) {
    char str[64];
    snprintf(str, 64, "%.1f%%", duet_status.progress_percents);
    M5.Lcd.setCursor(20, 100, 1);
    M5.Lcd.setTextSize(3);
    M5.Lcd.setTextDatum(CC_DATUM);
    M5.Lcd.drawString(str, (int)(M5.Lcd.width()/2), 120, 1);
    // Progress bar
    M5.Lcd.drawRect(20, 140, 280, 14, BLACK);
    M5.Lcd.fillRect(20, 140, (int)(2.8*duet_status.progress_percents), 14, BLACK);
  }
  
  if (statusConfig.display_z || statusConfig.display_temps) {
    M5.Lcd.drawFastHLine(0, 165, 320, BLACK);
  }

  if (statusConfig.display_z) {
    char str[15];
    snprintf(str, 15, "%0.2fmm", duet_status.z_height);
    M5.Lcd.setTextDatum(CC_DATUM);
    M5.Lcd.setTextSize(2);
    M5.Lcd.drawRightString("Z Height", 310, 180, 1);
    M5.Lcd.setTextSize(3);
    M5.Lcd.drawRightString(str, 310, 205, 1);
  }

  if (statusConfig.display_temps) {
    M5.Lcd.drawFastVLine((int)(M5.Lcd.width()/2), 165, 75, BLACK);
    M5.Lcd.drawXBitmap(10, 175, temp, temp_width, temp_height, BLACK);
    M5.Lcd.setCursor(130, 215, 1);
    M5.Lcd.setTextSize(3);
    M5.Lcd.setCursor(50, 175, 1);
    M5.Lcd.printf("%0.1f", duet_status.temp1);
    M5.Lcd.setCursor(50, 205, 1);
    M5.Lcd.printf("%0.1f", duet_status.temp2);
  }
}

void setup() {
  // put your setup code here, to run once:
  M5.begin();
  Serial.println();
  Serial.println("setup()");

  // Initialize SD library
  while (!SD.begin()) {
    Serial.println(F("Failed to initialize SD library"));
    drawFatalErrorScreen("SD card not found.");
    delay(1000);
  }

  // Should load default config if run for the first time
  Serial.println(F("Loading configuration..."));
  loadConfiguration(filename, config);

  // Setup Wifi AP.
  if (!wifiMulti.addAP(config.wifi_ssid, config.wifi_password)) {
    Serial.println(F("Failed to add Wifi AP"));
    drawFatalErrorScreen("Wifi setup failed.");
    return;
  }

  // Initialization done OK. Next we will connect to Wifi within loop().
  drawNoWifiScreen();
}

void loop() {
  // Fatal error. Stay in this screen.
  if (fatal_error) {
    delay(10000);
    return;
  }

  // No wifi connection. Try again.
  if ((wifiMulti.run(10000) != WL_CONNECTED)) {
    drawNoWifiScreen();
    delay(500);
    return;
  }

  // Just connected to WIFI, give a short notice.
  if (!wifi_connected) {
    drawWifiConnectedScreen();
    duet_error_allowance = 0;
    delay(500);
    return;
  }

  // Connect to duet and send a Get status http request.
  http.useHTTP10(true);
  http.begin(config.printer_ip);
  Serial.print("[HTTP] GET ");
  Serial.println(config.printer_ip);
  const int httpCode = http.GET();
  Serial.printf("[HTTP] GET... code: %d\n", httpCode);

  // No HTTP connection or an HTTP error status.
  if (httpCode != HTTP_CODE_OK) {
    // If already had a consecutive connection error.
    if (duet_error_allowance > 0) {
      Serial.println("Ignoring duet connection error");
      duet_error_allowance--;
    } else {
      drawNoHttpConnectionScreen(http.errorToString(httpCode).c_str());
    }
    http.end();  // remember to close the http client.
    delay(500);
    return;
  }

  StaticJsonDocument<96> filter;
  filter["status"] = true;
  filter["coords"]["xyz"] = true;
  filter["temps"]["current"] = true;
  filter["fractionPrinted"] = true;

  DynamicJsonDocument doc(3072);

  // Deserialize the JSON document
  DeserializationError error = deserializeJson(doc, http.getStream());
  if (error) {
    Serial.println(F("Failed to read status."));
    Serial.println(error.c_str());
    if (duet_error_allowance > 0) {
      Serial.println("Ignoring duet response parsing error.");
      duet_error_allowance--;
    } else {
      Serial.println("Message not ok");
      drawBadDuetResponseScreen();
    }
  }

  String status = doc["status"];

  DuetStatus current_duet_status;
  current_duet_status.state_char = status;
  current_duet_status.progress_percents = doc["fractionPrinted"].as<float>();
  current_duet_status.z_height = doc["coords"]["xyz"][2].as<float>();
  current_duet_status.temp1 = doc["temps"]["current"][0].as<float>();
  current_duet_status.temp2 = doc["temps"]["current"][1].as<float>();
  
  Serial.println("Done parsing, closing http");
  http.end();

  // We got a valid json response.
  const DuetStatus& duet_status = current_duet_status;
  drawInfoScreen(current_duet_status);
  // We will allow one transient duet error.
  duet_error_allowance = 1;

  delay(5000);
}
