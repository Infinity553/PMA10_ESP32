/*
 * Made for Waveshare ESP32 S3 Zero
 * made to read out an older M&C PMA 10/15 O2 Analyzer with 0-1V Analog Output
 *
 * Functionality:
 * - Analog In readout with linear interpolation on GPIO1.
 * - 11 Point Voltage Calibration via WebUI.
 * - Functionality to add a BME280 sensor for ambient values
 * - WebUI to show current Values and connect to a WIFI as well as an MQTT broker.
 * - Starts in AP mode for first Config or if saved wifi is not available.
 */

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include <ArduinoJson.h>

// pinout conf
#define ANALOG_PIN 1
#define I2C_SDA_PIN 3
#define I2C_SCL_PIN 2
#define NUM_CAL_POINTS 11

// variables
Preferences preferences;
struct Config {
  char wifi_ssid[32];
  char wifi_password[64];
  char mqtt_server[64];
  int mqtt_port;
  char mqtt_user[32];
  char mqtt_password[64];
  char mqtt_topic[64];
};
Config config;

// Cal. Data
struct CalPoint {
  uint16_t adc;
  uint16_t mv;
};
CalPoint calData[NUM_CAL_POINTS];
bool calibrationIsValid = false;
int calibrationStep = -1; // -1: idle, 0-10: calibrating, 11: done

Adafruit_BME280 bme;
float temperature = 0.0;
float humidity = 0.0;
float pressure = 0.0;
float analog_percentage = 0.0;
int current_voltage_mv = 0; // Aktuell gemessene Spannung

AsyncWebServer server(80);
WiFiClient espClient;
PubSubClient mqttClient(espClient);
unsigned long lastSensorRead = 0;
unsigned long lastMqttPublish = 0;
const long sensorReadInterval = 2000;
const long mqttPublishInterval = 10000;
const char* ap_ssid = "PMA10_Setup";

// functions
void loadConfiguration();
void saveConfiguration();
void loadCalibration();
void saveCalibration();
void setupWiFi();
void setupSensors();
void readSensors();
int getCalibratedVoltage(int raw_adc);
void setupWebServer();
void handleCalibrateRequest(AsyncWebServerRequest *request);
void handleCalibrateAction(AsyncWebServerRequest *request);
void setupMqtt();
void reconnectMqtt();
void publishMqtt();


// setup
void setup() {
  Serial.begin(115200);
  Serial.println("\nPMA Sensor-Gateway starting...");

  analogReadResolution(12);
  analogSetPinAttenuation(ANALOG_PIN, ADC_11db);

  loadConfiguration();
  loadCalibration();
  setupWiFi();
  setupSensors();
  setupWebServer();
  setupMqtt();

  lastSensorRead = millis();
}

void loop() {
  unsigned long currentMillis = millis();
  if (WiFi.status() == WL_CONNECTED) {
    if (!mqttClient.connected()) {
      reconnectMqtt();
    }
    mqttClient.loop();
  }
  if (currentMillis - lastSensorRead >= sensorReadInterval) {
    readSensors();
    lastSensorRead = currentMillis;
  }
  if (currentMillis - lastMqttPublish >= mqttPublishInterval && mqttClient.connected()) {
    publishMqtt();
    lastMqttPublish = currentMillis;
  }
}

// conf. management
void loadConfiguration() {
  preferences.begin("config", true);
  preferences.getString("wifi_ssid", config.wifi_ssid, sizeof(config.wifi_ssid));
  preferences.getString("wifi_password", config.wifi_password, sizeof(config.wifi_password));
  preferences.getString("mqtt_server", config.mqtt_server, sizeof(config.mqtt_server));
  config.mqtt_port = preferences.getInt("mqtt_port", 1883);
  preferences.getString("mqtt_user", config.mqtt_user, sizeof(config.mqtt_user));
  preferences.getString("mqtt_password", config.mqtt_password, sizeof(config.mqtt_password));
  preferences.getString("mqtt_topic", config.mqtt_topic, sizeof(config.mqtt_topic));
  preferences.end();
  Serial.println("Conf. read");
}

void saveConfiguration() {
  preferences.begin("config", false);
  preferences.putString("wifi_ssid", config.wifi_ssid);
  preferences.putString("wifi_password", config.wifi_password);
  preferences.putString("mqtt_server", config.mqtt_server);
  preferences.putInt("mqtt_port", config.mqtt_port);
  preferences.putString("mqtt_user", config.mqtt_user);
  preferences.putString("mqtt_password", config.mqtt_password);
  preferences.putString("mqtt_topic", config.mqtt_topic);
  preferences.end();
  Serial.println("Conf. saved");
}


// Cal.
void loadCalibration() {
  preferences.begin("adc_cal", true); // Read-only
  calibrationIsValid = preferences.getBool("valid", false);
  if (calibrationIsValid) {
    for (int i = 0; i < NUM_CAL_POINTS; i++) {
      String key_adc = "adc" + String(i);
      String key_mv = "mv" + String(i);
      calData[i].adc = preferences.getUShort(key_adc.c_str(), 0);
      calData[i].mv = preferences.getUShort(key_mv.c_str(), 0);
    }
    Serial.println("ADC Cal. was loaded");
  } else {
    Serial.println("No ADC Cal. found");
  }
  preferences.end();
}

void saveCalibration() {
  preferences.begin("adc_cal", false); // Read-write
  preferences.putBool("valid", true);
  for (int i = 0; i < NUM_CAL_POINTS; i++) {
    String key_adc = "adc" + String(i);
    String key_mv = "mv" + String(i);
    preferences.putUShort(key_adc.c_str(), calData[i].adc);
    preferences.putUShort(key_mv.c_str(), calData[i].mv);
  }
  preferences.end();
  calibrationIsValid = true;
  Serial.println("ADC Cal. saved");
}


// Sensor and Measurement Logic
void setupSensors() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  if (!bme.begin(0x76, &Wire)) {
    Serial.println("No BME280 found");
  } else {
    Serial.println("BME280 initialized");
  }
}

int getCalibratedVoltage(int raw_adc) {
    if (!calibrationIsValid) {
        // Fallback: rough estimate
        return map(raw_adc, 0, 4095, 0, 2500);
    }

    if (raw_adc <= calData[0].adc) {
        return calData[0].mv;
    }
    if (raw_adc >= calData[NUM_CAL_POINTS - 1].adc) {
        return calData[NUM_CAL_POINTS - 1].mv;
    }

    for (int i = 1; i < NUM_CAL_POINTS; i++) {
        if (raw_adc <= calData[i].adc) {
            long val = map(raw_adc, calData[i-1].adc, calData[i].adc, calData[i-1].mv, calData[i].mv);
            return val;
        }
    }
    return calData[NUM_CAL_POINTS - 1].mv;
}


void readSensors() {
  temperature = bme.readTemperature();
  humidity = bme.readHumidity();
  pressure = bme.readPressure() / 100.0F;

  int raw_adc = analogRead(ANALOG_PIN);
  current_voltage_mv = getCalibratedVoltage(raw_adc);

  analog_percentage = (current_voltage_mv / 1000.0) * 100.0;
  if (analog_percentage > 100.0) analog_percentage = 100.0;
  if (analog_percentage < 0.0) analog_percentage = 0.0;

  Serial.printf("Sensor values: Voltage=%dmV, Analog=%.1f%%, Temp=%.2f°C, Hum=%.2f%%, Pres=%.2fhPa\n", 
                current_voltage_mv, analog_percentage, temperature, humidity, pressure);
}

// Wifi
void setupWiFi() {
  if (strlen(config.wifi_ssid) == 0) {
    Serial.println("No Wifi conf. found, starting in AP mode");
    WiFi.softAP(ap_ssid);
    Serial.print("AP SSID: "); Serial.println(ap_ssid);
    Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());
  } else {
    Serial.print("Connecting to wifi: "); Serial.println(config.wifi_ssid);
    WiFi.begin(config.wifi_ssid, config.wifi_password);
    int retries = 0;
    while (WiFi.status() != WL_CONNECTED && retries < 20) {
      delay(500); Serial.print("."); retries++;
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nConnected!");
      Serial.print("IP-Address: "); Serial.println(WiFi.localIP());
    } else {
      Serial.println("\nConnection failed. Starting Access Point...");
      WiFi.mode(WIFI_AP);
      WiFi.softAP(ap_ssid);
      Serial.print("AP SSID: "); Serial.println(ap_ssid);
      Serial.print("AP IP-Address: "); Serial.println(WiFi.softAPIP());
    }
  }
}

// Webserver UI
void setupWebServer() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    String html = R"rawliteral(
<!DOCTYPE HTML><html><head><title>PMA Dashboard</title><meta name="viewport" content="width=device-width, initial-scale=1"><style>body { font-family: Arial, sans-serif; margin: 0; padding: 20px; background-color: #f4f4f4; }.container { max-width: 600px; margin: auto; background: white; padding: 20px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }h1 { color: #333; }.sensor { background: #eee; padding: 15px; margin-bottom: 10px; border-radius: 5px; display: flex; justify-content: space-between; align-items: center; }.sensor-label { font-weight: bold; }.sensor-value { font-size: 1.2em; color: #007BFF; }.footer { text-align: center; margin-top: 20px; }.footer a { color: #007BFF; text-decoration: none; }</style><script>setInterval(function() {var xhttp = new XMLHttpRequest();xhttp.onreadystatechange = function() {if (this.readyState == 4 && this.status == 200) {var data = JSON.parse(this.responseText);document.getElementById("analog").innerHTML = data.analog_percent;document.getElementById("temp").innerHTML = data.temperature_c;document.getElementById("hum").innerHTML = data.humidity_percent;document.getElementById("pres").innerHTML = data.pressure_hpa;}};xhttp.open("GET", "/values", true);xhttp.send();}, 2000);</script></head><body><div class="container"><h1>PMA Dashboard</h1><div class="sensor"><span class="sensor-label">O2:</span><span class="sensor-value"><span id="analog">0.0</span> %</span></div><div class="sensor"><span class="sensor-label">Temperature:</span><span class="sensor-value"><span id="temp">0.00</span> &deg;C</span></div><div class="sensor"><span class="sensor-label">Humidity:</span><span class="sensor-value"><span id="hum">0.00</span> %</span></div><div class="sensor"><span class="sensor-label">Pressure:</span><span class="sensor-value"><span id="pres">0.00</span> hPa</span></div><div class="footer"><a href="/settings">Settings</a></div></div></body></html>
)rawliteral";
    request->send(200, "text/html", html);
  });
  server.on("/values", HTTP_GET, [](AsyncWebServerRequest *request){
    StaticJsonDocument<256> doc;
    doc["analog_percent"] = round(analog_percentage * 10) / 10.0;
    doc["temperature_c"] = round(temperature * 100) / 100.0;
    doc["humidity_percent"] = round(humidity * 100) / 100.0;
    doc["pressure_hpa"] = round(pressure * 100) / 100.0;
    String json;
    serializeJson(doc, json);
    request->send(200, "application/json", json);
  });
  server.on("/settings", HTTP_GET, [](AsyncWebServerRequest *request){
    String html = R"rawliteral(
      <!DOCTYPE HTML><html><head>
      <title>Settings</title>
      <meta name="viewport" content="width=device-width, initial-scale=1">
      <style>
        body { font-family: Arial, sans-serif; margin: 0; padding: 20px; background-color: #f4f4f4; }
        .container { max-width: 600px; margin: auto; background: white; padding: 20px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }
        h1, h2 { color: #333; }
        form { display: flex; flex-direction: column; }
        label { font-weight: bold; margin-top: 10px; }
        input { padding: 10px; margin-top: 5px; border: 1px solid #ccc; border-radius: 4px; }
        button { background-color: #007BFF; color: white; padding: 12px; border: none; border-radius: 4px; cursor: pointer; margin-top: 20px; font-size: 1em; }
        button:hover { background-color: #0056b3; }
        .footer { text-align: center; margin-top: 20px; }
        .footer a { color: #007BFF; text-decoration: none; margin: 0 10px; }
        .cal-link { display:block; text-align:center; margin-top: 20px; padding: 10px; background-color: #6c757d; color: white; border-radius: 4px; text-decoration: none; }
      </style>
      </head><body>
      <div class="container">
        <h1>WiFi & MQTT Settings</h1>
        <form action="/save" method="POST">
          <h2>WLAN</h2>
          <label for="ssid">SSID</label>
          <input type="text" id="ssid" name="ssid" value="%WIFI_SSID%">
          <label for="pass">Password</label>
          <input type="password" id="pass" name="pass">
          <h2>MQTT Broker</h2>
          <label for="server">Server IP / Hostname</label>
          <input type="text" id="server" name="server" value="%MQTT_SERVER%">
          <label for="port">Port</label>
          <input type="number" id="port" name="port" value="%MQTT_PORT%">
          <label for="topic">Topic</label>
          <input type="text" id="topic" name="topic" value="%MQTT_TOPIC%">
          <label for="user">User (optional)</label>
          <input type="text" id="user" name="user" value="%MQTT_USER%">
          <label for="mqtt_pass">Password (optional)</label>
          <input type="password" id="mqtt_pass" name="mqtt_pass">
          <button type="submit">Save & Restart</button>
        </form>
        <a href="/calibrate" class="cal-link">ADC Calibration</a>
        <div class="footer"><a href="/">Back to Dashboard</a></div>
      </div></body></html>
    )rawliteral";
    html.replace("%WIFI_SSID%", config.wifi_ssid);
    html.replace("%MQTT_SERVER%", config.mqtt_server);
    html.replace("%MQTT_PORT%", String(config.mqtt_port));
    html.replace("%MQTT_TOPIC%", config.mqtt_topic);
    html.replace("%MQTT_USER%", config.mqtt_user);
    request->send(200, "text/html", html);
  });
  server.on("/save", HTTP_POST, [](AsyncWebServerRequest *request){
    if(request->hasParam("ssid", true)) strncpy(config.wifi_ssid, request->getParam("ssid", true)->value().c_str(), sizeof(config.wifi_ssid));
    if(request->hasParam("pass", true)) strncpy(config.wifi_password, request->getParam("pass", true)->value().c_str(), sizeof(config.wifi_password));
    if(request->hasParam("server", true)) strncpy(config.mqtt_server, request->getParam("server", true)->value().c_str(), sizeof(config.mqtt_server));
    if(request->hasParam("port", true)) config.mqtt_port = request->getParam("port", true)->value().toInt();
    if(request->hasParam("user", true)) strncpy(config.mqtt_user, request->getParam("user", true)->value().c_str(), sizeof(config.mqtt_user));
    if(request->hasParam("mqtt_pass", true)) strncpy(config.mqtt_password, request->getParam("mqtt_pass", true)->value().c_str(), sizeof(config.mqtt_password));
    if(request->hasParam("topic", true)) strncpy(config.mqtt_topic, request->getParam("topic", true)->value().c_str(), sizeof(config.mqtt_topic));
    saveConfiguration();
    String html = R"rawliteral(
<!DOCTYPE HTML><html><head><title>Saved</title><meta http-equiv="refresh" content="5;url=/"><style>body { font-family: Arial, sans-serif; padding: 50px; text-align: center; } .message { font-size: 1.2em; }</style></head><body><div class="message">Settings saved.<br>ESP is restarting</div></body></html>
)rawliteral";
    request->send(200, "text/html", html);
    delay(1000);
    ESP.restart();
  });
  
  // --- NEUE HANDLER FÜR DIE KALIBRIERUNG ---
  server.on("/calibrate", HTTP_GET, handleCalibrateRequest);
  server.on("/calibrate_action", HTTP_POST, handleCalibrateAction);
  server.on("/live_voltage", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", String(current_voltage_mv));
  });

  server.begin();
  Serial.println("Webserver started.");
}

void handleCalibrateRequest(AsyncWebServerRequest *request) {
    String html = R"rawliteral(
    <!DOCTYPE HTML><html><head><title>ADC Calibration</title><meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
      body { font-family: Arial, sans-serif; margin: 0; padding: 20px; background-color: #f4f4f4; text-align: center; }
      .container { max-width: 600px; margin: auto; background: white; padding: 20px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }
      h1, h2 { color: #333; }
      .voltage-display { font-size: 2.5em; color: #007BFF; margin: 20px 0; font-weight: bold; }
      .instructions { font-size: 1.2em; margin-bottom: 20px; }
      button { background-color: #007BFF; color: white; padding: 15px 30px; border: none; border-radius: 4px; cursor: pointer; margin: 5px; font-size: 1.1em; }
      button.start { background-color: #28a745; }
      button.cancel { background-color: #dc3545; }
      .footer a { color: #007BFF; text-decoration: none; }
    </style>
    <script>
      setInterval(function() {
        var xhttp = new XMLHttpRequest();
        xhttp.onreadystatechange = function() {
          if (this.readyState == 4 && this.status == 200) {
            document.getElementById("live_v").innerHTML = this.responseText;
          }
        };
        xhttp.open("GET", "/live_voltage", true);
        xhttp.send();
      }, 1000);
    </script>
    </head><body><div class="container">
    <h1>ADC Calibration</h1>
    )rawliteral";

    if (calibrationStep == -1) { // Startbildschirm
        html += R"rawliteral(
        <p class="instructions">Start the 11 point linearity calibration</p>
        <form action="/calibrate_action" method="POST">
          <button type="submit" name="action" value="start" class="start">Start</button>
        </form>
        )rawliteral";
    } else if (calibrationStep < NUM_CAL_POINTS) { // Kalibrierungsschritte
        float target_v = calibrationStep * 0.1;
        html += "<p class='instructions'>Step " + String(calibrationStep + 1) + "/" + String(NUM_CAL_POINTS) + "</p>";
        html += "<p class='instructions'>Please supply exaclty <strong>" + String(target_v, 1) + " V</strong> to the analog input.</p>";
        html += "<h2>actual measured Voltage:</h2>";
        html += "<div class='voltage-display'><span id='live_v'>...</span> mV</div>";
        html += R"rawliteral(
        <form action="/calibrate_action" method="POST">
          <button type="submit" name="action" value="save">Save and next</button>
          <button type="submit" name="action" value="cancel" class="cancel">Cancel</button>
        </form>
        )rawliteral";
    } else { // Fertig
        html += R"rawliteral(
        <h2>Calibration sucessfull!</h2>
        <p class="instructions">Calibration is saved and now in use.</p>
        <form action="/calibrate_action" method="POST">
          <button type="submit" name="action" value="finish">Quit</button>
        </form>
        )rawliteral";
    }
    
    html += R"rawliteral(
    <div class="footer" style="margin-top: 30px;"><a href="/">Back to the Dashboard</a></div>
    </div></body></html>
    )rawliteral";
    request->send(200, "text/html", html);
}

void handleCalibrateAction(AsyncWebServerRequest *request) {
    if (request->hasParam("action", true)) {
        String action = request->getParam("action", true)->value();
        if (action == "start") {
            calibrationStep = 0;
            calibrationIsValid = false;
            for(int i=0; i<NUM_CAL_POINTS; i++){
              calData[i] = {0, 0};
            }
        } else if (action == "save" && calibrationStep >= 0 && calibrationStep < NUM_CAL_POINTS) {
            calData[calibrationStep].adc = analogRead(ANALOG_PIN);
            calData[calibrationStep].mv = calibrationStep * 100; // Zielspannung in mV (0, 100, 200...)
            Serial.printf("Punkt %d saved: ADC=%d, mV=%d\n", calibrationStep, calData[calibrationStep].adc, calData[calibrationStep].mv);
            calibrationStep++;
            if (calibrationStep >= NUM_CAL_POINTS) {
                saveCalibration();
            }
        } else if (action == "cancel") {
            calibrationStep = -1;
            loadCalibration();
        } else if (action == "finish") {
            calibrationStep = -1;
        }
    }
    request->redirect("/calibrate");
}


// --- MQTT-Management ---
void setupMqtt() {
  if (strlen(config.mqtt_server) > 0) {
    mqttClient.setServer(config.mqtt_server, config.mqtt_port);
  }
}

void reconnectMqtt() {
  if (strlen(config.mqtt_server) == 0) return;
  
  Serial.print("Versuche MQTT-connecting...");
  String clientId = "PMA-";
  clientId += String(random(0xffff), HEX);
  
  if (mqttClient.connect(clientId.c_str(), config.mqtt_user, config.mqtt_password)) {
    Serial.println("connected!");
  } else {
    Serial.print("failed, rc=");
    Serial.print(mqttClient.state());
    Serial.println(" - retry in 5 seconds");
  }
}

void publishMqtt() {
  if (strlen(config.mqtt_topic) == 0 || !mqttClient.connected()) return;
  
  StaticJsonDocument<256> doc;
  doc["analog_percent"] = round(analog_percentage * 10) / 10.0;
  doc["temperature_c"] = round(temperature * 100) / 100.0;
  doc["humidity_percent"] = round(humidity * 100) / 100.0;
  doc["pressure_hpa"] = round(pressure * 100) / 100.0;

  char buffer[256];
  size_t n = serializeJson(doc, buffer);

  if (mqttClient.publish(config.mqtt_topic, buffer, n)) {
    Serial.println("MQTT message send.");
  } else {
    Serial.println("Error while publishing message");
  }
}