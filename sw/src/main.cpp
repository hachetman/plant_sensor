
#include "secrets.h"
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <Wire.h>

WiFiClient espClient;
IPAddress server(192, 168, 1, 161);

PubSubClient client(server, 1883, espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

Adafruit_BME280 bme;

int setup_wifi() {
  int timeout = 0;
  String hostname = "espressiv_" + WiFi.macAddress();
  hostname.replace(":", "");
  Serial.println(hostname);

  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(wifi_ssid);
  Serial.println(WiFi.macAddress());
  WiFi.setHostname(hostname.c_str());
  WiFi.begin(wifi_ssid, wifi_password);

  while (WiFi.status() != WL_CONNECTED and (timeout < 20)) {
    delay(500);
    Serial.print(".");
    timeout++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    return 0;
  } else {
    Serial.println("WiFi not connected");
    return -1;
  }
}

void callback(char *topic, byte *message, unsigned int length) {}

void setup() {
  unsigned status;
  float temperature;
  float humidity;
  float pressure;
  float bat_voltage = 0;
  int sensor_voltage = 0;
  pinMode(14, OUTPUT);
  digitalWrite(14, HIGH);
  Serial.begin(115200);
  status = bme.begin(0x76, &Wire);
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring, "
                   "address, sensor ID!");
    Serial.print("SensorID was: 0x");
    Serial.println(bme.sensorID(), 16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 "
                 "or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    ESP.deepSleep(20e6);
  }

  bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                  Adafruit_BME280::SAMPLING_X1, // temperature
                  Adafruit_BME280::SAMPLING_X1, // pressure
                  Adafruit_BME280::SAMPLING_X1, // humidity
                  Adafruit_BME280::FILTER_OFF,
                  Adafruit_BME280::STANDBY_MS_1000);
  Serial.println("-- Default Test --");
  Serial.print("Temperature = ");
  temperature = bme.readTemperature();
  Serial.print(temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  pressure = bme.readPressure() / 100.0F;
  Serial.print(pressure);
  Serial.println(" hPa");

  Serial.print("Humidity = ");
  humidity = bme.readHumidity();
  Serial.print(humidity);
  Serial.println(" %");
  Serial.println();
  bat_voltage = float(analogRead(A0)) / 4096 * 6.66;
  sensor_voltage = analogRead(A4);
  Serial.print("Sensor = ");
  Serial.print(sensor_voltage);
  Serial.println();
  digitalWrite(14, LOW);

  if (setup_wifi() == 0) {
    // client.setCallback(callback);
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("plant_sensor", mqtt_user, mqtt_password)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
    }
    if (client.connected()) {
      String hostname = "espressiv_" + WiFi.macAddress();
      hostname.replace(":", "");
      StaticJsonDocument<1024> doc;
      doc["battery"] = bat_voltage;
      doc["temperature"] = temperature;
      doc["humidity"] = humidity;
      doc["pressure"] = pressure;
      doc["soil"] = sensor_voltage;
      doc["rssi"] = WiFi.RSSI();
      char buffer[256];
      serializeJson(doc, buffer);
      Serial.println(buffer);
      String topic = "tele/" + hostname + "/" + mqtt_info_topic;
      client.publish(topic.c_str(), buffer);
      espClient.flush();
      delay(500);
    }
    client.disconnect();
    espClient.flush();
    delay(1000);
    // delay(10);
    // wait until connection is closed completely
    while (client.state() != MQTT_DISCONNECTED) {
      delay(10);
    }
    Serial.println("Send messages, sleeping zZz");
  } else {
    Serial.println("There was a connection Problem");
  }
  ESP.deepSleep(300e6);
}

void loop() {}
