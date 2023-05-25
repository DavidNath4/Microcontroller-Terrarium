#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include <PubSubClient.h>
#include <WiFi.h>

const char* ssid = "Rumah_Sitorus";
const char* password = "david221101";
const char* mqttServer = "test.mosquitto.org";
const int mqttPort = 1883;
const char* mqttUser = "";
const char* mqttPassword = "";
const char* mqttSubTopic = "topic/esp";
const char* mqttPubTopic = "topic/server";

const char deviceId[] = "gVMtxr";
int packetCount;

WiFiClient espClient;
PubSubClient subclient(espClient);
Adafruit_MQTT_Client pubClient(&espClient, mqttServer, mqttPort, mqttUser,
                               mqttPassword);

unsigned long previousPubMillis = 0;
const unsigned long pubInterval = 5000;

// declare function
void connectToWiFi();
void connectToPubSubMqtt();
void connectToAdafruitMqtt();
void publishMessage(const char* topic, char* payload);
void callback(char* topic, byte* payload, unsigned int length);
float randomFloat(float min, float max);
float tempValue();
float humdValue();
bool getRandomBoolean();

// setup function
void setup() {
  Serial.begin(115200);

  connectToWiFi();
  connectToPubSubMqtt();
  connectToAdafruitMqtt();
}

// main function
void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectToWiFi();
  }

  if (!pubClient.connected()) {
    connectToAdafruitMqtt();
  }

  if (!subclient.connected()) {
    connectToPubSubMqtt();
  }
  subclient.loop();

  // Non-blocking delay for publishing
  unsigned long currentMillis = millis();
  if (currentMillis - previousPubMillis >= pubInterval) {
    packetCount++;
    // char data[] = "gVMtxr" + "#" +
    //               String(getRandomBoolean() ? "true" : "false") + "#" +
    //               String(humdValue()) + "#" + String(tempValue()) + "#" +
    //               String(getRandomBoolean() ? "true" : "false") + "#" +
    //               String(getRandomBoolean() ? "true" : "false") + "#" +
    //               String(packetCount);
    // char payload[] = "test";
    // publishMessage(mqttPubTopic, data.c_str());

    char data[50];
    strcpy(data, deviceId);
    strcat(data, "#");
    strcat(data, String(getRandomBoolean() ? "true" : "false").c_str());
    strcat(data, "#");
    strcat(data, String(humdValue()).c_str());
    strcat(data, "#");
    strcat(data, String(tempValue()).c_str());
    strcat(data, "#");
    strcat(data, String(getRandomBoolean() ? "true" : "false").c_str());
    strcat(data, "#");
    strcat(data, String(getRandomBoolean() ? "true" : "false").c_str());
    strcat(data, "#");
    strcat(data, String(packetCount).c_str());

    publishMessage(mqttPubTopic, data);
    Serial.println(data);

    previousPubMillis = currentMillis;
  }
}

// function connect to WiFi
void connectToWiFi() {
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to the WiFi network");
}

// function connect to MQTT with PubSubClient Library
void connectToPubSubMqtt() {
  subclient.setServer(mqttServer, mqttPort);
  subclient.setCallback(callback);
  while (!subclient.connected()) {
    Serial.println("Connecting to PubSubClient MQTT...");

    if (subclient.connect("PNJ/ESP32/test", mqttUser, mqttPassword)) {
      Serial.println("Connected to PubSubClient MQTT broker");

      // Subscribe to the desired topic
      subclient.subscribe(mqttSubTopic);
    } else {
      Serial.print("Failed to connect to MQTT broker, state: ");
      Serial.println(subclient.state());
      delay(2000);
    }
  }
}

// function connect to MQTT with AdafruitMqtt Library
void connectToAdafruitMqtt() {
  int8_t ret;

  Serial.println("Connecting to Adafruit MQTT...");

  // Stop if already connected
  if (pubClient.connected()) {
    Serial.println("Connected to Adafruit MQTT broker");
    return;
  }

  while ((ret = pubClient.connect()) != 0) {
    Serial.println(pubClient.connectErrorString(ret));
    Serial.println("Retrying Adafruit MQTT connection in 5 seconds...");
    pubClient.disconnect();
    delay(5000);
  }

  Serial.println("Connected to Adafruit MQTT broker");
}

// function to publish message
void publishMessage(const char* topic, char* payload) {
  Adafruit_MQTT_Publish publish(&pubClient, topic);
  publish.publish(payload);
}

// callback function to retrive payload from MQTT
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.println("Message arrived in topic: " + String(topic));

  Serial.print("Message: ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  Serial.println("-----------------------");
}

// function to generate random value
float randomFloat(float min, float max) {
  float random = (float)rand() / RAND_MAX;
  return min + random * (max - min);
}
float tempValue() {
  float random = randomFloat(28, 36);
  return random;
}
float humdValue() {
  float random = randomFloat(20, 90);
  return random;
}
bool getRandomBoolean() { return random(0, 2) == 1; }