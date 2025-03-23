#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>

// Setup DHT sensor
#define DHTPIN 14
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// WiFi credentials
#define WIFI_NAME "Wokwi-GUEST"
#define WIFI_PASS ""

// MQTT credentials
#define MQTT_SERVER "broker.emqx.io"
#define MQTT_PORT 1883
#define MQTT_CLIENT_ID "67098633"

#define MQTT_TEMP_TOPIC "67098633/temp"
#define MQTT_HUM_TOPIC "67098633/hum"
#define MQTT_HUMI_TOPIC "67098633/humi"
#define MQTT_TEST_TOPIC "67098633/test"

// MQTT Relay Topics
#define MQTT_RELAY1 "67098633/relay1"
#define MQTT_RELAY2 "67098633/relay2"
#define MQTT_RELAY3 "67098633/relay3"
#define MQTT_RELAY4 "67098633/relay4"

// กำหนดขารีเลย์
const int relayPins[] = {19, 18, 17, 16};  // รีเลย์ 4 ตัว (GPIO 19, 18, 17, 16)

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

unsigned long lastReadTime = 0;
const long interval = 5000;  // อ่านค่า DHT ทุก 5 วินาที

void connectWiFi();
void connectMQTT();
void publishDHTData();
void controlRelay(String topic, String command);
void mqttCallback(char *topic, byte *payload, unsigned int length);

void setup() {
    Serial.begin(115200);
    Serial.println("\n🚀 ESP32 Booting up...");

    // ตั้งค่า GPIO สำหรับรีเลย์
    for (int i = 0; i < 4; i++) {
        pinMode(relayPins[i], OUTPUT);
        digitalWrite(relayPins[i], LOW); // ตั้งค่าเริ่มต้นให้รีเลย์ปิด (Active LOW)
    }

    Serial.println("📡 Connecting to WiFi...");
    connectWiFi();
    
    Serial.println("🔗 Connecting to MQTT...");
    mqtt.setServer(MQTT_SERVER, MQTT_PORT);
    mqtt.setCallback(mqttCallback);
    
    dht.begin();
    Serial.println("🌡️ DHT Sensor Initialized!");
}

void loop() {
    if (!mqtt.connected()) {
        connectMQTT();
    }
    mqtt.loop();

    // ใช้ millis() แทน delay() เพื่อให้ลูปทำงานต่อเนื่อง
    if (millis() - lastReadTime >= interval) {
        lastReadTime = millis();
        publishDHTData();
    }
}

void connectWiFi() {
    WiFi.begin(WIFI_NAME, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\n✅ WiFi Connected!");
    Serial.print("📶 IP Address: ");
    Serial.println(WiFi.localIP());
}

void connectMQTT() {
    while (!mqtt.connected()) {
        Serial.print("\n🔌 Connecting to MQTT... ");
        if (mqtt.connect(MQTT_CLIENT_ID)) {
            Serial.println("✅ Connected!");
            mqtt.publish(MQTT_TEST_TOPIC, "ESP32 Connected to MQTT!");

            // แจ้งสถานะรีเลย์
            mqtt.publish(MQTT_RELAY1, "Connected to relay1");
            mqtt.publish(MQTT_RELAY2, "Connected to relay2");
            mqtt.publish(MQTT_RELAY3, "Connected to relay3");
            mqtt.publish(MQTT_RELAY4, "Connected to relay4");

            // Subscribe MQTT Relay ทั้ง 4 ตัว
            mqtt.subscribe(MQTT_RELAY1);
            mqtt.subscribe(MQTT_RELAY2);
            mqtt.subscribe(MQTT_RELAY3);
            mqtt.subscribe(MQTT_RELAY4);

            Serial.println("📡 Subscribed to MQTT Relay Control!");
        } else {
            Serial.print("❌ Failed! (Error Code: ");
            Serial.print(mqtt.state());
            Serial.println(") Retrying in 5 seconds...");
            delay(5000);
        }
    }
}

void publishDHTData() {
    float temp = dht.readTemperature();
    float hum = dht.readHumidity();

    if (isnan(temp) || isnan(hum)) {
        Serial.println("⚠️ Failed to read from DHT sensor!");
        return;
    }

    Serial.println("--------------------------------------------------");
    Serial.printf("🌡️ Temperature: %.2f °C\n", temp);
    Serial.printf("💧 Humidity: %.2f %%\n", hum);
    Serial.println("--------------------------------------------------");

    String tempString = String(temp);
    String humString = String(hum);

    mqtt.publish(MQTT_TEMP_TOPIC, tempString.c_str());
    mqtt.publish(MQTT_HUM_TOPIC, humString.c_str());
    mqtt.publish(MQTT_HUMI_TOPIC, humString.c_str());

    Serial.println("📤 Data sent to MQTT!");
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    payload[length] = '\0';
    String command = String((char*)payload);

    Serial.println("\n📩 Incoming MQTT Message:");
    Serial.printf("📌 Topic: %s\n", topic);
    Serial.printf("📜 Message: %s\n", command.c_str());

    // ตรวจจับ MQTT Topics และควบคุมรีเลย์
    controlRelay(String(topic), command);
}

void controlRelay(String topic, String command) {
    Serial.println("🔧 Processing Relay Command...");

    if (topic == MQTT_RELAY1) {
        digitalWrite(relayPins[0], (command == "ON") ? HIGH : LOW);
        Serial.printf("✅ Relay 1 %s (Active LOW System)\n", command.c_str());
    } else if (topic == MQTT_RELAY2) {
        digitalWrite(relayPins[1], (command == "ON") ? HIGH : LOW);
        Serial.printf("✅ Relay 2 %s (Active LOW System)\n", command.c_str());
    } else if (topic == MQTT_RELAY3) {
        digitalWrite(relayPins[2], (command == "ON") ? HIGH : LOW);
        Serial.printf("✅ Relay 3 %s (Active LOW System)\n", command.c_str());
    } else if (topic == MQTT_RELAY4) {
        digitalWrite(relayPins[3], (command == "ON") ? HIGH : LOW);
        Serial.printf("✅ Relay 4 %s (Active LOW System)\n", command.c_str());
    } else {
        Serial.println("⚠️ Unknown Relay Command!");
    }
}