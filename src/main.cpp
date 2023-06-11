#include <Arduino.h>
#include <DHT.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <time.h>

#include "secrets.hpp"  // This file is not included in here. It just contains my WiFi SSID and password.

/*  Peripherals, sensors and other magical stuff */
/*  builtin led */
constexpr auto LED_PIN = 2;

/* DHT11 sensor */
constexpr auto DHT_PIN = 4;
constexpr auto DHT_TYPE = DHT11;
auto dht = DHT{DHT_PIN, DHT_TYPE};

/*  WiFi */
constexpr auto WIFI_SSID = SECRET_WIFI_SSID;
constexpr auto WIFI_PASSWORD = SECRET_WIFI_PASSWORD;
auto espClient = WiFiClient{};

/*  MQTT */
constexpr auto MQTT_BROKER_IP = "192.168.27.229";
constexpr auto MQTT_PORT = 1883;
constexpr auto MQTT_TOPIC_DATA = "iot/sensor_data";
constexpr auto MQTT_TOPIC_SHOULD_PUBLISH = "iot/should_publish";
auto client = PubSubClient{espClient};

/* NTP (for true ntp clock time. not necessary, but it's here for making the output look fancier) */
constexpr auto NTP_SERVER = "pool.ntp.org";
constexpr auto GMT_OFFSET_SEC = -18000;
constexpr auto DAYLIGHT_OFFSET_SEC = 0;
/* */

/*  State   */
long system_time = 0;
long last_publish = 0;

/*  Just starts blinking the led for 5s to let the user know there is something wrong.
    Not relevant */
void error_blink_5s() {
    for (int i = 0; i < 5; i++) {
        digitalWrite(LED_PIN, digitalRead(LED_PIN) == LOW ? HIGH : LOW);
        delay(500);
        digitalWrite(LED_PIN, digitalRead(LED_PIN) == LOW ? HIGH : LOW);
        delay(500);
    }
}

/* Loops until the ESP32 connects to the WiFi network */
void connect_to_wifi() {
    Serial.println("DEBUG: connecting to WiFi...");
    while (!WiFi.isConnected()) {
        Serial.println("ERROR: failed to connect to WiFi. Checking again in 5 seconds");
        error_blink_5s();
    }
    Serial.println("DEBUG: connected to WiFi");
}

/* Loops until the ESP32 connects to the MQTT broker */
void connect_to_mqtt() {
    Serial.println("DEBUG: connecting to MQTT broker");
    while (!client.connect("ESP8266Client")) {
        Serial.println("ERROR: failed to connect to MQTT broker. Retrying in 5 seconds");
        error_blink_5s();
    }
    Serial.println("DEBUG: connected to MQTT broker");
}

/*  Reads the current time from the ntp server and formats it as a string.
    Unnecessary, but pretty. */
size_t read_formatted_time(char* out, size_t size) {
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        return 0;
    }
    return strftime(out, size, "%x %X", &timeinfo);
}

/*  Publishes a new message to the MQTT broker.
    The message contains data about the current time, temperature and humidity.
    It also blinks the led if it works. */
bool publish_new_message() {
    const auto temperature = dht.readTemperature();
    const auto humidity = dht.readHumidity();
    if (isnan(temperature) || isnan(humidity)) {
        Serial.println("ERROR: failed to read from DHT sensor");
        return false;
    }

    char ftime[24];
    if (!read_formatted_time(ftime, sizeof(ftime))) {
        Serial.println("ERROR: failed to read time from NTP server");
        return false;
    }

    char payload[64];
    char tempString[8];
    char humString[8];
    dtostrf(temperature, 1, 2, tempString);
    dtostrf(humidity, 1, 2, humString);
    snprintf(payload, sizeof(payload), "{\"time\": \"%s\", \"tempt\": %s, \"hum\": %s}", ftime, tempString, humString);
    client.publish(MQTT_TOPIC_DATA, payload);

    Serial.print("DEBUG: Payload: ");
    Serial.println(payload);
    return true;
}

/*  The setup function.
    Just init everything and connect to WiFi and MQTT. */
void setup() {
    Serial.begin(9600);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    dht.begin();

    configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    connect_to_wifi();

    client.setServer(MQTT_BROKER_IP, MQTT_PORT);
    connect_to_mqtt();

    Serial.println("DEBUG: setup finished with no errors");
}

/*  The main loop.
    Checks if the WiFi and MQTT connections are still alive.
    If not, reconnects.
    Also checks if a new message should be published, and if so, it publishes new data. */
void loop() {
    if (!WiFi.isConnected()) {
        Serial.println("ERROR: WiFi connection lost.");
        connect_to_wifi();
    }

    if (!client.connected()) {
        Serial.println("ERROR: MQTT connection lost.");
        connect_to_mqtt();
    }

    client.loop();

    system_time = millis();
    if (system_time - last_publish > 5000) {
        Serial.println("DEBUG: publishing new message to the MQTT broker");
        const auto success = publish_new_message();
        if (!success) {
            Serial.println("ERROR: failed to publish new message to the MQTT broker. Retrying in 5 seconds");
            error_blink_5s();
            return;
        }

        Serial.println("DEBUG: message published successfully");
        last_publish = system_time;
    }
}
