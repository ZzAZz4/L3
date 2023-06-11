# L3

An MQTT client using ESP32.

You can run this project using PlatformIO or by just opening `src/main.cpp` and adding the required definitions {`SECRET_WIFI_SSID` and `SECRET_WIFI_PASSWORD`}.

Required external libraries:
```
	adafruit/DHT sensor library@^1.4.4       # for reading temperature/humidity
	adafruit/Adafruit Unified Sensor@^1.1.9  # for making the first library work
	knolleary/PubSubClient@^2.8              # for dealing with MQTT pub/sub logic
```
`Wifi` and `time` should come by default on most systems.
