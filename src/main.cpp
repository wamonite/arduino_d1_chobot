/*
arduino_d1_chobot

Copyright (c) 2019 Warren Moore

This software may be redistributed under the terms of the MIT License.
See the file LICENSE for details.
*/

#include <Arduino.h>
#include <Servo.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>

#ifndef STASSID
#define STASSID "my-ssid"
#define STAPSK "my-password"
#endif

#ifndef MDNS_NAME
#define MDNS_NAME "chobot"
#endif

#ifndef MQTT_HOST
#define MQTT_HOST "localhost"
#endif
#ifndef MQTT_PORT
#define MQTT_PORT 1883
#endif

#ifndef MQTT_USERNAME
#define MQTT_USERNAME "mqtt-user"
#define MQTT_PASSWORD "mqtt-password"
#endif

#ifndef MQTT_PREFIX
#define MQTT_PREFIX "sensor/chobot/"
#endif

// debug output

//#define SERIAL_PRINT 1

#define STRING_MAX 1000

enum service_enum
{
    S_HW = 0,
    S_CH
};

// wifi

const char *ssid = STASSID;
const char *password = STAPSK;
const char *mdns_name = MDNS_NAME;

WiFiClient wifi_client;
PubSubClient mqtt_client(wifi_client);
ESP8266WebServer web_server(80);

// mqtt

const char *mqtt_host = MQTT_HOST;
const int16_t mqtt_port = MQTT_PORT;
const char *mqtt_username = MQTT_USERNAME;
const char *mqtt_password = MQTT_PASSWORD;
const unsigned long mqtt_reconnect_delay = 5000;
unsigned long mqtt_reconnect_time = 0;
const char *mqtt_channel_state[2] = {
    MQTT_PREFIX "hot_water",
    MQTT_PREFIX "central_heating"
};
const char *mqtt_channel_set[2] = {
    MQTT_PREFIX "hot_water/set",
    MQTT_PREFIX "central_heating/set"
};
const unsigned long mqtt_delay = 60000; // how long to wait between mqtt status updates
unsigned long mqtt_last_time = 0; // last time status posted to mqtt

// leds

const int8_t led_pin_lookup[2] = {D7, D8}; // led output pins

// ldrs

const int8_t ldr_pin_lookup[2] = {D2, D1}; // ldr input pins
bool ldr_on[2]; // ldr input state
bool initial_state[2] = {false, false}; // has the ldr input state been set at all
bool desired_state[2] = {false, false}; // what state the ldr inputs should be in
bool ldr_last_state[2]; // temprary ldr input state used for debouncing
unsigned long ldr_last_time[2] = {0, 0}; // last time temporary ldr input state during debounce period
const unsigned long ldr_debounce = 100; // how long temporary ldr input state needs to remain stable

// servos

Servo servo_lookup[2]; // servo controllers
const int8_t servo_pin_lookup[2] = {D4, D3}; // servo pins
const int16_t servo_detach_delay = 1000; // time to wait after initial detach
const int16_t servo_controller_min[2] = {580, 580}; // servo setup param
const int16_t servo_controller_max[2] = {2350, 2530}; // servo setup param
const int16_t servo_pos_rest[2] = {180, 0}; // servo value for rest position
const int16_t servo_pos_press[2] = {180 - 115, 115}; // servo value for button pressing position
const int16_t servo_step = 5; // how much to move the servo position every loop
const unsigned long servo_delay = 50; // how long to wait between servo loops
int16_t servo_pos[2]; // current servo position value
unsigned long servo_last_time = 0; // last time the servo loop ran

// functions

void set_led(service_enum service, bool on)
{
    int pin = led_pin_lookup[service];
    digitalWrite(pin, on ? HIGH : LOW);
}

void set_angle(service_enum service, int angle)
{
    if (angle < 0)
        angle = 0;
    if (angle > 180)
        angle = 180;

    servo_lookup[service].write(angle);
}

void send_state_to_mqtt(service_enum service, bool on)
{
#if SERIAL_PRINT
    Serial.print("Sending status [");
    Serial.print(service);
    Serial.print("] to ");
    Serial.print(mqtt_channel_state[service]);
    Serial.print(" ");
    Serial.println(on ? "ON" : "OFF");
#endif

    mqtt_client.publish(mqtt_channel_state[service], on ? "ON" : "OFF", true);
}

void update_mqtt_connection(const unsigned long time_now)
{
    bool reconnected = false;

    if (!mqtt_client.connected())
    {
        if (mqtt_reconnect_time == 0)
            mqtt_reconnect_time = time_now;

        if (time_now - mqtt_reconnect_time > mqtt_reconnect_delay)
        {
#if SERIAL_PRINT
            Serial.print("Attempting MQTT connection... ");
#endif
            reconnected = mqtt_client.connect(mqtt_username, mqtt_username, mqtt_password);
            if (reconnected)
            {
                mqtt_client.subscribe(mqtt_channel_set[0]);
                mqtt_client.subscribe(mqtt_channel_set[1]);

#if SERIAL_PRINT
                Serial.println("connected");
#endif
            }
#if SERIAL_PRINT
            else
            {
                Serial.print("failed, rc=");
                Serial.print(mqtt_client.state());
                Serial.println(" trying again in 5 seconds");
            }
#endif

            mqtt_reconnect_time = 0;
        }
    }

    if (mqtt_client.connected())
    {
        mqtt_client.loop();

        if (reconnected || (time_now - mqtt_last_time > mqtt_delay))
        {
            mqtt_last_time = time_now;

            send_state_to_mqtt(S_HW, ldr_on[S_HW]);
            send_state_to_mqtt(S_CH, ldr_on[S_CH]);
        }
    }
}

void update_service_status(const unsigned long time_now)
{
    for (int8_t idx = 0; idx < 2; idx++)
    {
        bool ldr_state = bool(digitalRead(ldr_pin_lookup[idx]));

        if ((ldr_last_state[idx] != ldr_state) || (!initial_state[idx] && ldr_last_time[idx] == 0))
        {
            ldr_last_state[idx] = ldr_state;
            ldr_last_time[idx] = time_now;
        }

        if (ldr_last_time[idx] != 0 && (time_now - ldr_last_time[idx]) > ldr_debounce)
        {
            ldr_on[idx] = ldr_last_state[idx];
            if (!initial_state[idx])
            {
                desired_state[idx] = ldr_last_state[idx];
                initial_state[idx] = true;
            }
            set_led((service_enum)idx, ldr_last_state[idx]);
            send_state_to_mqtt((service_enum)idx, ldr_last_state[idx]);
            ldr_last_time[idx] = 0;

#if SERIAL_PRINT
            Serial.print("service[");
            Serial.print(idx);
            Serial.print("]: ");
            Serial.println(ldr_on[idx] ? "on" : "off");
#endif
        }
    }
}

void update_servos(const unsigned long time_now)
{
    if ((time_now - servo_last_time) <= servo_delay)
        return;
    servo_last_time = time_now;

    for (int8_t idx = 0; idx < 2; idx++)
    {
        if (!initial_state[idx])
            continue;

        if (servo_pos[idx] == servo_pos_rest[idx] && ldr_on[idx] == desired_state[idx])
        {
            if (servo_lookup[idx].attached())
            {
                servo_lookup[idx].detach();
#if SERIAL_PRINT
                Serial.print("service[");
                Serial.print(idx);
                Serial.println("]: detach");
#endif
            }

            continue;
        }

        if (!servo_lookup[idx].attached())
        {
            servo_lookup[idx].attach(servo_pin_lookup[idx], servo_controller_min[idx], servo_controller_max[idx]);
#if SERIAL_PRINT
            Serial.print("service[");
            Serial.print(idx);
            Serial.println("]: attach");
#endif
        }

        int16_t desired_pos = (ldr_on[idx] == desired_state[idx]) ? servo_pos_rest[idx] : servo_pos_press[idx];
        if (desired_pos > servo_pos[idx])
        {
            servo_pos[idx] += servo_step;
            if (servo_pos[idx] > desired_pos)
                servo_pos[idx] = desired_pos;
        }
        else
        {
            servo_pos[idx] -= servo_step;
            if (servo_pos[idx] < desired_pos)
                servo_pos[idx] = desired_pos;
        }

        set_angle((service_enum)idx, servo_pos[idx]);
#if SERIAL_PRINT
        Serial.print("service[");
        Serial.print(idx);
        Serial.print("]: pos ");
        Serial.println(servo_pos[idx]);
#endif
    }
}

void handle_root()
{
    char response[STRING_MAX];
    int sec = millis() / 1000;
    int min = sec / 60;
    int hr = min / 60;

    snprintf(
        response,
        STRING_MAX,
        "\
<!DOCTYPE html>\
<html lang=\"en\">\
  <head>\
    <meta charset=\"utf-8\">\
    <title>%s</title>\
    <link rel=\"stylesheet\" type=\"text/css\" href=\"https://fonts.googleapis.com/css?family=Chelsea+Market\">\
    <style>\
        body {background-color: #909090; color: #ffffff;}\
        h1 {text-align: center; font-family: 'Chelsea Market', serif; font-size: 40pt; padding-top: 100px;}\
        p {text-align: center; font-family: 'Chelsea Market', serif; font-size: 16pt; padding-top: 5px;}\
    </style>\
  </head>\
  <body>\
    <h1>%s</h1>\
    <p>Uptime: %02d:%02d:%02d</p>\
    <p>Hot water: %s</p>\
    <p>Central heating: %s</p>\
  </body>\
</html>\
",
        mdns_name,
        mdns_name,
        hr,
        min % 60,
        sec % 60,
        ldr_on[0] ? "on" : "off",
        ldr_on[1] ? "on" : "off"
    );

    web_server.send(200, "text/html", response);
}

void handle_not_found()
{
    char response[STRING_MAX];
    snprintf(
        response,
        STRING_MAX,
        "\
File Not Found\n\n\
URI: %s\n\n\
Args:\n\
",
        const_cast<char*>(web_server.uri().c_str())
    );

    for (uint8_t i = 0; i < web_server.args(); i++)
    {
        char temp[STRING_MAX];
        snprintf(
            temp,
            STRING_MAX,
            "  %s = '%s'\n",
            const_cast<char*>(web_server.argName(i).c_str()),
            const_cast<char*>(web_server.arg(i).c_str())
        );

        int16_t message_space = STRING_MAX - strlen(response) - 1;
        strncat(
            response,
            temp,
            message_space
        );
    }

   web_server.send(404, "text/plain", response);
}

void handle_mqtt(char* topic, byte* payload, unsigned int length)
{
#if SERIAL_PRINT
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    Serial.print(length);
    Serial.print(" '");
    for (int i = 0; i < length; i++) {
        Serial.print((char)payload[i]);
    }
    Serial.println("'");
#endif

    int16_t service = -1;
    for (int8_t idx = 0; idx < 2; idx++)
    {
        if (strcmp(mqtt_channel_set[idx], topic) == 0)
            service = idx;
    }
    if (service >= 0)
    {
        if (length == 2 && strncmp("ON", (char*)payload, 2) == 0)
        {
#if SERIAL_PRINT
            Serial.print("Service[");
            Serial.print(service);
            Serial.println("] ON");
#endif

            desired_state[service] = true;
        }
        else
        {
            if (length == 3 && strncmp("OFF", (char*)payload, 3) == 0)
            {
#if SERIAL_PRINT
                Serial.print("Service[");
                Serial.print(service);
                Serial.println("] OFF");
#endif

                desired_state[service] = false;
            }
#if SERIAL_PRINT
            else
            {
                Serial.print("Service[");
                Serial.print(service);
                Serial.println("] ?");
            }
#endif
        }
    }
}

void setup() {
#if SERIAL_PRINT
    Serial.begin(115200);
#endif

    // leds
    pinMode(led_pin_lookup[S_HW], OUTPUT);
    pinMode(led_pin_lookup[S_CH], OUTPUT);

    // ldrs
    pinMode(ldr_pin_lookup[S_HW], INPUT);
    pinMode(ldr_pin_lookup[S_CH], INPUT);

    // servos
    for (int8_t idx = 0; idx < 2; idx++)
    {
        servo_lookup[idx].attach(servo_pin_lookup[idx], servo_controller_min[idx], servo_controller_max[idx]);
        servo_lookup[idx].write(servo_pos_rest[idx]);
        servo_pos[idx] = servo_pos_rest[idx];
    }

    delay(servo_detach_delay);

    servo_lookup[0].detach();
    servo_lookup[1].detach();

    // wifi
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
#if SERIAL_PRINT
    Serial.println("");
#endif

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
#if SERIAL_PRINT
        Serial.print(".");
#endif
    }

#if SERIAL_PRINT
    Serial.println("");
    Serial.print("Connected to ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
#endif

    if (MDNS.begin(mdns_name)) {
#if SERIAL_PRINT
        Serial.print("MDNS responder started: ");
        Serial.print(mdns_name);
        Serial.println("");
#endif
    }

    web_server.on("/", handle_root);
    web_server.onNotFound(handle_not_found);
    web_server.begin();
#if SERIAL_PRINT
    Serial.println("HTTP server started");
#endif

    mqtt_client.setServer(mqtt_host, mqtt_port);
    mqtt_client.setCallback(handle_mqtt);
}

void loop() {
    web_server.handleClient();
    MDNS.update();

    const unsigned long time_now = millis();
    update_mqtt_connection(time_now);
    update_service_status(time_now);
    update_servos(time_now);
}
