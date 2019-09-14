#include <Arduino.h>
#include <Servo.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ArduinoJson.h>

#ifndef STASSID
#define STASSID "my-ssid"
#define STAPSK "my-password"
#endif

#ifndef MDNS_NAME
#define MDNS_NAME "chobot"
#endif

// debug output

//#define SERIAL_PRINT 1

// wifi

const char *ssid = STASSID;
const char *password = STAPSK;
const char *mdns_name = MDNS_NAME;

ESP8266WebServer server(80);

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
const unsigned long servo_delay = 250; // how long to wait between servo loops
int16_t servo_pos[2]; // current servo position value
unsigned long servo_last_time = 0; // last time the servo loop ran

// main loop

// message structures

enum service_enum
{
    S_HW = 0,
    S_CH
};

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

void update_service_status(const unsigned long time_now)
{
    for (int8_t idx = 0; idx < 2; idx++)
    {
        bool ldr_state = bool(digitalRead(ldr_pin_lookup[idx]));

        if (ldr_last_state[idx] != ldr_state)
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
                servo_lookup[idx].detach();

            continue;
        }

        if (!servo_lookup[idx].attached())
            servo_lookup[idx].attach(servo_pin_lookup[idx], servo_controller_min[idx], servo_controller_max[idx]);

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

void handleRoot() {
    char temp[400];
    int sec = millis() / 1000;
    int min = sec / 60;
    int hr = min / 60;

    snprintf(
        temp,
        400,
        "<html>\
    <head>\
        <meta http-equiv='refresh' content='5'/>\
        <title>%s</title>\
        <style>\
            body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }\
        </style>\
    </head>\
    <body>\
        <h1>%s</h1>\
        <p>Uptime: %02d:%02d:%02d</p>\
    </body>\
</html>",
        mdns_name,
        mdns_name,
        hr,
        min % 60,
        sec % 60
    );
    server.send(200, "text/html", temp);
}

void handleNotFound() {
    char temp[400];
    snprintf(
        temp,
        400,
        "File Not Found\n\n\
URI: %s\n\
",
        const_cast<char*>(server.uri().c_str())
    );

    server.send(404, "text/plain", temp);
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

    server.on("/", handleRoot);
    server.onNotFound(handleNotFound);
    server.begin();
#if SERIAL_PRINT
    Serial.println("HTTP server started");
#endif
}

unsigned long press_time = 0;
const unsigned long press_delay = 20000;

void loop() {
    server.handleClient();
    MDNS.update();

    const unsigned long time_now = millis();
    update_service_status(time_now);
    update_servos(time_now);
}
