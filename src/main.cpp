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

// wifi

const char *ssid = STASSID;
const char *password = STAPSK;
const char *mdns_name = MDNS_NAME;

ESP8266WebServer server(80);

// leds

const int8_t led_pin_lookup[2] = {D7, D8};

// ldrs

const int8_t ldr_pin_lookup[2] = {D2, D1};

// servos

#define SERVO_MIN 580
#define SERVO_MAX 2530
#define SERVO_DETACH_DELAY 1000

Servo servo_lookup[2];
const int8_t servo_pin_lookup[2] = {D4, D3};
bool servo_press[2] = {false, false};

#define PRESS_START_N 0
#define PRESS_START_I 180
#define PRESS_STEP 1
#define PRESS_SWEEP 115
#define PRESS_INVERT S_HW
#define PRESS_DELAY 5

// ldrs

bool ldr_on[2];

// main loop

// TODO comment
#define SERIAL_PRINT 1
#define LOOP_DELAY 100

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

bool update_service_status()
{
    bool updated = false;
    for (int8_t idx = 0; idx < 2; idx++)
    {
        bool ldr_state = bool(digitalRead(ldr_pin_lookup[idx]));

        if (ldr_on[idx] != ldr_state)
        {
            updated = true;
        }

#if SERIAL_PRINT
        if (ldr_on[idx] != ldr_state)
        {
            Serial.print("service[");
            Serial.print(idx);
            Serial.print("]: ");
            Serial.println(ldr_state ? "on" : "off");
        }
#endif
        ldr_on[idx] = ldr_state;
        set_led((service_enum)idx, ldr_state);
    }

    return updated;
}

void press_button(service_enum service)
{
    int16_t servo_pos = (service == PRESS_INVERT ? PRESS_START_I : PRESS_START_N);
    int16_t servo_max = (service == PRESS_INVERT ? PRESS_START_I - PRESS_SWEEP : PRESS_START_N + PRESS_SWEEP);
    int16_t servo_step = (service == PRESS_INVERT ? -PRESS_STEP : PRESS_STEP);

    servo_lookup[0].attach(servo_pin_lookup[S_HW], SERVO_MIN, SERVO_MAX);
    servo_lookup[1].attach(servo_pin_lookup[S_CH], SERVO_MIN, SERVO_MAX);

    bool service_state = ldr_on[service];
    while (true)
    {
        set_angle(service, servo_pos);
        servo_pos += servo_step;

        if (servo_step > 0 && servo_pos > servo_max)
            break;
        if (servo_step < 0 && servo_pos < servo_max)
            break;

        update_service_status();
        if (service_state != ldr_on[service])
            break;

        delay(PRESS_DELAY);
    }

    set_angle(service, service == PRESS_INVERT ? PRESS_START_I : PRESS_START_N);

    delay(SERVO_DETACH_DELAY);

    servo_lookup[0].detach();
    servo_lookup[1].detach();
}

//                     servo_press[S_HW] = true;
// #if SERIAL_PRINT
//                     Serial.println("pressing hot water");
// #endif

//                     servo_press[S_CH] = true;
// #if SERIAL_PRINT
//                     Serial.println("pressing central heating");

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
    servo_lookup[0].attach(servo_pin_lookup[S_HW], SERVO_MIN, SERVO_MAX);
    servo_lookup[1].attach(servo_pin_lookup[S_CH], SERVO_MIN, SERVO_MAX);
    servo_lookup[0].write(PRESS_INVERT == 0 ? PRESS_START_I : PRESS_START_N);
    servo_lookup[1].write(PRESS_INVERT == 1 ? PRESS_START_I : PRESS_START_N);

    delay(SERVO_DETACH_DELAY);

    servo_lookup[0].detach();
    servo_lookup[1].detach();

    // wifi
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    Serial.println("");

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.print("Connected to ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    if (MDNS.begin(mdns_name)) {
        Serial.print("MDNS responder started: ");
        Serial.print(mdns_name);
        Serial.println("");
    }

    server.on("/", handleRoot);
    server.onNotFound(handleNotFound);
    server.begin();
    Serial.println("HTTP server started");
}

void loop() {
    server.handleClient();
    MDNS.update();

    update_service_status();

    if (servo_press[S_HW])
    {
        press_button(S_HW);
        servo_press[S_HW] = false;
    }
    if (servo_press[S_CH])
    {
        press_button(S_CH);
        servo_press[S_CH] = false;
    }

    // TODO should debounce LDR inputs and remove this
    delay(LOOP_DELAY);
}
