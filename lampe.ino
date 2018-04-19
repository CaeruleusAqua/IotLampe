#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include <WiFi.h>
#include <ESPmDNS.h>
#include <HardwareSerial.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <stdio.h>
#include <MQTTClient.h>

void fadeTo(uint32_t pwm_value);

void touchTask(void *pvParameters);

void initPWM();

void outputOff();

void startUpdate();

void endUpdate();

void progressUpdate(unsigned int progress, unsigned int total);

void errorUpdate(ota_error_t error);


const char *ssid = "Sylvis-Wlan-NetG";
const char *password = "mi1ma2sy3js4";

int32_t ledState = 0;
int32_t direction = 1;


#define OUTPUT_PIN 25
#define MAX_BOARD_CURRENT (0.1/0.082)
#define MAX_LED_CURRENT 0.850
#define MAX_PWM_DUTY (uint32_t)(4096.0 * (MAX_LED_CURRENT / MAX_BOARD_CURRENT))
#define LEDC_TEST_FADE_TIME (3000)


WiFiClient net;
MQTTClient client;
ledc_channel_config_t ledc;

void fadeTo(uint32_t pwm_value) {
    ledc_set_fade_with_time(ledc.speed_mode, ledc.channel, pwm_value, LEDC_TEST_FADE_TIME);
    ledc_fade_start(ledc.speed_mode, ledc.channel, LEDC_FADE_NO_WAIT);
    vTaskDelay(LEDC_TEST_FADE_TIME / portTICK_PERIOD_MS);
}

void touchTask(void *pvParameters) {
    touch_pad_init();
    touch_pad_config(TOUCH_PAD_NUM9, 0);
    touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_0V5);
    touch_pad_set_cnt_mode(TOUCH_PAD_NUM9, TOUCH_PAD_SLOPE_7, TOUCH_PAD_TIE_OPT_HIGH);
    //vTaskDelay(100 / portTICK_PERIOD_MS);
    char buffer[40];

    while (true) {
        delay(10);
        uint16_t touch_value;
        touch_pad_read(TOUCH_PAD_NUM9, &touch_value);
        //itoa(touch_value,buffer,10);
        //client.publish("/touch", buffer);
        if (touch_value < 300) {
            delay(300);
            touch_pad_read(TOUCH_PAD_NUM9, &touch_value);

            // short press
            if (touch_value >= 300) {
                if (ledState == 0) {
                    ledState = MAX_PWM_DUTY/2;
                    direction = 1;
                    fadeTo(MAX_PWM_DUTY/2);
                    itoa((ledState/(float)MAX_PWM_DUTY)*100.0,buffer,10);
                    client.publish("/lamp_julian_g", buffer);
                } else {
                    ledState = 0;
                    direction = 1;
                    fadeTo(0);
                    itoa((ledState/(float)MAX_PWM_DUTY)*100.0,buffer,10);
                    client.publish("/lamp_julian_g", buffer);
                }
                
            }
                // long press
            else {
                while (touch_value < 300) {
                    static const int step_ms = 100;
                    ledState += (direction * ((MAX_PWM_DUTY / (10000 / step_ms))));
                    if (ledState < MAX_PWM_DUTY / 13 && direction == 1) {
                        ledState = (MAX_PWM_DUTY / 13);
                    } else if (ledState < MAX_PWM_DUTY / 13 && direction == -1) {
                        ledState = 0;
                        direction = 1;
                    }

                    if (ledState >= MAX_PWM_DUTY) {
                        ledState = MAX_PWM_DUTY;
                        direction = 1;
                    } else if (ledState <= 0) {
                        ledState = 0;
                        direction = 1;
                    }

                    touch_pad_read(TOUCH_PAD_NUM9, &touch_value);
                    ledc_set_fade_with_time(ledc.speed_mode, ledc.channel, ledState, step_ms);
                    ledc_fade_start(ledc.speed_mode, ledc.channel, LEDC_FADE_NO_WAIT);
                    itoa((ledState/(float)MAX_PWM_DUTY)*100.0,buffer,10);
                    client.publish("/lamp_julian_g", buffer);
                    delay(step_ms);
                
                }

            }
        }
    }
}


void pwmTask(void *pvParameters) {
    //  while (true) {
    //    for (int fadeValue = analog_0V3 ; fadeValue <= analog_2v5; fadeValue += 1) {
    //      dacWrite(OUTPUT_PIN, fadeValue);
    //      delay(100);
    //    }
    //
    //    for (int fadeValue = analog_2v5 ; fadeValue >= analog_0V3; fadeValue -= 1) {
    //      dacWrite(OUTPUT_PIN, fadeValue);
    //      delay(100);
    //
    //    }
    //  }
}


void initPWM() {
    ledc_timer_config_t ledc_timer;
    ledc_timer.duty_resolution = LEDC_TIMER_12_BIT;
    ledc_timer.freq_hz = 19000;
    ledc_timer.speed_mode = LEDC_HIGH_SPEED_MODE;
    ledc_timer.timer_num = LEDC_TIMER_0;

    ledc_timer_config(&ledc_timer);

    ledc.channel = LEDC_CHANNEL_0;
    ledc.duty = 0;
    ledc.gpio_num = OUTPUT_PIN;
    ledc.speed_mode = LEDC_HIGH_SPEED_MODE;
    ledc.timer_sel = LEDC_TIMER_0;

    ledc_channel_config(&ledc);

    ledc_fade_func_install(0);
}

void outputOff() {
    pinMode(OUTPUT_PIN, OUTPUT);
    digitalWrite(OUTPUT_PIN, 0);
}

void startUpdate() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
    else // U_SPIFFS
        type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
}

void endUpdate() {
    Serial.println("\nEnd");
}

void progressUpdate(unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
}

void errorUpdate(ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
}

void messageReceived(String &topic, String &payload) {
  if(topic == "/lamp_julian_s"){
     float val = atoi(payload.c_str());
     ledState = (MAX_PWM_DUTY / 100.0 * val);
     ledc_set_fade_with_time(ledc.speed_mode, ledc.channel, ledState, 100);
     ledc_fade_start(ledc.speed_mode, ledc.channel, LEDC_FADE_NO_WAIT);
  }
}


void setup() {
    outputOff();
    initPWM();
    fadeTo(MAX_PWM_DUTY / 2);
    ledState = MAX_PWM_DUTY / 2;
    direction = -1;
    Serial.begin(115200);

    xTaskCreatePinnedToCore(touchTask, "touchTask", 10000, NULL, 0, NULL, 1);


    Serial.println("Booting");
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    WiFi.waitForConnectResult();

    ArduinoOTA.setHostname("TouchLamp_Julian");
    ArduinoOTA.onStart(startUpdate);
    ArduinoOTA.onEnd(endUpdate);
    ArduinoOTA.onProgress(progressUpdate);
    ArduinoOTA.onError(errorUpdate);
    ArduinoOTA.begin();

    Serial.println("Ready");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());


    client.begin("192.168.178.20", net);
    client.onMessage(messageReceived);
    if (!client.connected()) {
      while (!client.connect("TouchLamp_Julian", "try", "try")) {
          Serial.print(".");
          delay(1000);
      }
          client.subscribe("/lamp_julian_s");
    }
    char buffer[40];
    itoa((ledState/(float)MAX_PWM_DUTY)*100.0,buffer,10);
    client.publish("/lamp_julian_g", buffer);
    
}

void loop() {
    ArduinoOTA.handle();
    if (!client.connected()) {
      while (!client.connect("TouchLamp_Julian", "try", "try")) {
          Serial.print(".");
          delay(1000);
      }
          client.subscribe("/lamp_julian_s");
    }

    
    client.loop();
    delay(10);
}

