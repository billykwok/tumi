#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <AudioFileSourcePROGMEM.h>
#include <AudioGeneratorAAC.h>
#include <AudioOutputI2S.h>
#include <BlynkSimpleEsp32.h>
#include <HX710B.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>

#include "snore.h"
#include "yawn.h"

#define NUM 12
#define PIN_VALVE_OUTPUT 35
#define PIN_LED_OUTPUT 13
#define PIN_AUDIO_OUTPUT 12
#define PIN_AUDIO_WORD_CLK 25
#define PIN_AUDIO_BIT_CLK 26
#define PIN_GAUGE_INPUT 32
#define PIN_GAUGE_CLOCK 33
#define PIN_PUMP_H_BRIDGE_1 14
#define PIN_PUMP_H_BRIDGE_2 2

enum class State {
  SLEEP_ALARM,
  SLEEPING,
};

Adafruit_NeoPixel *pixels =
    new Adafruit_NeoPixel(NUM, PIN_LED_OUTPUT, NEO_GRB + NEO_KHZ800);
HX710B *pressureSensor = new HX710B();
AudioOutputI2S *audioOutput = new AudioOutputI2S();

AudioFileSourcePROGMEM *audioSource = NULL;
AudioGeneratorAAC *audioPlayback = NULL;
volatile State state = State::SLEEP_ALARM;
bool debugFlash = false;
uint8_t squeezed = 0;

void yawn() {
  audioSource = new AudioFileSourcePROGMEM(yawningSound, sizeof(yawningSound));
  audioPlayback = new AudioGeneratorAAC();
  audioPlayback->begin(audioSource, audioOutput);
}

void snore() {
  audioSource = new AudioFileSourcePROGMEM(snoringSound, sizeof(snoringSound));
  audioPlayback = new AudioGeneratorAAC();
  audioPlayback->begin(audioSource, audioOutput);
}

void alert(void *parameters) {
  bool ringed = false;
  while (1) {
    vTaskDelay(10000);
    if (!ringed) {
      state = State::SLEEP_ALARM;
      ringed = true;
    }
    Serial.println("Alarm");
  }
}

void audio(void *parameters) {
  while (1) {
    if (audioPlayback->isRunning()) {
      audioPlayback->loop();
    } else {
      audioPlayback->stop();
      Serial.println("AAC done");
      switch (state) {
        case State::SLEEP_ALARM:
          vTaskDelay(10000);
          break;
        case State::SLEEPING:
          vTaskDelay(3000);
          break;
      }
      delete audioSource;
      delete audioPlayback;
      switch (state) {
        case State::SLEEP_ALARM:
          yawn();
          break;
        case State::SLEEPING:
          snore();
          break;
      }
    }
  }
}

void gauge(void *parameters) {
  float pressure = 500.0;
  while (1) {
    float newPressure = pressureSensor->pascal();
    if (!debugFlash) {
      if ((int)newPressure - (int)pressure > 12) {
        debugFlash = true;
        ++squeezed;
        if (squeezed > 3) {
          state = State::SLEEPING;
          squeezed = 0;
        }
      }
    } else {
      debugFlash = false;
    }
    Serial.println(newPressure);
    pressure = newPressure;
    vTaskDelay(10);
  }
}

void color(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i <= NUM; ++i) {
    pixels->setPixelColor(i, pixels->Color(r, g, b));
  }
  pixels->show();
}

void led(void *parameters) {
  uint16_t t = 0;
  while (1) {
    if (debugFlash) {
      color(0, 0, 255);
      vTaskDelay(1000);
    } else {
      double brightness = (-0.4 * cos(t * 2.0 * PI / (2047 + 1))) + 0.6;
      switch (state) {
        case State::SLEEP_ALARM:
          color(255 * brightness, 100 * brightness, 100 * brightness);
          break;
        case State::SLEEPING:
          color(100 * brightness, 100 * brightness, 255 * brightness);
          break;
      }
      t = (t + 1) % 2047;
    }
  }
}

void pump(void *parameters) {
  while (1) {
    digitalWrite(PIN_PUMP_H_BRIDGE_1, HIGH);
    digitalWrite(PIN_PUMP_H_BRIDGE_2, LOW);
    digitalWrite(PIN_VALVE_OUTPUT, LOW);
    vTaskDelay(state == State::SLEEPING ? 3000 : 1500);
    digitalWrite(PIN_PUMP_H_BRIDGE_1, LOW);
    digitalWrite(PIN_PUMP_H_BRIDGE_2, LOW);
    digitalWrite(PIN_VALVE_OUTPUT, HIGH);
    vTaskDelay(state == State::SLEEPING ? 3000 : 1500);
  }
}

void setup() {
  pinMode(PIN_LED_OUTPUT, OUTPUT);
  pinMode(PIN_VALVE_OUTPUT, OUTPUT);
  pinMode(PIN_PUMP_H_BRIDGE_1, OUTPUT);
  pinMode(PIN_PUMP_H_BRIDGE_2, OUTPUT);
  Serial.begin(115200);

  pixels->begin();
  audioOutput->SetGain(3);
  audioOutput->SetPinout(PIN_AUDIO_BIT_CLK, PIN_AUDIO_WORD_CLK,
                         PIN_AUDIO_OUTPUT);
  pressureSensor->begin(PIN_GAUGE_INPUT, PIN_GAUGE_CLOCK);

  switch (state) {
    case State::SLEEP_ALARM:
      yawn();
      break;
    case State::SLEEPING:
      snore();
      break;
  }
  xTaskCreatePinnedToCore(audio, "audio", 10240, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(led, "led", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(pump, "pump", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(gauge, "gauge", 2048, NULL, 1, NULL, 1);
  Serial.println("Ready");
}

void loop() {}
