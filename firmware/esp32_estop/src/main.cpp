#include <Arduino.h>
#include <esp_system.h>
#include "latch.h"

// Classic ESP32 DevKit only: NC mushroom between GPIO27 and GND; reset NO GPIO26.
// External 10k pullups to 3.3V are recommended. No connection to motor power.
constexpr int kStopPin = 27, kResetPin = 26;
StopLatch latch;
char device_id[13], boot_id[9], input[32];
size_t used = 0;
bool discarding = false;
uint32_t sequence = 0;

void sample_inputs() {
  latch.update(digitalRead(kStopPin) == LOW, digitalRead(kResetPin) == LOW, millis());
}

void reply() {
  if (used != 18 || input[0] != 'Q' || input[1] != ',') {
    latch.trip();
    return;
  }
  for (size_t i = 2; i < used; ++i) {
    if (!((input[i] >= '0' && input[i] <= '9') || (input[i] >= 'a' && input[i] <= 'f'))) {
      latch.trip();
      return;
    }
  }
  input[used] = '\0';
  latch.query(millis());
  sample_inputs();
  char payload[128], frame[144];
  snprintf(payload, sizeof(payload), "E1,%s,%s,%lu,%lu,%s,%d,%d", device_id, boot_id,
    static_cast<unsigned long>(++sequence), static_cast<unsigned long>(millis()),
    input + 2, latch.armed ? 0 : 1, latch.armed ? 1 : 0);
  snprintf(frame, sizeof(frame), "%s,%08lx\n", payload,
    static_cast<unsigned long>(wire_crc32(payload)));
  Serial.print(frame);
}

void setup() {
  pinMode(kStopPin, INPUT_PULLUP);
  pinMode(kResetPin, INPUT_PULLUP);
  uint8_t mac[6];
  if (esp_read_mac(mac, ESP_MAC_WIFI_STA) != ESP_OK) abort();
  snprintf(device_id, sizeof(device_id), "%02x%02x%02x%02x%02x%02x",
    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  snprintf(boot_id, sizeof(boot_id), "%08lx", static_cast<unsigned long>(esp_random()));
  Serial.begin(115200);
  latch.trip();
}

void loop() {
  sample_inputs();
  // Bound serial work so an input flood cannot starve GPIO/watchdog sampling.
  for (int count = 0; count < 32 && Serial.available(); ++count) {
    char c = static_cast<char>(Serial.read());
    if (c == '\n') {
      if (!discarding) reply();
      used = 0;
      discarding = false;
    } else if (!discarding) {
      if (used >= sizeof(input) - 1) {
        used = 0;
        discarding = true;
        latch.trip();
      } else input[used++] = c;
    }
    sample_inputs();
  }
  delay(1);
}
