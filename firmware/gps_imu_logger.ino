#include <esp_now.h>
#include <WiFi.h>

// Enforce 1-byte alignment — must match sender!
#pragma pack(push, 1)
typedef struct {
  uint8_t id;
  uint32_t da;  // date/time (e.g., Unix time)
  uint32_t ti;  // sensor uptime or timestamp
  float ax, ay, az;
  float gx, gy, gz;
  uint16_t crc;  // CRC16 of the data (must be last)
} SensorData;
#pragma pack(pop)
// Global: we'll use a local copy per callback
SensorData receivedData;

// Configurable: how often to check ESP-NOW health (ms)
const unsigned long CHECK_INTERVAL = 30000;  // every 30 seconds
unsigned long lastCheck = 0;

uint16_t calculateCRC16(const uint8_t *data, size_t len) {
  uint16_t crc = 0;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (int j = 0; j < 8; j++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

void OnDataRecv(const uint8_t *macAddr, const uint8_t *incomingData, int len) {
  // Validate size
  if (len != sizeof(SensorData)) {
    Serial.printf("⚠️ Bad size: expected %d, got %d\n", sizeof(SensorData), len);
    return;
  }

  // Copy safely
  memcpy(&receivedData, incomingData, len);

  // Format MAC address
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);

  // Output with labels and clean floats
  Serial.printf("📡 %s | ID=%d | DA=%u | TI=%u | "
                "A=(%.4f,%.4f,%.4f) | G=(%.4f,%.4f,%.4f)\n",
                macStr,
                receivedData.id, receivedData.da, receivedData.ti,
                receivedData.ax, receivedData.ay, receivedData.az,
                receivedData.gx, receivedData.gy, receivedData.gz);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);  // Wait for USB serial (if needed)

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();  // Ensure no AP connection

  if (initESPNow()) {
    Serial.println("✅ ESP-NOW running (open receiver)");
  } else {
    Serial.println("❌ ESP-NOW init failed, will retry...");
  }

  Serial.println("👂 Listening for any sensor...");
}

bool initESPNow() {
  // If already registered, unregister
  esp_now_unregister_recv_cb();

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ Failed to initialize ESP-NOW");
    return false;
  }

  // Register receive callback
  if (esp_now_register_recv_cb(OnDataRecv) != ESP_OK) {
    Serial.println("❌ Failed to register receive callback");
    return false;
  }

  return true;
}

void loop() {
  // Periodic health check
  unsigned long now = millis();
  if (now - lastCheck >= CHECK_INTERVAL) {
    lastCheck = now;

    // Check if ESP-NOW is still alive
    bool healthy = true;

    // Simple test: try to re-register (harmless if already registered)
    if (esp_now_register_recv_cb(OnDataRecv) != ESP_OK) {
      Serial.println("🔴 ESP-NOW seems down. Reinitializing...");
      WiFi.disconnect();
      delay(100);
      if (initESPNow()) {
        Serial.println("✅ ESP-NOW restored");
      } else {
        Serial.println("⚠️ Still failing to restore");
      }
    }
  }

  // No blocking delays — all work in callback
  delay(10);  // Small delay to avoid watchdog trigger
}
