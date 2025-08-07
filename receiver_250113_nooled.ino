#include <esp_now.h>
#include <WiFi.h>

// Structure example to receive data matches the sender structure
typedef struct struct_message {
  byte id;
  long da;
  long ti;
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  //Serial.printf("Board ID %u: %u bytes\n", myData.id, len);
    Serial.printf("%d  %d  %d  %f  %f  %f  %f  %f  %f   \n",
      myData.id,myData.da,myData.ti,myData.ax,myData.ay,myData.az,myData.gx,myData.gy,myData.gz);
}
 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  
  delay(1000);
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }else{
    Serial.println("ESPnow running");
  }
    
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  Serial.println("Listening...");

}
void loop() {
}