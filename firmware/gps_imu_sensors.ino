#include "FastIMU.h"
#include <Wire.h>
//#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include "RunningAverage.h"
#include <WiFi.h>
#include <esp_now.h>
/**************************************************************************
  TTGO T-Display ST7789 OLED based on Adafruit example
  https://github.com/adafruit/Adafruit-ST7735-Library/blob/master/examples/graphicstest/graphicstest.ino
 **************************************************************************/
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>
#include <TFT_eSPI.h>

//IMU I2C
TFT_eSPI tft = TFT_eSPI(); 

#define IMU_ADDRESS 0x69    //Change to the address of the IMU
#define PERFORM_CALIBRATION //Comment to disable startup calibration
BMI160 IMU;               //Change to the name of any supported IMU! 
calData calib = { 0 };  //Calibration data
AccelData accelData;    //Sensor data
GyroData gyroData;
//MagData magData;

//GPS
#define RXD2 26 //D8 //7
#define TXD2 27 //D10 //6
#define GPS_BAUD 9600
//SoftwareSerial ss(13,12);
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

//espNOW
//uint8_t broadcastAddress[] = {0xD8, 0xA0, 0x1D, 0x6A, 0x07, 0x34}; //pico R
//uint8_t broadcastAddress[] = {0xD8, 0xA0, 0x1D, 0x65, 0x7E, 0x9C}; //pico r
//uint8_t broadcastAddress[] = {0x3c,0x61,0x05,0x0b,0xbe,0xc8}; //T-display
uint8_t broadcastAddress[] = {0xd8, 0xa0, 0x1d, 0x69, 0xe6, 0xbc}; //pico r2
esp_now_peer_info_t peerInfo;

long delta, prsec;

//data: sensor CRC
#pragma pack(push, 1)
typedef struct {
  uint8_t id;
  uint32_t da, ti;
  float ax, ay, az;
  float gx, gy, gz;
  uint16_t crc;  // CRC16 of the data (must be last)
} SensorData;
SensorData myData;
#pragma pack(pop)

//runningAverage
RunningAverage RAAX(6);
RunningAverage RAAY(6);
RunningAverage RAAZ(6);
RunningAverage RAGX(6);
RunningAverage RAGY(6);
RunningAverage RAGZ(6);
int samples = 0;

// CRC Function
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

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Send: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : " Fail");
}

static void smartDelay(unsigned long ms){
  unsigned long start = millis();
  do {
    while (gpsSerial.available())
      gps.encode(gpsSerial.read());
  } while (millis() - start < ms);
}

void drawinfo(byte id, long time){
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(0, 0, 2);
  tft.setTextWrap(false);
  tft.setTextColor(TFT_WHITE,TFT_BLACK);
  tft.setTextSize(4);
  tft.print(id);
  tft.setCursor(48, 0);
  tft.setTextColor(TFT_RED,TFT_BLACK);
  tft.print(time);
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  tft.init();           // Initialize ST7789 240x135
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(2);
  tft.println("Starting...");

  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
  Wire.begin();
  Wire.setClock(400000); //400khz clock

  int err = IMU.init(calib, IMU_ADDRESS);
  if (err != 0) {
    Serial.print("IMU init failed: ");
    Serial.println(err);
    while (true);
  }
  
  #ifdef PERFORM_CALIBRATION
    Serial.println("IMU calibration...");
    Serial.println("Keep IMU level.");
    IMU.calibrateAccelGyro(&calib);
    Serial.println("Calibration done!");
    Serial.println("Accel biases X/Y/Z: ");
    Serial.print(calib.accelBias[0]);
    Serial.print(", ");
    Serial.print(calib.accelBias[1]);
    Serial.print(", ");
    Serial.println(calib.accelBias[2]);
    Serial.println("Gyro biases X/Y/Z: ");
    Serial.print(calib.gyroBias[0]);
    Serial.print(", ");
    Serial.print(calib.gyroBias[1]);
    Serial.print(", ");
    Serial.println(calib.gyroBias[2]);
    IMU.init(calib, IMU_ADDRESS);
  #endif

  //err = IMU.setGyroRange(500);      //USE THESE TO SET THE RANGE, IF AN INVALID RANGE IS SET IT WILL RETURN -1
  //err = IMU.setAccelRange(2);       //THESE TWO SET THE GYRO RANGE TO ±500 DPS AND THE ACCELEROMETER RANGE TO ±2g
  /*if (err != 0) {
    Serial.print("Error Setting range: ");
    Serial.println(err);
    while (true) { ; }
  }*/
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) { // Init ESP-NOW
    Serial.println("ESP-NOW init error");
    while (1);
  }
  // register Send CB to get the status of Transmitted packet
  esp_now_register_send_cb(OnDataSent);
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;   
  peerInfo.encrypt = false;
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    while (1);
  }
  myData.id= 2;
  Serial.println("Ready!");
}

void loop() {
  smartDelay(48);    //no need for if(gps.location.isUpdated()) {}

  long date = gps.date.value();
  long time = gps.time.value();
  long sec = (time/100)%100;
  if (sec == prsec){
    delta++;
    } else {
    delta = 0;
    drawinfo(myData.id,time);
  };

  Serial.printf(" %u ", myData.id);
  Serial.printf(" %d ", date); // Raw date in DDMMYY format (u32)
  Serial.printf(" %d ", time + 5*delta); // Raw time in HHMMSSCC format (u32)
  myData.da = date;
  myData.ti = time + 5*delta;

  IMU.update();
  IMU.getAccel(&accelData);
  IMU.getGyro(&gyroData);

  RAAX.addValue(accelData.accelX);
  RAAY.addValue(accelData.accelY);
  RAAZ.addValue(accelData.accelZ);
  RAGX.addValue(gyroData.gyroX);
  RAGY.addValue(gyroData.gyroY);
  RAGZ.addValue(gyroData.gyroZ);

  myData.ax=(float)RAAX.getAverage();
  myData.ay=(float)RAAY.getAverage();
  myData.az=(float)RAAZ.getAverage();
  myData.gx=(float)RAGX.getAverage();
  myData.gy=(float)RAGY.getAverage();
  myData.gz=(float)RAGZ.getAverage(); 

  Serial.printf(" %f ",myData.ax);
  Serial.printf(" %f ",myData.ay);
  Serial.printf(" %f ",myData.az);
  Serial.printf(" %f ", myData.gx);
  Serial.printf(" %f ", myData.gy);
  Serial.printf(" %f\n",myData.gz);

  // 🔐 Compute CRC over all fields EXCEPT the crc field itself
  myData.crc = calculateCRC16((uint8_t*)&myData, sizeof(myData) - sizeof(myData.crc));

  // Set values to send via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
  if (result != ESP_OK) {
    Serial.println("Error sending the data");
  }
  prsec = sec;
}
