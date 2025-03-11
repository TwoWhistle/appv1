/***********************************************************
 *  Example: wristbandv1_xiao_nRF52840.ino
 *  - Collect ECG (analog pin) and PPG (MAX30102) data
 *  - Collect SCD41 data (CO2, Temp, Humidity) every 5s
 *  - Bundle raw samples into a single chunk + attach SCD41
 *  - Send chunk to iOS for advanced processing
 *  - Ported to Seeed Studio XIAO nRF52840 using Bluefruit
 ***********************************************************/

#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "SparkFun_SCD4x_Arduino_Library.h"
#include <bluefruit.h> 
#include "LSM6DS3.h"
#include <math.h>

//LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A               HEEEERRREEEEEEE
      
float IMUAccelXSum = 0.0;
float IMUAccelYSum = 0.0;
float IMUAccelZSum = 0.0;

float IMUGyroXSum = 0.0;
float IMUGyroYSum = 0.0;
float IMUGyroZSum = 0.0;


// ========== BLE DEFINITIONS ==========
#define SERVICE_UUID              "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define DATA_CHARACTERISTIC_UUID  "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

#define MTU_SIZE 20

// Create a BLE Service using the same UUID
BLEService dataService(SERVICE_UUID);

// Create a BLE Characteristic for sending data
BLECharacteristic dataCharacteristic(DATA_CHARACTERISTIC_UUID);

// Track connection status
bool deviceConnected = false;

// ======== Bluefruit Connect/Disconnect Callbacks ========
void connect_callback(uint16_t conn_handle)
{
  deviceConnected = true;
  Serial.println("✅ Device connected!");
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  deviceConnected = false;
  Serial.println("❌ Disconnected! Restarting advertising...");
  Bluefruit.Advertising.start();
}

// ========== SENSOR DEFINITIONS & GLOBALS ==========

// -- MAX30102 (PPG) --
//MAX30105 particleSensor;                           ///HEREEEEEEEEEEE

// -- SCD41 (CO2, Temp, Humidity) --
//SCD4x scd41;                                      ///HEREEEEEEEEEEE
float latestCO2 = 0.0;
float latestTemp = 0.0;
float latestHumidity = 0.0;

// We'll read SCD41 every 5 seconds
unsigned long lastSCD41Read = 0;
const unsigned long SCD41_INTERVAL_MS = 5000;

// -- ECG & PPG Data Buffering --
#define SAMPLING_FREQUENCY 100  // e.g., 100 Hz
#define N_SAMPLES 50           // e.g., 1-second chunk at 100 Hz

float ecgBuffer[N_SAMPLES];
float ppgBuffer[N_SAMPLES];
int sampleIndex = 0;

unsigned long lastSampleTime = 0;
unsigned long sampleInterval = 1000 / SAMPLING_FREQUENCY; // ms per sample

// -----------------------
// Helper Function to Send Data in Chunks (MTU=150)
// -----------------------
void sendInChunks(const String &fullStr, size_t maxChunkSize) {
    if (!Bluefruit.connected()) {  
        Serial.println("❌ Device disconnected while sending in chunks");
        return;
    }

    size_t totalLen = fullStr.length();
    size_t offset = 0;

    while (offset < totalLen) {
        size_t chunkLen = min(maxChunkSize, totalLen - offset);
        String sub = fullStr.substring(offset, offset + chunkLen);

        // Convert to byte array
        uint8_t chunkData[chunkLen + 1];
        memcpy(chunkData, sub.c_str(), chunkLen);
        chunkData[chunkLen] = '\0';  // Null terminate for safety

        // Send via BLE
        if (dataCharacteristic.notify(chunkData, chunkLen)) {
            Serial.print("📡 Sent chunk: ");
            Serial.println(sub);
        } else {
            Serial.println("⚠️ Failed to send chunk");
        }

        offset += chunkLen;

        // A short delay ensures the BLE stack has time to process the next packet
        delay(20);
    }
}

void setup() {
  Serial1.begin(115200);
  //Wire.begin();

  // ========== Initialize SCD41 ==========                                       ///HEREEEEEEEEEEE
  /*
  if (!scd41.begin()) {
    Serial.println("Failed to initialize SCD41! Check wiring.");
  } else {
    Serial.println("SCD41 initialized.");
    scd41.startPeriodicMeasurement(); // Start measuring in periodic mode
  }

  // ========== Initialize MAX30102 ==========
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 not found. Check wiring!");
    while (1);
  }
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x3F);
  particleSensor.setPulseAmplitudeGreen(0);
*/

  // ========== Initialize IMU ==========                 HERREEEEEE
  /*if (myIMU.begin() != 0) {
      Serial.println("Device error");
  } else {
      Serial.println("Device OK!");
  }*/
  



  // ========== BLE SETUP with Bluefruit ==========
  Bluefruit.begin();
  // Optionally set transmit power, name, etc.
  Bluefruit.setTxPower(4);  // 0 dB = default, 4 dB is slightly stronger
  Bluefruit.setName("nrfwrist");  // keep the same name if you wish

  // Register connection callbacks
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // Create and start our service
  dataService.begin();

  // Set up our characteristic: only notify needed
  dataCharacteristic.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
  // No reads/writes from the peer, so:
  dataCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  //dataCharacteristic.setFixedLen(0); // variable length is fine here
  dataCharacteristic.begin();

  // Configure advertising
  //Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addService(dataService);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addName();

  // Start advertising
  Bluefruit.Advertising.start(0);
  Serial.println("📡 BLE Advertising started (Bluefruit)");
}

void loop() {
  if (deviceConnected) {

    /*
    int ecgSignal = analogRead(1);                                      ///HEREEEEEEEEEEE
    float irValue = particleSensor.getIR();

    IMUAccelXSum += round(fabs(myIMU.readFloatAccelX()) * 10000.0) / 10000.0;
    IMUAccelYSum += round(fabs(myIMU.readFloatAccelY()) * 10000.0) / 10000.0;
    IMUAccelZSum += round(fabs(myIMU.readFloatAccelZ()) * 10000.0) / 10000.0;

    IMUGyroXSum += round(fabs(myIMU.readFloatGyroX()) * 10000.0) / 10000.0;
    IMUGyroYSum += round(fabs(myIMU.readFloatGyroY()) * 10000.0) / 10000.0;
    IMUGyroZSum += round(fabs(myIMU.readFloatGyroZ()) * 10000.0) / 10000.0;
    */

    int ecgSignal = 800;
    float irValue = 10000;

    IMUAccelXSum += 5;
    IMUAccelYSum += 5;
    IMUAccelZSum += 5;

    IMUGyroXSum += 5;
    IMUGyroYSum += 5;
    IMUGyroZSum += 5;

    // Save the sample in the buffer
    ecgBuffer[sampleIndex] = ecgSignal;
    ppgBuffer[sampleIndex] = irValue;

    sampleIndex++;

    // When enough samples have been accumulated, build and send the chunk.
    if (sampleIndex >= N_SAMPLES) {
      // Get the current timestamp (in milliseconds)
      unsigned long timestamp = millis();
      
      // Build the chunk string in the format:
      // TIMESTAMP;EEG,sample0,sample1,...,sampleN*
      String chunk = String(timestamp);
      chunk += ";ECG";
      for (int i = 0; i < N_SAMPLES; i++) {
        chunk += ",";
        chunk += String(ecgBuffer[i]);
      }

      chunk += ";PPG";
      for (int i = 0; i < N_SAMPLES; i++) {
        chunk += ",";
        chunk += String(ppgBuffer[i]);
      }

      // Attempt SCD41 read                                      ///HEREEEEEEEEEEE
      /*
      if (scd41.readMeasurement()) {
        latestCO2 = scd41.getCO2();
        latestTemp = scd41.getTemperature();
        latestHumidity = scd41.getHumidity();
      } else {
        Serial.println("Failed to read SCD41.");
      }
      */
      
      latestCO2 = 850;
      latestTemp = 20;
      latestHumidity = 35;

      // Append SCD41 data
      chunk += ";SCD," + String(latestCO2, 2) + "," 
               + String(latestTemp, 2) + "," 
               + String(latestHumidity, 2);

      
      // IMU readings
      chunk += ";IMU";
      chunk += ",";
      chunk += String(IMUAccelXSum);
      chunk += ",";
      chunk += String(IMUAccelYSum);
      chunk += ",";
      chunk += String(IMUAccelZSum);
      chunk += ",";
      chunk += String(IMUGyroXSum);
      chunk += ",";
      chunk += String(IMUGyroYSum);
      chunk += ",";
      chunk += String(IMUGyroZSum);
      
      chunk += "*"; // Mark the end of the chunk
      
      Serial.print("📡 Sending Chunk: ");
      Serial.println(chunk);
      
      // ✅ Use sendInChunks() to properly handle MTU limitations
      sendInChunks(chunk, MTU_SIZE);

      // Reset the buffer index for the next chunk.
      sampleIndex = 0;

      IMUAccelXSum = 0.0;
      IMUAccelYSum = 0.0;
      IMUAccelZSum = 0.0;

      IMUGyroXSum = 0.0;
      IMUGyroYSum = 0.0;
      IMUGyroZSum = 0.0;
    }

    // Delay between samples (simulate 100 Hz sampling rate)
    delay(10);  // 10 ms delay approximates 100 samples per second.
    
  } else {
    // Not connected, do other tasks or idle.
    delay(500);
  }
}
