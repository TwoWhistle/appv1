#include <Arduino.h>
#include <bluefruit.h>

// -----------------------
// Match your original UUIDs
// -----------------------
#define SERVICE_UUID          "7B36A001-7C44-4D49-B574-2E18D904AF71"
#define EEG_CHARACTERISTIC    "7B36A002-7C44-4D49-B574-2E18D904AF71"

// Define how many samples per chunk you want to send.
#define N_SAMPLES 100
#define MTU_SIZE 23  // BLE Max Transmission Unit size

// -----------------------
// Create BLE Service & Characteristic
// -----------------------
BLEService eegService = BLEService(SERVICE_UUID);
BLECharacteristic eegCharacteristic = BLECharacteristic(EEG_CHARACTERISTIC);

// Track connection state
bool deviceConnected = false;

// Buffer for EEG samples
int eegBuffer[N_SAMPLES];
uint8_t sampleIndex = 0;

// -----------------------
// Forward Declarations
// -----------------------
void connect_callback(uint16_t conn_handle);
void disconnect_callback(uint16_t conn_handle, uint8_t reason);
void sendInChunks(const String &fullStr, size_t maxChunkSize = MTU_SIZE);

void setup() {
  Serial1.begin(115200);
  Serial.println("🚀 Bluefruit BLE Setup Starting...");

  // 1) Initialize the Bluefruit stack
  Bluefruit.begin();
  
  // (Optional) Set max transmission power
  Bluefruit.setTxPower(4);            // +4 dBm
  // (Optional) Set local device name for advertising
  Bluefruit.setName("nrfear");
  
  // 2) Set up connect/disconnect callbacks
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // 3) Create and start the BLE Service
  eegService.begin();

  // 4) Configure the EEG Characteristic
  eegCharacteristic.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
  eegCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  eegCharacteristic.begin();

  // 5) Configure advertising
  Bluefruit.Advertising.addService(eegService);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.start(0);

  Serial.println("📡 BLE Advertising Started with Service UUID");
}

void loop() {
  if (deviceConnected) {
    // Simulate EEG reading (or use analogRead if connected to a sensor)
    int eegSignal = 900; // or analogRead(A1);
    
    // Save the sample in the buffer
    eegBuffer[sampleIndex] = eegSignal;
    sampleIndex++;

    // When enough samples have been accumulated, build and send the chunk.
    if (sampleIndex >= N_SAMPLES) {
      // Get the current timestamp (in milliseconds)
      unsigned long timestamp = millis();
      
      // Build the chunk string in the format:
      // TIMESTAMP;EEG,sample0,sample1,...,sampleN*
      String chunk = String(timestamp);
      chunk += ";EEG";
      for (int i = 0; i < N_SAMPLES; i++) {
        chunk += ",";
        chunk += String(eegBuffer[i]);
      }
      chunk += "*"; // Mark the end of the chunk
      
      Serial.print("📡 Sending EEG Chunk: ");
      Serial.println(chunk);
      
      // ✅ Use sendInChunks() to properly handle MTU limitations
      sendInChunks(chunk, MTU_SIZE);

      // Reset the buffer index for the next chunk.
      sampleIndex = 0;
    }

    // Delay between samples (simulate 100 Hz sampling rate)
    delay(10);  // 10 ms delay approximates 100 samples per second.
    
  } else {
    // Not connected, do other tasks or idle.
    delay(500);
  }
}

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
        if (eegCharacteristic.notify(chunkData, chunkLen)) {
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

// -----------------------
// Callback Implementations
// -----------------------
void connect_callback(uint16_t conn_handle) {
  deviceConnected = true;
  Serial.println("✅ Device connected!");
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  deviceConnected = false;
  Serial.println("❌ Disconnected! Restarting advertising...");
  Bluefruit.Advertising.start(0);
}
