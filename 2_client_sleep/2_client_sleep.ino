#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <math.h>

// Configs
#define DEVICE_NAME "ESP32-Current-Sensor2"
#define SERVICE_UUID "12345678-1234-1234-1234-123456789ab2"
#define CHAR_UUID "abcdabcd-1234-5678-abcd-123456789abc"
#define DATA_INTERVAL 25    // Interval between sensor readings unit second(s)
#define SEND_INTERVAL 1000    // Interval between sending data

// MCP3221 Configuration
#define SDA_PIN 8   // SDA ของ ESP32-C3 Supermini
#define SCL_PIN 9   // SCL ของ ESP32-C3 Supermini
#define MCP3221_ADDRESS 0x4D // I2C Address ของ MCP3221
#define clockFrequency 400000
#define offset 126

// Variables for current measurement
signed long long sum_all = 0;
signed long long check_overflow = 0;
uint32_t count_Value = 0;
uint32_t count_overflow = 0;
uint32_t count_sensorReading = 0;
double current = 0;
double V = 0;
double V_rms = 0;
double gain = 100;
double R = 100;

// New RTC memory variables to track sleep count
RTC_DATA_ATTR float sensorReadings[20];
RTC_DATA_ATTR uint32_t sleepCount = 0;  // Track number of deep sleep cycles
RTC_DATA_ATTR uint8_t start = 0; 

// BLE Variables
BLEServer *pServer = nullptr;
BLECharacteristic *pCharacteristic = nullptr;
bool deviceConnected = false;
unsigned long lastNotifyTime = 0;
unsigned long lastReadingTime = 0;
unsigned long lastSendingTime = 0;
int measurementCycle = 0;

class CharacteristicCallbacks: public BLECharacteristicCallbacks {
    void onStatus(BLECharacteristic* pCharacteristic, Status s, uint32_t code) {
        if (s == Status::SUCCESS_NOTIFY) {
            Serial.printf("Notification success!\n");
        } else {
            Serial.printf("Notification failed: status=%d, code=%d\n", s, code);
        }
    }
};

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        Serial.printf("Device connected\n");
    }
    
    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        Serial.printf("Device disconnected\n");
        BLEDevice::startAdvertising();
    }
};

// MCP3221 Reading Function
uint16_t readMCP3221() {
    unsigned int rawData = 0;
    Wire.beginTransmission(MCP3221_ADDRESS);
    Wire.requestFrom(MCP3221_ADDRESS, 2);
    if (Wire.available() == 2) {
        rawData = (Wire.read() << 8) | (Wire.read());
    } else {
        Wire.beginTransmission(MCP3221_ADDRESS);
        Wire.endTransmission();
    }
    return rawData;
}

void readSensorData() {
    uint16_t adcValue = readMCP3221(); // Read ADC value
    V = (adcValue * 3.3*1000) / 4096.0; // คำนวณแรงดันไฟฟ้า (VREF = 3.3V) mV
    // Update running sum for RMS calculation
    sum_all += (V - offset) * (V - offset);
    count_Value++;
    
    // Check for overflow
    if(sum_all < check_overflow) {
        count_overflow++;
        sum_all = 0;
        count_Value = 0;
        sum_all += (V - offset) * (V - offset);
        count_Value++;
    }
    
    // Calculate RMS current every 4000 samples
    if(count_Value == 1000) {
        V_rms = sqrt(sum_all / count_Value);
        sum_all = 0;
        count_Value = 0;
        current = V_rms/R;
        sensorReadings[count_sensorReading] = current;
        count_sensorReading++;
        
        // Increment sleep count before going to deep sleep
        sleepCount++;
        
        Serial.printf("Sleep Count: %lu\n", sleepCount);
        Serial.printf("Going to Deep Sleep\n");
        esp_deep_sleep_start();
    }
}

void sendRealtimeData() {
    if (!deviceConnected) {
        return;
    }

    unsigned long currentTime = millis();
    if (currentTime - lastNotifyTime < SEND_INTERVAL) {
        return;
    }

    // Prepare data string with measurement cycle, sleep count, and sensor readings
    String data = String(measurementCycle) + "," + String(sleepCount) + ",";
    
    // Add all sensor readings to the data string
    for (int i = 0; i < count_sensorReading; i++) {
        data += String(sensorReadings[i]);
        if (i < count_sensorReading - 1) {
            data += ",";
        }
    }

    Serial.printf("Sending real-time data: %s\n", data.c_str());
    
    pCharacteristic->setValue(data.c_str());
    pCharacteristic->notify();
    
    lastNotifyTime = currentTime;
    measurementCycle++;

    // Reset count_sensorReading after sending
    count_sensorReading = 0;
}

void setup() {
    Serial.begin(115200);
    delay(100);
    
    // Initialize I2C
    Wire.begin();
    Wire.setClock(clockFrequency);
    Serial.printf("MCP3221 I2C Reader Initialized\n");
    
    // Set sleep time (e.g., 30 seconds)
    // esp_sleep_enable_timer_wakeup(DATA_INTERVAL * 1000000); // 30 seconds = 30,000,000 microseconds
    // Print sleep count on startup
    Serial.printf("Total Sleep Cycles: %lu\n", sleepCount);
    // Initialize BLE
    BLEDevice::init(DEVICE_NAME);
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    
    BLEService *pService = pServer->createService(SERVICE_UUID);
    pCharacteristic = pService->createCharacteristic(
        CHAR_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_NOTIFY
    );
    
    pCharacteristic->setCallbacks(new CharacteristicCallbacks());
    pCharacteristic->addDescriptor(new BLE2902());
    pService->start();
    
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);

    BLEDevice::startAdvertising();
    
    Serial.printf("Current Sensor ready and advertising...\n");
}

void loop() {
    if(start == 0){
      start = 1;
      esp_deep_sleep_start();
    }
    readSensorData();
    if(!deviceConnected) {
        BLEDevice::startAdvertising();
    }
    if(count_sensorReading == 20){
        if(deviceConnected) {
            sendRealtimeData();
        }
    }
}