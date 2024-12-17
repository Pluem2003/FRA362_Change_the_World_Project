#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <math.h>

// Configs
#define DEVICE_NAME "ESP32-Current-Sensor1"
#define SERVICE_UUID "12345678-1234-1234-1234-123456789ab1"
#define CHAR_UUID "abcdabcd-1234-5678-abcd-123456789abc"
#define DATA_INTERVAL 1000    // Interval between sending data
#define MAX_CYCLES 100        // Maximum number of test cycles

// I2C and MCP3221 Configurations
#define SDA_PIN 8             // SDA of ESP32-C3 Supermini
#define SCL_PIN 9             // SCL of ESP32-C3 Supermini
#define MCP3221_ADDRESS 0x4D  // I2C Address of MCP3221
#define CLOCK_FREQUENCY 400000
#define OFFSET 100.708008
#define Gain 10
#define CURRENT_RESISTOR 100  // Shunt resistor value in ohms

// Variables for sensor and BLE
volatile float currentReading = 0.0;
volatile float current = 0.0;
volatile float voltageReading = 0.0;
volatile float vrms = 0.0;
unsigned long lastNotifyTime = 0;
unsigned long lastReadingTime = 0;

// Test tracking variables
int measurementCycle = 0;
float sentReadings[MAX_CYCLES];
unsigned long sentTimestamps[MAX_CYCLES];
int successfulTransmissions = 0;
int failedTransmissions = 0;

// Accumulation variables for RMS calculation
volatile signed long long sum_all = 0;
volatile uint32_t count_Value = 0;
volatile uint32_t count_overflow = 0;

// BLE
BLEServer *pServer = nullptr;
BLECharacteristic *pCharacteristic = nullptr;
bool deviceConnected = false;

class CharacteristicCallbacks: public BLECharacteristicCallbacks {
    void onStatus(BLECharacteristic* pCharacteristic, Status s, uint32_t code) {
        if (s == Status::SUCCESS_NOTIFY) {
            Serial.println("Notification success!");
        } else {
            Serial.printf("Notification failed: status=%d, code=%d\n", s, code);
        }
    }
};

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        Serial.println("Device connected");
        delay(1000);
    }
    
    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        Serial.println("Device disconnected");
        BLEDevice::startAdvertising();
    }
};

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

void IRAM_ATTR readSensorData() {
    // Read ADC value
    uint16_t adcValue = readMCP3221();
    
    // Calculate voltage (in mV)
    float instantVoltage = (adcValue * 3.3 * 1000) / 4096.0;
    Serial.printf("%f, %f, %f\n",instantVoltage - OFFSET,instantVoltage,currentReading);
    // Accumulate for RMS calculation
    signed long long check_overflow = sum_all;
    sum_all += (instantVoltage - OFFSET) * (instantVoltage - OFFSET);
    count_Value++;
    
    // Handle potential overflow
    if(sum_all < check_overflow) {
        sum_all = 0;
        count_Value = 0;
        count_overflow++;
        sum_all += (instantVoltage - OFFSET) * (instantVoltage - OFFSET);
        count_Value++;
    }
    
    // Update volatile variables
    voltageReading = instantVoltage;
    
    // Calculate RMS and current every 4000 samples
    if(count_Value == 1000) {
        vrms = sqrt(sum_all / count_Value);
        current = vrms / CURRENT_RESISTOR;
        currentReading = current*Gain;
        sum_all = 0;
        count_Value = 0;
    }
}

void sendRealtimeData() {
    
    unsigned long currentTime = millis();
    if (currentTime - lastNotifyTime < DATA_INTERVAL) {
        return;
    }
    
    // Prepare data string with measurement cycle, timestamp, voltage, RMS, and current
    String data = String(measurementCycle) + "," +
                  String(currentTime) + "," +
                  String(currentReading, 3);
    
    Serial.printf("Sending real-time data: %s\n", data.c_str());
    
    pCharacteristic->setValue(data.c_str());
    pCharacteristic->notify();
    
    // Store sent data for verification
    sentReadings[measurementCycle] = currentReading;
    sentTimestamps[measurementCycle] = currentTime;
    
    lastNotifyTime = currentTime;
    measurementCycle++;
}

void printTestResults() {
    Serial.println("\n===== TEST RESULTS =====");
    Serial.printf("Total Cycles: %d\n", MAX_CYCLES);
    Serial.printf("Successful Transmissions: %d\n", successfulTransmissions);
    Serial.printf("Failed Transmissions: %d\n", failedTransmissions);
    Serial.println("=======================\n");
}

void setup() {
    Serial.begin(115200);
    
    // Initialize I2C
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(CLOCK_FREQUENCY);
    
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
    
    Serial.println("Sensor ready and advertising...");
}

void loop() {
    readSensorData();
    unsigned long currentTime = millis();
    
    // Check if BLE Client has enabled notifications
    uint8_t notifyStatus = pCharacteristic->getDescriptorByUUID(BLEUUID((uint16_t)0x2902))->getValue()[0];
    if (notifyStatus != 0x01) {
        Serial.println("Notifications not enabled by client yet.");
        return;  // Wait for client to enable notifications
    }

    if (currentTime - lastReadingTime >= DATA_INTERVAL) {
        lastReadingTime = currentTime;
        sendRealtimeData();
    }
    
    // End test after MAX_CYCLES
}