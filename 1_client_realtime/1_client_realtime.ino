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
#define DATA_INTERVAL 1000    // Interval between sensor readings
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
double currentReading = 0;
double gain = 0;
double currentReal = 0;

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
    }
    
    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        Serial.println("Device disconnected");
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

double calculateGain(double current) {
    double gain;

    if (current >= 0 && current < 8) {
        // Fixed value for 0 <= x < 30
        gain = 0.048;
  } else if (current >=8  && current < 53.8) {
        // Fixed value for 30 <= x < 50
        gain = 0.043;
    } else if (current >= 25 && current < 53.8) {
        // Fixed value for 30 <= x < 50
        gain = 0.0813;
    } else if (current >= 53.8 && current < 100) {
        // Fixed value for 30 <= x < 50
        gain = 0.0773;
    } else if (current >= 100 && current < 400) {
        // Equation for 50 <= x < 400
        gain = 0.0773;
    } else if (current >= 400) {
        // Fixed value for x > 400
        gain = 0.01361;
    } else {
        // Default case (outside defined ranges)
        gain = 0.0; // or handle error
    }

    return gain;
}

void readSensorData() {
    uint16_t adcValue = readMCP3221(); // Read ADC value
    
    // Update running sum for RMS calculation
    sum_all += (adcValue - offset) * (adcValue - offset);
    count_Value++;
    
    // Check for overflow
    if(sum_all < check_overflow) {
        count_overflow++;
        sum_all = 0;
        count_Value = 0;
        sum_all += (adcValue - offset) * (adcValue - offset);
        count_Value++;
    }
    
    // Calculate RMS current every 4000 samples
    if(count_Value == 4000) {
        currentReading = sqrt(sum_all / count_Value);
        
        sum_all = 0;
        count_Value = 0;
        gain = calculateGain(currentReading);
        currentReal = currentReading*gain;
        
        // Serial.printf("%f,  %f, %f\n",current,gain,currentReal);
        
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

    // Prepare data string with measurement cycle and timestamp
    String data = String(measurementCycle) + "," + 
                  String(millis()) + "," + 
                  String(currentReading);

    Serial.printf("Sending real-time data: %s\n", data.c_str());
    
    pCharacteristic->setValue(data.c_str());
    pCharacteristic->notify();
    
    lastNotifyTime = currentTime;
    measurementCycle++;
}

void setup() {
    Serial.begin(115200);
    
    // Initialize I2C
    Wire.begin();
    Wire.setClock(clockFrequency);
    Serial.println("MCP3221 I2C Reader Initialized");
    
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
    
    Serial.println("Current Sensor ready and advertising...");
}

void loop() {
    unsigned long currentTime = millis();
    // uint16_t adcValue = readMCP3221(); // Read ADC value
    // Serial.println(adcValue-126);
    readSensorData();
    if (currentTime - lastSendingTime >= SEND_INTERVAL) {
        lastSendingTime = currentTime;
        sendRealtimeData();
    }
    
}