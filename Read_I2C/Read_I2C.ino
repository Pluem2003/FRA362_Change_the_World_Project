#include <Wire.h>
#include <math.h>
#include <stdint.h>

#define SDA_PIN 8   // SDA ของ ESP32-C3 Supermini
#define SCL_PIN 9   // SCL ของ ESP32-C3 Supermini
#define MCP3221_ADDRESS 0x4D // I2C Address ของ MCP3221
#define clockFrequency 400000
#define offset 100.708008

signed long long sum_all = 0;
signed long long check_overflow = 0;
uint32_t count_Value = 0;
uint32_t count_overflow = 0;
double current = 0;
double V = 0;
double V_rms = 0;
double gain = 100;
double R = 100;

uint16_t readMCP3221() {
  unsigned int rawData = 0;
  Wire.beginTransmission(MCP3221_ADDRESS);
  Wire.requestFrom(MCP3221_ADDRESS, 2);
  if (Wire.available() == 2)
  {
    rawData = (Wire.read() << 8) | (Wire.read());
  }
  else
  {
    Wire.beginTransmission(MCP3221_ADDRESS);
    Wire.endTransmission();
  }
  return rawData;
}

// float calculateGain(float x) {
//   if (x <= 50) {
//     // Use the first equation for x <= 50
//     float a1 = -0.169;
//     float b1 = 0.094;
//     float c1 = 0.111;
//     return a1 * exp(b1 * x) + c1;
//   } else {
//     // Use the second equation for x > 50
//     float a2 = 0.000006;
//     float b2 = -0.0047;
//     float c2 = 0.9148;
//     return a2 * x * x + b2 * x + c2;
//   }
// }

void setup() {
  Wire.begin();
  Wire.setClock(clockFrequency);
  Serial.begin(115200);
  Serial.println("MCP3221 I2C Reader Initialized");
}

void loop() {
  check_overflow = sum_all;
  uint16_t adcValue = readMCP3221(); // อ่านค่า ADC
  V = (adcValue * 3.3*1000) / 4096.0; // คำนวณแรงดันไฟฟ้า (VREF = 3.3V) mV
  // Serial.println(adcValue - offset);
  // Serial.printf("%d, %f,  %f, %f\n",adcValue,current,gain,currentReal);
  Serial.printf("%f, %f, %f\n",V-offset,V_rms,current);
  sum_all += (V - offset) * (V - offset);
  count_Value++;
  if(sum_all < check_overflow) {
    sum_all = 0;
    count_Value = 0;
    count_overflow++;
    sum_all += (V - offset) * (V - offset);
    count_Value++;
  }
  if(count_Value == 4000){
    V_rms = sqrt(sum_all / count_Value);
    sum_all = 0;
    count_Value = 0;
    // gain = calculateGain(V_rms);
    current = V_rms/R;
  }

}