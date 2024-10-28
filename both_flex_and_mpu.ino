#include <ESP8266WiFi.h>
#include <ThingSpeak.h>
#include <Wire.h>

const char* ssid = "hp";            // Replace with your Wi-Fi credentials
const char* password = "KK123456";

const unsigned long channelID = 2326023;   // Replace with Your ThingSpeak Channel ID and APIkey
const char* thingSpeakApiKey = "TEQC2LB9ZPG6DHAQ";

// Constants for flex 
// Calibrate flex sensor when you are using
const int flexPin = A0;
const float VCC = 5;      // Voltage at Arduino 5V line
const float R_DIV = 30000.0;  // Resistor used to create a voltage divider
const float flatResistance = 30000.0; // Resistance when flat
const float bendResistance = 80000.0; // Resistance at 180 deg

// MPU6050 Slave Device Address
const uint8_t MPU6050SlaveAddress = 0x68;

const uint8_t scl = 4;  // Replace as per your connection // GPIO5, D1
const uint8_t sda = 0;  // GPIO4, D2

const uint16_t AccelScaleFactor = 16384;
const uint16_t GyroScaleFactor = 131;

const uint8_t MPU6050_REGISTER_SMPLRT_DIV = 0x19;
const uint8_t MPU6050_REGISTER_USER_CTRL = 0x6A;
const uint8_t MPU6050_REGISTER_PWR_MGMT_1 = 0x6B;
const uint8_t MPU6050_REGISTER_PWR_MGMT_2 = 0x6C;
const uint8_t MPU6050_REGISTER_CONFIG = 0x1A;
const uint8_t MPU6050_REGISTER_GYRO_CONFIG = 0x1B;
const uint8_t MPU6050_REGISTER_ACCEL_CONFIG = 0x1C;
const uint8_t MPU6050_REGISTER_FIFO_EN = 0x23;
const uint8_t MPU6050_REGISTER_INT_ENABLE = 0x38;
const uint8_t MPU6050_REGISTER_ACCEL_XOUT_H = 0x3B;
const uint8_t MPU6050_REGISTER_SIGNAL_PATH_RESET = 0x68;

int16_t AccelX, AccelY, AccelZ, Temperature, GyroX, GyroY, GyroZ;
float elapsedTime, currentTime, previousTime;
double Ax, Ay, Az, T, Gx, Gy, Gz;
double Axp, Ayp, Azp = 0;
float dx, dy, dz = 0;
WiFiClient client;

unsigned long previousMillis = 0;
const unsigned long interval = 100; // Send data to ThingSpeak every 100 milliseconds

void setup() {
  Serial.begin(115200);
  Wire.begin(sda, scl);
  MPU6050_Init();
  pinMode(BUILTIN_LED, OUTPUT);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  ThingSpeak.begin(client);
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Read the ADC, and calculate voltage and resistance from it
    int ADCflex = analogRead(flexPin);
    float Vflex = ADCflex * VCC / 1023.0;
    float Rflex = (R_DIV * (VCC / Vflex - 1.0));
    float deflection = (-(0.0893) * Rflex) + (438.3058 - 45);
    ThingSpeak.setField(1, static_cast<float>(deflection));

    Read_RawValue(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_XOUT_H);
    previousTime = currentTime;
    currentTime = millis();
    elapsedTime = (currentTime - previousTime) / 1000;

    Ax = ((double)AccelX / AccelScaleFactor) + 0.02;
    Ay = ((double)AccelY / AccelScaleFactor) + 0.01;
    Az = ((double)AccelZ / AccelScaleFactor) - 1.06;
    T = (double)Temperature / 340 + 36.53;
    Gx = (double)GyroX / GyroScaleFactor;
    Gy = (double)GyroY / GyroScaleFactor;
    Gz = (double)GyroZ / GyroScaleFactor;
    dx = ((Ax - Axp) * (elapsedTime * elapsedTime) * 0.5) * 1000;
    dy = ((Ay - Ayp) * (elapsedTime * elapsedTime) * 0.5) * 1000;
    dz = ((Az - Azp) * (elapsedTime * elapsedTime) * 0.5) * 1000;
 //Serial.print("Ax: "); Serial.print(Ax);
  //Serial.print(" Ay: "); Serial.print(Ay);
  Serial.print(" Az: "); Serial.println(Az);
  //Serial.print(" T: "); Serial.print(T);
  //Serial.print(" Gx: "); Serial.print(Gx);
  //Serial.print(" Gy: "); Serial.print(Gy);
  //Serial.print(" Gz: "); Serial.println(Gz);
    ThingSpeak.setField(2, static_cast<float>(T));
    ThingSpeak.setField(3, static_cast<float>(Ax));
    ThingSpeak.setField(4, static_cast<float>(Ay));
    ThingSpeak.setField(5, static_cast<float>(Az));
    ThingSpeak.setField(6, static_cast<float>(Gx));
    ThingSpeak.setField(7, static_cast<float>(Gy));
    ThingSpeak.setField(8, static_cast<float>(Gz));
    Axp = Ax;
    Ayp = Ay;
    Azp = Az;

    int response = ThingSpeak.writeFields(channelID, thingSpeakApiKey);
  }

  // Other non-blocking code can go here
}

void I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.write(data);
  Wire.endTransmission();
}

void Read_RawValue(uint8_t deviceAddress, uint8_t regAddress) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, (uint8_t)14);
  AccelX = (((int16_t)Wire.read() << 8) | Wire.read());
  AccelY = (((int16_t)Wire.read() << 8) | Wire.read());
  AccelZ = (((int16_t)Wire.read() << 8) | Wire.read());
  Temperature = (((int16_t)Wire.read() << 8) | Wire.read());
  GyroX = (((int16_t)Wire.read() << 8) | Wire.read());
  GyroY = (((int16_t)Wire.read() << 8) | Wire.read());
  GyroZ = (((int16_t)Wire.read() << 8) | Wire.read());
}

void MPU6050_Init() {
  delay(100);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SMPLRT_DIV, 0x07);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_1, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_2, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_CONFIG, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_GYRO_CONFIG, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_CONFIG, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_FIFO_EN, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_INT_ENABLE, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SIGNAL_PATH_RESET, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_USER_CTRL, 0x00);
}
