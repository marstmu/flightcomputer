#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GPS.h>
#include <Adafruit_BMP280.h>
#include <SD.h>

#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO, BMP_SCK);

#define GPSSerial Serial8
Adafruit_GPS GPS(&GPSSerial);

const unsigned long interval = 100;
unsigned long previousMillis = 0;

const int MPU = 0x68; // MPU6050 I2C address
String filename;  // For dynamic filename generation
File dataFile;

void setup() {
  Serial2.begin(9600); // antenna
  Serial.begin(9600);
  GPS.begin(9600);
  bmp.begin();

  delay(1000);

  // Initialize SD card
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("Card failed, or not present");
    while (1);  // Don't proceed further if no SD card
  }

  // Find the next available filename
  for (int i = 0; i < 10000; i++) {
    filename = "datalog" + String(i) + ".csv";
    if (!SD.exists(filename.c_str())) {  // Convert String to const char* using c_str()
      break; // Found an unused filename
    }
  }

  // Open the new file for writing
  dataFile = SD.open(filename.c_str(), FILE_WRITE);  // Convert String to const char* using c_str()
  if (dataFile) {
    dataFile.println("Logging session started"); // Optional: Write a start marker in the file
    dataFile.close();
  } else {
    Serial.println("Error opening " + filename);
    while (1);
  }

  // IMU init
  Wire.begin(); // start communication with MPU6050
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00); // reset device
  Wire.endTransmission(true);

  // BMP280 init
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);
}


String readAccValues() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total

  int16_t AccX_raw = (Wire.read() << 8 | Wire.read()); // X-axis value
  int16_t AccY_raw = (Wire.read() << 8 | Wire.read()); // Y-axis value
  int16_t AccZ_raw = (Wire.read() << 8 | Wire.read()); // Z-axis value

  double AccX = (AccX_raw / 16384.0) * 9.81; // Convert to m/s^2
  double AccY = (AccY_raw / 16384.0) * 9.81;
  double AccZ = (AccZ_raw / 16384.0) * 9.81;

  return String(AccX) + "," + String(AccY) + "," + String(AccZ);
}

void pr(const String &data) {
  Serial.print(data);
  Serial2.print(data);

  // Write to SD card
  dataFile = SD.open(filename.c_str(), FILE_WRITE);  // Convert String to const char* using c_str()
  if (dataFile) {
    dataFile.println(data);
    dataFile.close();
  } else {
    Serial.println("error opening " + filename);
  }
}


void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    String accelerometerData = readAccValues();
    pr(String(millis()) + "," + accelerometerData + "," + String(bmp.readTemperature()) + "," + String(bmp.readPressure()) + "," + String(bmp.readAltitude(1027)) + "\n");
  }

  GPS.read();
  if (GPS.newNMEAreceived()) {
    String accelerometerData = readAccValues();
    if (!GPS.parse(GPS.lastNMEA())) return; // Parse NMEA

    if (GPS.fix) {
      pr(String(millis()) + "," + accelerometerData + "," + String(bmp.readTemperature()) + "," + String(bmp.readPressure()) + "," + String(bmp.readAltitude(1027)) + "," +
         String(GPS.latitude, 4) + GPS.lat + "," + String(GPS.longitude, 4) + GPS.lon + "\n");
    }
  }
}