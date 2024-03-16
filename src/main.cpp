#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GPS.h>
#include <Adafruit_BMP280.h>
#include <SD.h>
#include <SPI.h>

#define BMP_SCK (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS (10)

Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO, BMP_SCK);

#define GPSSerial Serial8
Adafruit_GPS GPS(&GPSSerial);

const unsigned long interval = 100;
unsigned long previousMillis = 0;

const int MPU = 0x68; // MPU6050 I2C address

float AccX, AccY, AccZ;
int16_t AccX_raw, AccY_raw, AccZ_raw;
String accelerometerData;

// SD Card Stuff
Sd2Card card;
SdVolume volume;
SdFile root;
File myFile;
File counterFile;
const int chipSelect = BUILTIN_SDCARD;
String fileName;

void initSDCard()
{
  Serial.println("Initializing SD Card...");

  if (!card.init(SPI_HALF_SPEED, chipSelect))
  {
    Serial.println({"initialization Failed!"});
  }

  fileName = String(SD.usedSize()) + ".csv";
}

void setup()
{
  // Wait until serial port is initialized
  /**
   * TESTING PURPOSES ONLY SHOULD BE COMMENTED ON PRODUCTION
  */
  // while (!Serial)
  // {
  //   delay(100);
  // }
  Serial2.begin(9600); // antenna
  Serial.begin(9600);  // PC Port for testing
  GPS.begin(4800);     // GPS Modules
  bmp.begin();         // BMP 280

  delay(1000);

  initSDCard();

  // IMU init
  Wire.begin(); // start communication
  Wire.beginTransmission(MPU);

  Wire.write(0x6B);
  Wire.write(0x00); // reset device
  Wire.endTransmission(true);

  // BMP280 init
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

/**
 * Reads the acceleration values and returns as a string
 */
String readAccValues()
{
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(static_cast<uint8_t>(MPU),
                   static_cast<uint8_t>(6),
                   static_cast<bool>(true)); // Read 6 registers total, each axis value is stored in 2 registers

  int16_t AccX_raw = (Wire.read() << 8 | Wire.read()); // X-axis value
  int16_t AccY_raw = (Wire.read() << 8 | Wire.read()); // Y-axis value
  int16_t AccZ_raw = (Wire.read() << 8 | Wire.read()); // Z-axis value

  double AccX = (AccX_raw / 16384.0) * 9.81; // Convert to actual accelerometer value
  double AccY = (AccY_raw / 16384.0) * 9.81;
  double AccZ = (AccZ_raw / 16384.0) * 9.81;

  String result = String(AccX) + "," + String(AccY) + "," + String(AccZ);
  return result;
}

void pr(const String &data)
{
  Serial.println(data);
  Serial2.println(data);
}

/**
 * Ensure string is formatted as CSV. Saves the string to data file
 * Checks if -1 is data value
 */
void saveData(String data)
{
  myFile = SD.open(fileName.c_str(), FILE_WRITE);
  if (data.equals("-1"))
    return;
  if (myFile)
  {
    myFile.println(data);
    myFile.close();
  }
  else
  {
    Serial.print("Error opening the file :");
    Serial.println(fileName);
  }
}
/**
 * Returns values in CSV format based on whether or not GPS data is received
 * returns -1 when no data is available to read
 * Returns in this format: Millseconds running, Acceleration X, Y and Z, Temperature, Pressure, Altitude, Latitude (Most sig figs), Lat (Least Sig Figs)
 * Longitude (Most Sig Figs), Latitude (Least Sig Figs)
 */
String getAllData()
{
  unsigned long currentMillis = millis();

  delay(10);
  GPS.read();
  if (GPS.newNMEAreceived())
  {
    accelerometerData = readAccValues();
    // Someone please do this string formatting better please
    if (GPS.lat)
    {
      return String(millis()) + "," + accelerometerData + "," + String(bmp.readTemperature()) + "," + bmp.readPressure() + "," + bmp.readAltitude(1027) + "," +
             String(GPS.latitude, 4) + String(GPS.lat) + "," + String(GPS.longitude, 4) + String(GPS.lon);
    }
    if (!GPS.parse(GPS.lastNMEA()))
    {
    }
  }

  else if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;
    accelerometerData = readAccValues();

    /*
    pr(String(millis()) + ",");
    pr(accelerometerData + ",");
    pr(String(bmp.readTemperature()) + ",");
    pr(String(bmp.readPressure()) + ",");
    pr(String(bmp.readAltitude(1027)) + "\n");
    */
   // Same here
    return String(millis()) + "," + accelerometerData + "," + String(bmp.readTemperature()) + "," + bmp.readPressure() + "," + bmp.readAltitude(1027) + ",,,,";
  }
  return "-1";
}

void loop()
{
  String currentData = getAllData();

  if (!(currentData.equals("-1")))
  {
    pr(currentData);
    saveData(currentData);
  }
}
