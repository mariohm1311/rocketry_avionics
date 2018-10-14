#include <Arduino.h>
// #include <Wire.h>
// #include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_MPL3115A2.h>
#include <utility/imumaths.h>
#include <TinyGPS++.h>
#include <SPI.h>
#include <SD.h>

File myFile;
#define FILE_TRUNC (O_READ | O_WRITE | O_CREAT | O_TRUNC)
// TinyGPSPlus gps;
// SoftwareSerial gpsSerial(2, 3);
float lat = 0.0;
float lon = 0.0;
float alt = 0.0;
String fString;

uint8_t updateN = 1;
int writeState = 1;
const int buttonPin = 5;
const int ledPin = 13;

#define BNO055_SAMPLERATE_DELAY_MS (10)
Adafruit_BNO055 bno = Adafruit_BNO055();
Adafruit_MPL3115A2 mpl = Adafruit_MPL3115A2();

int freeRam()
{
    extern int __heap_start, *__brkval;
    int v;
    return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

void writeHeader(File f)
{
    f.print("c_sys,");
    f.print("c_gyr,");
    f.print("c_acc,");
    f.print("c_mag,");
    f.print("acc_x,");
    f.print("acc_y,");
    f.print("acc_z,");
    f.print("ang_x,");
    f.print("ang_y,");
    f.print("ang_z,");
    f.print("g_x,");
    f.print("g_y,");
    f.print("g_z,");
    f.print("press,");
    f.print("alt,");
    f.print("temp,");
    f.print("mill");
    f.println();
}

void writeData(File f,
               imu::Vector<3> acc, imu::Vector<3> ang, imu::Vector<3> grav,
               float pres_pa, float alt_m, float temp_C,
               uint8_t sys, uint8_t gyro, uint8_t accel, uint8_t mag,
               unsigned long mil)
{
    f.print(sys);
    f.print(",");
    f.print(gyro);
    f.print(",");
    f.print(accel);
    f.print(",");
    f.print(mag);
    f.print(",");

    f.print(acc.x(), 3);
    f.print(",");
    f.print(acc.y(), 3);
    f.print(",");
    f.print(acc.z(), 3);
    f.print(",");

    f.print(ang.x(), 2);
    f.print(",");
    f.print(ang.y(), 2);
    f.print(",");
    f.print(ang.z(), 2);
    f.print(",");

    f.print(grav.x(), 3);
    f.print(",");
    f.print(grav.y(), 3);
    f.print(",");
    f.print(grav.z(), 3);
    f.print(",");

    f.print(pres_pa, 1);
    f.print(",");
    f.print(alt_m, 2);
    f.print(",");
    f.print(temp_C, 1);
    f.print(",");

    f.print(mil);
    f.println();
}

void setup()
{
    Serial.begin(9600);
    // gpsSerial.begin(9600);

    while (!Serial)
    {
        ; // wait for serial port to connect. Needed for native USB port only
    }

    pinMode(buttonPin, INPUT);
    pinMode(ledPin, OUTPUT);

    Serial.println("##############################################");
    Serial.println("          IMU + ALTIMETER + GPS TEST          ");
    Serial.println("##############################################");

    Serial.print("Initializing SD card...");
    if (!SD.begin(4))
    {
        Serial.println("initialization failed!");
        while (1)
            ;
    } else {
        Serial.println();
    }
    delay(1000);

    myFile = SD.open("testi.txt", FILE_TRUNC);
    if (myFile)
    {
        writeHeader(myFile);
        myFile.close();
    } else {
        Serial.println("Error opening file");
    }
    

    if (!bno.begin())
    {
        Serial.println("No IMU detected");
        while (1)
            ;
    }

    delay(1000);
    bno.setExtCrystalUse(true);

    if (!mpl.begin())
    {
        Serial.println("No altimeter detected");
        return;
    }
    delay(1000);

    Serial.println("Calibration values:\n0=uncalibrated, 3=fully calibrated");
    Serial.println();
    Serial.println("----------------------------------------------");

    myFile = SD.open("testi.txt", FILE_WRITE);
    if (!myFile)
    {
        Serial.println("Error opening file");
    }
    delay(500);
}

void loop()
{
    float pres_pa, temp_C, alt_m = 0.0;
    uint8_t sys, gyro, accel, mag;

    if (writeState == 1 and digitalRead(buttonPin)) {
        writeState = 0;
        myFile.close();
    }

    pres_pa = mpl.getPressure();
    alt_m = 44330.77 * (1 - pow(pres_pa / 101326.0, 0.1902632));
    temp_C = mpl.getTemperature();

    imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu::Vector<3> ang = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    imu::Vector<3> grav = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);

    /* Display calibration status for each sensor. */
    bno.getCalibration(&sys, &gyro, &accel, &mag);

    if (writeState == 1) {
        digitalWrite(ledPin, HIGH);
        writeData(myFile, acc, ang, grav, pres_pa, alt_m, temp_C, sys, gyro, accel, mag, millis());
        myFile.flush();
        digitalWrite(ledPin, LOW);
    }

    Serial.print("Update #");
    Serial.println(updateN);
    updateN++;

    if (gyro == 3 and accel == 3 and mag == 3)
    {
        /* Display the floating point data */
        Serial.print("ACCELERATION:  ");
        Serial.print("X: ");
        Serial.print(acc.x(), 2);
        Serial.print(" Y: ");
        Serial.print(acc.y(), 2);
        Serial.print(" Z: ");
        Serial.print(acc.z(), 2);
        Serial.println();

        Serial.print("ROTATION:      ");
        Serial.print("X: ");
        Serial.print(ang.x(), 2);
        Serial.print(" Y: ");
        Serial.print(ang.y(), 2);
        Serial.print(" Z: ");
        Serial.print(ang.z(), 2);
        Serial.println();

        Serial.print("GRAVITY VEC:   ");
        Serial.print("X: ");
        Serial.print(grav.x(), 2);
        Serial.print(" Y: ");
        Serial.print(grav.y(), 2);
        Serial.print(" Z: ");
        Serial.print(grav.z(), 2);
        Serial.println();

        Serial.print("AMBIENT PRESSURE:    ");
        Serial.print(pres_pa / 101325, 4);
        Serial.println(" atm");

        // Altitude can be calculated directly through the following equation
        
        // alt_m = mpl.getAltitude();
        Serial.print("BAROMETRIC ALTITUDE: ");
        Serial.print(alt_m, 2);
        Serial.println(" m");

        Serial.print("AMBIENT TEMPERATURE:  ");
        Serial.print(temp_C, 2);
        Serial.println(" *C");

        Serial.println("----------------------------------------------");
    } else {
        Serial.print("CALIBRATION: Sys=");
        Serial.print(sys, DEC);
        Serial.print(" Gyro=");
        Serial.print(gyro, DEC);
        Serial.print(" Accel=");
        Serial.print(accel, DEC);
        Serial.print(" Mag=");
        Serial.println(mag, DEC);
        Serial.println("----------------------------------------------");
    }

    delay(10);
}



    // #include <Arduino.h>
    // #include <Wire.h>
    // #include <SoftwareSerial.h>
    // #include <Adafruit_Sensor.h>
    // #include <Adafruit_BNO055.h>
    // #include <Adafruit_MPL3115A2.h>
    // #include <utility/imumaths.h>
    // #include <TinyGPS++.h>
    // #include <SPI.h>
    // #include <SD.h>

    // File myFile;
    // TinyGPSPlus gps;
    // SoftwareSerial gpsSerial(2, 3);
    // float lat = 0.0;
    // float lon = 0.0;
    // float alt = 0.0;

    // #define BNO055_SAMPLERATE_DELAY_MS (1000)
    // Adafruit_BNO055 bno = Adafruit_BNO055();
    // Adafruit_MPL3115A2 mpl = Adafruit_MPL3115A2();

    // void setup()
    // {
    //     Serial.begin(9600);
    //     gpsSerial.begin(9600);

    //     Serial.println("##############################################");
    //     Serial.println("          IMU + ALTIMETER + GPS TEST          ");
    //     Serial.println("##############################################");

    //     Serial.print("Initializing SD card...");
    //     if (!SD.begin(4))
    //     {
    //         Serial.println("initialization failed!");
    //         while (1)
    //             ;
    //     }

    //     myFile = SD.open("avionics_test.txt", FILE_WRITE);
    //     myFile.println("acc_x,acc_y,acc_z,ang_x,ang_y,ang_z,g_x,g_y,g_z,press,alt,temp");

    //     if (!bno.begin())
    //     {
    //         Serial.println("No IMU detected");
    //         while (1)
    //             ;
    //     }

    //     delay(1000);
    //     bno.setExtCrystalUse(true);

    //     if (!mpl.begin())
    //     {
    //         Serial.println("No altimeter detected");
    //         return;
    //     }
    //     delay(1000);

    //     Serial.println("Calibration values:\n0=uncalibrated, 3=fully calibrated");
    //     Serial.println("");
    //     Serial.println("----------------------------------------------");
    // }

    // void loop()
    // {
    //     float pres_pa, temp_C, alt_m = 0.0;

    //     /* Display calibration status for each sensor. */
    //     uint8_t sys, gyro, accel, mag;
    //     bno.getCalibration(&sys, &gyro, &accel, &mag);

    //     if (gyro == 3 and accel == 3 and mag == 3)
    //     {
    //         // Possible vector values can be:
    //         // - VECTOR_ACCELEROMETER - m/s^2
    //         // - VECTOR_MAGNETOMETER  - uT
    //         // - VECTOR_GYROSCOPE     - rad/s
    //         // - VECTOR_EULER         - degrees
    //         // - VECTOR_LINEARACCEL   - m/s^2
    //         // - VECTOR_GRAVITY       - m/s^2

    //         // while (gpsSerial.available())
    //         // {
    //         //     if (gps.encode(gpsSerial.read()))
    //         //     {
    //         //         lat = gps.location.lat();
    //         //         lon = gps.location.lng();
    //         //         alt = gps.altitude.meters();

    //         //         Serial.print(F("Sats: "));
    //         //         Serial.println(gps.satellites.value());
    //         //         Serial.print("Latitude: ");
    //         //         Serial.println(lat, 6);
    //         //         Serial.print("Longitude: ");
    //         //         Serial.println(lon, 6);
    //         //         Serial.print("Altitude: ");
    //         //         Serial.println(alt, 2);
    //         //         Serial.println();
    //         //     }
    //         // }

    //         imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    //         imu::Vector<3> ang = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    //         imu::Vector<3> grav = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);

    //         /* Display the floating point data */
    //         Serial.print("ACCELERATION:  ");
    //         Serial.print("X: ");
    //         Serial.print(acc.x(), 2);
    //         Serial.print(" Y: ");
    //         Serial.print(acc.y(), 2);
    //         Serial.print(" Z: ");
    //         Serial.print(acc.z(), 2);
    //         Serial.println();

    //         Serial.print("ROTATION:      ");
    //         Serial.print("X: ");
    //         Serial.print(ang.x(), 2);
    //         Serial.print(" Y: ");
    //         Serial.print(ang.y(), 2);
    //         Serial.print(" Z: ");
    //         Serial.print(ang.z(), 2);
    //         Serial.println();

    //         Serial.print("GRAVITY VEC:   ");
    //         Serial.print("X: ");
    //         Serial.print(grav.x(), 2);
    //         Serial.print(" Y: ");
    //         Serial.print(grav.y(), 2);
    //         Serial.print(" Z: ");
    //         Serial.print(grav.z(), 2);
    //         Serial.println();

    //         pres_pa = mpl.getPressure();
    //         Serial.print("AMBIENT PRESSURE:    ");
    //         Serial.print(pres_pa / 101325, 4);
    //         Serial.println(" atm");

    //         // Altitude can be calculated directly through the following equation
    //         alt_m = 44330.77 * (1 - pow(pres_pa / 101326.0, 0.1902632));
    //         // alt_m = mpl.getAltitude();
    //         Serial.print("BAROMETRIC ALTITUDE: ");
    //         Serial.print(alt_m, 2);
    //         Serial.println(" m");

    //         temp_C = mpl.getTemperature();
    //         Serial.print("AMBIENT TEMPERATURE:  ");
    //         Serial.print(temp_C, 2);
    //         Serial.println(" *C");

    //         Serial.println("----------------------------------------------");
    //     }
    //     else
    //     {
    //         Serial.print("CALIBRATION: Sys=");
    //         Serial.print(sys, DEC);
    //         Serial.print(" Gyro=");
    //         Serial.print(gyro, DEC);
    //         Serial.print(" Accel=");
    //         Serial.print(accel, DEC);
    //         Serial.print(" Mag=");
    //         Serial.println(mag, DEC);
    //         Serial.println("----------------------------------------------");
    //     }

    //     delay(BNO055_SAMPLERATE_DELAY_MS);
    // }