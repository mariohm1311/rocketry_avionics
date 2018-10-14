#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_MPL3115A2.h>
#include <utility/imumaths.h>
// #include <TinyGPS++.h>
#include <NMEAGPS.h>
#include <GPSport.h>
#include <GPSfix_cfg.h>
#include <Streamers.h>

// #define NMEAGPS_INTERRUPT_PROCESSING

#ifndef NMEAGPS_INTERRUPT_PROCESSING
#error You must define NMEAGPS_INTERRUPT_PROCESSING in NMEAGPS_cfg.h!
#endif

static NMEAGPS gps;
static gps_fix fix_data;

// TinyGPSPlus gps;
// SoftwareSerial gpsSerial(2, 3);
float lat = 0.0;
float lon = 0.0;
float alt = 0.0;

#define BNO055_SAMPLERATE_DELAY_MS (1000)
int calib_data_addr = 0x0000;
const int imu_id = 55;
adafruit_bno055_offsets_t calib_data;
Adafruit_BNO055 bno = Adafruit_BNO055(imu_id);
sensor_t imu_sensor;
uint8_t sys, gyro, accel, mag;

Adafruit_MPL3115A2 mpl = Adafruit_MPL3115A2();
float pres_pa, temp_C, alt_m;

static void GPSisr(uint8_t c)
{
    gps.handle(c);
} // GPSisr

void displayCalStatus(void)
{
    sys = gyro = accel = mag = 0;
    bno.getCalibration(&sys, &gyro, &accel, &mag);

    if (!sys)
    {
        Serial.print("! ");
    }

    Serial.print("Sys=");
    Serial.print(sys, DEC);
    Serial.print(" Gyro=");
    Serial.print(gyro, DEC);
    Serial.print(" Accel=");
    Serial.print(accel, DEC);
    Serial.print(" Mag=");
    Serial.println(mag, DEC);
}

void displaySensorOffsets(adafruit_bno055_offsets_t &calib_data)
{
    Serial.print("Accelerometer: ");
    Serial.print(calib_data.accel_offset_x);
    Serial.print(" ");
    Serial.print(calib_data.accel_offset_y);
    Serial.print(" ");
    Serial.print(calib_data.accel_offset_z);
    Serial.print(" ");

    Serial.print("\nGyro: ");
    Serial.print(calib_data.gyro_offset_x);
    Serial.print(" ");
    Serial.print(calib_data.gyro_offset_y);
    Serial.print(" ");
    Serial.print(calib_data.gyro_offset_z);
    Serial.print(" ");

    Serial.print("\nMag: ");
    Serial.print(calib_data.mag_offset_x);
    Serial.print(" ");
    Serial.print(calib_data.mag_offset_y);
    Serial.print(" ");
    Serial.print(calib_data.mag_offset_z);
    Serial.print(" ");

    Serial.print("\nAccel Radius: ");
    Serial.print(calib_data.accel_radius);

    Serial.print("\nMag Radius: ");
    Serial.print(calib_data.mag_radius);
}

void bnoStartup(void)
{
    long bno_id;
    bool found_calib = false;

    EEPROM.get(calib_data_addr, bno_id);

    bno.getSensor(&imu_sensor);
    if (bno_id != imu_sensor.sensor_id)
    {
        Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
        delay(500);
    }
    else
    {
        Serial.println("\nFound Calibration for this sensor in EEPROM.");
        calib_data_addr += sizeof(long);
        EEPROM.get(calib_data_addr, calib_data);

        displaySensorOffsets(calib_data);

        Serial.println("\n\nRestoring Calibration data to the BNO055...");
        bno.setSensorOffsets(calib_data);

        Serial.println("\n\nCalibration data loaded into BNO055");
        found_calib = true;
    }

    delay(1000);
    bno.setExtCrystalUse(true); // Crystal must be configured AFTER calibration

    if (found_calib)
    {
        Serial.println("Move sensor slightly to calibrate magnetometers");
        while (!bno.isFullyCalibrated())
        {
            delay(BNO055_SAMPLERATE_DELAY_MS);
        }
    }
    else
    {
        Serial.println("Please Calibrate Sensor: ");
        while (!bno.isFullyCalibrated())
        {
            imu::Vector<3> ang = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

            Serial.print("X: ");
            Serial.print(ang.x(), 4);
            Serial.print("\tY: ");
            Serial.print(ang.y(), 4);
            Serial.print("\tZ: ");
            Serial.print(ang.z(), 4);

            Serial.print("\t");
            displayCalStatus();
            Serial.println();
            delay(BNO055_SAMPLERATE_DELAY_MS);
        }
    }

    Serial.println("\nFully calibrated!");
    Serial.println("--------------------------------");
    Serial.println("Calibration Results: ");
    bno.getSensorOffsets(calib_data);
    displaySensorOffsets(calib_data);

    Serial.println("\n\nStoring calibration data to EEPROM...");

    calib_data_addr = 0x0000;
    bno.getSensor(&imu_sensor);
    bno_id = imu_sensor.sensor_id;
    EEPROM.put(calib_data_addr, bno_id);

    calib_data_addr += sizeof(long);
    EEPROM.put(calib_data_addr, calib_data);
    Serial.println("Data stored to EEPROM.");
}

void setup()
{
    Serial.begin(9600);

    gpsPort.attachInterrupt(GPSisr);
    gpsPort.begin(9600);
    // gpsSerial.begin(9600);

    Serial.println("##############################################");
    Serial.println("          IMU + ALTIMETER + GPS TEST          ");
    Serial.println("##############################################");

    if (!bno.begin())
    {
        Serial.println("No IMU detected");
        while (1)
            ;
    }

    delay(1000);

    bnoStartup();

    if (!mpl.begin())
    {
        Serial.println("No altimeter detected");
        return;
    }
    delay(1000);

    Serial.println("Calibration values:\n0=uncalibrated, 3=fully calibrated");
    Serial.println("");
    Serial.println("----------------------------------------------");
}

void loop()
{
    pres_pa = temp_C = alt_m = 0.0;

    // Possible vector values can be:
    // - VECTOR_ACCELEROMETER - m/s^2
    // - VECTOR_MAGNETOMETER  - uT
    // - VECTOR_GYROSCOPE     - rad/s
    // - VECTOR_EULER         - degrees
    // - VECTOR_LINEARACCEL   - m/s^2
    // - VECTOR_GRAVITY       - m/s^2
    Serial.println(fix_data.altitude());
    if (gps.available())
    {
        // Print all the things!
        fix_data = gps.fix();
        Serial.println(fix_data.altitude());
        trace_all(Serial, gps, fix_data);
    }

    // while (gpsSerial.available())
    // {
    //     if (gps.encode(gpsSerial.read()))
    //     {
    //         lat = gps.location.lat();
    //         lon = gps.location.lng();
    //         alt = gps.altitude.meters();

    //         Serial.print(F("Sats: "));
    //         Serial.println(gps.satellites.value());
    //         Serial.print("Latitude: ");
    //         Serial.println(lat, 6);
    //         Serial.print("Longitude: ");
    //         Serial.println(lon, 6);
    //         Serial.print("Altitude: ");
    //         Serial.println(alt, 2);
    //         Serial.println();
    //     }
    // }
    Serial.print("CALIBRATION: ");
    displayCalStatus();

    imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu::Vector<3> ang = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    imu::Vector<3> grav = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);

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

    pres_pa = mpl.getPressure();
    Serial.print("AMBIENT PRESSURE:    ");
    Serial.print(pres_pa / 101325, 4);
    Serial.println(" atm");

    // Altitude can be calculated directly through the following equation
    alt_m = 44330.77 * (1 - pow(pres_pa / 101326.0, 0.1902632));
    // alt_m = mpl.getAltitude();
    Serial.print("BAROMETRIC ALTITUDE: ");
    Serial.print(alt_m, 2);
    Serial.println(" m");

    temp_C = mpl.getTemperature();
    Serial.print("AMBIENT TEMPERATURE:  ");
    Serial.print(temp_C, 2);
    Serial.println(" *C");

    Serial.println("----------------------------------------------");

    delay(BNO055_SAMPLERATE_DELAY_MS);
}

// #include <Arduino.h>
// #include <Wire.h>
// #include <SoftwareSerial.h>
// #include <Adafruit_Sensor.h>
// #include <Adafruit_BNO055.h>
// #include <Adafruit_MPL3115A2.h>
// #include <utility/imumaths.h>
// // #include <TinyGPS++.h>
// #include <NMEAGPS.h>
// #include <GPSport.h>
// #include <GPSfix_cfg.h>
// #include <Streamers.h>

// // #define NMEAGPS_INTERRUPT_PROCESSING

// #ifndef NMEAGPS_INTERRUPT_PROCESSING
//     #error You must define NMEAGPS_INTERRUPT_PROCESSING in NMEAGPS_cfg.h!
// #endif

// static NMEAGPS gps;
// static gps_fix fix_data;

// // TinyGPSPlus gps;
// // SoftwareSerial gpsSerial(2, 3);
// float lat = 0.0;
// float lon = 0.0;
// float alt = 0.0;

// #define BNO055_SAMPLERATE_DELAY_MS (1000)
// Adafruit_BNO055 bno = Adafruit_BNO055();
// Adafruit_MPL3115A2 mpl = Adafruit_MPL3115A2();

// static void GPSisr(uint8_t c)
// {
//     gps.handle(c);
// } // GPSisr

// void setup()
// {
//     Serial.begin(9600);

//     gpsPort.attachInterrupt(GPSisr);
//     gpsPort.begin(9600);
//     // gpsSerial.begin(9600);

//     Serial.println("##############################################");
//     Serial.println("          IMU + ALTIMETER + GPS TEST          ");
//     Serial.println("##############################################");

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

//     // Possible vector values can be:
//     // - VECTOR_ACCELEROMETER - m/s^2
//     // - VECTOR_MAGNETOMETER  - uT
//     // - VECTOR_GYROSCOPE     - rad/s
//     // - VECTOR_EULER         - degrees
//     // - VECTOR_LINEARACCEL   - m/s^2
//     // - VECTOR_GRAVITY       - m/s^2
//     Serial.println(fix_data.altitude());
//     if (gps.available())
//     {
//         // Print all the things!
//         fix_data = gps.fix();
//         Serial.println(fix_data.altitude());
//         trace_all(Serial, gps, fix_data);
//     }

//     // while (gpsSerial.available())
//     // {
//     //     if (gps.encode(gpsSerial.read()))
//     //     {
//     //         lat = gps.location.lat();
//     //         lon = gps.location.lng();
//     //         alt = gps.altitude.meters();

//     //         Serial.print(F("Sats: "));
//     //         Serial.println(gps.satellites.value());
//     //         Serial.print("Latitude: ");
//     //         Serial.println(lat, 6);
//     //         Serial.print("Longitude: ");
//     //         Serial.println(lon, 6);
//     //         Serial.print("Altitude: ");
//     //         Serial.println(alt, 2);
//     //         Serial.println();
//     //     }
//     // }
//     Serial.print("CALIBRATION: Sys=");
//     Serial.print(sys, DEC);
//     Serial.print(" Gyro=");
//     Serial.print(gyro, DEC);
//     Serial.print(" Accel=");
//     Serial.print(accel, DEC);
//     Serial.print(" Mag=");
//     Serial.println(mag, DEC);
    
//     imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
//     imu::Vector<3> ang = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
//     imu::Vector<3> grav = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);

//     /* Display the floating point data */
//     Serial.print("ACCELERATION:  ");
//     Serial.print("X: ");
//     Serial.print(acc.x(), 2);
//     Serial.print(" Y: ");
//     Serial.print(acc.y(), 2);
//     Serial.print(" Z: ");
//     Serial.print(acc.z(), 2);
//     Serial.println();

//     Serial.print("ROTATION:      ");
//     Serial.print("X: ");
//     Serial.print(ang.x(), 2);
//     Serial.print(" Y: ");
//     Serial.print(ang.y(), 2);
//     Serial.print(" Z: ");
//     Serial.print(ang.z(), 2);
//     Serial.println();

//     Serial.print("GRAVITY VEC:   ");
//     Serial.print("X: ");
//     Serial.print(grav.x(), 2);
//     Serial.print(" Y: ");
//     Serial.print(grav.y(), 2);
//     Serial.print(" Z: ");
//     Serial.print(grav.z(), 2);
//     Serial.println();

//     pres_pa = mpl.getPressure();
//     Serial.print("AMBIENT PRESSURE:    ");
//     Serial.print(pres_pa / 101325, 4);
//     Serial.println(" atm");

//     // Altitude can be calculated directly through the following equation
//     alt_m = 44330.77 * (1 - pow(pres_pa / 101326.0, 0.1902632));
//     // alt_m = mpl.getAltitude();
//     Serial.print("BAROMETRIC ALTITUDE: ");
//     Serial.print(alt_m, 2);
//     Serial.println(" m");

//     temp_C = mpl.getTemperature();
//     Serial.print("AMBIENT TEMPERATURE:  ");
//     Serial.print(temp_C, 2);
//     Serial.println(" *C");

//     Serial.println("----------------------------------------------");

//     delay(BNO055_SAMPLERATE_DELAY_MS);
// }