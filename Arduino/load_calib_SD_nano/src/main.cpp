#include <Arduino.h>
#include <Wire.h>
// #include <EEPROM.h>
#include <SD.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Serialize.h>

File calib_file;
bool load_calib = false;

#define BNO055_SAMPLERATE_DELAY_MS (1000)
int calib_data_addr = 0x0000;
const int imu_id = 54;
adafruit_bno055_offsets_t calib_data;
Adafruit_BNO055 bno = Adafruit_BNO055(imu_id);
sensor_t imu_sensor;
uint8_t sys, gyro, accel, mag;

typedef Serialize<adafruit_bno055_offsets_t> CalFrame;
CalFrame calib_frame;

bool saveSensorOffsets(void)
{
    bno.getSensorOffsets(calib_data);
    calib_frame.data = calib_data;

    calib_file = SD.open(String("imu") + String(imu_id) + String(".cal"), FILE_WRITE);
    if (calib_file)
    {
        calib_file.seek(0);
        calib_file.write(calib_frame.bytes, sizeof(calib_data));
        calib_file.close();
        return true;
    }
    else
    {
        Serial.println("Failed to save calibration");
        return false;
    }
}

bool readSensorOffsets(void)
{
    bool do_read = SD.exists(String("imu") + String(imu_id) + String(".cal"));
    if (do_read)
    {
        calib_file = SD.open(String("imu") + String(imu_id) + String(".cal"), FILE_READ);
    }

    if (calib_file and calib_file.available())
    {
        calib_file.seek(0);
        calib_file.read(calib_frame.bytes, sizeof(calib_data));
        calib_file.close();
        calib_data = calib_frame.data;
        bno.setSensorOffsets(calib_data);
        return true;
    }
    else
    {
        Serial.println("Failed to save calibration");
        return false;
    }
    return false;
}

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
    bool found_calib = false;
    bool load_calib = SD.exists(String("imu") + String(imu_id) + String(".cal"));

    if (!load_calib)
    {
        Serial.println("\nNot loading calibration data");
        delay(500);
    }
    else
    {
        Serial.println("\nFound calibration data");
        found_calib = readSensorOffsets();
        calib_data = calib_frame.data;
        displaySensorOffsets(calib_data);

        Serial.println("\n\nRestoring Calibration data to the BNO055...");
        bno.setSensorOffsets(calib_data);

        Serial.println("\n\nCalibration data loaded into BNO055");
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
    // Serial.println("--------------------------------");
    // Serial.println("Calibration Results: ");
    bno.getSensorOffsets(calib_frame.data);
    calib_data = calib_frame.data;
    displaySensorOffsets(calib_data);

    // Serial.println("\n\nStoring calibration data to SD card...");

    calib_data_addr += sizeof(long);
    saveSensorOffsets();
    Serial.println("Data stored to SD card.");
}

void setup()
{
    Serial.begin(9600);

    // Serial.println("##############################################");
    // Serial.println("          IMU + ALTIMETER + GPS TEST          ");
    // Serial.println("##############################################");

    Serial.print("Initializing SD card...");
    if (!SD.begin(4))
    {
        Serial.println("initialization failed!");
        while (1)
            ;
    }
    else
    {
        Serial.println();
    }
    delay(1000);

    if (!bno.begin())
    {
        // Serial.println("No IMU detected");
        while (1)
            ;
    }

    delay(1000);

    bnoStartup();

    // Serial.println("Calibration values:\n0=uncalibrated, 3=fully calibrated");
    // Serial.println("");
    // Serial.println("----------------------------------------------");
}

void loop()
{

    // Possible vector values can be:
    // - VECTOR_ACCELEROMETER - m/s^2
    // - VECTOR_MAGNETOMETER  - uT
    // - VECTOR_GYROSCOPE     - rad/s
    // - VECTOR_EULER         - degrees
    // - VECTOR_LINEARACCEL   - m/s^2
    // - VECTOR_GRAVITY       - m/s^2
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

    // Serial.println("----------------------------------------------");

    delay(BNO055_SAMPLERATE_DELAY_MS);
}