#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS (1000)
Adafruit_BNO055 bno = Adafruit_BNO055();

void setup()
{
    Serial.begin(9600);

    Serial.println("##############################################");
    Serial.println("                   IMU TEST                   ");
    Serial.println("##############################################");

    if (!bno.begin())
    {
        Serial.print("No IMU detected");
        while (1)
            ;
    }

    delay(1000);
    bno.setExtCrystalUse(true);

    Serial.println("Calibration values:\n0=uncalibrated, 3=fully calibrated");
    Serial.println("");
    Serial.println("----------------------------------------------");
}

void loop()
{
    /* Display calibration status for each sensor. */
    uint8_t sys, gyro, accel, mag;
    bno.getCalibration(&sys, &gyro, &accel, &mag);

    if (gyro == 3 and accel == 3 and mag == 3)
    {
        // Possible vector values can be:
        // - VECTOR_ACCELEROMETER - m/s^2
        // - VECTOR_MAGNETOMETER  - uT
        // - VECTOR_GYROSCOPE     - rad/s
        // - VECTOR_EULER         - degrees
        // - VECTOR_LINEARACCEL   - m/s^2
        // - VECTOR_GRAVITY       - m/s^2
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

        Serial.println("----------------------------------------------");
    }
    else
    {
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

    delay(BNO055_SAMPLERATE_DELAY_MS);
}