#include <Arduino.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

TinyGPSPlus gps;
SoftwareSerial gpsSerial(2,3);
float lat = 0.0;
float lon = 0.0;
float alt = 0.0;

// void clearScreen()
// {
//     for (int i = 0; i < 100; i++)
//     {
//         Serial.println("");
//     }
// }

void setup()
{
    Serial.begin(9600);
    gpsSerial.begin(9600);
}

void loop()
{
    // clearScreen();
    while (gpsSerial.available())
    {
        if (gps.encode(gpsSerial.read()))
        {
            lat = gps.location.lat();
            lon = gps.location.lng();
            alt = gps.altitude.meters();

            Serial.print(F("\n\nSats: "));
            Serial.print(gps.satellites.value());
            Serial.print("\nLatitude: ");
            Serial.print(lat, 6);
            Serial.print("\nLongitude: ");
            Serial.print(lon, 6);
            Serial.print("\nAltitude: ");
            Serial.print(alt, 2);
        }
    }

    delay(500);
}